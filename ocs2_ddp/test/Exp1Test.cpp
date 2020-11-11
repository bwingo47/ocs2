/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/EXP1.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>

class Exp1Test : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr ocs2::scalar_t expectedCost = 5.4399;
  static constexpr ocs2::scalar_t expectedStateInputEqConstraintISE = 0.0;
  static constexpr ocs2::scalar_t expectedStateEqConstraintISE = 0.0;

  Exp1Test() {
    // event times
    const ocs2::scalar_array_t eventTimes{0.2262, 1.0176};
    const std::vector<size_t> subsystemsSequence{0, 1, 2};
    modeScheduleManagerPtr.reset(new ocs2::ModeScheduleManager({eventTimes, subsystemsSequence}));

    // partitioning times
    partitioningTimes = ocs2::scalar_array_t{startTime, eventTimes[0], eventTimes[1], finalTime};

    // rollout settings
    const auto rolloutSettings = []() {
      ocs2::rollout::Settings rolloutSettings;
      rolloutSettings.absTolODE_ = 1e-10;
      rolloutSettings.relTolODE_ = 1e-7;
      rolloutSettings.maxNumStepsPerSecond_ = 10000;
      return rolloutSettings;
    }();

    // dynamics and rollout
    systemPtr.reset(new ocs2::EXP1_System(modeScheduleManagerPtr));
    rolloutPtr.reset(new ocs2::TimeTriggeredRollout(*systemPtr, rolloutSettings));

    // cost function
    costPtr.reset(new ocs2::EXP1_CostFunction(modeScheduleManagerPtr));

    // constraint
    constraintPtr.reset(new ocs2::ConstraintBase);

    // operatingTrajectories
    const auto stateOperatingPoint = ocs2::vector_t::Zero(STATE_DIM);
    const auto inputOperatingPoint = ocs2::vector_t::Zero(INPUT_DIM);
    operatingPointsPtr.reset(new ocs2::OperatingPoints(stateOperatingPoint, inputOperatingPoint));
  }

  ocs2::ddp::Settings getSettings(ocs2::ddp::Algorithm algorithmType, size_t numThreads, ocs2::search_strategy::Type strategy,
                                  bool display = false) const {
    ocs2::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.nThreads_ = numThreads;
    ddpSettings.preComputeRiccatiTerms_ = false;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.maxNumIterations_ = 30;
    ddpSettings.checkNumericalStability_ = true;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.useNominalTimeForBackwardPass_ = true;
    ddpSettings.useFeedbackPolicy_ = false;
    ddpSettings.debugPrintRollout_ = false;
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength_ = 0.0001;
    return ddpSettings;
  }

  std::string getTestName(const ocs2::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "EXP1 Test { ";
    testName += "Algorithm: " + ocs2::ddp::toAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + ocs2::search_strategy::toString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void performanceIndexTest(const ocs2::ddp::Settings& ddpSettings, const ocs2::PerformanceIndex& performanceIndex) const {
    const auto testName = getTestName(ddpSettings);
    EXPECT_LT(fabs(performanceIndex.totalCost - expectedCost), 10 * ddpSettings.minRelCost_)
        << "MESSAGE: " << testName << ": failed in the total cost test!";
    EXPECT_LT(fabs(performanceIndex.stateInputEqConstraintISE - expectedStateInputEqConstraintISE), 10 * ddpSettings.constraintTolerance_)
        << "MESSAGE: " << testName << ": failed in state-input equality constraint ISE test!";
    EXPECT_LT(fabs(performanceIndex.stateEqConstraintISE - expectedStateEqConstraintISE), 10 * ddpSettings.constraintTolerance_)
        << "MESSAGE: " << testName << ": failed in state-only equality constraint ISE test!";
  }

  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 3.0;
  const ocs2::vector_t initState = (ocs2::vector_t(STATE_DIM) << 2.0, 3.0).finished();
  ocs2::scalar_array_t partitioningTimes;
  std::shared_ptr<ocs2::ModeScheduleManager> modeScheduleManagerPtr;

  std::unique_ptr<ocs2::SystemDynamicsBase> systemPtr;
  std::unique_ptr<ocs2::TimeTriggeredRollout> rolloutPtr;
  std::unique_ptr<ocs2::CostFunctionBase> costPtr;
  std::unique_ptr<ocs2::ConstraintBase> constraintPtr;
  std::unique_ptr<ocs2::OperatingPoints> operatingPointsPtr;
};

constexpr size_t Exp1Test::STATE_DIM;
constexpr size_t Exp1Test::INPUT_DIM;
constexpr ocs2::scalar_t Exp1Test::expectedCost;
constexpr ocs2::scalar_t Exp1Test::expectedStateInputEqConstraintISE;
constexpr ocs2::scalar_t Exp1Test::expectedStateEqConstraintISE;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1Test, slq_single_thread_linesearch) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::SLQ, 1, ocs2::search_strategy::Type::LINE_SEARCH);

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1Test, slq_multi_thread_linesearch) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::SLQ, 3, ocs2::search_strategy::Type::LINE_SEARCH);

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1Test, ilqr_single_thread_linesearch) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::ILQR, 1, ocs2::search_strategy::Type::LINE_SEARCH);

  // instantiate
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1Test, ilqr_multi_thread_linesearch) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::ILQR, 3, ocs2::search_strategy::Type::LINE_SEARCH);

  // instantiate
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1Test, slq_single_thread_levenberg_marquardt) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::SLQ, 1, ocs2::search_strategy::Type::LEVENBERG_MARQUARDT);

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1Test, slq_multi_thread_levenberg_marquardt) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::SLQ, 3, ocs2::search_strategy::Type::LEVENBERG_MARQUARDT);

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1Test, ilqr_single_thread_levenberg_marquardt) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::ILQR, 1, ocs2::search_strategy::Type::LEVENBERG_MARQUARDT);

  // instantiate
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1Test, ilqr_multi_thread_levenberg_marquardt) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::ILQR, 3, ocs2::search_strategy::Type::LEVENBERG_MARQUARDT);

  // instantiate
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}