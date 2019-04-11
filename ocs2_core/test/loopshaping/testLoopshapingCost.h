//
// Created by rgrandia on 10.04.19.
//

#pragma once

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <experimental/filesystem>

#include "ocs2_core/cost/QuadraticCostFunction.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingCost.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

template <class CONFIG>
class TestFixtureLoopShapingCost : public ::testing::Test {
 protected:
  static constexpr size_t FULL_STATE_DIM = CONFIG::FULL_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = CONFIG::FULL_INPUT_DIM;
  static constexpr size_t SYSTEM_STATE_DIM = CONFIG::SYSTEM_STATE_DIM;
  static constexpr size_t SYSTEM_INPUT_DIM = CONFIG::SYSTEM_INPUT_DIM;
  static constexpr size_t FILTER_STATE_DIM = CONFIG::FILTER_STATE_DIM;
  static constexpr size_t FILTER_INPUT_DIM = CONFIG::FILTER_INPUT_DIM;

  using TestSystemCost = QuadraticCostFunction<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using TestLoopshapingCost = LoopshapingCost<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  using scalar_t = typename TestLoopshapingCost::scalar_t;
  using state_vector_t = typename TestLoopshapingCost::state_vector_t;
  using input_vector_t = typename TestLoopshapingCost::input_vector_t;
  using state_matrix_t = typename TestLoopshapingCost::state_matrix_t;
  using input_matrix_t = typename TestLoopshapingCost::input_matrix_t;
  using state_input_matrix_t = typename TestLoopshapingCost::state_input_matrix_t;
  using input_state_matrix_t = typename TestLoopshapingCost::input_state_matrix_t;
  using system_state_vector_t = typename TestSystemCost::state_vector_t;
  using system_input_vector_t = typename TestSystemCost::input_vector_t;
  using system_state_matrix_t = typename TestSystemCost::state_matrix_t;
  using system_input_matrix_t = typename TestSystemCost::input_matrix_t;
  using system_input_state_matrix_t = typename TestSystemCost::input_state_matrix_t;
  using system_state_input_matrix_t = typename TestSystemCost::state_input_matrix_t;
  using filter_state_vector_t = typename TestLoopshapingCost::filter_state_vector_t;
  using filter_input_vector_t = typename TestLoopshapingCost::filter_input_vector_t;

  void SetUp() override {
    const std::experimental::filesystem::path pathToTest = std::experimental::filesystem::path(__FILE__);
    const std::string settingsFile = std::string(pathToTest.parent_path()) + "/" + CONFIG::fileName;

    // Load loopshaping definition
    loopshapingDefinition_.reset(new LoopshapingDefinition());
    loopshapingDefinition_->loadSettings(settingsFile);

    // Set up state and input
    t = 0.5;
    getRandomStateInput(x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);

    // Create system costs
    system_state_matrix_t Q, Q_final;
    system_input_matrix_t R;
    system_input_state_matrix_t P;
    Q.setRandom();
    Q_final.setRandom();
    R.setRandom();
    P.setRandom();

    // Make symmetric
    Q_final = (0.5*Q_final.transpose() + 0.5*Q_final).eval();
    Q = (0.5*Q.transpose() + 0.5*Q).eval();
    R = (0.5*R.transpose() + 0.5*R).eval();
    testSystemCost.reset(new TestSystemCost(Q, R, x_sys_, u_sys_, Q_final, x_sys_, P));

    // Create Loopshaping costs
    testLoopshapingCost.reset(new TestLoopshapingCost(*testSystemCost, loopshapingDefinition_));
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<TestSystemCost> testSystemCost;
  std::unique_ptr<TestLoopshapingCost> testLoopshapingCost;

  double t;
  state_vector_t x_;
  input_vector_t u_;
  system_state_vector_t x_sys_;
  system_input_vector_t u_sys_;
  filter_state_vector_t x_filter_;
  filter_input_vector_t u_filter_;

  void getRandomStateInput(system_state_vector_t &x_sys,
                           system_input_vector_t &u_sys,
                           filter_state_vector_t &x_filter,
                           filter_input_vector_t &u_filter,
                           state_vector_t &x,
                           input_vector_t &u,
                           double range = 1.0) {
    // Set random state
    x_sys.setRandom();
    u_sys.setRandom();
    x_filter.setRandom();
    u_filter.setRandom();

    // Scale the randomness
    x_sys *= range;
    u_sys *= range;
    x_filter *= range;
    u_filter *= range;

    // Get total state
    loopshapingDefinition_->concatenateSystemAndFilterState(x_sys, x_filter, x);
    loopshapingDefinition_->concatenateSystemAndFilterInput(u_sys, u_filter, u);
  }
};

}; // namespace ocs2
