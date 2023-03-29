//
// Created by bruce on 10/4/22.
//

#include <iostream>

#include "ocs2_legged_robot/constraint/EndEffectorLinearConstraintCppAd.h"

namespace ocs2 {
namespace legged_robot {


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLinearConstraintCppAd::EndEffectorLinearConstraintCppAd(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                   size_t numConstraints,
                                                                   size_t stateDim,
                                                                   size_t inputDim,
                                                                   const std::string& modelName,
                                                                   const std::string& modelFolder,
                                                                   bool recompileLibraries,
                                                                   bool verbose,
                                                                   ConfigCppAd configCppAd)
    : StateInputConstraint(ConstraintOrder::Linear),
      numConstraints_(numConstraints),
      configCppAd_(std::move(configCppAd)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone())
{

//  size_t eePosDim = config_.Ax.cols();
//  size_t eeVelDim = config_.Av.cols();
//  size_t adFuncStateDim = eePosDim + eeVelDim;

  size_t adFuncStateDim = stateDim + inputDim;

  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
  }


  // constraint function handle
  auto constraintFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
    const ad_vector_t state = x.head(stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = getConstraintValueCppAd(configCppAd_, *endEffectorKinematicsPtr_, state, input);
  };

  constraintCppAdInterfacePtr_.reset(new CppAdInterface(constraintFunc,
                                                        adFuncStateDim,
                                                        modelName,
                                                        modelFolder));

  if (recompileLibraries) {
    constraintCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    constraintCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLinearConstraintCppAd::EndEffectorLinearConstraintCppAd(const ocs2::legged_robot::EndEffectorLinearConstraintCppAd &rhs)
  : StateInputConstraint(rhs),
    endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
    numConstraints_(rhs.numConstraints_),
    configCppAd_(rhs.configCppAd_),
    constraintCppAdInterfacePtr_(new CppAdInterface(*rhs.constraintCppAdInterfacePtr_)) {}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorLinearConstraintCppAd::configure(ConfigCppAd&& config) {
  assert(config.b.rows() == numConstraints_);
  assert(config.Ax.size() > 0 || config.Av.size() > 0);
  assert((config.Ax.size() > 0 && config.Ax.rows() == numConstraints_) || config.Ax.size() == 0);
  assert((config.Ax.size() > 0 && config.Ax.cols() == 3) || config.Ax.size() == 0);
  assert((config.Av.size() > 0 && config.Av.rows() == numConstraints_) || config.Av.size() == 0);
  assert((config.Av.size() > 0 && config.Av.cols() == 3) || config.Av.size() == 0);
  configCppAd_ = std::move(config);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t EndEffectorLinearConstraintCppAd::getConstraintValueCppAd(const ConfigCppAd &configCppAd,
                                                                      EndEffectorKinematics<scalar_t> &eefKinematics,
                                                                      const ocs2::ad_vector_t &state,
                                                                      const ocs2::ad_vector_t &input)
{
  std::cout << "================inside getConstraintValueCppAd====================" << std::endl;
  auto* pinocchioEefKinematicsCppAdPtr = dynamic_cast<PinocchioEndEffectorKinematicsCppAd*>(&eefKinematics);
  // check for null pointer, i.e. dynamic casting failure, eefKinematics is not of type PinocchioEndEffectorKinematicsCppAd
  assert(pinocchioEefKinematicsCppAdPtr);

  // check for null pointer using print statements
  if (pinocchioEefKinematicsCppAdPtr == nullptr) {
    std::cout << "**************pinocchioEefKinematicsCppAdPtr is a null pointer*****************" << std::endl;
  }

  std::cout << "================after assert====================" << std::endl;
  std::cout << "configCppAd.b size: " << configCppAd.b.size() << std::endl;
  std::cout << "configCppAd.Ax size: " << configCppAd.Ax.size() << std::endl;
  std::cout << "configCppAd.Av size: " << configCppAd.Av.size() << std::endl;

  ad_vector_t f = configCppAd.b;
  if (configCppAd.Ax.size() > 0) {
    std::cout << "================inside 1st if====================" << std::endl;
    pinocchioEefKinematicsCppAdPtr->getPositionCppAdExternal(state);
//    f.noalias() += configCppAd.Ax * pinocchioEefKinematicsCppAdPtr->getPositionCppAdExternal(state).front();
//    ad_vector_t dummy = ad_vector_t::Ones(3);
    f.noalias() += configCppAd.Ax * state.segment<3>(0, 3);
  }
  if (configCppAd.Av.size() > 0) {
    std::cout << "================inside 2nd if====================" << std::endl;
//    f.noalias() += configCppAd.Av * pinocchioEefKinematicsCppAdPtr->getVelocityCppAdExternal(state, input).front();
//    ad_vector_t dummy = ad_vector_t::Ones(3);
    f.noalias() += configCppAd.Av * (state.segment<3>(0, 3) + input.segment<3>(0, 3));
  }
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorLinearConstraintCppAd::getValue(scalar_t time,
                                                    const vector_t &state,
                                                    const vector_t &input,
                                                    const PreComputation &preComp) const
{
  vector_t stateInput(state.rows() + input.rows());
  std::cout << "================inside getValue====================" << std::endl;
  stateInput << state, input;
  vector_t result = constraintCppAdInterfacePtr_->getFunctionValue(stateInput);
  std::cout << "================after getFunctionValue====================" << std::endl;
//  vector_t result(3);
//  result << 1.0, 1.0, 1.0;
  return result;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorLinearConstraintCppAd::getLinearApproximation(scalar_t time,
                                                                                           const vector_t &state,
                                                                                           const vector_t &input,
                                                                                           const PreComputation &preComp) const
{
  vector_t stateInput(state.rows() + input.rows());
  stateInput << state, input;

  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

  linearApproximation.f = constraintCppAdInterfacePtr_->getFunctionValue(stateInput);

  matrix_t constraintJacobian = constraintCppAdInterfacePtr_->getJacobian(stateInput);

  linearApproximation.dfdx = constraintJacobian.block(0, 0, numConstraints_, state.rows());
  linearApproximation.dfdu = constraintJacobian.block(0, state.rows(), numConstraints_, input.rows());

  return linearApproximation;
}

} // legged_robot
} // ocs2