//
// Created by bruce on 10/4/22.
//

#ifndef OCS2_LEGGED_ROBOT_INCLUDE_OCS2_LEGGED_ROBOT_CONSTRAINT_ENDEFFECTORLINEARCONSTRAINTCPPAD_H_
#define OCS2_LEGGED_ROBOT_INCLUDE_OCS2_LEGGED_ROBOT_CONSTRAINT_ENDEFFECTORLINEARCONSTRAINTCPPAD_H_

#include <functional>
#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include "ocs2_legged_robot/constraint/EndEffectorLinearConstraint.h"

namespace ocs2 {
namespace legged_robot {

class EndEffectorLinearConstraintCppAd final : public StateInputConstraint {
 public:

  /**
   * Coefficients of the linear constraints of the form:
   * g(xee, vee) = Ax * xee + Av * vee + b
   */

  struct ConfigCppAd {
    ad_vector_t b;
    ad_matrix_t Ax;
    ad_matrix_t Av;

//    ConfigCppAd() = default;
//    ConfigCppAd(const ConfigCppAd& rhs) = default;

    void fromStandardConfig(EndEffectorLinearConstraint::Config& config)
    {
      b = config.b.template cast<ad_scalar_t>();
      Ax = config.Ax.template cast<ad_scalar_t>();
      Av = config.Av.template cast<ad_scalar_t>();
    }
  };

//  struct Config {
//    vector_t b;
//    matrix_t Ax;
//    matrix_t Av;
//
//    ConfigCppAd toCppAd() {
//      ConfigCppAd config = ConfigCppAd();
//      config.b = b.template cast<ad_scalar_t>();
//      config.Ax = Ax.template cast<ad_scalar_t>();
//      config.Av = Av.template cast<ad_scalar_t>();
//
//      return config;
//    }
//  };


  /**
   * Constructor
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] numConstraints: The number of constraints {1, 2, 3}
   * @param [in] config: The constraint coefficients, g(xee, vee) = Ax * xee + Av * vee + b
   */
  EndEffectorLinearConstraintCppAd(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                   size_t numConstraints,
                                   size_t stateDim,
                                   size_t inputDim,
                                   const std::string& modelName,
                                   const std::string& modelFolder,
                                   bool recompileLibraries,
                                   bool verbose,
                                   ConfigCppAd config = ConfigCppAd());

  ~EndEffectorLinearConstraintCppAd() override = default;
  EndEffectorLinearConstraintCppAd* clone() const override { return new EndEffectorLinearConstraintCppAd(*this); }

  /** Sets a new constraint coefficients.
   *  rvalue reference of Config struct facilitates std::move()
   *  instead of copying for assigning member variable config_ */
  void configure(ConfigCppAd&& config);
  /** Sets a new constraint coefficients. */
  void configure(const ConfigCppAd& config)
  {
    // copy constructor Config(const Config& ) takes the lvalue reference
    // config and creates a temporary Config obj that gets passed as a rvalue to the
    // rvalue version of the configure() function.
    this->configure(ConfigCppAd(config));
  }

  /** Gets the underlying end-effector kinematics interface. */
  EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:

  EndEffectorLinearConstraintCppAd(const EndEffectorLinearConstraintCppAd& rhs);

  ad_vector_t getConstraintValueCppAd(const ConfigCppAd& configCppAd,
                                      EndEffectorKinematics<scalar_t>& eefKinematics,
                                      const ad_vector_t& state,
                                      const ad_vector_t& input);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t numConstraints_;
  ConfigCppAd configCppAd_;

  std::unique_ptr<CppAdInterface> constraintCppAdInterfacePtr_;

};

}
}


#endif //OCS2_LEGGED_ROBOT_INCLUDE_OCS2_LEGGED_ROBOT_CONSTRAINT_ENDEFFECTORLINEARCONSTRAINTCPPAD_H_
