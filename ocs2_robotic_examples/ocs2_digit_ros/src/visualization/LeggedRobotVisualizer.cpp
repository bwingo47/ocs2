//
// Created by bruce on 7/20/22.
//

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_digit_ros/visualization/LeggedRobotVisualizer.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include "ocs2_legged_robot/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ocs2 {
namespace digit {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DigitVisualizer::DigitVisualizer(PinocchioInterface pinocchio_interface,
                                 CentroidalModelInfo centroidal_model_info,
                                 const PinocchioEndEffectorKinematics end_effector_kinematics,
                                 ros::NodeHandle &node_handle, scalar_t max_update_freq)
    : endEffectorKinematicsPtr_(end_effector_kinematics.clone()),
      centroidalModelInfo_(centroidal_model_info),
      pinocchioInterface_(pinocchio_interface),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / max_update_freq)
{
    endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
    launchVisualizerNode(node_handle);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DigitVisualizer::update(const SystemObservation &observation, const PrimalSolution &primal_solution,
                             const CommandData &command)
{

}

}
}
