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