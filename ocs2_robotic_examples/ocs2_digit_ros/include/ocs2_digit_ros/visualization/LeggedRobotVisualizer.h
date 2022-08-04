//
// Created by bruce on 7/20/22.
//

#ifndef OCS2_DIGIT_ROS_LEGGEDROBOTVISUALIZER_H
#define OCS2_DIGIT_ROS_LEGGEDROBOTVISUALIZER_H

#pragma once //(see https://en.wikipedia.org/wiki/Pragma_once)

// ros includes
#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>

// ocs2 core functionality includes
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>

namespace ocs2 {
namespace digit {

class DigitVisualizer : public DummyObserver {
public:
    /** Visualization settings (publicly available) */
    std::string frameId_ = "odom";              // Frame name all messages are published in
    scalar_t footMarkerDiameter_ = 0.03;        // Size of the spheres at the feet
    scalar_t footAlphaWhenLifted_ = 0.3;        // Alpha value when a foot is lifted.
    scalar_t forceScale_ = 1000.0;              // Vector scale in N/m
    scalar_t velScale_ = 5.0;                   // Vector scale in m/s
    scalar_t copMarkerDiameter_ = 0.03;         // Size of the sphere at the center of pressure
    scalar_t supportPolygonLineWidth_ = 0.005;  // LineThickness for the support polygon
    scalar_t trajectoryLineWidth_ = 0.01;       // LineThickness for trajectories
    std::vector<Color> feetColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple};  // Colors for markers per feet

    /**
     *
     * @param pinocchio_interface :
     * @param centroidal_model_info :
     * @param end_effector_kinematics :
     * @param node_handle :
     * @param max_update_frequency : maximum publish frequency measured in MPC time.
     */
    DigitVisualizer(PinocchioInterface pinocchio_interface, CentroidalModelInfo centroidal_model_info,
                    const PinocchioEndEffectorKinematics end_effector_kinematics, ros::NodeHandle& node_handle,
                    scalar_t max_update_freq = 100.0);

    ~DigitVisualizer() override = default;

    void update(const SystemObservation& observation, const PrimalSolution& primal_solution, const CommandData& command) override;

    void launchVisualizerNode(ros::NodeHandle& nodeHandle);

    void publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed = 1.0);

    void publishObservation(ros::Time timeStamp, const SystemObservation& observation);

    void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories);

    void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                         const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule);


private:

    DigitVisualizer(const DigitVisualizer&) = delete;

    void publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const;
    void publishBaseTransform(ros::Time timeStamp, const vector_t& basePose);
    void publishCartesianMarkers(ros::Time timeStamp, const legged_robot::contact_flag_t& contactFlags,
                                 const std::vector<legged_robot::vector3_t>& feetPositions,
                                 const std::vector<legged_robot::vector3_t>& feetForces) const;

    PinocchioInterface pinocchioInterface_;
    const CentroidalModelInfo centroidalModelInfo_;
    std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

    tf::TransformBroadcaster tfBroadcaster_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

    ros::Publisher costDesiredBasePositionPublisher_;
    std::vector<ros::Publisher> costDesiredFeetPositionPublishers_;

    ros::Publisher stateOptimizedPublisher_;

    ros::Publisher currentStatePublisher_;

    scalar_t lastTime_;
    scalar_t minPublishTimeDifference_;


};

}
}

#endif //OCS2_DIGIT_ROS_LEGGEDROBOTVISUALIZER_H
