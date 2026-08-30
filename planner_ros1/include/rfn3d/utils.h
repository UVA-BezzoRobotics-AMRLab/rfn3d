#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <rfn3d/rfn_types.h>
#include <rfn3d/tinycolormap.hpp>

// ROS1-only helpers: convert the core's ROS-free trajectory to messages and
// visualize it. The trajectory is already sampled by the core, so this just
// maps each state to a message point (and copies out the jerks the wrapper
// needs to seed the next replan's initial state).
namespace utils
{

    inline trajectory_msgs::MultiDOFJointTrajectory
    convert_traj_to_msg(const std::vector<rfn_state_t> &trajectory,
                        const std::string &frame_str,
                        std::vector<Eigen::Vector3d> &jerks)
    {
        trajectory_msgs::MultiDOFJointTrajectory traj_msg;
        traj_msg.header.frame_id = frame_str;
        traj_msg.header.stamp = ros::Time::now();

        jerks.clear();
        for (const rfn_state_t &x : trajectory)
        {
            trajectory_msgs::MultiDOFJointTrajectoryPoint p;
            p.time_from_start = ros::Duration(x.t);

            p.transforms.resize(1);
            p.transforms[0].translation.x = x.pos(0);
            p.transforms[0].translation.y = x.pos(1);
            p.transforms[0].translation.z = x.pos(2);

            p.velocities.resize(1);
            p.velocities[0].linear.x = x.vel(0);
            p.velocities[0].linear.y = x.vel(1);
            p.velocities[0].linear.z = x.vel(2);

            p.accelerations.resize(1);
            p.accelerations[0].linear.x = x.accel(0);
            p.accelerations[0].linear.y = x.accel(1);
            p.accelerations[0].linear.z = x.accel(2);

            jerks.push_back(x.jerk);

            // Yaw points along the direction of motion.
            Eigen::Vector3d vel(x.vel(0), x.vel(1), x.vel(2));
            vel.normalize();
            double yaw = atan2(vel(1), vel(0));

            p.transforms[0].rotation.x = 0.;
            p.transforms[0].rotation.y = 0.;
            p.transforms[0].rotation.z = sin(yaw / 2.);
            p.transforms[0].rotation.w = cos(yaw / 2.);

            traj_msg.points.push_back(p);
        }

        return traj_msg;
    }

    inline void visualize_traj(const trajectory_msgs::MultiDOFJointTrajectory &trajectory,
                               ros::Publisher &trajVizPub,
                               const std::string &frame_str)
    {
        if (trajectory.points.size() < 2)
        {
            return;
        }

        visualization_msgs::MarkerArray markerArray;
        for (size_t i = 0; i < trajectory.points.size() - 1; ++i)
        {
            const trajectory_msgs::MultiDOFJointTrajectoryPoint &startPoint = trajectory.points[i];
            const trajectory_msgs::MultiDOFJointTrajectoryPoint &endPoint = trajectory.points[i + 1];

            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_str;
            marker.header.stamp = ros::Time::now();
            marker.ns = "joint_trajectory";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = startPoint.transforms[0].translation.x;
            marker.pose.position.y = startPoint.transforms[0].translation.y;
            marker.pose.position.z = startPoint.transforms[0].translation.z;

            double delta_x = endPoint.transforms[0].translation.x - startPoint.transforms[0].translation.x;
            double delta_y = endPoint.transforms[0].translation.y - startPoint.transforms[0].translation.y;
            double delta_z = endPoint.transforms[0].translation.z - startPoint.transforms[0].translation.z;

            Eigen::Vector3d direction(delta_x, delta_y, delta_z);
            direction.normalize();

            Eigen::Quaterniond quat;
            quat.setFromTwoVectors(Eigen::Vector3d::UnitX(), direction);

            marker.pose.orientation.x = quat.x();
            marker.pose.orientation.y = quat.y();
            marker.pose.orientation.z = quat.z();
            marker.pose.orientation.w = quat.w();

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

            Eigen::Vector3d vel_vec(startPoint.velocities[0].linear.x,
                                    startPoint.velocities[0].linear.y,
                                    startPoint.velocities[0].linear.z);
            double vel = vel_vec.norm();

            tinycolormap::Color color = tinycolormap::GetColor(vel / 3.0, tinycolormap::ColormapType::Plasma);

            marker.color.r = color.r();
            marker.color.g = color.g();
            marker.color.b = color.b();
            marker.color.a = 1.0;

            markerArray.markers.push_back(marker);
        }

        trajVizPub.publish(markerArray);
    }

} // namespace utils
