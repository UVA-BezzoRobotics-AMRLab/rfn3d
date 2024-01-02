#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <vector>
#include <Eigen/Eigen>

#include <rfn3d/tinycolormap.hpp>
#include <faster/faster_types.hpp>

namespace utils
{

    bool truncate_path(const std::vector<Eigen::Vector3d> &path,
                       std::vector<Eigen::Vector3d> &resultPath,
                       double d)
    {
        if (path.size() < 2)
        {
            // Not enough points to create a path
            return false;
        }

        double remainingDistance = d;
        Eigen::Vector3d resultPoint;
        resultPath.clear();
        resultPath.push_back(path[0]); // Start point of the new path

        for (size_t i = 1; i < path.size(); ++i)
        {
            double segmentLength = (path[i] - path[i - 1]).norm();

            if (remainingDistance <= 0)
            {
                // Requested distance is longer than the entire path
                return false;
            }

            if (remainingDistance <= segmentLength)
            {
                // The target point lies on this segment
                double t = remainingDistance / segmentLength;
                resultPoint = (1 - t) * path[i - 1] + t * path[i];
                resultPath.push_back(resultPoint);
                return true;
            }

            resultPath.push_back(path[i]); // Add entire segment to the new path
            remainingDistance -= segmentLength;
        }

        return false; // Requested distance is longer than the entire path
    }

    void visualize_traj(const trajectory_msgs::MultiDOFJointTrajectory &trajectory,
                        ros::Publisher &trajVizPub,
                        const std::string &frame_str)
    {

        visualization_msgs::MarkerArray markerArray;
        for (size_t i = 0; i < trajectory.points.size() - 1; ++i)
        {
            const trajectory_msgs::MultiDOFJointTrajectoryPoint &startPoint = trajectory.points[i];
            const trajectory_msgs::MultiDOFJointTrajectoryPoint &endPoint = trajectory.points[i + 1];

            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "joint_trajectory";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = startPoint.transforms[0].translation.x;
            marker.pose.position.y = startPoint.transforms[0].translation.y;
            marker.pose.position.z = startPoint.transforms[0].translation.z;

            // Calculate the quaternion for the marker orientation
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

            // Set the dimensions of the marker as a cylinder
            marker.scale.x = 0.1;                                                             // Diameter of the cylinder
            marker.scale.y = 0.1;                                                             // Diameter of the cylinder
            marker.scale.z = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z); // Length of the cylinder

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

        ROS_INFO("publishing trajectory visualization");
        trajVizPub.publish(markerArray);
    }

    trajectory_msgs::MultiDOFJointTrajectory convert_traj_to_msg(const std::vector<state> &trajectory,
                                                                 double traj_dt,
                                                                 const std::string &frame_str,
                                                                 std::vector<Eigen::Vector3d> &jerks)
    {
        trajectory_msgs::MultiDOFJointTrajectory traj_msg;
        traj_msg.header.frame_id = frame_str;
        traj_msg.header.stamp = ros::Time::now();

        jerks.clear();
        double next_t = 0.;
        for (int i = 0; i < trajectory.size(); ++i)
        {
            state x = trajectory[i];
            if (x.t >= next_t)
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

                // setup the yaw to always point in the direction of motion
                Eigen::Vector3d vel(x.vel(0), x.vel(1), x.vel(2));
                vel.normalize();

                double yaw = atan2(vel(1), vel(0));

                p.transforms[0].rotation.x = 0.;
                p.transforms[0].rotation.y = 0.;
                p.transforms[0].rotation.z = sin(yaw / 2.);
                p.transforms[0].rotation.w = cos(yaw / 2.);

                traj_msg.points.push_back(p);

                next_t += traj_dt;
            }
        }

        return traj_msg;
    }

} // end namespace utils
