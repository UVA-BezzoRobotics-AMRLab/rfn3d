#include <cstdio>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_server/OctomapServer.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


class OctomapFromPointCloudNode
{
public:
    OctomapFromPointCloudNode() : nh("~"), tfBuffer(), tfListener(tfBuffer)
    {
        // Parameters
        nh.param<double>("/octomap_res", octomapRes, .1);

        nh.param<std::string>("/frame_id", frameID, "uav1/os_sensor");

        nh.param<std::string>("/topic_octomap", octomapTopic, "/local_octomap");
        nh.param<std::string>("/topic_octomap_viz", visualizationTopic, "/octomap_viz");
        nh.param<std::string>("/topic_odom", odomTopic, "/uav1/estimation_manager/odom_main");
        nh.param<std::string>("/topic_pointcloud", pointcloudTopic, "/uav1/point_cloud_manager/merged_pointcloud");

        // Subscribe to PointCloud2 and Odometry topics
        pointcloudSub = nh.subscribe(pointcloudTopic, 1, &OctomapFromPointCloudNode::pointcloudCallback, this);

        odomSub = nh.subscribe(odomTopic, 1, &OctomapFromPointCloudNode::odomCallback, this);

        // Advertise the Octomap topic
        octomapPub = nh.advertise<octomap_msgs::Octomap>(octomapTopic, 1);

        // Advertise the visualization topic
        visualizationPub = nh.advertise<visualization_msgs::MarkerArray>(visualizationTopic, 1);

        laserScanPub = nh.advertise<sensor_msgs::LaserScan>("/front/scan", 1);

        _is_init = false;
        
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        // Process the Odometry data, e.g., convert it to Eigen::Vector3d and store it in a member variable
        _odom(0) = odom_msg->pose.pose.position.x;
        _odom(1) = odom_msg->pose.pose.position.y;
        _odom(2) = odom_msg->pose.pose.position.z;

        _is_init = true;
    }

    // Callback for the PointCloud2 topic
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
    {
        // Process the PointCloud2 data, e.g., convert it to octomap::Pointcloud and update the octree
        if (!_is_init)
            return;

        // transform from uav1/os_lidar to gps_garmin_origin
        sensor_msgs::PointCloud2 transformed_pc_msg;
        try
        {
            geometry_msgs::TransformStamped transformStamped = 
              tfBuffer.lookupTransform(frameID, "uav1/os_sensor", ros::Time(0));

            tf2::doTransform(*pc_msg, transformed_pc_msg, transformStamped);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Could not transform pointcloud: %s", ex.what());
            return;
        }

        pcl::fromROSMsg(transformed_pc_msg, pointcloud);

        publishLaserScanFromPointCloud();
        publishOctomap();
    }

    void publishLaserScanFromPointCloud(double epsilon = 0.05)
    {
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.header.frame_id = frameID;
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = 0.005;
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 15.0;

        int num_rays = std::round((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
        scan_msg.ranges.assign(num_rays, scan_msg.range_max);

        for (const auto& pt : pointcloud.points)
        {
            if (std::abs(pt.z - _odom(2)) > epsilon) continue;  // Filter for near-2D slice

            double angle = std::atan2(pt.y, pt.x);
            double range = std::hypot(pt.x, pt.y);

            int idx = std::round((angle - scan_msg.angle_min) / scan_msg.angle_increment);
            if (idx < 0 || idx >= num_rays) continue;

            if (range < scan_msg.ranges[idx]) {
                scan_msg.ranges[idx] = range;
            }
        }

        laserScanPub.publish(scan_msg);
    }

    void publishOctomapMarkers(const octomap::OcTree& octree)
    {
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker marker;
        marker.header.frame_id = frameID;
        marker.header.stamp = ros::Time::now();
        marker.ns = "occupied_voxels";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = octree.getResolution();
        marker.scale.y = octree.getResolution();
        marker.scale.z = octree.getResolution();
        marker.color.a = .5;
        marker.color.r = 0.0;
        marker.color.g = 0.1;
        marker.color.b = 0.3;

        int count = 0;
        for (octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
        {
            if (octree.isNodeOccupied(*it))
            {
                count++;
                // Add point to the cube list
                geometry_msgs::Point point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();
                marker.points.push_back(point);
            }
        }

        ROS_INFO("publishing, %d occupied voxels", count);
        markerArray.markers.push_back(marker);
        visualizationPub.publish(markerArray);
    }

    // Publish the octomap
    void publishOctomap()
    {
        // Create an octomap instance and fill it with data (modify as needed)
        octomap::OcTree octree(octomapRes); // Set your desired resolution

        // add data to octree
        const double min_radius = .45;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = pointcloud.begin(); it != pointcloud.end(); ++it)
        {
            
          // Check if the point is within a certain radius to avoid cluttering the octomap
          Eigen::Vector3d point(it->x - _odom(0), it->y - _odom(1), it->z - _odom(2));
          if (point.norm() < min_radius)
            continue;

            /*octree.updateNode(octomap::point3d(it->x + _odom(0), it->y + _odom(1), it->z + _odom(2)), true);*/
            octree.updateNode(octomap::point3d(it->x, it->y, it->z), true);
        }

        // Convert the octomap to octomap_msgs::Octomap
        octomap_msgs::Octomap octomapMsg;
        octomap_msgs::binaryMapToMsg(octree, octomapMsg);

        // Stamp the message with the current time
        octomapMsg.header.frame_id = frameID;
        octomapMsg.header.stamp = ros::Time::now();

        // Publish the octomap message
        ROS_INFO("publishing, %lu nodes", octomapMsg.data.size());
        octomapPub.publish(octomapMsg);

        // // convert octree into a marker array for visualization
        // visualization_msgs::MarkerArray markerArray;
        // octomap_msgs::fullMapToMsg(octree, markerArray);
        // visualizationPub.publish(markerArray);
        publishOctomapMarkers(octree);

    }

private:
    ros::NodeHandle nh;

    ros::Subscriber pointcloudSub;
    ros::Subscriber odometrySub;
    ros::Subscriber odomSub;

    ros::Publisher octomapPub;
    ros::Publisher visualizationPub;
    ros::Publisher laserScanPub;

    double octomapRes;

    std::string frameID;

    std::string odomTopic;
    std::string octomapTopic;
    std::string pointcloudTopic;
    std::string visualizationTopic;

    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    Eigen::Vector3d _odom;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    bool _is_init;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_from_pointcloud_node");
    OctomapFromPointCloudNode node;
    ros::spin();
    return 0;
}
