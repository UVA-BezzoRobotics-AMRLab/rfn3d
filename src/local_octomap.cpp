#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_server/OctomapServer.h>

class OctomapFromPointCloudNode
{
public:
    OctomapFromPointCloudNode() : nh("~")
    {
        // Parameters
        nh.param<std::string>("/frame_id", frameID, "uav1/local_origin");
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

        pcl::fromROSMsg(*pc_msg, pointcloud);

        publishOctomap();
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
        octomap::OcTree octree(0.1); // Set your desired resolution

        // add data to octree
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = pointcloud.begin(); it != pointcloud.end(); ++it)
        {
            octree.updateNode(octomap::point3d(it->x + _odom(0), it->y + _odom(1), it->z + _odom(2)), true);
        }

        // Convert the octomap to octomap_msgs::Octomap
        octomap_msgs::Octomap octomapMsg;
        octomap_msgs::binaryMapToMsg(octree, octomapMsg);

        // Stamp the message with the current time
        octomapMsg.header.frame_id = "/uav1/local_origin";
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

    std::string frameID;

    std::string odomTopic;
    std::string octomapTopic;
    std::string pointcloudTopic;
    std::string visualizationTopic;

    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    Eigen::Vector3d _odom;

    bool _is_init;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_from_pointcloud_node");
    OctomapFromPointCloudNode node;
    ros::spin();
    return 0;
}
