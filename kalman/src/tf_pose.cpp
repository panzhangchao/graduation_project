#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <cluster/obstaclePose.h>
#include <cluster/obstaclePoseArray.h>


#include <ros/ros.h>

void posecallback(const cluster::obstaclePoseArray &in_pose)
{

}

int main(int argc, char **argv)
{
    int i = 10;
    ros::init(argc,argv,"tf_pose");
    ros::NodeHandle nh;
    ros::Subscriber  sub_obstacle_pose;

    sub_obstacle_pose = nh.subscribe("obstacle_pose",5, posecallback); 

    tf::TransformListener listener;
    geometry_msgs::PointStamped pose_base_link;
    pose_base_link.header.stamp = ros::Time();
    pose_base_link.header.frame_id = "base_link";
    pose_base_link.point.x = 1;
    pose_base_link.point.y = 1;
    pose_base_link.point.z = 1;
    geometry_msgs::PointStamped pose_odom;

    try{
        listener.transformPoint("odom", pose_base_link, pose_odom);
    }
    catch( tf::TransformException ex)
    {
        ROS_WARN("transfrom exception : %s",ex.what());
        ros::Duration(0.05).sleep();
    }

    std::cout << pose_odom.point << std::endl;
    std::cout << "end" <<std::endl;
    return 0;
}