#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "goal_publisher");

  ros::NodeHandle nh;
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  geometry_msgs::PoseStamped goal;

  goal.header.frame_id = "map";
  goal.pose.position.x = 6;
  goal.pose.position.y = 1.5;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 4);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    goal.header.stamp = ros::Time::now();
    goal_pub.publish(goal);
    loop_rate.sleep();
  }

  ros::spin();
  
  return 0;
}
