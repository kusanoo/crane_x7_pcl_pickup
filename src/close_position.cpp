#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

class RsjRobotTestNode
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_odom_;
  ros::Publisher pub_twist_;
  ros::Subscriber sub_clusters_;

  visualization_msgs::MarkerArray marker_array_;

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
  }

  void cbCluster(const visualization_msgs::MarkerArray::ConstPtr &msg)
  {
    //ROS_INFO("clusters: %zu", msg->markers.size());
    const visualization_msgs::Marker *target = NULL;
    for (visualization_msgs::MarkerArray::_markers_type::const_iterator
             it = msg->markers.begin(),
             it_end = msg->markers.end();
         it != it_end; ++it)
    {
      const visualization_msgs::Marker &marker = *it;
      if (marker.ns == "target_cluster")
      {
        target = &marker;
      }
    }
    ROS_INFO("clusters: %zu", msg->markers.size());
    if (target != NULL)
    {
      float dx = target->pose.position.x;
      float dy = target->pose.position.y;
      ROS_INFO("target: %f, %f", dx, dy);
    } 
  }

public:
  RsjRobotTestNode()
    : nh_()
  {
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>(
        "cmd_vel", 5);
    sub_odom_ = nh_.subscribe(
        "odom", 5, &RsjRobotTestNode::cbOdom, this);
    sub_clusters_ = nh_.subscribe(
        "clusters", 5, &RsjRobotTestNode::cbCluster, this);    
  }
  void mainloop()
  {
    ROS_INFO("Hello ROS World!");

    ros::Rate rate(10.0);
    while (ros::ok())
    {
      ros::spinOnce();
      // ここに速度指令の出力コード
      rate.sleep();
    }
    // ここに終了処理のコード
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rsj_robot_test_node");

  RsjRobotTestNode robot_test;

  robot_test.mainloop();
}
