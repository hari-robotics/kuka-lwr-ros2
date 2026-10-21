#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>

int main( int argc, char** argv )
{
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("points_and_lines");
  auto marker_pub = n->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

  // TODO(ros2-migration): ROS 1 Rate followed ROS time; rclcpp::Rate uses the system clock
  rclcpp::Rate r(30);

  float f = 0.0;
  while (rclcpp::ok())
  {

    visualization_msgs::msg::Marker point;
    // ROS 2: tf2 frame ids have no leading slash (ROS 1 "/world")
    point.header.frame_id= "world";
    point.header.stamp= n->now();
    point.ns = "points_and_lines";
    point.action = visualization_msgs::msg::Marker::ADD;
    point.pose.orientation.w = 1.0;



    point.id = 0;




    point.type = visualization_msgs::msg::Marker::POINTS;




    // POINTS markers use x and y scale for width/height respectively
    point.scale.x = 0.02;
    point.scale.y = 0.02;





    // Points are green
    point.color.g = 1.0f;
    point.color.a = 1.0;



      
      geometry_msgs::msg::Point p;
      p.x = 0.3;
      p.y = 0.3;
      p.z = 0.3;

      point.points.push_back(p);



    marker_pub->publish(point);


    r.sleep();

  }
  rclcpp::shutdown();
}
