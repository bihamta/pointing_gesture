#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#define WINDOW_SIZE 15
#define COV_THRESHOLD .05f

ros::Publisher cmdVelPub;
geometry_msgs::PointStamped endPoints[WINDOW_SIZE];
geometry_msgs::PointStamped finalPoint;
int iter = 0;
bool finalPointFound = false;
double prev_dist = 1000.0;

void endPointCallback(geometry_msgs::PointStamped endPoint)
{
  if (finalPointFound){
    return;
  }
  endPoints[iter] = endPoint;
  iter ++;
  if (iter == WINDOW_SIZE) {
    iter = 0;
    finalPoint.point.x = finalPoint.point.y = finalPoint.point.z = 0;
    for(int i = 0; i < WINDOW_SIZE; i++){
      finalPoint.point.x += endPoints[i].point.x / WINDOW_SIZE;
      finalPoint.point.y += endPoints[i].point.y / WINDOW_SIZE;
      finalPoint.point.z += endPoints[i].point.z / WINDOW_SIZE;
    }

    double cov = 0.;
    for(int j = 0; j < WINDOW_SIZE; j++)
    {
      double err = sqrt(pow(finalPoint.point.x - endPoints[j].point.x, 2)+
          pow(finalPoint.point.y - endPoints[j].point.y, 2)+
          pow(finalPoint.point.z - endPoints[j].point.z, 2));
      cov += err / WINDOW_SIZE;
    }
    if(cov < COV_THRESHOLD ){
      finalPointFound = true;
      std::cout << "Goal found, start navigating..." << std::endl;
      std::cout << "Goal: " << finalPoint.point.x << " " << finalPoint.point.y << " " << finalPoint.point.z << std::endl;
    }
  }
}

void odometryCallback(nav_msgs::Odometry odom)
{
  if (! finalPointFound) {
    return;
  }
  geometry_msgs::Twist twist;  
  twist.linear.x = twist.linear.y = twist.linear.z = 0;  
  twist.angular.x = twist.angular.y = twist.angular.z = 0;  

  tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  double finalYaw = atan2(-finalPoint.point.x, finalPoint.point.z); 
  if ( fabs(finalYaw - yaw) < 0.08  ){
    twist.linear.x = 0.1;
    double dist = sqrt(pow(-finalPoint.point.x - odom.pose.pose.position.y, 2)+
        pow(finalPoint.point.z - odom.pose.pose.position.x, 2));
    std::cout << "dist: " << dist << " final: " << finalPoint.point.x << "," << finalPoint.point.y << "," << finalPoint.point.z << " -- " << " odom: " << odom.pose.pose.position.x << ","  << odom.pose.pose.position.y << "," << odom.pose.pose.position.z << std::endl;
    if (dist > prev_dist) {
      finalPointFound = false;
      prev_dist = 1000.0;
    } else {
      prev_dist = dist;
    }
  }else{
    std::cout << "Yaw: " << yaw << ", Final Yaw: " << finalYaw << std::endl;
    if(finalYaw < yaw) {
      twist.angular.z = -0.1;
    }else{
      twist.angular.z = 0.1;
    }
  }
  cmdVelPub.publish(twist);
}

// main
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "navigation");
  ROS_INFO_STREAM("Navigation Initaited");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Subscriber endPointSub = nh.subscribe<geometry_msgs::PointStamped> ("/3dr/end_point", 5, &endPointCallback);
  ros::Subscriber odometrySub = nh.subscribe<nav_msgs::Odometry> ("/odom", 5, &odometryCallback);

  cmdVelPub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);

  ros::spin();
  return 0;
}
