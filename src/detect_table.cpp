#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "opencv2/opencv.hpp"

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
  
  if (object->data[0] == 1)
  {
    float object_width = object->data[1];
    float object_height = object->data[2];
    float x_pos;
    
    // find corners 
    cv::Mat cvHomography(3, 3, CV_32F);
    std::vector<cv::Point2f> inpt_array, outpt_array;

    cvHomography.at<float>(0, 0) = object->data[3];
    cvHomography.at<float>(1, 0) = object->data[4];
    cvHomography.at<float>(2, 0) = object->data[5];
    cvHomography.at<float>(0, 1) = object->data[6];
    cvHomography.at<float>(1, 1) = object->data[7];
    cvHomography.at<float>(2, 1) = object->data[8];
    cvHomography.at<float>(0, 2) = object->data[9];
    cvHomography.at<float>(1, 2) = object->data[10];
    cvHomography.at<float>(2, 2) = object->data[11];

    inpt_array.push_back(cv::Point2f(0, 0));
    inpt_array.push_back(cv::Point2f(object_width, 0));
    inpt_array.push_back(cv::Point2f(0, object_height));
    inpt_array.push_back(cv::Point2f(object_width, object_height));
    cv::perspectiveTransform(inpt_array, outpt_array, cvHomography);
    
    ROS_INFO("first corner x => %f | y => %f \n second corner x => %f | y => %f \n third corner x => %f | y => %f \n fourth corner x => %f | y => %f", outpt_array.at(0).x, outpt_array.at(0).y, outpt_array.at(1).x, outpt_array.at(1).y,
    outpt_array.at(2).x, outpt_array.at(2).y,
    outpt_array.at(3).x, outpt_array.at(3).y,)
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_table_corner");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
  ros::spin()
}

