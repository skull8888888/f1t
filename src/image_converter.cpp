#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
 
#include "std_msgs/Int16.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Bool.h"
#include <math.h>  

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <boost/bind.hpp>


static const std::string OPENCV_WINDOW = "Image window";

void callback(const sensor_msgs::CompressedImageConstPtr& image_msg, const std_msgs::Int16::ConstPtr& steer_msg)
{

  ROS_INFO("steer %d", steer_msg->data);
  
}


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber image_sub_;
  ros::Subscriber steer_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher steer_pub_;
  ros::Publisher throttle_pub_;


  int steer = 0;
  int img_count = 0;

  int throttle = 0;
  int dev_steer = 0;
  int one_lane_steer = 0;
  int red_threshold = 0;

  double right_slope = 0.0;
  double left_slope = 0.0;

  std::string model_path;
  std::string img_dir;
  std::string steer_file;

  std::ofstream myfile;

public:
  ImageConverter(): it_(nh_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = nh_.subscribe("/camera/color/image_raw/compressed", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/image", 1);
    
    steer_sub_ = nh_.subscribe("/car1/rc_cmd/steer", 1, &ImageConverter::steerCb, this);

    cv::namedWindow(OPENCV_WINDOW);
    cv::startWindowThread();  

    nh_.getParam("image_converter/steer_file", steer_file);
    nh_.getParam("image_converter/img_dir", img_dir);

    myfile.open(steer_file);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    myfile.close();
  }


  void steerCb(const std_msgs::Int16::ConstPtr& msg)
  {
    // ROS_INFO("steerd %d", msg->data);
    this->steer = msg->data;
  }

  void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cv_ptr->image;
    cv::cvtColor(cv_ptr->image, img, CV_BGR2RGB);

    cv::Rect roi;

    int offset_x = 0;
    int offset_y = img.size().height / 2;

    roi.x = offset_x;
    roi.y = offset_y;
    roi.width = img.size().width;
    roi.height = img.size().height - offset_y;

    //  
    // 640x480 original
    // 640x240 cut
    // ROS_INFO("%d %d", roi.width, roi.height);

    img = img(roi);
    cv::resize(img, img, cv::Size(320,120));

    // cv::imwrite(to_string(msg->header.stamp) + ".jpeg", img);

    imshow(OPENCV_WINDOW, img);
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, img).toImageMsg();
    image_pub_.publish(out_msg);   

    //////////////////////////////////////////////////


    ROS_INFO("steerd %d",this->steer); 
    this->myfile << std::to_string(this->steer) + "\n";

    int n_zero = 4;
    std::string old_string = std::to_string(img_count);
    std::string new_string = std::string(n_zero - old_string.length(), '0') + old_string;

    std::string filename = this->img_dir + new_string + ".jpg";
    cv::imwrite(filename,img);

    this->img_count += 1;

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}