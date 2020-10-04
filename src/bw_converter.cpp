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

#include <torch/script.h>

#include <iostream>
#include <memory>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher steer_pub_;
  ros::Publisher throttle_pub_;


  int prev_steer = 0;

  int throttle = 0;
  int dev_steer = 0;
  int one_lane_steer = 0;
  int red_threshold = 0;

  double right_slope = 0.0;
  double left_slope = 0.0;

  std::string model_path;

public:
  ImageConverter(): it_(nh_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = nh_.subscribe("/camera/color/image_raw/compressed", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/image", 1);
    
    steer_pub_ = nh_.advertise<std_msgs::Int16>("/auto_cmd/steer",5);
    throttle_pub_ = nh_.advertise<std_msgs::Int16>("/auto_cmd/throttle",5);

    nh_.getParam("throttle", throttle);
    nh_.getParam("dev_steer", dev_steer);
    nh_.getParam("one_lane_steer", one_lane_steer);
    nh_.getParam("red_threshold", red_threshold);
    
    nh_.getParam("right_slope", right_slope);
    nh_.getParam("left_slope", left_slope);
    
    nh_.getParam("model_path", model_path);

    cv::namedWindow(OPENCV_WINDOW);
    cv::startWindowThread();  

    torch::jit::script::Module module;
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
      module = torch::jit::load("/home/user/catkin_ws/src/f1t/jn/balanced.pt");
    }
    catch (const c10::Error& e) {
      std::cerr << e.what();
    }

    torch::Device device(torch::kCUDA);

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::ones({1, 3, 120, 320}, device));

    // Execute the model and turn its output into a tensor.
    at::Tensor output = module.forward(inputs).toTensor();
    std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    cv::Mat img;
    cv::cvtColor(cv_ptr->image, img, CV_RGB2GRAY);

    cv::Rect roi;

    int offset_x = 0;
    int offset_y = img.size().height / 2;

    roi.x = offset_x;
    roi.y = offset_y;
    roi.width = img.size().width;
    roi.height = img.size().height - offset_y;

    // 640x240 cut 
    // 640x480 original
    // ROS_INFO("%d %d", roi.width, roi.height);
    img = img(roi);
    cv::resize(img, img, cv::Size(320,120));

    // cv::imwrite(to_string(msg->header.stamp) + ".jpeg", img);

    imshow(OPENCV_WINDOW, img);
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, img).toImageMsg();
    image_pub_.publish(out_msg);   

    //////////////////////////////////////////////////
    
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}