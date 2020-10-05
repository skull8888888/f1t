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

class ModelNode
{
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;

  ros::Publisher steer_pub_;
  ros::Publisher throttle_pub_;

  int steer = 0;

  std::string model_path;


  torch::jit::script::Module module;
  
public:
  ModelNode() {
    // Subscribe to input video
    image_sub_ = nh_.subscribe("/camera/color/image_raw/compressed", 1, &ModelNode::imageCb, this);
    
    steer_pub_ = nh_.advertise<std_msgs::Int16>("car1/auto_cmd/steer",5);
    throttle_pub_ = nh_.advertise<std_msgs::Int16>("car1/auto_cmd/throttle",5);

    nh_.getParam("model_path", model_path);

    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
      module = torch::jit::load("/home/user/catkin_ws/src/f1t/jn/balanced.pt");
      module.eval();
    }
    catch (const c10::Error& e) {
      std::cerr << e.what();
    }


    // std::vector<torch::jit::IValue> inputs;
    // inputs.push_back(torch::ones({1, 3, 120, 320}, device));

    // Execute the model and turn its output into a tensor.
    // std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';
    
    cv::namedWindow(OPENCV_WINDOW);
    cv::startWindowThread();  

    // cv::Mat img = cv::imread("/home/user/catkin_ws/images/test/left0000.jpg");
    // model_forward(img);

  }

  ~ModelNode()
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

    cv::Mat img = cv_ptr->image;
    model_forward(img);
    // sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, img).toImageMsg();
    // image_pub_.publish(out_msg);   

    //////////////////////////////////////////////////
    
  }

  void model_forward(cv::Mat& img){

    cv::cvtColor(img, img, CV_RGB2GRAY);
    cv::cvtColor(img, img, CV_GRAY2RGB);
    
    cv::Rect roi;

    int offset_x = 0;
    int offset_y = img.size().height / 2;

    roi.x = offset_x;
    roi.y = offset_y;
    roi.width = img.size().width;
    roi.height = img.size().height - offset_y;

    // 480x640 original
    // 240x640 cut 
    // ROS_INFO("%d %d", roi.width, roi.height);
    img = img(roi);
    cv::resize(img, img, cv::Size(320,120));
    // ROS_INFO("%u %u", img.rows, img.cols);  
    imshow(OPENCV_WINDOW, img);
    
    // normalizing image to fit torch tensor input format
    img.convertTo(img, CV_32FC3, 1.0/255.0);
    // imshow(OPENCV_WINDOW, img);
    // std::cout << img << std::endl;

    auto tensor_image = torch::from_blob(img.data, {1, img.rows, img.cols, img.channels()});
    tensor_image = tensor_image.permute({0,3,1,2});
    
    // std::cout << tensor_image << std::endl;

    std::vector<torch::jit::IValue> inputs;
    torch::Device device(torch::kCUDA);
    inputs.push_back(tensor_image.to(device));

    // auto out_tensor = tensor_image;
    // auto s = out_tensor.sizes();
    
    // out_tensor = out_tensor.squeeze().permute({2, 1, 0});
    
    // // ROS_INFO("%u %u %u %u", s[0], s[1], s[2], s[3]);
    // out_tensor = out_tensor.to(torch::kU8);
    // out_tensor = out_tensor.to(torch::kCPU);

    // cv::Mat resultImg(120, 320, CV_8UC3);
    // std::memcpy((void *) resultImg.data, out_tensor.data_ptr(), sizeof(torch::kU8) * out_tensor.numel());

    // imshow(OPENCV_WINDOW, resultImg);
    
    at::Tensor output = module.forward(inputs).toTensor();
    std::cout << output[0][0].item<float>() << '\n';

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_node");
  ModelNode mn;
  ros::spin();
  return 0;
}