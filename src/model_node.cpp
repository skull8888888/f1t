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

#include <torch/data.h>
#include <torch/script.h>

#include <iostream>
#include <fstream>
#include <memory>

static const std::string OPENCV_WINDOW = "Image window";

int SEQ_L = 8;

std::ofstream myfile;

class ModelNode
{
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;

  ros::Publisher steer_pub_;
  ros::Publisher throttle_pub_;

  int steer = 0;

  std::string model_path;

  torch::jit::script::Module backbone;
  torch::jit::script::Module decoder;
  
  std::vector<at::Tensor> frames_cache;
  std::vector<at::Tensor> angles_cache;
 
public:
  ModelNode() {
    // Subscribe to input video
    image_sub_ = nh_.subscribe("/camera/color/image_raw/compressed", 1, &ModelNode::imageCb, this);
    
    steer_pub_ = nh_.advertise<std_msgs::Int16>("car1/auto_cmd/steer",5);
    throttle_pub_ = nh_.advertise<std_msgs::Int16>("car1/auto_cmd/throttle",5);

    nh_.getParam("model_path", model_path);

    try {

      torch::Device device(torch::kCUDA);

      backbone = torch::jit::load("/home/user/catkin_ws/src/f1t/backbone_script.pt");
      
      backbone.to(device);
      backbone.eval();

      decoder = torch::jit::load("/home/user/catkin_ws/src/f1t/decoder_script.pt");

      decoder.to(device);
      decoder.eval();
    
    }
    catch (const c10::Error& e) {
      std::cerr << e.what();
    }
    
    cv::namedWindow(OPENCV_WINDOW);
    cv::startWindowThread();  

		myfile.open("/home/user/catkin_ws/src/f1t/val_pred.csv");
  }

  ~ModelNode()
  {
    cv::destroyWindow(OPENCV_WINDOW);
		myfile.close();
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
    at::Tensor img_emb = backbone_forward(img);

    this->frames_cache.push_back(img_emb.detach());

		if(this->frames_cache.size() < SEQ_L){

			this->angles_cache.push_back(torch::tensor({0.0}).to(at::kCUDA));
			return;
		}

		torch::Tensor x = torch::stack(this->frames_cache, 1);
		torch::Tensor steer_angles = torch::stack(this->angles_cache);
		steer_angles = torch::transpose(steer_angles, 0,1);

		std::vector<torch::jit::IValue> inputs = {x, steer_angles};
		
		torch::Tensor y = this->decoder.forward(inputs).toTensor().flatten().detach();
		y = torch::clamp(y, -1.0, 1.0); 

		float current_steer = y.item<float>();
		
		myfile << std::to_string(current_steer) + "\n";

		std::cout << current_steer << std::endl;

		this->angles_cache.push_back(y);		
		this->angles_cache = std::vector<torch::Tensor>(this->angles_cache.begin() + 1, this->angles_cache.end());

		this->frames_cache = std::vector<torch::Tensor>(this->frames_cache.begin() + 1, this->frames_cache.end());
		
		
  }

  at::Tensor backbone_forward(cv::Mat& img){

    cv::cvtColor(img, img, CV_BGR2RGB);
    cv::Rect roi;

    int offset_x = 0;
    int offset_y = img.size().height / 2;

    roi.x = offset_x;
    roi.y = offset_y;
    roi.width = img.size().width;
    roi.height = img.size().height - offset_y;

    // 480x640 original
    // 240x640 cut 

    img = img(roi);
    cv::resize(img, img, cv::Size(320,120));
    // ROS_INFO("%u %u", img.rows, img.cols);  
    imshow(OPENCV_WINDOW, img);
    
    img.convertTo(img, CV_32FC3, 1.0/255.0);
 

    auto tensor_image = torch::from_blob(img.data, {1, img.rows, img.cols, img.channels()});
    tensor_image = tensor_image.permute({0,3,1,2});
    

		std::vector<double> norm_mean = {0.485, 0.456, 0.406};
		std::vector<double> norm_std = {0.229, 0.224, 0.225};

		tensor_image = torch::data::transforms::Normalize<>(norm_mean, norm_std)(tensor_image);
		tensor_image = tensor_image.to(at::kCUDA);
  
    std::vector<torch::jit::IValue> inputs;

    inputs.push_back(tensor_image);
    

    at::Tensor output = backbone.forward(inputs).toTensor();
    
    return output;

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_node");
  ModelNode mn;
  ros::spin();
  return 0;
}