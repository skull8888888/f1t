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

// #include <torch/data.h>
// #include <torch/script.h>

#include <iostream>
#include <fstream>
#include <memory>

#include <cuda_runtime_api.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudaarithm.hpp>
#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>

int SEQ_L = 8;

// std::ofstream myfile;

class Logger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char* msg) override {
        // remove this 'if' if you need more logged info
        if ((severity == Severity::kERROR) || (severity == Severity::kINTERNAL_ERROR)) {
            std::cout << msg << "\n";
        }
    }
} gLogger;

// destroy TensorRT objects if something goes wrong
struct TRTDestroy
{
    template <class T>
    void operator()(T* obj) const
    {
        if (obj)
        {
            obj->destroy();
        }
    }
};

template <class T>
using TRTUniquePtr = std::unique_ptr<T, TRTDestroy>;

// calculate size of tensor
size_t getSizeByDim(const nvinfer1::Dims& dims)
{
    size_t size = 1;
    for (size_t i = 0; i < dims.nbDims; ++i)
    {
        size *= dims.d[i];
    }
    return size;
}

// initialize TensorRT engine and parse ONNX model --------------------------------------------------------------------
void parseOnnxModel(const std::string& model_path, TRTUniquePtr<nvinfer1::ICudaEngine>& engine,
                    TRTUniquePtr<nvinfer1::IExecutionContext>& context)
{
    TRTUniquePtr<nvinfer1::IBuilder> builder{nvinfer1::createInferBuilder(gLogger)};
    TRTUniquePtr<nvinfer1::INetworkDefinition> network{builder->createNetworkV2(1U << static_cast<int>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH))};
    
    
    TRTUniquePtr<nvonnxparser::IParser> parser{nvonnxparser::createParser(*network, gLogger)};

    TRTUniquePtr<nvinfer1::IBuilderConfig> config{builder->createBuilderConfig()};
    // parse ONNX
    if (!parser->parseFromFile(model_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO)))
    {
        std::cerr << "ERROR: could not parse the model.\n";
        return;
    }
    // allow TensorRT to use up to 1GB of GPU memory for tactic selection.
    config->setMaxWorkspaceSize(1ULL << 30);
    // use FP16 mode if possible
    if (builder->platformHasFastFp16())
    {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
    // we have only one image in batch
    builder->setMaxBatchSize(1);
    // generate TensorRT engine optimized for the target platform
    engine.reset(builder->buildEngineWithConfig(*network, *config));
    context.reset(engine->createExecutionContext());


    ROS_INFO("finished parsing model");
}


void preprocessImage(cv::Mat& frame, float* gpu_input, const nvinfer1::Dims& dims)
{
    cv::cuda::GpuMat gpu_frame;
    // upload image to GPU
    gpu_frame.upload(frame);

    auto input_width = dims.d[2];
    auto input_height = dims.d[1];
    auto channels = dims.d[0];
    auto input_size = cv::Size(input_width, input_height);
    // resize
    cv::cuda::GpuMat resized;
    cv::cuda::resize(gpu_frame, resized, input_size, 0, 0, cv::INTER_NEAREST);
    // normalize
    cv::cuda::GpuMat flt_image;
    resized.convertTo(flt_image, CV_32FC3, 1.f / 255.f);
    cv::cuda::subtract(flt_image, cv::Scalar(0.485f, 0.456f, 0.406f), flt_image, cv::noArray(), -1);
    cv::cuda::divide(flt_image, cv::Scalar(0.229f, 0.224f, 0.225f), flt_image, 1, -1);
    // to tensor
    std::vector<cv::cuda::GpuMat> chw;
    for (size_t i = 0; i < channels; ++i)
    {
        chw.emplace_back(cv::cuda::GpuMat(input_size, CV_32FC1, gpu_input + i * input_width * input_height));
    }
    cv::cuda::split(flt_image, chw);
}

  
void resizeImage(cv::Mat& img){

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
  
}

class ModelNode
{
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;

  ros::Publisher steer_pub_;
  ros::Publisher mode_pub_;
  ros::Publisher throttle_pub_;

  // std::vector<at::Tensor> frames_cache;
  // std::vector<at::Tensor> angles_cache;



  TRTUniquePtr<nvinfer1::ICudaEngine> backbone_engine{nullptr};
  TRTUniquePtr<nvinfer1::IExecutionContext> backbone_context{nullptr};
  

public:
  ModelNode() {
    // Subscribe to input video
    image_sub_ = nh_.subscribe("/camera/color/image_raw/compressed", 1, &ModelNode::imageCb, this);
    
    steer_pub_ = nh_.advertise<std_msgs::Int16>("/auto_cmd/steer",1);
    throttle_pub_ = nh_.advertise<std_msgs::Int16>("/auto_cmd/throttle",1);
    mode_pub_ = nh_.advertise<std_msgs::Bool>("/auto_mode",1);


    std::string backbone_path = "/home/team7/catkin_ws/src/f1t/backbone.onnx";
    parseOnnxModel(backbone_path, this->backbone_engine, this->backbone_context);
    

  }

  ~ModelNode()
  {
    
    std_msgs::Bool mode_msg;
    mode_msg.data = false;
    
    mode_pub_.publish(mode_msg);
      
    std::cout << "MODE 0" << std::endl;
//    myfile.close();
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

    resizeImage(img);

    std::vector<nvinfer1::Dims> input_dims; // we expect only one input
    std::vector<nvinfer1::Dims> output_dims; // and one output

    // int nbBindings = this->backbone_engine->getNbBindings();
    // std::cout << nbBindings << std::endl;

    std::vector<void*> buffers(this->backbone_engine->getNbBindings()); // buffers for input and output data

    int batch_size = 1;
    // ROS_INFO("trying to alloc buffers");
    for (size_t i = 0; i < this->backbone_engine->getNbBindings(); ++i)
    {
      auto binding_size = getSizeByDim(this->backbone_engine->getBindingDimensions(i)) * batch_size * sizeof(float);
      cudaMalloc(&buffers[i], binding_size);
      if (this->backbone_engine->bindingIsInput(i))
      {
          input_dims.emplace_back(this->backbone_engine->getBindingDimensions(i));
      }
      else
      {
          output_dims.emplace_back(this->backbone_engine->getBindingDimensions(i));
      }
    }

    preprocessImage(img, (float *) buffers[0], input_dims[0]);
    this->backbone_context->execute(batch_size, buffers.data());
    
    
    // at::Tensor img_emb = backbone_forward(img);

    // this->frames_cache.push_back(img_emb.detach());

    // if(this->frames_cache.size() < SEQ_L){

    //     this->angles_cache.push_back(torch::tensor({0.0}).to(torch::kCUDA));
    //     return;
    // }

    // torch::Tensor x = torch::stack(this->frames_cache, 1);
    // torch::Tensor steer_angles = torch::stack(this->angles_cache);
    // steer_angles = torch::transpose(steer_angles, 0,1);

    // std::vector<torch::jit::IValue> inputs = {x, steer_angles};

    // torch::Tensor y = this->decoder.forward(inputs).toTensor().flatten().detach();
    // y = torch::clamp(y, -1.0, 1.0); 

    // this->angles_cache.push_back(y);
    // this->angles_cache = std::vector<torch::Tensor>(this->angles_cache.begin() + 1, this->angles_cache.end());

    // this->frames_cache = std::vector<torch::Tensor>(this->frames_cache.begin() + 1, this->frames_cache.end());
    
    // //////////////////////////////////////
    
    // std_msgs::Bool mode_msg;
    // mode_msg.data = true;
    
    // mode_pub_.publish(mode_msg);
      
    int steer = 1500 ;
    // std::cout << steer << std::endl;
    
    std_msgs::Int16 steer_msg;
    steer_msg.data = steer;

    this->steer_pub_.publish(steer_msg);


    // std_msgs::Int16 throttle_msg;
    // throttle_msg.data = 1465;

    // this->throttle_pub_.publish(throttle_msg);
      
  }
   

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_node");

  ModelNode mn;
  ros::spin();
  return 0;
}
