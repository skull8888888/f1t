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
#include <NvInferRuntime.h>
#include <NvOnnxParser.h>

int SEQ_L = 8;
int D_MODEL = 512;

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

void preprocessImage(cv::Mat& frame, float* gpu_input, const nvinfer1::Dims& dims)
{
    cv::cuda::GpuMat gpu_frame;
    // upload image to GPU
    gpu_frame.upload(frame);

    auto input_width = dims.d[3];
    auto input_height = dims.d[2];
    auto channels = dims.d[1];
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

std::vector<int> tokenize(std::vector<float>& cpu_angles){

  std::vector<int> cpu_tokens;

  std::vector<float> bins{-1.1, -0.5, 0.15, 1.1};

  for(auto angle: cpu_angles){

    for(int i=1; i<bins.size(); i++){
      if(bins[i-1] < angle && angle <= bins[i]) {
        cpu_tokens.push_back(i);
        break;
      }
    }
  }

  return cpu_tokens;

}

// initialize TensorRT engine and parse ONNX model --------------------------------------------------------------------
void loadEngineFromFile(const std::string& model_path, TRTUniquePtr<nvinfer1::ICudaEngine>& engine,
                    TRTUniquePtr<nvinfer1::IExecutionContext>& context)
{
  
  std::vector<char> trtModelStream_;
  size_t size{ 0 };
  std::ifstream file(model_path, std::ios::binary);
  
  if (file.good()) {
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream_.resize(size);
    file.read(trtModelStream_.data(), size);
    file.close();
  }

  TRTUniquePtr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(gLogger)};

  // TRTUniquePtr<nvinfer1::ICudaEngine> engine{runtime->deserializeCudaEngine(trtModelStream_.data(), size)};

  engine.reset(runtime->deserializeCudaEngine(trtModelStream_.data(), size));
  context.reset(engine->createExecutionContext());
  ROS_INFO("finished loading model");
}

class ModelNode
{
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;

  ros::Publisher steer_pub_;
  ros::Publisher mode_pub_;
  ros::Publisher throttle_pub_;

  std::vector<std::vector<float>> frames_cache;
  std::vector<float> cpu_angles_cache;
  // std::vector<at::Tensor> angles_cache;

  TRTUniquePtr<nvinfer1::ICudaEngine> backbone_engine{nullptr};
  TRTUniquePtr<nvinfer1::IExecutionContext> backbone_context{nullptr};
  

  TRTUniquePtr<nvinfer1::ICudaEngine> decoder_engine{nullptr};
  TRTUniquePtr<nvinfer1::IExecutionContext> decoder_context{nullptr};
  
public:
  ModelNode() {
    // Subscribe to input video
    image_sub_ = nh_.subscribe("/camera/color/image_raw/compressed", 1, &ModelNode::imageCb, this);
    
    steer_pub_ = nh_.advertise<std_msgs::Int16>("/auto_cmd/steer",1);
    throttle_pub_ = nh_.advertise<std_msgs::Int16>("/auto_cmd/throttle",1);
    mode_pub_ = nh_.advertise<std_msgs::Bool>("/auto_mode",1);


    loadEngineFromFile("/home/team7/catkin_ws/src/f1t/backbone.engine", this->backbone_engine, this->backbone_context);
    loadEngineFromFile("/home/team7/catkin_ws/src/f1t/decoder.engine", this->decoder_engine, this->decoder_context);


  }

  ~ModelNode()
  {
    
    std_msgs::Bool mode_msg;
    mode_msg.data = false;
    
    mode_pub_.publish(mode_msg);
      
    // for(float * buf: this->frames_cache){
    //   cudaFree(buf);
    // }
    
    std::cout << "MODE 0" << std::endl;
//    myfile.close();
  }

  void backboneForward(cv::Mat& img, std::vector<void*>& buffers){

    std::vector<nvinfer1::Dims> input_dims; // we expect only one input
    std::vector<nvinfer1::Dims> output_dims; // and one output

    int batch_size = 1;
    
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

    preprocessImage(img, static_cast<float *>(buffers[0]), input_dims[0]);

    this->backbone_context->execute(batch_size, buffers.data());

    

  }

  float decoderForward(){
    std::vector<void*> buffers(this->decoder_engine->getNbBindings());

    std::vector<nvinfer1::Dims> input_dims; // x: (1, SEQ_L, d_model), y: (1, SEQ_L - 1)
    std::vector<nvinfer1::Dims> output_dims; // x: (1, )

    int batch_size = 1;
    
    for (size_t i = 0; i < this->decoder_engine->getNbBindings(); ++i)
    {

      auto binding_size = getSizeByDim(this->decoder_engine->getBindingDimensions(i)) * batch_size * sizeof(float);
      
      cudaMalloc(&buffers[i], binding_size);

      if (this->decoder_engine->bindingIsInput(i))
      {
        input_dims.emplace_back(this->decoder_engine->getBindingDimensions(i));
      }
      else
      {
          output_dims.emplace_back(this->decoder_engine->getBindingDimensions(i));
      }
    }

    // copying cached backbone outputs to first input buffer of decoder

    std::vector<float> frames_cache_flatten(getSizeByDim(input_dims[0]));

    for (int i = 0; i < this->frames_cache.size(); i++) {
      
      for (int j = 0; j < D_MODEL; j++){
        // std::cout << frames_cache[i][j] << std::endl;
        frames_cache_flatten[i*D_MODEL + j] = frames_cache[i][j];

      }

    }
    
    cudaMemcpy(buffers[0], frames_cache_flatten.data(), getSizeByDim(input_dims[0]) * sizeof(float), cudaMemcpyHostToDevice);
    
    auto cpu_tokens = tokenize(this->cpu_angles_cache);

    cudaMemcpy(buffers[1], cpu_tokens.data(), getSizeByDim(input_dims[1]) * sizeof(int), cudaMemcpyHostToDevice);
    
    this->decoder_context->execute(batch_size, buffers.data());

    std::vector<float> pred_angle_buffer(getSizeByDim(output_dims[0]));
    cudaMemcpy(pred_angle_buffer.data(), buffers[2], getSizeByDim(output_dims[0]) * sizeof(float), cudaMemcpyDeviceToHost);

    for( void * buf: buffers){
      cudaFree(buf);
    }

    return pred_angle_buffer[0];

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

    std::vector<void*> backbone_buffers(this->backbone_engine->getNbBindings()); // buffers for input and output data

    backboneForward(img, backbone_buffers);

    std::vector<float> cpu_output(D_MODEL);
    cudaMemcpy(cpu_output.data(), backbone_buffers[1], cpu_output.size() * sizeof(float), cudaMemcpyDeviceToHost);

    this->frames_cache.push_back(cpu_output);

    if(this->frames_cache.size() < SEQ_L) {
       std::cout << "pred angle " << 0.0 << std::endl;
      this->cpu_angles_cache.push_back(0.0);
      return;
    }

    float pred_angle = decoderForward();

    std::cout << "pred angle " << pred_angle << std::endl;

    this->cpu_angles_cache.push_back(pred_angle);
    this->cpu_angles_cache = std::vector<float>(this->cpu_angles_cache.begin() + 1, this->cpu_angles_cache.end());

    this->frames_cache = std::vector<std::vector<float>>(this->frames_cache.begin() + 1, this->frames_cache.end());
    
    for(auto buf: backbone_buffers){
      cudaFree(buf);
    }

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
