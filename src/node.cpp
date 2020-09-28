// #include <ros/ros.h>

// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/CompressedImage.h>
 
// #include "std_msgs/Int16.h"
// #include "std_msgs/Int64.h"
// #include "std_msgs/Bool.h"
// #include <math.h>  

// #include <vector>

// static const std::string OPENCV_WINDOW = "Image window";

// class ImageConverter
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
//   image_transport::Subscriber image_sub_;
//   image_transport::Publisher image_pub_;

//   ros::Publisher steer_pub_;
//   ros::Publisher throttle_pub_;


//   int prev_steer = 0;

//   int throttle = 0;
//   int dev_steer = 0;
//   int one_lane_steer = 0;
//   int red_threshold = 0;

//   double right_slope = 0.0;
//   double left_slope = 0.0;

// public:
//   ImageConverter(): it_(nh_) {
//     // Subscrive to input video feed and publish output video feed
//     image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
//     image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
//     steer_pub_ = nh_.advertise<std_msgs::Int16>("/auto_cmd/steer",5);
//     throttle_pub_ = nh_.advertise<std_msgs::Int16>("/auto_cmd/throttle",5);

//     nh_.getParam("throttle", throttle);
//     nh_.getParam("dev_steer", dev_steer);
//     nh_.getParam("one_lane_steer", one_lane_steer);
//     nh_.getParam("red_threshold", red_threshold);
    
//     nh_.getParam("right_slope", right_slope);
//     nh_.getParam("left_slope", left_slope);
    
//     cv::namedWindow(OPENCV_WINDOW);
//   }

//   ~ImageConverter()
//   {
//     cv::destroyWindow(OPENCV_WINDOW);
//   }

//   void imageCb(const sensor_msgs::ImageConstPtr& msg)
//   {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

//     }
//     catch (cv_bridge::Exception& e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }

//     cv::Mat out_img;
//     cv::cvtColor(cv_ptr->image, out_img, CV_RGB2HSV);
  
//     cv::Mat mask1, mask2;

//     cv::inRange(out_img, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
//     cv::inRange(out_img, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);

//     cv::Mat red_img = mask1 | mask2;

//     int lowThreshold = 50;
//     int kernel_size = 3;
//     const int ratio = 2;
//     cv::inRange(out_img, cv::Scalar(14, 140, 60), cv::Scalar(120, 255, 255), out_img);
    
//     cv::Canny(out_img, out_img, lowThreshold, lowThreshold*ratio, kernel_size);

//     float portion = 0.56;

//     cv::Size out_size = out_img.size();

//     cv::Mat mask = cv::Mat::zeros(out_size, out_img.type());
//     mask(cv::Rect(0, out_size.height * portion, out_size.width, out_size.height * (1-portion))) = 255;
//     cv::bitwise_and(out_img, mask, out_img);
    
//     double red_portion = 0.3;

//     cv::Mat red_mask = cv::Mat::zeros(out_size, out_img.type());
//     red_mask(cv::Rect(out_size.width / 6 , out_size.height * red_portion,out_size.width - out_size.width / 6, out_size.height * (1-red_portion))) = 255;

//     cv::bitwise_and(red_img, red_mask, red_img);
    
//     cv::imshow(OPENCV_WINDOW, red_img);
//     cv::waitKey(3);

//     double red_sum = cv::sum(red_img)[0];
//     ROS_INFO("RED SUM %f", red_sum);

//     // line detection 
//     std::vector<cv::Vec4i> lines;
//     cv::HoughLinesP(out_img, lines, 1, CV_PI/180, 60, 10, 20);


//     std::vector<cv::Vec2d> left_lines;
//     std::vector<cv::Vec2d> middle_lines;
//     std::vector<cv::Vec2d> right_lines;

//     for(auto line: lines) {
      
//       double slope = double(line[3]-line[1]) / double(line[2]-line[0]);
      
//       double y_inter = double(line[1]) - slope*double(line[0]);
      
//       ROS_INFO("slope %f %f %f", slope, left_slope, right_slope);

//       if(slope < left_slope) {
//         left_lines.push_back(cv::Vec2d(slope, y_inter));
//       } else if (slope > right_slope) {
//         right_lines.push_back(cv::Vec2d(slope, y_inter));
//       } else if(slope > left_slope + 0.08 && slope < right_slope - 0.08){
//         middle_lines.push_back(cv::Vec2d(slope, y_inter));  
//       }

//     }

//     // averaging lines 
//     int right_x = -1;
//     int left_x = -1;

//     double right_mean_slope = 0;
//     double left_mean_slope = 0;

//     if(right_lines.size() > 0) {

//       cv::Scalar right_mean = cv::mean(right_lines);
//       right_mean_slope = right_mean[0];
//       double right_mean_inter = right_mean[1];
      
//       right_x = int(std::max(0.0,(out_size.height * portion - right_mean_inter) / right_mean_slope));

//       cv::line( 
//           cv_ptr->image, 
//           cv::Point(right_x, int(out_size.height * portion)),
//           cv::Point(out_size.width, int(out_size.width * right_mean_slope + right_mean_inter)), 
//           cv::Scalar(255,0,0), 
//           16, 
//           8);

//     }

//     if(left_lines.size() > 0) {
      
//       cv::Scalar left_mean = cv::mean(left_lines);
//       left_mean_slope = left_mean[0];
//       double left_mean_inter = left_mean[1];

//       left_x = int(std::max(0.0,(out_size.height * portion - left_mean_inter) / left_mean_slope));

//       cv::line(
//           cv_ptr->image, 
//           cv::Point(0, int(left_mean_inter)),
//           cv::Point(left_x, int(out_size.height * portion)),
//           cv::Scalar(0,0,255), 
//           16, 
//           8);
//     }

//     if(middle_lines.size() > 0) {
      
//       cv::Scalar middle_mean = cv::mean(middle_lines);
//       double middle_mean_slope = middle_mean[0];
//       double middle_mean_inter = middle_mean[1];

//       int middle_x = int(std::max(0.0,(out_size.height * portion - middle_mean_inter) / middle_mean_slope));

//       cv::line( 
//           cv_ptr->image, 
//           cv::Point(middle_x, int(out_size.height * portion)),
//           cv::Point(out_size.width, int(out_size.width * middle_mean_slope + middle_mean_inter)), 
//           cv::Scalar(255,255,0), 
//           16, 
//           8);

//     }

//     //////////////////////////////////////////////////
    
//     if(left_lines.size() > 0 && right_lines.size() > 0) {
      
//       double middle_slope = double(out_size.height * portion - out_size.height) / double(out_size.width/2 - (left_x + right_x) / 2);
        
//       steer(middle_slope);

//       cv::line(
//         cv_ptr->image, 
//         cv::Point((left_x + right_x) / 2, int(out_size.height * portion)),
//         cv::Point(out_size.width/2, out_size.height),
//         cv::Scalar(0,255,0), 
//         16, 
//         8);
//     } else if(red_sum > red_threshold && (middle_lines.size() > 0)){
//       steer_value(-400);
//     } else if (left_lines.size() > 0 && right_lines.size() == 0){
//       steer_value(one_lane_steer);
//     } else if (left_lines.size() == 0 && right_lines.size() > 0){
//       steer_value(-one_lane_steer);
//     } else {
//       // steer_value(0);
//     }
  
//     // Output modified video stream
//     image_pub_.publish(cv_ptr->toImageMsg());
//   }
//   void steer_value(int val){

//     ROS_INFO("%d", val);

//     if (abs(prev_steer - val) < dev_steer) {
//       prev_steer = val;
//     } 

//     std_msgs::Int16 msg;
//     msg.data = 1500 + val;

//     steer_pub_.publish(msg);


//     std_msgs::Int16 throttle_msg;
//     throttle_msg.data = throttle;

//     throttle_pub_.publish(throttle_msg);

//   }

//   void steer(double slope){

//     double angle = atan(slope) * 180 / M_PI;
//     // ROS_INFO("%f", angle);
//     double p = 0;
//     if(angle > 0) {
//       p = 1.0 - angle / 90.0;
//     } else {
//       p = -(1.0 + angle / 90.0);
//     }

//     steer_value(p * 400);

//   }


// };

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "image_converter");
//   ImageConverter ic;
//   ros::spin();
//   return 0;
// }