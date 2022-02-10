 #include <ros/ros.h>

 // Include opencv2
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/core/core.hpp>
//  #include <opencv2/core/types.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #define IMG_HEIGHT (240)
 #define IMG_WIDTH (320)

  // OpenCV Window Name
 static const std::string OPENCV_WINDOW = "Image window";

 // Topics
 static const std::string IMAGE_TOPIC = "/camera/color/image_raw";
 static const std::string PUBLISH_TOPIC = "/image_converter/output_video";

 // Publisher
 ros::Publisher pub;


void image_callback(const sensor_msgs::ImageConstPtr& msg){
   std_msgs::Header msg_header = msg->header;
   std::string frame_id = msg_header.frame_id.c_str();
   ROS_INFO_STREAM("New Image from " << frame_id);

   cv_bridge::CvImagePtr cv_ptr;

   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   cv::Mat frame = cv_ptr->image; 
   cv::imshow("original", frame);
   cv::waitKey(3);

}


 int main(int argc, char** argv)
 {
   // Initialize the ROS Node "roscpp_example"
   ros::init(argc, argv, "roscpp_example");

   // Instantiate the ROS Node Handler as nh
   ros::NodeHandle nh;

   // Print "Hello ROS!" to the terminal and ROS log file
   ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());
   ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1000, image_callback);
   // Program succesful
   ros::spin();
   return 0;
 }
