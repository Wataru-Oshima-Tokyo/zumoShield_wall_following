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


void img_modified(cv::Mat &frame){
    // cvtColor(frame, frame,COLOR_BGR2RGB);
    for(int i=1; i<((IMG_HEIGHT)-1); i++)
    {

        // Skip first and last column, no neighbors to convolve with
        for(int j=1; j<((IMG_WIDTH)-1); j++)
        {
            cv::Vec3b &color = frame.at<cv::Vec3b>(cv::Point(j,i));
            if(i<4 || i>IMG_HEIGHT-4 || j<4 || j>IMG_WIDTH-4 || i == IMG_HEIGHT/2 || j==IMG_WIDTH/2){
                //BGR
                color.val[0] = 0;
                color.val[1] = 255;
                color.val[2] = 255;
             }
        }
    }

}

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
   int low_c[3] = {17, 123, 121};
   int high_c[3] ={37, 143, 201};
   cv::Mat frame = cv_ptr->image, frame_HSV, frame_threshold, mask,resized;
   int fheight = frame.cols, fwidth = frame.rows;

   //change the image from BGR to HSV
   cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
   //mask the image in yellow
   cv::inRange(frame_HSV, cv::Scalar(low_c[0],low_c[1],low_c[1]), cv::Scalar(high_c[0],high_c[1],high_c[2]),mask);
   //resize the image
  //  cv::resize(mask, resized, cv::Size(), fwidth/3, fheight/3);
   int search_top = fheight/4*3; 
   int search_bot = search_top+20;//focus on just the front of cam <- need to know only 20 rows
  //  mask[0:search_top, 0:w] =0;
  //  mask[search_bot:fheight, 0:w] = 0;
  //  for(int i=1; i<mask.rows-1; i++){
  //    for(int j=1; j<mask.cols-1; j++){
  //      cv::Vec3b &color = mask.at<cv::Vec3b>(cv::Point(j,i));
  //      if(j< search_top || (j>search_bot && j<fheight)){
  //         color.val[0] =0;
  //         color.val[1] =0;
  //         color.val[2] =0;
  //      }
  //    }
  //  }

   cv::Moments M = cv::moments(mask); // get the center of gravity
   if (M.m00 >0){
   			int cx = int(M.m10/M.m00); //重心のx座標
   			int cy = int(M.m01/M.m00); //重心のy座標
      
      cv::circle(frame, cv::Point(cx,cy), 5, cv::Scalar(0, 0, 255));
   }
   
   cv::imshow("original", frame);
   //cv::imshow("Masked", mask);
   cv::waitKey(3);

}

//  void image_cb(const sensor_msgs::ImageConstPtr& msg)
//  {
//    std_msgs::Header msg_header = msg->header;
//    std::string frame_id = msg_header.frame_id.c_str();
//    ROS_INFO_STREAM("New Image from " << frame_id);

//    cv_bridge::CvImagePtr cv_ptr;

//    try
//    {
//      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }
//    img_modified(cv_ptr->image);
//    // Draw an example crosshair
// //    cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

//    // Update GUI Window
//    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//    cv::waitKey(3);

//    // Output modified video stream
// //    pub.publish(cv_ptr->toImageMsg());
//  }

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
