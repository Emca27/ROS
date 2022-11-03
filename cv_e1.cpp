#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Edge Detection";

class Edge_Detector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  Edge_Detector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
    //  &Edge_Detector::imageCb, this);
      image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &Edge_Detector::imageCb, this);
      
    image_pub_ = it_.advertise("/edge_detector/raw_image", 1);
    cv::namedWindow(OPENCV_WINDOW);

  }

  ~Edge_Detector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){

	detect_edges(cv_ptr->image);
    	image_pub_.publish(cv_ptr->toImageMsg());

	}
  }
  void detect_edges(cv::Mat img)
  {

   	cv::Mat src, src_gray;
	cv::Mat dst, detected_edges;
	cv::Mat binarization;

	int edgeThresh = 1;
	int lowThreshold = 200;
	int highThreshold =300;
	int kernel_size = 5;

	img.copyTo(src);

	cv::cvtColor( img, src_gray, CV_BGR2GRAY );
        cv::blur( src_gray, detected_edges, cv::Size(5,5) );
	cv::Canny( detected_edges, detected_edges, lowThreshold, highThreshold, kernel_size );
	
	double threshold_value=97;
	double max_binary_value=255;
	int threshold_type =3;
	
	
        cv::threshold(src_gray, binarization, threshold_value, max_binary_value, threshold_type );
        
        // Declare the output variables
        cv::Mat cdst, cdstP;
        
        
            // Copy edges to the images that will display the results in BGR
    cvtColor(detected_edges, cdst, cv::CV_BGR2GRAY);
    cdstP = cdst.clone();
        
        // Standard Hough Line Transform
        std::vector<cv::Vec2f> lines; // will hold the results of the detection
        //cv::HoughLines(detected_edges, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }


  	dst = cv::Scalar::all(0);
  	img.copyTo( dst, detected_edges);
  	//img.copyTo( dst, binarization);
  	//img.copyTo( dst, cdst);
  	
	dst.copyTo(img);

    	cv::imshow(OPENCV_WINDOW, src);
    	cv::imshow(OPENCV_WINDOW_1, dst);
    	cv::waitKey(3);

  }	
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Edge_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}
