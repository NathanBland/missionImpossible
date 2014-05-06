#include <iostream>
#include <iomanip>
#include <ros/publisher.h>
#include <std_msgs/String.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <geometry_msgs/Twist.h>
#include <zbar.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <memory>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <fstream>
#include <math.h>
#define PI 3.14159265

using namespace std;
using namespace cv;
using namespace zbar;
namespace enc = sensor_msgs::image_encodings;

class rec{
public:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    ros::Publisher loc;
    ros::Publisher message;
    string paperModel;
    cv::NormalBayesClassifier paperClassifier;
    cv_bridge::CvImagePtr cv_ptr;

    rec() :  it(nh){
        sub = it.subscribe("ardrone/image_raw", 1, &rec::imageCallback,this);
        loc = nh.advertise<geometry_msgs::Twist>("/QR_twist",1);
        message = nh.advertise<std_msgs::String>("/QR_mgs",1);

        ros::param::param<std::string>("~paperModel", paperModel, "paper.yml");

        if(fileExists(paperModel)) {
          paperClassifier.load(paperModel.c_str());
          ROS_INFO("Loaded %s", paperModel.c_str());
        } else {
          ROS_WARN("No paper model loaded");
        }
    }

    bool fileExists(const std::string& fname)
    {
      std::ifstream f(fname.c_str());
      if(f.good())
      {
        f.close();
        return true;
      } else
      {
        f.close();
        return false;
      }
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg){

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8); // extract image
        } catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        Mat frame = cv_ptr->image;

        cv::resize(cv_ptr->image, frame, cv::Size(0,0), 0.25, 0.25);
        cv::Mat vectorizedFrame = frame.reshape(1, frame.rows*frame.cols);
        cv::Mat testf, labelf;
        vectorizedFrame.convertTo(testf, CV_32FC1, 1.0/255.0, 1.0);
        paperClassifier.predict(testf, &labelf);
        labelf = labelf.reshape(1, frame.rows);
        cv::dilate(labelf, labelf, cv::Mat::ones(3,3, CV_32FC1), cv::Point(-1,-1), 2);
        cv::erode(labelf, labelf, cv::Mat::ones(3,3, CV_32FC1), cv::Point(-1,-1), 2);
        // find the centroid of the largest blob
        cv::Mat labeli;
        labelf.convertTo(labeli, CV_8UC1);
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(labeli, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        if(contours.size() <= 0){
            ROS_WARN("Nothing detected!!");
            return;
        }
          // look for largest blob
        size_t bigIdx = 0;
        double contourArea = 0.0;
        for(size_t ii = 0; ii < contours.size(); ii++){
            double tempA = cv::contourArea(contours.at(ii));
            if(tempA > contourArea){
                contourArea = tempA;
                bigIdx = ii;

            }
        }
        cv::Moments mu = cv::moments(contours.at(bigIdx));
        cv::Point2f paper = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        cv::Point2f distance = cv::Point2f( (msg->height/2)-paper.x, (msg->width/2)-paper.y);

        if(!(isnan(distance.x) || isnan(distance.y))){
            geometry_msgs::Twist t;
            t.linear.x = distance.x;
            t.linear.y = distance.y;
            //sleep(1);
            loc.publish(t);
        }

    }

    cv::Point2f detectModel(const sensor_msgs::ImageConstPtr& msg, bool sphero){


    }

private:

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "image_rec");
    rec image;
    ros::spin();
    return 0;
}
