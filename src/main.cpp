#include <iostream>
#include <iomanip>
#include <ros/publisher.h>
#include <std_msgs/String.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <geometry_msgs/Twist.h>
#include <zbar.h>

using namespace std;
using namespace cv;
using namespace zbar;

class rec{
public:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    ros::Publisher loc;
    ros::Publisher message;

    rec() :  it(nh){
        sub = it.subscribe("ardrone/image_raw", 1, &rec::imageCallback,this);
        loc = nh.advertise<geometry_msgs::Twist>("/QR_twist",1);
        message = nh.advertise<std_msgs::String>("/QR_mgs",1);
    }
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        ImageScanner scanner;

        // Configure the reader
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
        // Capture an OpenCV frame
        cv::Mat frame, frame_grayscale;
        frame = cv_ptr->image;

        // Convert to grayscale
        cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

        // Obtain image data
        int width = frame_grayscale.cols;
        int height = frame_grayscale.rows;
        uchar *raw = (uchar *)(frame_grayscale.data);

        // Wrap image data
        Image image(width, height, "Y800", raw, width * height);

        // Scan the image for barcodes
        //int n = scanner.scan(image);
        scanner.scan(image);

        // Extract results
        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

           int n = symbol->get_location_size();

           vector<Point> vp;

            for(int i=0;i<n;i++){
                vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
            }

            RotatedRect r = minAreaRect(vp);

            geometry_msgs::Twist twist;
            twist.linear.x = 100*((double)r.center.y/frame.cols);
            twist.linear.y = 100*((double)r.center.x/frame.rows);
            twist.linear.z = 100*((double)r.size.area() / frame.size().area());
            loc.publish(twist);

            std_msgs::String QR_msg;
            QR_msg.data = symbol->get_data();
            message.publish(QR_msg);
        }

        // Show captured frame, now with overlays!
        imshow("captured", frame);

        // clean up
        image.set_data(NULL, 0);

        waitKey(30);
    }

private:

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "image_rec");
    rec image;
    ros::spin();
    return 0;
}
