#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include <iomanip>
#include <ros/publisher.h>
#include <std_msgs/String.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

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
        sub = it.subscribe("ardrone/image_raw", 1, &image_rec::imageCallback,this);
        loc = nh.advertise<geometry_msgs::Twist>("/QR_twist",1);
        message = it.advertise("/QR_mgs",1);
    }
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
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
        cv_ptr >> frame;

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
            vector<Point> vp;

            // do something useful with results



           int n = symbol->get_location_size();
           for(int i=0;i<n;i++){
                vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
           }
           RotatedRect r = minAreaRect(vp);
           Point2f pts[4];
           r.points(pts);

            // Draw location of the symbols found
            if (symbol->get_location_size() == 4) {
                //rectangle(frame, Rect(symbol->get_location_x(i), symbol->get_location_y(i), 10, 10), Scalar(0, 255, 0));
                line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 0, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(255, 255, 255), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(255, 255, 255), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(255, 255, 255), 2, 8, 0);
            }
            vector<Point> vp;
            // do something useful with results
            cout << "decoded " << symbol->get_type_name()
            << " symbol \"" << symbol->get_data() << '"' <<" "<< endl;
            int n = symbol->get_location_size();
            for(int i=0;i<n;i++){
                vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
            }
            RotatedRect r = minAreaRect(vp);

            geometry_msgs::Twist twist;

            twist.linear.x = 100*((double)r.center.y/frame.cols);
            twist.linear.y = 100*((double)r.center.x/frame.rows);
            twist.linear.z = 100*((double)r.size / frame.size);
            std_msgs::String QR_msg;
            QR_msg.data = symbol->get_data();
            QR_msg_pub.publish(QR_msg);
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
    ros::init(argc, argv, "missinImpossible");
    ros::NodeHandle nh_;

    int cam_idx = 0;

    if (argc == 2) {
        cam_idx = atoi(argv[1]);
    }

    VideoCapture cap(cam_idx);
    if (!cap.isOpened()) {
        cerr << "Could not open camera." << endl;
        exit(EXIT_FAILURE);
    }
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    namedWindow("captured", CV_WINDOW_AUTOSIZE);

    // Create a zbar reader


    return 0;
}
