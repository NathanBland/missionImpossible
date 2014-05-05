/*
 *ROS program to control an AR Parrot drone with an xbox controller and with automated control.
 *
 *Written by Malachi Mart and David Meier
 */
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/LedAnim.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/FlightAnim.h>
#include <string>

using namespace std;

class ControlARDrone
{
public:
  ControlARDrone();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void navCallback(const ardrone_autonomy::Navdata::ConstPtr& nav);
  void normalCallBack(const geometry_msgs::Twist::ConstPtr& obj);
  void QRCallBack(const std_msgs::String::ConstPtr& command);
  void startUp();

  ros::NodeHandle nh_;

  bool hold_a, hold_b, hold_x, hold_y, hold_lbump, hold_rbump, hold_start, hold_reset, hold_xbox, auto_on, flying;
  int linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, buttoncount, height;
  int droneState; // -1 == user control, 0 == searching, 1 == start up, 2 == positioning
  double l_scale_, a_scale_, p, lastXPos, lastYPos, lastZPos;
  string QR_msg;
  ros::Time timeOfLastCycle;
  geometry_msgs::Twist kinetic;

  ros::Publisher vel_pub_;
  ros::Publisher ar_launch;
  ros::Publisher ar_land;
  ros::Subscriber joy_sub_;
  ros::Subscriber norm_sub;
  ros::Subscriber tag_sub;
  ros::Subscriber QR_sub;
  ros::Publisher ar_reset;
  ros::ServiceClient cam_toggle;
  ros::ServiceClient led_change;
  ardrone_autonomy::CamSelect camsrv;
  ardrone_autonomy::FlightAnim aminsrv;
  ardrone_autonomy::FlightAnimRequest animreq;
  ardrone_autonomy::FlightAnimResponse animresponse;
  ardrone_autonomy::LedAnim ledsrv;
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response response;
  std_msgs::Empty empty_msg;
};

ControlARDrone::ControlARDrone() {
  linear_x = 4; linear_y=3; linear_z=1; angular_x=0; angular_y=0; angular_z=0;
  a_scale_= 4.0; l_scale_= 4.0; auto_on = false; droneState = -1; flying = false;
  nh_.param("axis_linearx", linear_x, linear_x);
  nh_.param("axis_lineary", linear_y, linear_y);
  nh_.param("axis_linearz", linear_z, linear_z);
  nh_.param("axis_angularx", angular_x, angular_x);
  nh_.param("axis_angulary", angular_y, angular_y);
  nh_.param("axis_angularz", angular_z, angular_z);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ar_land = nh_.advertise<std_msgs::Empty>("/ardrone/land", 10);
  ar_launch = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
  ar_reset = nh_.advertise<std_msgs::Empty>("/ardrone/reset", 1);

  cam_toggle = nh_.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel");
  led_change = nh_.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");
  ledsrv.request.duration = 5;
  ledsrv.request.freq = 4;
  camsrv.request.channel = 0;
  buttoncount=0;

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &ControlARDrone::joyCallback, this);
  tag_sub = nh_.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 5, &ControlARDrone::navCallback, this);
  //norm_sub = nh_.subscribe<geometry_msgs::Twist>("/normalModeling/obj_loc", 1000, &ControlARDrone::normalCallBack, this);
  norm_sub = nh_.subscribe<geometry_msgs::Twist>("/QR_twist", 10, &ControlARDrone::normalCallBack, this);
  QR_sub = nh_.subscribe<std_msgs::String>("/QR_msg", 10, &ControlARDrone::QRCallBack, this);
}

void ControlARDrone::startUp() {
    if (QR_msg == ""){

    }
}

void ControlARDrone::normalCallBack(const geometry_msgs::Twist::ConstPtr& obj){
    geometry_msgs::Twist vel;
    double x = obj->linear.x;
    double y = obj->linear.y;
    double z = obj->linear.z;
    double scale= .04;
    if (droneState == 2) {
        if(x < 0){
            vel.linear.y = scale*(100 - x)/100;
        }
        if(x > 0) {
            vel.linear.y = -scale*abs(100 - x)/100;
        }if(y < 0){
            vel.linear.z= scale* (100 - y)/100;
        }
        if(y > 0){
            vel.linear.z= -scale*abs(100 - y)/100;
        }
        if (z < 3000){
            vel.linear.x = scale*abs(3000 - z)/3000;
        }
        if (z > 6000){
            vel.linear.x = scale*abs(z - 6000)/6000;
        }
        vel_pub_.publish(vel);
    }
}

void ControlARDrone::QRCallBack(const std_msgs::String::ConstPtr& command){
    geometry_msgs::Twist vel;
    if(droneState != 2){
        droneState = 2;
        QR_msg = command->data;
        ar_land.publish(empty_msg);
    }

    if (command->data == "left"){
        vel.angular.z = 30;
    } else if(command->data == "right") {
        vel.angular.z = -30;
    }else if(command->data == "take off") {
        ar_launch.publish(empty_msg);
    }else if(command->data == "land") {
        ar_land.publish(empty_msg);
    }
    vel_pub_.publish(vel);
    droneState=-1;
}

void ControlARDrone::navCallback(const ardrone_autonomy::Navdata::ConstPtr& nav){
    geometry_msgs::Twist vel;
    height = nav->altd;
    if (droneState == 1){
        if(!flying){
            ar_launch.publish(empty_msg);
            flying = true;
        }
        if(height < 1200 || height > 1400){
            if (height > 1400){
                vel.linear.z = -.5;
            }else{
                vel.linear.z = .5;
            }
            vel_pub_.publish(vel);
        }else{
            droneState = 0;
        }
    }
    if(droneState == 0) {
        vel.angular.z = -.2;
        vel_pub_.publish(vel);
    }
}

void ControlARDrone::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    if(joy->buttons[0]&&!hold_a){
        droneState = 1;
        cout << droneState << endl;
        hold_a=true;
    }else if(joy->buttons[0]==0){hold_a=false;}

    if(joy->buttons[1]&&!hold_b){
        vel.angular.z = 30;
        vel_pub_.publish(vel);
        hold_b=true;
    }else if(joy->buttons[1]==0){hold_b=false;}
/*
  if(joy->buttons[2]&&!hold_x){
        buttoncount++;
        if (buttoncount>13){
            buttoncount = 0;
        }
        ledsrv.request.type = buttoncount;
        led_change.call(ledsrv);
        hold_x=true;
    }else if(joy->buttons[2]==0){hold_x=false;}

    if(joy->buttons[3]&&!hold_y){

        hold_y=true;
    }else if(joy->buttons[3]==0){hold_y=false;}
*/
    if(joy->buttons[4]&&!hold_rbump){
        ar_land.publish(empty_msg);
        droneState = -1;
        hold_rbump=true;
    }else if(joy->buttons[4]==0){hold_rbump=false;}

    if(joy->buttons[5]&&!hold_lbump){
        ar_launch.publish(empty_msg);
        hold_lbump=true;
    }else if(joy->buttons[5]==0){hold_lbump=false;}

    if(joy->buttons[6]&&!hold_reset){
        ar_reset.publish(empty_msg);
        hold_reset=true;
        droneState = -1;
        auto_on = false;
    }else if(joy->buttons[6]==0){hold_reset=false;}

    if(joy->buttons[7]&&!hold_start){
        if (camsrv.request.channel == 0){camsrv.request.channel =1;}else{camsrv.request.channel =0;}
        cam_toggle.call(camsrv);
        hold_start=true;
    }else if(joy->buttons[7]==0){hold_start=false;}

    if(joy->buttons[8]&&!hold_xbox){
        if (auto_on){
            auto_on = false;
            ledsrv.request.type = 0;
            led_change.call(ledsrv);
        }else{
            auto_on = true;
            ledsrv.request.type = 6;
            led_change.call(ledsrv);
        }
        hold_xbox=true;
    }else if(joy->buttons[8]==0){hold_xbox=false;}


  //if (!auto_on){
      vel.linear.x = 3*joy->axes[linear_x];
      vel.linear.y = 3*joy->axes[linear_y];
      vel.linear.z = 3*joy->axes[linear_z];
      //vel.angular.z = 3*joy->axes[angular_z];
      vel_pub_.publish(vel);
  //}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tele_op");
  ControlARDrone tele_op;

  ros::spin();
}
