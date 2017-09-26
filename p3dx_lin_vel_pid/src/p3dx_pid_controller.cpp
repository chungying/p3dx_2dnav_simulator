#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "message_filters/subscriber.h"
#include "message_filters/cache.h"
#include <boost/bind.hpp>
#include <iostream>

using namespace nav_msgs;
using namespace geometry_msgs;
using namespace message_filters;

ros::Publisher stamped_cmd_pub;
ros::Publisher cmd_pub;
ros::Publisher err_pub;
boost::shared_ptr< Cache<Odometry> > odo_cache_ptr;
boost::shared_ptr< Cache<TwistStamped> > ref_cache_ptr;

Twist pid(TwistStampedConstPtr& latest_ref, std::vector<OdometryConstPtr>& odo_vec);

int seq = 0;
void making_stamp_cb(const Twist& msg)
{
  TwistStamped stamped_cmd_vel;
  stamped_cmd_vel.header.seq = seq++;
  stamped_cmd_vel.header.stamp = ros::Time::now();
  stamped_cmd_vel.twist = msg;

  stamped_cmd_pub.publish(stamped_cmd_vel);
}

struct Gains{
  Gains(){
    Kp = 1;
    Ki = 0;
    Kd = 0;
  }
  double Kp;
  double Ki;
  double Kd;
} gains_x;

struct State{
  State(){
    err = 0;
    integral = 0;
    differential = 0;
  }
  double err;
  double integral;
  double differential;
} cur_sta_x, pre_sta_x; 

bool ref_updated = false;

void odo_callback( const OdometryConstPtr& odo_msg){
  TwistStampedConstPtr latest_ref = ref_cache_ptr->getElemBeforeTime(ref_cache_ptr->getLatestTime());
  ref_updated = false;

  std::vector<OdometryConstPtr> odo_vec = odo_cache_ptr->getInterval(odo_cache_ptr->getOldestTime(), odo_cache_ptr->getLatestTime());

//TODO use ROSWARN
  if(latest_ref.get()==0){
//    std::cout << "ref_cmd is null" << std::endl;
    
  }
  else if(odo_vec.size()<2){
//    std::cout << "amount of odometry data is not enough" << std::endl;
  }
  else{
    Twist cmd_msg;
    std_msgs::Float64 err_msg;
    if(latest_ref->twist.linear.x!=0 || latest_ref->twist.angular.z!=0){
      cmd_msg = pid(latest_ref, odo_vec);
      err_msg.data = cur_sta_x.err;
    }
    err_pub.publish(err_msg);
    cmd_pub.publish(cmd_msg);
  }
}

Twist pid(TwistStampedConstPtr& latest_ref, std::vector<OdometryConstPtr>& odo_vec){

    double dt = (odo_vec[1]->header.stamp - odo_vec[0]->header.stamp).toSec();
    
    cur_sta_x.err = latest_ref->twist.linear.x - odo_vec[1]->twist.twist.linear.x;
    cur_sta_x.integral = cur_sta_x.err*dt + pre_sta_x.integral;//integral = err*dt +pre_integral
    cur_sta_x.differential = (cur_sta_x.err - pre_sta_x.err) / dt;
    
    double output_x = gains_x.Kp * cur_sta_x.err + 
                      gains_x.Ki * cur_sta_x.integral + 
                      gains_x.Kd * cur_sta_x.differential;
    //TODO maximum speed should be an argument of this node
    if(abs(output_x)>1.0)
      output_x = 0.5;//TODO this saturation should be an argument as well.
    pre_sta_x.err = cur_sta_x.err;
    pre_sta_x.integral = cur_sta_x.integral;
    pre_sta_x.differential = cur_sta_x.differential;

    Twist cmd_msg;
    cmd_msg.linear.x = output_x;
    cmd_msg.linear.y = 0;
    cmd_msg.linear.z = 0;
    cmd_msg.angular.x = 0;
    cmd_msg.angular.y = 0;
    cmd_msg.angular.z = latest_ref->twist.angular.z;

    return cmd_msg;
}


void ref_callback( const TwistStampedConstPtr& ref_msg){
  ref_updated = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "p3dx_pid_controller");

  if(argc == 4){
    gains_x.Kp = strtod(argv[1], NULL);
    gains_x.Ki = strtod(argv[2], NULL);
    gains_x.Kd = strtod(argv[3], NULL);
    std::cout << "gains_x: " << gains_x.Kp << " " << gains_x.Ki << " " << gains_x.Kd << std::endl;
  }

  ros::NodeHandle nh ;
  stamped_cmd_pub = nh.advertise<TwistStamped>("/move_base/stamped_cmd_vel", 10) ;
  cmd_pub = nh.advertise<Twist>("/p3dx/cmd_vel", 10);
  err_pub = nh.advertise<std_msgs::Float64>("p3dx/pid/err", 10);
  ros::Subscriber raw_cmd_sub = nh.subscribe("/move_base/cmd_vel", 10, &making_stamp_cb) ;

  Subscriber<Odometry> odo_sub(nh, "/p3dx/odom", 1);
  odo_cache_ptr.reset(new Cache<Odometry>(odo_sub, 2));
  odo_cache_ptr->registerCallback(boost::bind( &odo_callback, _1));

  Subscriber<TwistStamped> ref_sub(nh, "/move_base/stamped_cmd_vel", 1);
  ref_cache_ptr.reset(new Cache<TwistStamped>(ref_sub, 10));
  ref_cache_ptr->registerCallback(boost::bind( &ref_callback, _1));

  ros::spin() ;
  odo_cache_ptr.reset();
  ref_cache_ptr.reset();
  return 0;
}
