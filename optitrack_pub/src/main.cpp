#include "toolset.h"

ExternalGPS optitrack;

int main(int argc, char **argv) {
  int id=1;
  ros::init(argc,argv,"publish_pos");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("optitrack/pos",1000);

  optitrack.set_values(id,ONBOARD_IPADDRESS,EXTERNALGPS_IPADDRESS);
    if(optitrack.init() != 1){
        printf("ExternalGPS error: fail to initialize\n");
        exit(-1);
    }

  ros::Rate rate(30);

  while(ros::ok()){
    geometry_msgs::PoseStamped msg;
    optitrack.loop();
    
    msg.pose.position.x=optitrack.g_x;
    msg.pose.position.y=-optitrack.g_z;
    msg.pose.position.z=optitrack.g_y;
    msg.pose.orientation.x=optitrack.g_qx;
    msg.pose.orientation.y=optitrack.g_qy;
    msg.pose.orientation.z=optitrack.g_qz;
    msg.pose.orientation.w=optitrack.g_qw;

    pub.publish(msg);
    rate.sleep();
  }
  




}

