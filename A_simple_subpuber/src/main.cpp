#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "/home/ubuntu/catkin_ws/src/A_simple_subpuber/include/A_simple_subpuber/mavlink/v1.0/autoquad/mavlink.h"
#include "A_simple_subpuber/Mavlink.h"
#include "SerialRDM.h"
#define RADIUS_EARTH_METER (6378.137*1000.0)

int j=0;

SerialRDM serial;
void RadiansToDegrees(float *value);
void GetEulers(float qx, float qy, float qz, float qw, float *angle1,float *angle2, float *angle3);

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    //pub_ = n_.advertise<A_simple_subpuber::Mavlink>("/mavlink/to", 1000);
   //pub_ = n_.advertise<std::vector>("/mavlink/to", 1000);
    //Topic you want to subscribe
    sub_ = n_.subscribe("optitrack/pos", 1000, &SubscribeAndPublish::callback, this);
  }

 void callback(const geometry_msgs::PoseStamped& input)
  {
    //geometry_msgs::PoseStamped input;
    //.... do something with the input and generate the output...
float x = input.pose.position.x;
float y = input.pose.position.y;
float z = input.pose.position.z;
float qx = input.pose.orientation.x;
float qy = input.pose.orientation.y;
float qz = input.pose.orientation.z;
float qw = input.pose.orientation.w;
float yaw_l, pitch_l, roll_l;
GetEulers(qx,qy,qz,qw,&pitch_l,&roll_l,&yaw_l);

//pitch_c=roll_l;
//roll_c=-pitch_l;

//x_n=-z*1000;
//y_n=x*1000;
//z_n=-y*1000;
//yaw_n=-yaw_l;
//printf("pos:[%3.2f,%3.2f,%3.2f,%3.2f]\n",x_n,y_n,z_n,yaw_n);
double Lat_mav = x/RADIUS_EARTH_METER*180.0f/3.14159265f;
double Lon_mav = y/RADIUS_EARTH_METER*180.0f/3.14159265f;
double Alt_mav = z;
float Yaw_mav = -yaw_l;
   


 mavlink_message_t message;
  //  A_simple_subpuber::Mavlink rosmavlink_msg;
   static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
   static int messageLength;
    if(j==0){
    mavlink_msg_extern_gps_position_pack(1,1,&message,3, 0.1, 0.1, 0.1 ,0.1f, 1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,10000.0f, 0.0f,0.0f, 0.0f);
 
    messageLength = mavlink_msg_to_send_buffer(buffer, &message);
     /*for(int i=0;i<=messageLength/8;i++)
      printf("%X\n",buffer[i]);*/
  
 j++;
   }
    else{
    
    mavlink_msg_extern_gps_position_pack(1,1,&message,4, Lat_mav, Lon_mav, Alt_mav ,0.0f, 0.0f,Yaw_mav,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    messageLength = mavlink_msg_to_send_buffer(buffer, &message);
   
 }
    serial.serialsendf((const char*)buffer,messageLength); 




  /*smavlink_msg.sysid = 138;
    rosmavlink_msg.compid = 190;
    rosmavlink_msg.msgid = message.msgid;
	for (int i = 0; i < message.len / 8; i++)
	{
	(rosmavlink_msg.payload64).push_back(message.payload64[i]);
	}
    pub_.publish(rosmavlink_msg);
	pub_.publish(buffer);*/
  }

private:
  ros::NodeHandle n_; 
  //ros::Publisher pub_;
  ros::Subscriber sub_;
  
};//End of class SubscribeAndPublish */

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "serial");
  ROS_INFO_STREAM("Seiral node starting\n");

    serial.set_values("/dev/ttyAMA0");
    if(serial.init() != 1){
        printf("SerialComm error: fail to initialize\n");
        exit(-1);
    }
 
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}


void RadiansToDegrees(float *value)
{
    *value = (*value)*(180.0f/3.14159265f);
}

void GetEulers(float qx, float qy, float qz, float qw, float *angle1,float *angle2, float *angle3)
{
    float &heading = *angle1;
	float &attitude = *angle2;
	float &bank = *angle3;
	double test = qw*qx + qy*qz;
	if (test > 0.499) { 					// singularity at north pole
		heading = (float) 2.0f * atan2(qy,qw);
		attitude = 3.14159265f/2.0f;
		bank = 0;
		RadiansToDegrees(&heading);
		RadiansToDegrees(&attitude);
		RadiansToDegrees(&bank);
		return;
	}
	if (test < -0.499) {						// singularity at south pole
		heading = (float) -2.0f * atan2(qy,qw);
		attitude = - 3.14159265f/2.0f;
		bank = 0;
		RadiansToDegrees(&heading);
		RadiansToDegrees(&attitude);
		RadiansToDegrees(&bank);
		return;
	}
	double sqx = qx*qx;
	double sqy = qy*qy;
	double sqz = qz*qz;
	heading = (float) atan2((double)2.0*qw*qz-2.0*qx*qy , (double)1 - 2.0*sqz - 2.0*sqx);
	attitude = (float)asin(2.0*test);
	bank = (float) atan2((double)2.0*qw*qy-2.0*qx*qz , (double)1.0 - 2.0*sqy - 2.0*sqx);
	RadiansToDegrees(&heading);
	RadiansToDegrees(&attitude);
	RadiansToDegrees(&bank);
}

