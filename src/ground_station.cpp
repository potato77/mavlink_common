//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>

#include <common/mavlink.h>

#include "generic_port.h"
#include "serial_port.h"
#include "udp_port.h"
#include "autopilot_interface.h"
#include "math_utils.h"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>


using std::string;
using namespace std;

int	system_id    = 0; // system id
int	component_id = 0; // component id
//---------------------------------------gazebo真值相关------------------------------------------
Eigen::Vector3d pos_drone_gazebo;
Eigen::Quaterniond q_gazebo;
Eigen::Vector3d Euler_gazebo;   
geometry_msgs::Point setpoint;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void read_messages(Autopilot_Interface &api);
void send_messages(Autopilot_Interface &api);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void gazebo_cb(const nav_msgs::Odometry::ConstPtr& msg, Generic_Port *port)
{
    q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Euler_gazebo = quaternion_to_euler(q_gazebo);
    
    
    // NED frame
    mavlink_vision_position_estimate_t pose_vision = { 0 };
    ros::Time stamp = ros::Time::now();
    pose_vision.usec = stamp.toNSec() / 1000;
    pose_vision.x = msg->pose.pose.position.x;
    pose_vision.y = - msg->pose.pose.position.y;
    pose_vision.z = - msg->pose.pose.position.z;
    pose_vision.roll = Euler_gazebo[0];
    pose_vision.pitch = Euler_gazebo[1];
    pose_vision.yaw = -Euler_gazebo[2];
    //pose_vision.covariance = { 0 };

    mavlink_message_t message;
    mavlink_msg_vision_position_estimate_encode(system_id, component_id, &message, &pose_vision);

    int len = port->write_message(message);
}
void sp_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    setpoint = *msg;
    printf("Got a new setpoint \n");
    printf("    setpoint  (NED):  % f % f % f (m/s^2)\n", setpoint.x , setpoint.y , setpoint.z );
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station");
    ros::NodeHandle nh("~");

    bool use_udp = false;
    // 串口参数
    int baudrate,tx_port, rx_port;
    string uart_name,udp_ip;

    nh.param<bool>("use_udp", use_udp, false);
    nh.param<string>("uart_name", uart_name, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 57600);
    nh.param<string>("udp_ip", udp_ip, "127.0.0.1");
    nh.param<int>("rx_port", rx_port, -1);
    nh.param<int>("tx_port", tx_port, -1);

	const char *uart_name_adr = uart_name.c_str();
	const char *udp_ip_adr = udp_ip.c_str();

	Generic_Port *port;
	if(use_udp)
	{
		port = new UDP_Port(udp_ip_adr, rx_port, tx_port);
	}
	else
	{
		port = new Serial_Port(uart_name_adr, baudrate);
	}

	//Instantiate an autopilot interface object
    Autopilot_Interface autopilot_interface(port);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	port->start();
	autopilot_interface.start();

    //至此初始化结束，缺省了关闭串口的初始化

    ros::Subscriber gazebo_sub = nh.subscribe<nav_msgs::Odometry>("/prometheus/ground_truth/p300_basic", 100, boost::bind(&gazebo_cb,_1,port));
    ros::Subscriber sp_sub = nh.subscribe<geometry_msgs::Point>("/setpoint", 100, sp_cb);

    //初始起飞点
    setpoint.x = 0.0;
    setpoint.y = 0.0;
    setpoint.z = -1.0;

    // 频率
    ros::Rate rate(1.0);

    // 启动offboard模式
	autopilot_interface.enable_offboard_control();
	usleep(100); // give some time to let it sink in

	// 自主起飞
    // 解锁
    autopilot_interface.arm_disarm(true);
    usleep(100); // give some time to let it sink in

    while(ros::ok())
    {
        send_messages(autopilot_interface);
        read_messages(autopilot_interface);

        //回调一次 更新传感器状态
        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}

void send_messages(Autopilot_Interface &api)
{
	printf("SEND OFFBOARD COMMANDS\n");

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

	sp.x   = setpoint.x;
	sp.y   = setpoint.y;
	sp.z   = setpoint.z;
    sp.yaw = 0.0;

	// SEND THE COMMAND , 这里调用的是Autopilot_Interface的子函数，Autopilot_Interface会负责持续发送这个消息到飞控
	api.update_setpoint(sp);
}

void read_messages(Autopilot_Interface &api)
{
    printf("READ SOME MESSAGES \n");

    // copy current messages
    Mavlink_Messages messages = api.current_messages;

    // local position in ned frame
    mavlink_local_position_ned_t pos = messages.local_position_ned;
    printf("Got message LOCAL_POSITION_NED \n");
    printf("    time_in_msg:    %u    (ms)\n", pos.time_boot_ms );
    printf("    time_in_local:  %u    (ms)\n", (uint32_t)(messages.time_stamps.local_position_ned/1000));
    printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

    // hires imu
    mavlink_highres_imu_t imu = messages.highres_imu;
    printf("Got message HIGHRES_IMU \n");
    printf("    time_in_msg:     %lu  (us)\n", imu.time_usec);
    printf("    time_in_local:   %lu  (ms)\n", messages.time_stamps.highres_imu );
    printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
    printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
    printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
    printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
    printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
    printf("    temperature: %f C \n"       , imu.temperature );

    printf("\n");
}
