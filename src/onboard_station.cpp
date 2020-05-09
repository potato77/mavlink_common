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
#include <std_msgs/String.h>

using std::string;
using namespace std;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void read_messages(Autopilot_Interface &api);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void timerCallback1(const ros::TimerEvent& e, Generic_Port *port)
{
    // 发送心跳包
    mavlink_heartbeat_t heartbeat = { 0 };
    heartbeat.custom_mode = 0;
    heartbeat.type  = 0;
    heartbeat.autopilot = 0;
    heartbeat.base_mode  = 0;
    heartbeat.system_status = 0;
    heartbeat.mavlink_version = 0;

    mavlink_message_t message;
    int	system_id    = 1; // system id
    int	component_id = 1; // component id
    mavlink_msg_heartbeat_encode(system_id, component_id, &message, &heartbeat);

    int len = port->write_message(message);
}

void timerCallback2(const ros::TimerEvent& e, Generic_Port *port)
{
    // 发送当前位置
    mavlink_local_position_ned_t local_position_ned = { 0 };
    local_position_ned.time_boot_ms = (uint32_t) (get_time_usec()/1000);
    local_position_ned.x  = 1.0;
    local_position_ned.y  = 2.0;
    local_position_ned.z  = 3.0;
    local_position_ned.vx = 4.0;
    local_position_ned.vy = 5.0;
    local_position_ned.vz = 6.0;

    mavlink_message_t message;
    int	system_id    = 1; // system id
    int	component_id = 1; // component id
    mavlink_msg_local_position_ned_encode(system_id, component_id, &message, &local_position_ned);

    int len = port->write_message(message);

	char buf[300];

	// Translate message to buffer
	unsigned len2 = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    printf("Sending a new message, length is %u.\n ", len2);
    for (int i=0; i<len2; i++) 
    {
        printf("%02X - ", (uint8_t)buf[i]);
    }
    printf("\n");


}

void timerCallback3(const ros::TimerEvent& e, Generic_Port *port)
{
    // 发送当前位置
    mavlink_attitude_t attitude = { 0 };
    attitude.time_boot_ms = (uint32_t) (get_time_usec()/1000);
    attitude.roll  = 0.1;
    attitude.pitch  = 0.2;
    attitude.yaw  = 0.3;
    attitude.rollspeed = 0.0;
    attitude.pitchspeed = 0.0;
    attitude.yawspeed = 0.0;

    mavlink_message_t message;
    int	system_id    = 1; // system id
    int	component_id = 1; // component id
    mavlink_msg_attitude_encode(system_id, component_id, &message, &attitude);

    int len = port->write_message(message);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "onboard_station");
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
    //Autopilot_Interface autopilot_interface(port);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	port->start();
	//autopilot_interface.start();

    //至此初始化结束，缺省了关闭串口的初始化
    // 发送心跳包
    //ros::Timer timer1 = nh.createTimer(ros::Duration(1.0), boost::bind(&timerCallback1,_1,port));
    // 发送local_position_ned
    ros::Timer timer2 = nh.createTimer(ros::Duration(1.0), boost::bind(&timerCallback2,_1,port));
    // 发送姿态
    //ros::Timer timer3 = nh.createTimer(ros::Duration(1.0), boost::bind(&timerCallback3,_1,port));

    // 频率
    ros::Rate rate(100.0);

    while(ros::ok())
    {
        //read_messages(autopilot_interface);

        //回调一次 更新传感器状态
        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}

void read_messages(Autopilot_Interface &api)
{
    printf("READ SOME MESSAGES \n");

    // copy current messages
    Mavlink_Messages messages = api.current_messages;

    // position_setpoint
    mavlink_set_position_target_local_ned_t pos_sp = messages.position_setpoint;
    printf("Got message SET_POSITION_TARGET_LOCAL_NED \n");
    printf("    time_in_msg:    %u    (ms)\n", pos_sp.time_boot_ms );
    printf("    time_in_local:  %u    (ms)\n", (uint32_t)(messages.time_stamps.position_target_local_ned/1000));
    printf("    pos_sp  (NED):  %f %f %f (m)\n", pos_sp.x, pos_sp.y, pos_sp.z );

    // vision_estimation
    mavlink_vision_position_estimate_t vision_est = messages.vision_est;
    printf("Got message VISION_POSITION_ESTIMATE \n");
    printf("    time_in_msg:     %lu  (us)\n", vision_est.usec);
    printf("    time_in_local:   %lu  (us)\n", messages.time_stamps.vision_est );
    printf("    vision_est:  %f %f %f (m)\n", vision_est.x, vision_est.y, vision_est.z );

    printf("\n");
}
