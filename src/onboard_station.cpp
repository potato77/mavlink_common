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
#include "mavlink_aes.c"



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
void print_hex(BYTE str[], int len)
{
	int idx;

	for(idx = 0; idx < len; idx++)
		printf("%02x", str[idx]);
}

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
    // 本地信息转储为mavlink payload结构体
    mavlink_local_position_ned_t local_position_ned = { 0 };
    mavlink_local_position_ned_t local_position_ned_cipher = { 0 };
    local_position_ned.time_boot_ms = (uint32_t) get_time_msec();
    local_position_ned.x  = 1.0;
    local_position_ned.y  = 2.0;
    local_position_ned.z  = 3.0;
    local_position_ned.vx = 4.0;
    local_position_ned.vy = 5.0;
    local_position_ned.vz = 6.0;

    // 明文加密，使用AES算法

    // 初始化
    //256 bit密码
	BYTE key[1][32] = {
		{0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81,0x1f,0x35,0x2c,0x07,0x3b,0x61,0x08,0xd7,0x2d,0x98,0x10,0xa3,0x09,0x14,0xdf,0xf4}
	};
    WORD key_schedule[60];
	BYTE iv[1][16] = {
		{0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff},
	};
    BYTE plaintext[1][MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
	BYTE enc_buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];

    // 设置密钥
	aes_key_setup(key[0], key_schedule, 256);

	// printf(  "Key          : ");
	// print_hex(key[0], 32);
	// printf("\nIV           : ");
	// print_hex(iv[0], 16);

    // 将mavlink payload结构体转为明文
    memcpy(&plaintext, &local_position_ned, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);

    // 使用AES算法加密，并将密文存储到enc_buf中
	aes_encrypt_ctr(plaintext[0], MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, enc_buf, key_schedule, 256, iv[0]);
	// printf("\nPlaintext    : ");
	// print_hex(plaintext[0], MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
	// printf("\n-encrypted to: ");
	// print_hex(enc_buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
    // printf("\n\n");

    memcpy(&local_position_ned_cipher, &enc_buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);

    // 将加密后的密文编码为mavlink_message标准结构体
    mavlink_message_t message;
    int	system_id    = 1; // system id
    int	component_id = 1; // component id
    mavlink_msg_local_position_ned_encode(system_id, component_id, &message, &local_position_ned_cipher);

    // 发送
    int len = port->write_message(message);

	// char buf[300];
	// // Translate message to buffer
	// unsigned len2 = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    printf("Sending message LOCAL_POSITION_NED \n");
    printf("    magic:          %02X    \n", message.magic );
    printf("    len:            %02X    \n", message.len );
    printf("    incompat_flags: %02X    \n", message.incompat_flags );
    printf("    compat_flags:   %02X    \n", message.compat_flags );
    printf("    seq:            %02X    \n", message.seq );
    printf("    sysid:          %02X    \n", message.sysid );
    printf("    compid:         %02X    \n", message.compid );
    printf("    msgid:          %02X    \n", message.msgid );
    printf("    ckecksum:       %02X-%02X    \n", message.ck[0],message.ck[1] );
    printf("    link id:        %02X    \n", message.signature[0] );
    printf("    time stamp:     %02X-%02X-%02X-%02X-%02X-%02X    \n", message.signature[1], message.signature[2], message.signature[3], message.signature[4], message.signature[5], message.signature[6] );
    printf("    signature:      %02X-%02X-%02X-%02X-%02X-%02X    \n", message.signature[7], message.signature[8], message.signature[9], message.signature[10], message.signature[11], message.signature[12] );
    printf("    payload_cipher:   \n" );
    printf("    time_in_msg:    %u       (ms)\n", local_position_ned_cipher.time_boot_ms );
    printf("    pos  (NED):     %f %f %f (m)\n", local_position_ned_cipher.x, local_position_ned_cipher.y, local_position_ned_cipher.z );
    printf("    vel  (NED):     %f %f %f (m/s)\n", local_position_ned_cipher.vx, local_position_ned_cipher.vy, local_position_ned_cipher.vz );
    printf("    payload_plain:   \n" );
    printf("    time_in_msg:    %u       (ms)\n", local_position_ned.time_boot_ms );
    printf("    pos  (NED):     %f %f %f (m)\n", local_position_ned.x, local_position_ned.y, local_position_ned.z );
    printf("    vel  (NED):     %f %f %f (m/s)\n", local_position_ned.vx, local_position_ned.vy, local_position_ned.vz );

    printf("\n");

}

void timerCallback3(const ros::TimerEvent& e, Generic_Port *port)
{
    // 发送当前位置
    mavlink_attitude_t attitude = { 0 };
    attitude.time_boot_ms = (uint32_t) get_time_msec();
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
