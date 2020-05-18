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

// hash密钥
static const uint8_t secret_key_ground[32] = {
    0x42, 0x71, 0xb5, 0xe9,
    0x42, 0x71, 0xb5, 0xe9,
    0x42, 0x71, 0xb5, 0xe9,
    0x42, 0x71, 0xb5, 0xe9,
    0x42, 0x71, 0xb5, 0xe9,
    0x42, 0x71, 0xb5, 0xe9,
    0x42, 0x71, 0xb5, 0xe9,
    0x42, 0x71, 0xb5, 0xe9
};

mavlink_signing_t signing;
mavlink_signing_streams_t signing_streams;
void print_hex(BYTE str[], int len)
{
	int idx;

	for(idx = 0; idx < len; idx++)
		printf("%02x", str[idx]);
}


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
    //Autopilot_Interface autopilot_interface(port);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	port->start();
	//autopilot_interface.start();

    //至此初始化结束，缺省了关闭串口的初始化

    //ros::Subscriber gazebo_sub = nh.subscribe<nav_msgs::Odometry>("/prometheus/ground_truth/p300_basic", 100, boost::bind(&gazebo_cb,_1,port));
    ros::Subscriber sp_sub = nh.subscribe<geometry_msgs::Point>("/setpoint", 100, sp_cb);

    //初始起飞点
    setpoint.x = 0.0;
    setpoint.y = 0.0;
    setpoint.z = -1.0;

    // 频率
    ros::Rate rate(10.0);

    //密钥初始化及读取
    memset(&signing, 0, sizeof(signing));
    memcpy(signing.secret_key, secret_key_ground, 32);


    //签名stream初始化
    memset(&signing_streams, 0, sizeof(signing_streams));
    signing_streams.num_signing_streams = 0;
//     signing_streams.stream
    
// typedef struct __mavlink_signing_streams {
//     uint16_t num_signing_streams;
//     struct __mavlink_signing_stream {
//         uint8_t link_id;              ///< ID of the link (MAVLINK_CHANNEL)
//         uint8_t sysid;                ///< Remote system ID
//         uint8_t compid;               ///< Remote component ID
//         uint8_t timestamp_bytes[6];   ///< Timestamp, in microseconds since UNIX epoch GMT
//     } stream[MAVLINK_MAX_SIGNING_STREAMS];
// } mavlink_signing_streams_t;
    while(ros::ok())
    {
        // send_messages(autopilot_interface);
        // read_messages(autopilot_interface);
        bool success;               // receive success flag

        mavlink_message_t message;
        success = port->read_message(message);

		if( success )
		{
			// Handle Message ID
			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
                    mavlink_local_position_ned_t local_position_ned_cipher;
                    mavlink_local_position_ned_t local_position_ned_plain = { 0 };
					mavlink_msg_local_position_ned_decode(&message, &(local_position_ned_cipher));

                    // 初始化
                    //256 bit密码
                    BYTE key[1][32] = {
                        {0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81,0x1f,0x35,0x2c,0x07,0x3b,0x61,0x08,0xd7,0x2d,0x98,0x10,0xa3,0x09,0x14,0xdf,0xf4}
                    };
                    WORD key_schedule[60];
                    BYTE iv[1][16] = {
                        {0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff},
                    };
                    BYTE ciphertext[1][MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
                    BYTE enc_buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
                    
                    // 设置密钥
                    aes_key_setup(key[0], key_schedule, 256);
                    // 将mavlink payload结构体转为密文
                    memcpy(&ciphertext, &local_position_ned_cipher, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
                    //解密
                    aes_decrypt_ctr(ciphertext[0], MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, enc_buf, key_schedule, 256, iv[0]);
                    // printf("\nCiphertext   : ");
                    // print_hex(ciphertext[0], MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
                    // printf("\n-decrypted to: ");
                    // print_hex(enc_buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);

                    memcpy(&local_position_ned_plain, &enc_buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
                    //pass = pass && !memcmp(enc_buf, plaintext[0], MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
                    
                    printf("Got message LOCAL_POSITION_NED \n");
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
                    printf("    time_in_msg:    %u       (ms)\n", local_position_ned_plain.time_boot_ms );
                    printf("    pos  (NED):     %f %f %f (m)\n", local_position_ned_plain.x, local_position_ned_plain.y, local_position_ned_plain.z );
                    printf("    vel  (NED):     %f %f %f (m/s)\n", local_position_ned_plain.vx, local_position_ned_plain.vy, local_position_ned_plain.vz );
                    printf("\n");

                    uint64_t timestamp_test = get_time_msec();
	                signing.timestamp = timestamp_test; 

                    bool check_signature = mavlink_signature_check(&signing, &signing_streams,&message);

                    if(check_signature)
                    {
                        printf("check_signature pass ! \n");
                    }else
                    {
                        printf("check_signature not pass ! \n");
                    }
                    

					break;
				}
            }
        }else
        {
            //printf("No message !!! \n");
        }
        

        //回调一次 更新传感器状态
        //ros::spinOnce();
        usleep(100);
        //rate.sleep();
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
    printf("    time_in_local:   %lu  (us)\n", messages.time_stamps.highres_imu );
    printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
    printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
    printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
    printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
    printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
    printf("    temperature: %f C \n"       , imu.temperature );

    printf("\n");
}
