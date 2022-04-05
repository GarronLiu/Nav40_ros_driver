#include <ros/ros.h>
#include <fcntl.h>      //open函数的头文件
#include <termios.h>    //串口驱动函数
#include <unistd.h>
#include <errno.h>    
#include <stdio.h>      //标准输入输出头文件
#include <string.h>
#include "std_msgs/Float32.h"
#include "nav40_demo.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <sensor_msgs/Imu.h>
#include "tf/transform_datatypes.h"//转换函数头文件
using namespace std;

#define NAV_DL     122    //数据长度
#define NAV_DH1    0xEB   //帧头
#define NAV_DH2    0x90   //帧头
#define MAXSIZE    2048   //缓冲区长度

typedef struct
{
	unsigned char Recbuf[MAXSIZE];  //缓冲数组
	int tail;              //尾指针
	int head;              //头指针
}Suqueue;

Suqueue queue_cycle;          //创建缓冲数组            
APM_Datatype APM;            //创建帧结构体
unsigned int checksum = 0;  //校验和
unsigned int checkRes_L, checkRes_H; //4个字节
unsigned char temp_buf[122]={0};
unsigned char buf[1];
int len;
//设置波特率，初始化串口
int set_uart_baudrate(const int _fd, unsigned int baud)
{
	int speed;
	switch (baud) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
  case 460800: speed = B460800; break;
	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	tcgetattr(_fd, &uart_config); //获取终端参数

  uart_config.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
  uart_config.c_cflag &= ~PARENB;  /* no parity bit */
  uart_config.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
  uart_config.c_cflag &= ~CSIZE;
  uart_config.c_cflag |= CS8;    /* 8-bit characters */
  uart_config.c_cflag &= ~CRTSCTS;/* no hardware flowcontrol */

  /* setup for non-canonical mode */
	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	uart_config.c_iflag = 0;
	uart_config.c_oflag = 0;

  /* fetch bytes as they become available */
	uart_config.c_cc[VTIME] = 0;
	uart_config.c_cc[VMIN] = 1;

	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return 0;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return 0;
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		return 0;
	}

	return 1;
}

int main(int argc, char **argv)
{
  std::string frame_id;
  int baud_;
  std::string imu_port;
  
    //初始化
    ros::init(argc, argv, "nav40_node");
    ros::NodeHandle nh;
    //参数初始化
    nh.param<std::string>("frame_id", frame_id, "imu");
    nh.param("baud", baud_, 460800);
    nh.param<std::string>("device_port", imu_port, std::string("/dev/ttyUSB"));

    int fd = open(imu_port.c_str(), O_RDWR);              //打开串口
    memset(queue_cycle.Recbuf, 0, MAXSIZE);            //初始化缓冲数组
    queue_cycle.tail = 0;                           //初始化缓冲数组指针
    queue_cycle.head = 0;
    if (fd == -1)
    {
       printf("open error.\n");
       return 0;
     }

     set_uart_baudrate(fd, baud_);                 //串口初始化
    //定义发布话题；
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu> ("imu_data",20);
    //设置循环频率
    ros::Rate loop_rate(200);
    cout << "open success"<< endl;

    struct tm* stm_ptr;
    time_t timer;
    time(&timer);
    stm_ptr=localtime(&timer);
    
    while(ros::ok())
    {
	//循环队列读取串口数据
         sensor_msgs::Imu msg;
         msg.header.stamp = ros::Time::now();
        len = read(fd, buf, 1);
	      memcpy(queue_cycle.Recbuf + queue_cycle.tail, buf, len);

	      queue_cycle.tail = (queue_cycle.tail + 1) % MAXSIZE; 
       //进入帧结构判断
     //循环队列大于等于2倍的长度，才进入帧结构的判断
        while ((queue_cycle.tail>queue_cycle.head && queue_cycle.tail - queue_cycle.head >= 2 * NAV_DL) || (queue_cycle.tail<queue_cycle.head && (MAXSIZE - queue_cycle.head + queue_cycle.tail) >= 2 * NAV_DL))
		{             
			if (queue_cycle.Recbuf[queue_cycle.head] == NAV_DH1)   //校验帧头
			{
				queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
				if (queue_cycle.Recbuf[queue_cycle.head] == NAV_DH2)   //校验帧头
				{      
					queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
					for (int k = 0; k <= 117; k++)
					{
					    checksum += queue_cycle.Recbuf[queue_cycle.head];
					    queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
					}
					checksum = checksum + 0xEB + 0x90;
					checkRes_L = checksum & 0x00ff;
					checkRes_H = (checksum >> 8) & 0x00ff;
					checksum = 0;       //必须清零
					//检验和
					if (queue_cycle.Recbuf[queue_cycle.head] == checkRes_L && queue_cycle.Recbuf[(queue_cycle.head + 1) % MAXSIZE] == checkRes_H)
					{   //校验和通过
					    for (int j = 121; j>=0; j--)
              {
                temp_buf[121-j]= queue_cycle.Recbuf[(queue_cycle.head + MAXSIZE - j+1) % MAXSIZE];
              }

              memcpy(&APM, temp_buf,122 );  // 将一帧完整的数据帧拷贝到结构体
              //printf("time_stamped:%d\r\n",APM.gps_ms);//打印数据量；
            /*
            printf("temperary frame:");
            for(int i=0;i<121;i++){
                printf("%x",temp_buf[i]);
              }
			       printf("\n");
              //在这里访问结构体成员即可
                
              printf("length:%d\r\n",APM.zhen_len);
            printf("apm_accel_x:%f\r\n",APM.accel_x);
             printf("apm_accel_y:%f\r\n",APM.accel_y);
              printf("apm_accel_z%f\r\n",APM. accel_z);
               printf("apm_accel_x:%f\r\n",APM.pitch_rate);
             printf("apm_accel_y:%f\r\n",APM.roll_rate);
              printf("apm_accel_z%f\r\n",APM. yaw_rate);*/
            
              // calculate measurement time

            
             /* if(APM.gps_status!=0&&APM.gps_status!=1)
              {
               //printf("gps time valid, gps status:%d\r\n",APM.gps_status);
                stm_ptr->tm_hour=APM.gps_hh;
                stm_ptr->tm_min  =APM.gps_mm;
                stm_ptr->tm_sec   =APM.gps_ss;
               
                uint32_t t = mktime(stm_ptr);   
                msg.header.stamp=ros::Time(t,APM.gps_ms*1000000);
              }*/
              //else
              {
               // printf("No gps time\n");
               //double t1 = ros::Time::now().toSec();
               //msg.header.stamp = ros::Time::now()
                 
              }
              //printf("IMU time_stamped:%f ms\r\n",t1*1000);//打印时间戳；
              
              //printf("System time_stamped:%d\r\n",APM.gps_ms);
              msg.header.frame_id = frame_id;
              
              /*tf::Quaternion q;
              q = tf::createQuaternionFromRPY(APM.roll,APM.pitch,APM.yaw);
              msg.orientation.x = q.x();
              msg.orientation.y = q.y();
              msg.orientation.z = q.z();
              msg.orientation.w = q.w();
              msg.linear_acceleration.x = APM.accel_x;
              msg.linear_acceleration.y = APM.accel_y;
              msg.linear_acceleration.z = APM.accel_z;
              msg.angular_velocity.x = APM.roll_rate;
              msg.angular_velocity.y = APM.pitch_rate;
              msg.angular_velocity.z = APM.yaw_rate;*/

              msg.angular_velocity.x = APM.roll_rate;
	            msg.angular_velocity.y = -APM.pitch_rate;
	            msg.angular_velocity.z = -APM.yaw_rate;
              Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(APM.roll, Eigen::Vector3f::UnitX()));
	            Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(-APM.pitch, Eigen::Vector3f::UnitY()));
            	Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(-APM.yaw, Eigen::Vector3f::UnitZ())); 
             	Eigen::Quaternionf rotation;
	            rotation=yawAngle * pitchAngle * rollAngle;
	// 东北地坐标系转成XYZ坐标系
            	Eigen::Vector3f accel(APM.accel_x, -APM.accel_y, -APM.accel_z);
	            accel = rotation.inverse() * accel;
	           msg.linear_acceleration.x = accel.x();
	           msg.linear_acceleration.y = accel.y();
	           msg.linear_acceleration.z = accel.z();
	           msg.orientation.x = rotation.x();
	           msg.orientation.y = rotation.y();
	           msg.orientation.z = rotation.z();
	           msg.orientation.w = rotation.w();


            pub_imu.publish(msg);
            ros::spinOnce();
             loop_rate.sleep();
					}
				}
			}
			else queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
		}
    
    }

    return 0;
}
