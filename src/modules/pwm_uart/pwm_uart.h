#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_accel.h>


#include <px4_platform_common/tasks.h>
#include <poll.h>
#include <px4_platform_common/posix.h>
//外部声明serv_sys_uart_main主函数
extern "C" __EXPORT int pwm_uart_main(int argc, char *argv[]);

/*
   PX4 init_d rc.mc_default文件机架设置PWM范围
   PX4_PWM_MAIN_MAX   解锁状态最大值
   PX4_PWM_MAIN_MIN   解锁状态最小值
*/
#define PX4_PWM_MAIN_MAX 1950    
#define PX4_PWM_MAIN_MIN 900


/*
   用户设置PWM范围
   PWM_MAIN_MAX   解锁状态最大值
   PWM_MAIN_MIN   解锁状态最小值
*/
#define USER_PWM_MAIN_MAX 900    
#define USER_PWM_MAIN_MIN 0

//#define GPIO_MTRI3_DRDY_OFF	(GPIO_OUTPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTC|GPIO_PIN14)  //mjx  SPI up_to_down 2th node

//#define MTI3_DRDY		px4_arch_gpioread(GPIO_MTRI3_DRDY_OFF)   //mjx




static bool thread_should_exit = false; /*Ddemon exit flag*///定义查看进程存在标志变量
static bool thread_running = false;  /*Daemon status flag*///定义查看进程运行标志变量
static int daemon_task;//定义进程变量


//定义线程主函数
int pwm_uart_thread_main(int argc, char *argv[]);
//静态声明函数
static int uart_init(const char * uart_name);//串口初始化函数，形参为串口路径
static int set_uart_baudrate(const int fd, unsigned int baud);//设置串口波特率函数
static void usage(const char *reason);//进程提示函数

struct actuator_controls_s actuators_0_in;
uORB::Subscription _actuator_controls_0_sub{ORB_ID(actuator_controls_0)};	//控制组0输出量

struct actuator_outputs_s _actuator_outputs;
struct actuator_outputs_s _actuator_outputs_poll;
uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};	//PWM输出量

struct rc_channels_s rc_channels_test;
//uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};	//订阅输入通道信息

struct vehicle_status_s _vehicle_status;
uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};	//无人机状态量

struct sensor_accel_s sensor_accel_test;





