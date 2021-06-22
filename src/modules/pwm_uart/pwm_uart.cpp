
#include "pwm_uart.h"
 
 

//设置串口波特率函数
int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;
    //选择波特率
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }
    //实例化termios结构体，命名为uart_config
    struct termios uart_config;
 
    int termios_state;
    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);// 获取终端参数
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }
 
    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }
    // 设置与终端相关的参数，TCSANOW 立即改变参数
    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }
 
    return true;
}
 
//串口初始化函数，传入形参为"/dev/ttyS2"
int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY| O_NONBLOCK);//调用Nuttx系统的open函数，形参为串口文件配置模式，可读写，
    // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}
//进程提示函数，用来提示可输入的操作
static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }
 
    fprintf(stderr, "usage: serv_sys_uart {start|stop|status} [param]\n\n");
    exit(1);
}
//主函数入口
int pwm_uart_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
    }
    //输入为start
    if (!strcmp(argv[1], "start")) {
        if (thread_running) {//进程在运行
            warnx("already running\n");//打印提示已经在运行
            return 0;//跳出代码
        }
        //如果是第一次运行
        thread_should_exit = false;
        //建立名为serv_sys_uart进程SCHED_PRIORITY_MAX - 55,
        daemon_task = px4_task_spawn_cmd("pwm_uart",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         pwm_uart_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);//正常命令形式为serv_sys_uart start /dev/ttyS2
        return 0;//跳出代码
    }
    //如果是stop
    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;//进程标志变量置true
        return 0;
    }
    //如果是status
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("running");
            return 0;
 
        } else {
            warnx("stopped");
            return 1;
        }
 
        return 0;
    }
    //若果是其他，则打印不支持该类型
    usage("unrecognized command");
    return 1;
}
 
int pwm_uart_thread_main(int argc, char *argv[])
{
    //正常命令形式为serv_sys_uart start /dev/ttyS2 
    const char *uart_name = "/dev/ttyS6";
    warnx("opening port %s", uart_name);
    
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
 
    /*配置串口*/
    int serv_uart = uart_init(uart_name);//初始化串口路径-
    if(false == serv_uart)return -1;
    if(false == set_uart_baudrate(serv_uart,115200)){//设置串口波特率为115200
        printf("set_uart_baudrate is failed\n");
        return -1;
    }
    printf("uart init is successful\n");
 
    /*进程标志变量*/
    thread_running = true;//进程正在运行


	uint16_t duty_precent[8];
	uint8_t duty_temp = 1;
	char speedControlBuffer[31];
	char arm[]="arm\r\n";
	char start[]="start\r\n";
	bool zero_check = false;
	//uint16_t user_

	//_vehicle_status_sub.update(&_vehicle_status);

	
 //if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED){
	/*for(int count_arm=0; count_arm<4; count_arm++)
		{
		write(serv_uart, &arm[count_arm], 1);
		}*/
	/*usleep(1000);
	for(int count_start=0; count_start<6; count_start++)
		{
		write(serv_uart, &start[count_start], 1);
		}*/
 //	}

 
    //定义串口事件阻塞结构体及变量
    
	int rc_channels_sub_fd = orb_subscribe(ORB_ID(rc_channels));
	orb_set_interval(rc_channels_sub_fd, 50);

	/*int actuator_outputs_poll_sub_fd = orb_subscribe(ORB_ID(actuator_outputs));
	orb_set_interval(actuator_outputs_poll_sub_fd, 50);*/

	//int sensor_accel_sub_fd = orb_subscribe(ORB_ID(sensor_accel));

	/*px4_pollfd_struct_t fds[] = {
	  { .fd = sensor_accel_sub_fd,	 .events = POLLIN },
	};*/



 
    while(!thread_should_exit)
		{
		//int poll_ret = px4_poll(fds, 1, 150);

		/*if (poll_ret < 0) {
			// this is seriously bad - should be an emergency 
			if (error_counter < 10 || error_counter % 50 == 0) {
				// use a counter to prevent flooding (and slowing us down)
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}
		} */
		//else {

		//	if (fds[0].revents & POLLIN) {
		//		_actuator_controls_0_sub.update(&actuators_0_in);
		        _actuator_outputs_sub.update(&_actuator_outputs);
			    _vehicle_status_sub.update(&_vehicle_status);
	    //        orb_copy(ORB_ID(rc_channels), rc_channels_sub_fd, &rc_channels_test);

		/*PX4_INFO("channel 1:%4.1f, channel 2:%4.1f, channel 3:%4.1f, channel 4:%4.1f",
                (double)_actuator_outputs.output[0],
                (double)_actuator_outputs.output[1],
                (double)_actuator_outputs.output[2],
                (double)_actuator_outputs.output[3]
		        );*/

  //for(int loop_count = 0;loop_count < 10;loop_count ++){
		for(int i=0; i<8; i++)
			{
			duty_precent[i] = (int)((USER_PWM_MAIN_MAX - USER_PWM_MAIN_MIN)*((int)_actuator_outputs.output[i]- PX4_PWM_MAIN_MIN)/(PX4_PWM_MAIN_MAX - PX4_PWM_MAIN_MIN));
			duty_temp = duty_temp & duty_precent[i];
			}

		if(duty_temp == 0) zero_check = true;
		if(zero_check && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED){
		for(int count_arm=0; count_arm<4; count_arm++)
		{
		write(serv_uart, &arm[count_arm], 1);
		}
		usleep(500);
		for(int count_start_loop=0; count_start_loop<6; count_start_loop++)
		{
		write(serv_uart, &start[count_start_loop], 1);
		}
		usleep(500);

		 //write(serv_uart, &arm, sizeof(arm));
		 //write(serv_uart, &start, sizeof(start));
		 //PX4_INFO("sending start");
		}
		//else PX4_INFO("dont send start");

		
		/*sprintf(speedControlBuffer, "gzs%03d%03d%03d%03d%03d%03d%03d%03d",
		duty_precent[0],
		duty_precent[1], 
		duty_precent[2],  
		duty_precent[3], 
		duty_precent[4], 
		duty_precent[5], 
		duty_precent[6], 
		duty_precent[7]);*/

		/** 通信指令数据转换 **/
		speedControlBuffer[0] = 'g';
		speedControlBuffer[1] = 'z';
		speedControlBuffer[2] = 's';
		for(uint8_t ESC_ID = 0; ESC_ID < 8; ESC_ID++){
			speedControlBuffer[ESC_ID * 3 + 3] = duty_precent[ESC_ID] / 100 % 10 + '0';
			speedControlBuffer[ESC_ID * 3 + 4] = duty_precent[ESC_ID] / 10 % 10 + '0';
			speedControlBuffer[ESC_ID * 3 + 5] = duty_precent[ESC_ID] / 1 % 10 + '0';
			}
		/** 通信指令数据转换 **/

		/*PX4_INFO("channel 1:%d, channel 2:%d, channel 3:%d, channel 4:%d",
                duty_precent[0],
                duty_precent[1],
                duty_precent[2],
                duty_precent[3]
		        );*/
		
        uint8_t LRC = 0;
        for(uint8_t i = 0; i < 27; i++) LRC += speedControlBuffer[i];
        speedControlBuffer[27] = LRC / 16 + '0';
        if(speedControlBuffer[27] > '9') speedControlBuffer[27] = (speedControlBuffer[27] - '9' + 'a' - 1);
        speedControlBuffer[28] = LRC % 16 + '0';
        if(speedControlBuffer[28] > '9') speedControlBuffer[28] = (speedControlBuffer[28] - '9' + 'a' - 1);
        speedControlBuffer[29] = '\r';
        speedControlBuffer[30] = '\n';
	    /*PX4_INFO("%c%c",
                speedControlBuffer[27],
                speedControlBuffer[28]
		        );*/

		for(int count_str=0; count_str<31; count_str++)
			{
            write(serv_uart, &speedControlBuffer[count_str], 1);
			}
		

		
		
		usleep(2000);
   // }
		
//	}
    }    
//    }
    //如果标志位置flase应该退出进程
    warnx("exiting");
    thread_running = false;
    close(serv_uart);
 
    fflush(stdout);
	return 0;

}




