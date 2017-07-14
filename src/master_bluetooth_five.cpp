#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h> 
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>

class SerialPort
{
    public :
		// set neccessary params  
		bool Set_Parity(int fd,int databits,int stopbits,int parity);	
		// set the speed of the serial port
		void Set_Speed(int fd, int speed);
		//receive the cmd and deal it
		char* itoa(int num, char*str, int radix);
        //create two static variables   
		static int speed_arr[];	
		static int name_arr[];
		//some useful variables
		bool already_open_flag;
		int fd;
		char send_buffer[200];//连接设备时，每个步骤分配三个字节
		int count;
		bool blueteeth_connect_status; // true is connected, false is unconnected
};

SerialPort serialport;
fd_set rfds;

int SerialPort::speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int SerialPort::name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200, 300};

char* SerialPort::itoa(int num, char* str, int radix)
{
	char index[]="0123456789ABCDEF";
	unsigned unum;/*中间变量*/
	int i=0,j,k;
	/*确定unum的值*/
	if(radix==10 && num<0)/*十进制负数*/
	{
		unum=(unsigned)-num;
		str[i++]='-';
	}
	else 
		unum=(unsigned)num;/*其他情况*/
		
	do
	{
		str[i++]=index[unum%(unsigned)radix];
		unum/=radix;
	}while(unum);
	
	str[i]='\0';
	if(str[0]=='-')
		k=1;/*十进制负数*/
	else
		k=0;
	char temp;
	for(j=k;j<=(i-1)/2;j++)
	{
		temp=str[j];
		str[j]=str[i-1+k-j];
		str[i-1+k-j]=temp;
	}
	return str;
}

bool SerialPort::Set_Parity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;   
    if  ( tcgetattr( fd, &options) !=  0) 
    {   
        perror("SetupSerial failed!");       
        return false;    
    }  
    options.c_cflag &= ~CSIZE;   
    switch (databits)   
    {     
        case 7:       
            options.c_cflag |= CS7;   
            break;  
        case 8:       
            options.c_cflag |= CS8;  
            break;     
        default:      
            fprintf(stderr,"Unsupported data size\n"); 
            return false;    
    }  
    switch (parity)   
    {     
        case 'n':  
        case 'N':      
            options.c_cflag &= ~PARENB;   /* Clear parity enable */ 
            options.c_iflag &= ~INPCK;     /* Enable parity checking */   
            break;    
        case 'o':     
        case 'O':       
            options.c_cflag |= (PARODD | PARENB);   
            options.c_iflag |= INPCK;             /* Disnable parity checking */   
            break;    
        case 'e':    
        case 'E':     
            options.c_cflag |= PARENB;     /* Enable parity */      
            options.c_cflag &= ~PARODD;      
            options.c_iflag |= INPCK;       /* Disnable parity checking */  
            break;  
        case 'S':   
        case 's':  /*as no parity*/     
            options.c_cflag &= ~PARENB;  
            options.c_cflag &= ~CSTOPB;
            break;    
        default:     
            fprintf(stderr,"Unsupported parity\n");      
            return false;    
    }    
      
    switch (stopbits)  
    {     
        case 1:      
            options.c_cflag &= ~CSTOPB;    
            break;    
        case 2:      
            options.c_cflag |= CSTOPB;    
           break;  
        default:      
            fprintf(stderr,"Unsupported stop bits\n");    
            return false;   
    }   
    /* Set input parity option */   
    if (parity != 'n')     
        options.c_iflag |= INPCK;   

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &=~(ICANON | ECHO | ECHOE | ISIG);
 
    options.c_oflag &=~OPOST;
 
    options.c_iflag &=~ICRNL;
    options.c_iflag &=~INLCR;
	
    //tcflush(fd,TCIFLUSH);
    tcflush(fd,TCIOFLUSH);
    //options.c_cc[VTIME] = 150;   
    //options.c_cc[VMIN] = 0; /* Update the options and do it NOW */  
    /* 这样设置表示read函数只有在读取一个字节后才能返回
     * 否则一直等待, 上面注释里面的方式是正确的
     */
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;
    if (tcsetattr(fd,TCSANOW,&options) != 0)     
    {   
        perror("SetupSerial failed!\n");     
        return false;    
    }   
    return true;
}

void SerialPort::Set_Speed(int fd, int speed)
{  
  int   i;   
  int   status;   
  struct termios Opt;
  
  tcgetattr(fd, &Opt);   
  for ( i= 0;  i < sizeof(serialport.speed_arr) / sizeof(int);  i++) 
  {   
    if(speed == serialport.name_arr[i]) 
	{       
      tcflush(fd, TCIOFLUSH);       
      cfsetispeed(&Opt, serialport.speed_arr[i]);    
      cfsetospeed(&Opt, serialport.name_arr[i]);     
      status = tcsetattr(fd, TCSANOW, &Opt);    
      if  (status != 0) 
	  	{          
          perror("tcsetattr ttyUSB0");    
          return;       
        }      
      tcflush(fd, TCIOFLUSH);
	  printf(" set serial 115200...\n");
	  
	  return;
    }    } 
  
}

void MessageCallBack(const std_msgs::Int16& toggle_msg) // 5hz
{
	int ret = -1;
	struct timeval time;
	if (serialport.blueteeth_connect_status == true)
	{
		if (toggle_msg.data == 1 && serialport.already_open_flag == false /*&& serialport.blueteeth_connect_status == true*/) // 1: open the door 0: close the door
		{
			printf("in data = 1, should open the door!\n");
			std::cout<<std::endl;
			
			serialport.itoa(1, &serialport.send_buffer[0], 10);	
			serialport.send_buffer[1] = '\0';	
			serialport.count = 0;
			//send 1 to the blueteeth module
			//write(serialport.fd, &serialport.send_buffer[0], 1);
			ret = system("echo '1' >> /dev/blueteethserial0");
			//serialport.count++;
			serialport.already_open_flag = true;
		}
		else if (serialport.count < 5 )
		{
			serialport.count++;  
		}
		else if (serialport.count >= 5 )
		{
			serialport.count = 0;
			if (serialport.already_open_flag == true)
			{
			  serialport.itoa(0, &serialport.send_buffer[0], 10);	
			  serialport.send_buffer[1] = '\0';	
			  //send 0 flag to module
			  //write(serialport.fd, &serialport.send_buffer[0], 1);	
			  ret = system("echo '0' >> /dev/blueteethserial0");
			  serialport.already_open_flag = false;
			}
		}
	}
	/*
	//每次完成开门动作之后，检测蓝牙是否还处于连接状态
	usleep(10000);
	tcflush(serialport.fd, TCIOFLUSH);
        time.tv_sec = 0;
	time.tv_usec = 80000; //100ms, because the callback hz is 10
	ret = system("echo 'AT+GETSTAT\r' >> /dev/blueteethserial0");
	//实现串口异步I/O
	ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
	if (ret < 0)
	{
		perror("select");
		return;
	}
	else if ( ret == 0 )
	{
		std::cout<<"timeout!\n"<<std::endl;
		return;
	}
	else
	{
		printf("......in blueteeth query connect status......");
		std::cout<<std::endl;
		ret = read(serialport.fd, &serialport.send_buffer[171], 10);
		//print the string
		printf("ret = %d, %s\n", ret, &serialport.send_buffer[171]);
		std::cout<<std::endl;
		
		if ( strncmp(&serialport.send_buffer[171+ret-1], "1", 1) == 0)
		{
				printf("status: still connect!\n");
				std::cout<<std::endl;
				serialport.blueteeth_connect_status = true;
		}
		else if ( strncmp(&serialport.send_buffer[171+ret-1], "0", 1) == 0)
		{
			        pthread_mutex_lock(&mutex);
				//do something or not
				serialport.blueteeth_connect_status = false;
				printf("has lost connect to the slave device!\n");
				std::cout<<std::endl;
				pthread_cond_signal(&cond);
				pthread_mutex_unlock(&mutex);
		}
	}
	*/
}

/*
void *thread(void *arg)  
{  
	int ret;
	struct timeval time;
	printf("in new thread!\n");
	//serialport.blueteeth_connect_status = false;
	while(1)
	{
		//发送命令确定模块是否已经连接上，返回１表示连接上了，返回０表示没有连接上
		
		if (serialport.blueteeth_connect_status == false)
		{
			printf("......in blueteeth connect start......");
			std::cout<<std::endl;
			bool already_set_unconnect_ok = false;
			bool already_set_name_ok = false;
			bool already_set_master_ok = false;
			//只要有数据返回，就说明成功
			bool already_set_searchdev_ok = false;
			//等待次数
			int current_count = 0;
			//确保断开，发断开指令
			char unconnect_cmd[] = "AT+DISC";
			unconnect_cmd[strlen(unconnect_cmd)] = '\n';
			//unconnect_cmd[strlen(unconnect_cmd)] = '\r';
			//unconnect_cmd[strlen(unconnect_cmd)+1] = '\n';
			
			strcpy(&serialport.send_buffer[2], unconnect_cmd);
			
			//设置超时为10s, 如果10之内没有数据返回，就认为没有数据可读
			time.tv_sec = 10;
			time.tv_usec = 0;
			ret = write(serialport.fd, &serialport.send_buffer[2], 2);
			printf("disc, write ret = %d\n", ret);
			std::cout<<std::endl;
			//实现串口异步I/O
			ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
			//根据返回值来判断串口是否已经有了数据
			if (ret < 0)
			{
				perror("select");
				continue;
			}
			else if ( ret == 0 )
			{
				std::cout<<"timeout!\n"<<std::endl;
				continue;
			}
			else
			{
				printf("......in blueteeth set disc......");
				std::cout<<std::endl;
				ret = read(serialport.fd, &serialport.send_buffer[11], 2);//只是返回"OK"
				//print the string
				printf("ret = %d, %s\n", ret, &serialport.send_buffer[11]);
				std::cout<<std::endl;
				
				if ( strncmp(&serialport.send_buffer[11], "OK", 2) == 0)
				{
						printf("set disc successfully!\n");
						std::cout<<std::endl;
				}
			}
			
			while (read(serialport.fd, &serialport.send_buffer[11], 1) == 1 && current_count < 5)
			{
				printf("in blueteeth set disc......");
				std::cout<<std::endl;
				if (strcmp(&serialport.send_buffer[11], "O") == 0 && already_set_unconnect_ok == false)
				{
					continue;
				}
				else if (strcmp(&serialport.send_buffer[11], "K") == 0 && already_set_unconnect_ok == true)
				{
					printf("succeed in set disc!\n");
					std::cout<<std::endl;
					already_set_unconnect_ok = true;
					current_count = 0;
					break;
				}
				else
				{
					already_set_unconnect_ok = false;
					//如果执行到了这里，就加1
				    current_count++;
				    sleep(1);
				}
					
			}
			
			
			//设置模块名字
			if (already_set_unconnect_ok == true)
			{
				char setname_cmd[]= "AT+NAMEIIM_MASTER";
				strcpy(&serialport.send_buffer[1], setname_cmd);
				write(serialport.fd, &serialport.send_buffer[1], strlen(setname_cmd));
				current_count = 0;
				//延迟一会再读
			    sleep(1);
				while (read(serialport.fd, &serialport.send_buffer[11], 1) == 1 && current_count < 5)
				{
					if (strcmp(&serialport.send_buffer[11], "O") == 0)
					{
						continue;
					}
					else if (strcmp(&serialport.send_buffer[11], "K") == 0)
					{
						printf("succeed in set name!\n");
						already_set_name_ok = true;
						current_count = 0;
						break;
					}
					else
					{
						already_set_name_ok = false;
						//如果执行到了这里，就加1
						current_count++;
						sleep(1);
					}	
				}
			}
			//设置主机模式
			if (already_set_name_ok == true)
			{
				char setname_cmd[]= "AT+ROLE1";
				strcpy(&serialport.send_buffer[1], setname_cmd);
				write(serialport.fd, &serialport.send_buffer[1], strlen(setname_cmd));
				current_count = 0;
				//延迟一会再读
				sleep(1);
				while (read(serialport.fd, &serialport.send_buffer[11], 1) == 1 && current_count < 5)
				{
					if (strcmp(&serialport.send_buffer[11], "O") == 0)
					{
						continue;
					}
					else if (strcmp(&serialport.send_buffer[11], "K") == 0)
					{
						printf("succeed in set role master!\n");
						already_set_master_ok = true;
						current_count = 0;
						break;
					}
					else
					{
						already_set_master_ok = false;
						//如果执行到了这里，就加1
						current_count++;
						sleep(1);
					}		
				}
			}
			//查询
			if (already_set_master_ok == true)
			{
				char setname_cmd[]= "AT+INQ";
				strcpy(&serialport.send_buffer[1], setname_cmd);
				write(serialport.fd, &serialport.send_buffer[1], strlen(setname_cmd));
				current_count = 0;
				//延迟一会再读
				sleep(1);
				
				while(current_count < 5)
				{
					if (read(serialport.fd, &serialport.send_buffer[11], 1) == 1)
					{
						printf("succeed in set inq!\n");
						already_set_searchdev_ok = true;
						current_count = 0;
						break;
					}
					else if (current_count < 10)
					{
						already_set_searchdev_ok = false;
						current_count++;
						sleep(1);
					}	
				}
				
			}
			//连接设备
			int count = 0;
			if (already_set_searchdev_ok == true && count < 3)
			{
				char setname_cmd[]= "AT+CONN1";
				strcpy(&serialport.send_buffer[1], setname_cmd);
				write(serialport.fd, &serialport.send_buffer[1], strlen(setname_cmd));
				current_count = 0;
				//延迟一会再读
				sleep(1);
				
				while (read(serialport.fd, &serialport.send_buffer[11], 1) == 1 && current_count < 5)
				{
					if (strcmp(&serialport.send_buffer[11], "+") == 0)
					{
						continue;
					}
					else if (strcmp(&serialport.send_buffer[11], "C") == 0)
					{
						printf("succeed in set connet!\n");
						serialport.blueteeth_connect_status = true;
						current_count = 0;
						break;
					}
					else
					{
						serialport.blueteeth_connect_status = false;
						current_count++;
						sleep(1);
					}
						
				}
				if (serialport.blueteeth_connect_status == false)
				{
					count++;
				}
			}
		}		
	}
	printf("outside while recycle!\n");
	//关闭文件
	close(serialport.fd);
		     
    return ((void *)0);  
}
*/  

int main(int argc, char **argv)
{
  
	ros::init(argc, argv, "master_bluetooth_five");
	ros::NodeHandle n;
	
	serialport.already_open_flag = false;
	serialport.fd = -1;
	serialport.send_buffer[100] = {0};
	serialport.send_buffer[0] = -1;
	serialport.blueteeth_connect_status = true;
	
	//serialport initialize
	//need have a special device name according the ID
	serialport.fd = open("/dev/blueteethserial0", O_RDWR);  
	if(serialport.fd == -1)  
	{  
		perror("set serialport error\n");  
	}  
	else  
	{  
		printf("%s",ttyname(serialport.fd));  
		printf(" succesfully\n");  
	}  
	//set the parameters of serialport  
	serialport.Set_Speed(serialport.fd, 115200);
	if (serialport.Set_Parity(serialport.fd, 8, 1, 'S') == true)  
	{  
		//printf("Set Parity Error\n");  
		//exit (0);
		printf("set data format succesfully!\n");  
	}
	else
	{
		printf("set data format failed!\n");
		exit(-1);
	}
	
	/*
	//将文件描述符加入读描述符集合
	FD_ZERO(&rfds);
	FD_SET(serialport.fd, &rfds);
	// create a thread to check whether the blueteeth is connected ok.
	printf("create new thread!\n");  
	pthread_t ntid;  
    if( pthread_create(&ntid, NULL, thread, NULL) != 0)  
    {  
        printf("can't create thread for deal with the blueteeth !\n");  
        return -1;  
    }
    */
    
    //等待蓝牙连接成功
    //this is a temp program.
    //sleep(5);
    //callback function
	ros::Subscriber sub = n.subscribe("/door_enabler", 5, MessageCallBack);
	
	ros::Rate loop_rate(10);
	
	while (ros::ok())
	{	
		ros::spinOnce();

		loop_rate.sleep();
	}
	close(serialport.fd);

	return 0;
}
