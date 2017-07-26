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
		//线程休眠唤醒
		
};

SerialPort serialport;
fd_set rfds;
static pthread_cond_t cond;
static pthread_mutex_t mutex;
unsigned short query_count = 0;

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
    options.c_cc[VTIME] = 100;
    options.c_cc[VMIN] = 30;
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
          perror("tcsetattr error");    
          return;       
        }      
      tcflush(fd, TCIOFLUSH);
	  printf(" set serial 115200...\n");
	  
	  return;
    }    
  } 
  
}

void MessageCallBack(const std_msgs::Int16& toggle_msg) // 5hz
{
	int ret = -1;
	struct timeval time;
	if (serialport.blueteeth_connect_status == true)
	{
		if (toggle_msg.data == 1 && serialport.already_open_flag == false /*&& serialport.blueteeth_connect_status == true*/) // 1: open the door 0: close the door
		{		
			//serialport.itoa(1, &serialport.send_buffer[0], 10);	
			//serialport.send_buffer[1] = '\0';	
			serialport.count = 0;
			//send 1 to the blueteeth module
			ret = write(serialport.fd, "1\r", 2);
			//ret = system("echo '1\r' >> /dev/blueteethserial0");
			std::cout<<"data == 1, should open the door!\n"<<std::endl;
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
			  //serialport.itoa(0, &serialport.send_buffer[0], 10);	
			  //serialport.send_buffer[1] = '\0';	
			  //send 0 flag to module
			  ret = write(serialport.fd, "0\r", 2);	
			  //ret = system("echo '0\r' >> /dev/blueteethserial0");
			  serialport.already_open_flag = false;
			}
		}

		//每次完成开门动作之后，检测蓝牙是否还处于连接状态
	if (serialport.count == 0)
	{
		usleep(10000);
		tcflush(serialport.fd, TCIOFLUSH);
	    time.tv_sec = 0;
		time.tv_usec = 80000; //100ms, because the callback hz is 10
		//ret = system("echo 'AT+STATE?\r' >> /dev/blueteethserial0");
		/*实现串口异步I/O*/
		//ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
		ret = write(serialport.fd, "AT+STATE?\r", 10);
		if (ret < 0)
		{
			perror("select");
			return;
		}
		else if ( ret == 0 )
		{
			std::cout<<"in callback, getstat timeout!\n"<<std::endl;
			return;
		}
		else
		{
			//printf("......in blueteeth query connect status......");
			//std::cout<<std::endl;
			ret = read(serialport.fd, &serialport.send_buffer[171], 13);
			//print the string
			//printf("ret = %d, %s\n", ret, &serialport.send_buffer[171]);
			//std::cout<<std::endl;
			
			if ( strncmp(&serialport.send_buffer[171], "CONNECTED", 9) == 0)
			{
					//printf("in callback, status: still connect!\n");
					//std::cout<<std::endl;
					serialport.blueteeth_connect_status = true;
			}
			else if ( strncmp(&serialport.send_buffer[171], "DISCONNECTED", 12) == 0)
			{
				    pthread_mutex_lock(&mutex);
					//do something or not
					serialport.blueteeth_connect_status = false;
					std::cout<<"has lost connect to the slave device!\n"<<std::endl;
					pthread_cond_signal(&cond);
					pthread_mutex_unlock(&mutex);
			}
			//clear the io buffer
			tcflush(serialport.fd, TCIOFLUSH);
		}
	}
     }
		
}

void *thread(void *arg)  
{  
	int ret;
	struct timeval time;
	printf("in new thread!\n");
	//serialport.blueteeth_connect_status = false;
	while(1)
	{
		if (serialport.blueteeth_connect_status == false)
		{
			printf("......in blueteeth connect start......");
			std::cout<<std::endl;
			usleep(500000);
			//query the status
			tcflush(serialport.fd, TCIOFLUSH);
			time.tv_sec = 2;
			time.tv_usec = 0; 
			//ret = system("echo 'AT+STATE?\r' >> /dev/blueteethserial0");
			ret = write(serialport.fd, "AT+STATE?\r", 10);
			//get the info from the module
			ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
			if (ret < 0)
			{
				perror("select");
				continue;
			}
			else if( ret == 0 )
			{
				std::cout<<"getstat timeout!\n"<<std::endl;
				continue;
			}
			else
			{
				printf("......in blueteeth query connect status......");
				std::cout<<std::endl;
				ret = read(serialport.fd, &serialport.send_buffer[151], 10);
				//print the string
				printf("ret = %d, %s\n", ret, &serialport.send_buffer[151]);
				std::cout<<std::endl;
				
				if ( strncmp(&serialport.send_buffer[151], "CONNECTED", 9) == 0)
				{
						//printf("status: still connect!\n");
						//std::cout<<std::endl;
						serialport.blueteeth_connect_status = true;
						goto LABEL_THREAD_SLEEP;
						
				}
				else if ( strncmp(&serialport.send_buffer[151], "DISCONNECT", 10) == 0)
				{
						std::cout<<"status: unconnected!\n"<<std::endl;
						serialport.blueteeth_connect_status = false;
				}
				//clear uart data.
            	tcflush(serialport.fd, TCIOFLUSH);
				
			}
			
			//get master role
			usleep(10000);

			//clear uart data.
            tcflush(serialport.fd, TCIOFLUSH);
			time.tv_sec = 1;
			time.tv_usec = 0;
			ret = system("echo 'AT+ROLE?\r' >> /dev/blueteethserial0");
			/*实现串口异步I/O*/
			ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
			//根据返回值来判断串口是否已经有了数据
			if (ret < 0)
			{
				perror("select");
				continue;
			}
			else if ( ret == 0 )
			{
				std::cout<<"get role timeout!\n"<<std::endl;
				continue;
			}
			else
			{
				printf("......in blueteeth get masterrole......");
				std::cout<<std::endl;
				ret = read(serialport.fd, &serialport.send_buffer[161], 1);//只是返回"OK"
				//print the string
				printf("ret = %d, %s\n", ret, &serialport.send_buffer[161]);
				printf("return string is  %s", &serialport.send_buffer[161]);
				std::cout<<std::endl;
				
				if ( strncmp(&serialport.send_buffer[161], "M", 1) == 0)
				{
						std::cout<<"it's already masterrole!\n"<<std::endl;
						goto LABEL_INQ;
				}
				//clear uart data.
            	tcflush(serialport.fd, TCIOFLUSH);
			}

			
			//clear uart data.
            tcflush(serialport.fd, TCIOFLUSH);
			//set master role
			time.tv_sec = 5;
			time.tv_usec = 0;
			ret = system("echo 'AT+ROLE=M\r' >> /dev/blueteethserial0");
			/*实现串口异步I/O*/
			ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
			//根据返回值来判断串口是否已经有了数据
			if (ret < 0)
			{
				perror("select");
				continue;
			}
			else if ( ret == 0 )
			{
				std::cout<<"set role timeout!\n"<<std::endl;
				continue;
			}
			else
			{
				printf("......in blueteeth set masterrole......");
				std::cout<<std::endl;
				ret = read(serialport.fd, &serialport.send_buffer[61], 3);//只是返回"OK"
				//print the string
				printf("ret = %d, %s", ret, &serialport.send_buffer[61]);
				printf("return string is  %s", &serialport.send_buffer[61]);
				std::cout<<std::endl;
				
				if ( strncmp(&serialport.send_buffer[61], "OK", 2) == 0)
				{
						std::cout<<"set masterrole successfully!\n"<<std::endl;
				}
				else
				{
						continue;
				}
				//clear uart data.
            	tcflush(serialport.fd, TCIOFLUSH);
			}

			LABEL_CLR:
			//clear uart data.
            tcflush(serialport.fd, TCIOFLUSH);

			//at first, clear bond status
			time.tv_sec = 5;
			time.tv_usec = 0;
			ret = system("echo 'AT+CLRBOND\r' >> /dev/blueteethserial0");
			//query the return string
			ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
			//根据返回值来判断串口是否已经有了数据
			if (ret < 0)
			{
				perror("select");
				continue;
			}
			else if ( ret == 0 )
			{
				std::cout<<"unbond slave bluetooth timeout!\n"<<std::endl;
				continue;
			}
			else
			{        
				printf("......in blueteeth unbond slave device......");
				std::cout<<std::endl;
				ret = read(serialport.fd, &serialport.send_buffer[71], 3);
				//print the string
				//printf("ret = %d, %s\n", ret, &serialport.send_buffer[91]);
				//std::cout<<std::endl;
				
				if( strncmp(&serialport.send_buffer[71], "OK", 2) == 0)
				{
						std::cout<<"unbond slave bluetooth successfully!\n"<<std::endl;
				}
				else
				{
						continue;
				}
			}		

			LABEL_INQ:
			//clear uart data.
            tcflush(serialport.fd, TCIOFLUSH);

			//query slave device
			time.tv_sec = 10;
			time.tv_usec = 0;
			ret = system("echo 'AT+SCAN:RSSI=-99\r' >> /dev/blueteethserial0");
			//query the return string
			ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
			//根据返回值来判断串口是否已经有了数据
			if (ret < 0)
			{
				perror("select");
				continue;
			}
			else if ( ret == 0 )
			{
				std::cout<<"query slave blue timeout!\n"<<std::endl;
				continue;
			}
			else
			{        
				printf("......in blueteeth query slave device......");
				std::cout<<std::endl;
				ret = read(serialport.fd, &serialport.send_buffer[91], 30);
				//print the string
				//printf("ret = %d, %s\n", ret, &serialport.send_buffer[91]);
				//std::cout<<std::endl;
				
				if ( ret > 21)
				{
				   
			        printf("%s", &serialport.send_buffer[91]);
			        std::cout<<std::endl;
					std::cout<<"query slave device successfully, has a slave device!\n"<<std::endl;
				}
				else
				{
					std::cout<<"query slave blue failed!\n"<<std::endl;
					query_count++;
					std::cout<<"query_count = "<<query_count<<std::endl;
					if(query_count >= 5)
					{
						query_count = 0;
						goto LABEL_CLR;
					}
					continue;
				}
					
			}
			
			//clear uart data.
            tcflush(serialport.fd, TCIOFLUSH);

			//connect slave device
			time.tv_sec = 15;
			time.tv_usec = 0;
			char dest[24] = "AT+CONNECT=";
			char src[12];
			strncpy(src, &serialport.send_buffer[91+8], 12);
			strcat(dest, src);
			dest[23] = '\r';
			printf("dest = %s\n", dest);
			std::cout<<std::endl;
			
			//ret = system("echo 'AT+CONN1\r' >> /dev/blueteethserial0");
			//must write the info to uart
			ret = write(serialport.fd, (unsigned char*)&dest, strlen(dest));
			//detect the return info
			ret = select(serialport.fd+1, &rfds, NULL, NULL, &time);
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
				//clear the query count;
			    query_count = 0;
				std::cout<<"......in blueteeth connect slave device......"<<std::endl;
				ret = read(serialport.fd, &serialport.send_buffer[131], 3);
				//print the string
				printf("ret = %d, %s\n", ret, &serialport.send_buffer[131]);
				printf("return string is  %s", &serialport.send_buffer[131]);
				std::cout<<std::endl;
				
				if ( strncmp(&serialport.send_buffer[131], "OK", 2) == 0)
				{
						serialport.blueteeth_connect_status = true;
						std::cout<<"connect to the slave device successfully!\n"<<std::endl;
				}
				else
				{
						continue;
				}
				//clear uart data.
            	tcflush(serialport.fd, TCIOFLUSH);
					
			}
		}
		
		LABEL_THREAD_SLEEP:
		//进入休眠
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond, &mutex);
		//do something or not
		std::cout<<"start reconnect to the slave device!\n"<<std::endl;
		pthread_mutex_unlock(&mutex);
		goto LABEL_INQ;
				
	}
	//clear uart data.
    tcflush(serialport.fd, TCIOFLUSH);
	std::cout<<"wrong! outside while recycle!\n"<<std::endl;	     

	return ((void *)0);
	
}  

int main(int argc, char **argv)
{
  
	ros::init(argc, argv, "master_bluetooth_five");
	ros::NodeHandle n;
	//线程相关参数初始化
	pthread_mutex_init(&mutex, NULL);
	pthread_cond_init(&cond, NULL);
	
	serialport.already_open_flag = false;
	serialport.fd = -1;
	serialport.send_buffer[100] = {0};
	serialport.send_buffer[0] = -1;
	serialport.blueteeth_connect_status = false;
	
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
	if (serialport.Set_Parity(serialport.fd, 8, 1, 'N') == true)  
	{  
		printf("set data format succesfully!\n");  
	}
	else
	{
		printf("set data format failed!\n");
        exit(-1);
	}
	//wait for bluetooth's pair
	sleep(2);
	/*将文件描述符加入读描述符集合*/
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

