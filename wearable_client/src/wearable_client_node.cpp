#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <sys/socket.h>
#include <sys/time.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>

sensor_msgs::Imu wearable_dataFirst;
sensor_msgs::Imu wearable_dataSecond;

ros::NodeHandle* nh_pointer;

void doprocessing(int sock);

void clear_Imu(){
	wearable_dataFirst.angular_velocity.x = NAN;
	wearable_dataFirst.angular_velocity.y = NAN;
	wearable_dataFirst.angular_velocity.z = NAN;
	
	wearable_dataFirst.linear_acceleration.x = NAN;
	wearable_dataFirst.linear_acceleration.y = NAN;
	wearable_dataFirst.linear_acceleration.z = NAN;
	
	wearable_dataSecond.angular_velocity.x = NAN;
	wearable_dataSecond.angular_velocity.y = NAN;
	wearable_dataSecond.angular_velocity.z = NAN;
	
	wearable_dataSecond.linear_acceleration.x = NAN;
	wearable_dataSecond.linear_acceleration.y = NAN;
	wearable_dataSecond.linear_acceleration.z = NAN;
		
}
		
float acc_x, acc_y, acc_z, vel_x, vel_y, vel_z = 0;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "wearable_client_node");
	
	
	
	//nh_pointer = &nh;
	
	
	
	
	int sockfd, newsockfd, portno;
	unsigned int clilen;
	char buffer[256];
	struct sockaddr_in serv_addr, cli_addr;
	int n, pid;
	int option = 1;
	
	/* First call to socket() function */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	setsockopt(sockfd,SOL_SOCKET,(SO_REUSEPORT | SO_REUSEADDR),(char*)&option,sizeof(option));
	if (sockfd < 0) {
		perror("ERROR opening socket");
		exit(1);
	}

	/* Initialize socket structure */
	bzero((char *) &serv_addr, sizeof(serv_addr));
	portno = 8888;

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);

	/* Now bind the host address using bind() call.*/
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		perror("ERROR on binding");
		exit(1);
	}

	/* Now start listening for the clients, here
	 * process will go in sleep mode and will wait
	 * for the incoming connection
	 */

	listen(sockfd, 5);
	clilen = sizeof(cli_addr);

	//clear_Imu();
	
	while(true){
				std::cout << "entering"<<std::endl;

		newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
		std::cout << "entering"<<std::endl;

		if (newsockfd < 0) {
			perror("ERROR on accept");
			exit(1);
		}

		/* Create child process */
		pid = fork();

		if (pid < 0) {
			perror("ERROR on fork");
			exit(1);
		}
				
		if (pid == 0) {
			/* This is the client process */
			close(sockfd);
			doprocessing(newsockfd);
			exit(0);
		} else {
			close(newsockfd);
		}
		
			
		
		//ros::Duration(0.1).sleep();
	} /* end of while */
	
	return 0;
}


void doprocessing(int sock) {
	int n;
	char buffer[256];
	
	float stuff;
	char letter;
	bzero(buffer, 256);
	ros::NodeHandle nh;
	ros::Duration(1).sleep();
	ros::Publisher wearableSensorFirst_pub = nh.advertise<sensor_msgs::Imu>("/wearS_1", 1);
	ros::Publisher wearableSensorSecond_pub = nh.advertise<sensor_msgs::Imu>("/wearS_2", 1);
	ros::Duration(1).sleep();
	
	while (true) {
		n = read(sock, buffer, 255);
	
		if (n < 0) {
			std::cout << "ERROR reading from socket" << std::endl;
			perror("ERROR reading from socket");
			exit(1);
		}
		
		
		std::string str = std::string(buffer);
		
		for (int i=0; i<str.length(); i++)
		{
    		if (str[i] == ';')
        	str[i] = ' ';
		}

		std::stringstream ss(str);		
		std::string var;
		while(ss>>var){
			
			if(*var.c_str() == 'f'){
				ss >> var;
				if(*var.c_str() == 'a'){
					ss>> stuff;
					ss>> acc_x >> acc_y >> acc_z;
					wearable_dataFirst.linear_acceleration.x = acc_x;
					wearable_dataFirst.linear_acceleration.y = acc_y;
					wearable_dataFirst.linear_acceleration.z = acc_z;
				}else if(*var.c_str() == 'y'){
					ss>> stuff >> vel_x >> vel_y >> vel_z;
					wearable_dataFirst.angular_velocity.x = vel_x;
					wearable_dataFirst.angular_velocity.y = vel_y;
					wearable_dataFirst.angular_velocity.z = vel_z;
				}
				
				

				
			}else if(*var.c_str() == 's'){
				ss >> var;
				if(*var.c_str() == 'a'){
					ss>> stuff;
					ss>> acc_x >> acc_y >> acc_z;
					wearable_dataSecond.linear_acceleration.x = acc_x;
					wearable_dataSecond.linear_acceleration.y = acc_y;
					wearable_dataSecond.linear_acceleration.z = acc_z;
					
				}else if(*var.c_str() == 'y'){
					ss>> stuff >> vel_x >> vel_y >> vel_z;
					wearable_dataSecond.angular_velocity.x = vel_x;
					wearable_dataSecond.angular_velocity.y = vel_y;
					wearable_dataSecond.angular_velocity.z = vel_z;
				}
				
			}
		}
		vel_x = vel_y = vel_z = acc_x = acc_y = acc_z = 0;
		//std::cout << acc_x <<" "<< acc_y <<" "<< acc_z <<" "<< vel_x <<" "<< vel_y <<" "<< vel_z <<std::endl;
		//printf("%s", buffer);
		//std::cout <<" fine "<<std::endl;
		
		
		wearableSensorFirst_pub.publish(wearable_dataFirst);
		wearableSensorSecond_pub.publish(wearable_dataSecond);
		
		
		ss.str("");
		memset(buffer, 0, sizeof(buffer));
		ros::Duration(0.025).sleep();
		
		
					
	}
	
}
