#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define PORT 9099 
#define BUFFER_SIZE 180*240

int sockfd;
int sock_init = 0;

int isConnected() {
	return sock_init;
}

void initClient() {
	
	if (sock_init > 0)
	{
		printf("socket already initialized \n");
		return;
	}
    struct sockaddr_in server_addr;
    
    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        exit(EXIT_FAILURE);
    }

    // Initialize server address
    memset(&server_addr, '0', sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr) <= 0) {
        perror("Invalid address/ Address not supported");
        exit(EXIT_FAILURE);
    }

    // Connect to the server
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection Failed");
        exit(EXIT_FAILURE);
    }

	sock_init = 1;
}


void sendFloatData(float * data_f, int data_f_size) {
	if (sock_init == 1)
	{
		// Write the data to the socket
		if ( (write(sockfd, (void*)data_f , data_f_size*4)) < 0) {
			perror("Write error");
			exit(EXIT_FAILURE);
		}
		printf("Data sent successfully\n");
	}
	else {
		printf("Socket not initialized., Cannot send data. Call function initClient first \n");
	}
}

int readCommand()
{
	char recv_buf[1];
	if (sock_init == 1) {
		if ( (read(sockfd, recv_buf, 1)) < 0) {
			perror("Read error");
			exit(EXIT_FAILURE);
		}
		else {
			if (recv_buf[0] == 1) {
				return 1;
			}
			else {
				return 0;
			}
		}
		
	}
	else 
	{
		sleep(1);
		return 0;
	}
}

void endClient()
{
	if (sock_init == 1) {
		close(sockfd);
	}
}

