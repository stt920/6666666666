#include <stdio.h>  
#include <stdlib.h>  
#include <unistd.h>  
#include <string.h>  
#include <netinet/in.h>  
#include <sys/socket.h>  
#include <sys/types.h>  
  
#define SERVER_IP "2345::1147:ff:fe00:3525"  
#define SERVER_PORT 0xf0b3  
  
  
int main(int argc, char *argv[])  
{  
    int sock = -1;  
    int ret  = -1;  
    int addr_len = 0;  
     int j;
    struct sockaddr_in6 server_addr;  
      
    sock = socket(AF_INET6, SOCK_DGRAM, 0);  
    if (sock < 0) {  
        perror("Fail to socket");  
        exit(1);  
     }  
  
    memset(&server_addr, 0, sizeof(server_addr));  
      
    server_addr.sin6_family = AF_INET6;  
    server_addr.sin6_port = htons(SERVER_PORT);  
    inet_pton(AF_INET6, SERVER_IP, &(server_addr.sin6_addr.s6_addr));  
  
    addr_len = sizeof(server_addr);  
  
    char buf[] =  {0x66,0xAB,0x0D,0x01,0x31,0x00,0xFE,0x03,0x21,0x08,0x11,02,00};  
    printf("Send data:");  
    for(j=0;j<sizeof(buf)/sizeof(buf[0]);j++)
		printf("0x%x,",*(buf+j));
    printf("\n:"); 
      
    addr_len = sizeof(server_addr);  
    ret = sendto(sock, buf, sizeof(buf)/sizeof(buf[0]), 0, (struct sockaddr *)&server_addr, addr_len);  
    if (ret < 0) {  
        perror("Fail to recvfrom");  
        exit(1);  
     }  
  
    printf("Send data successfully\n");  
    sleep(2);  
      
    return 0;  
}
