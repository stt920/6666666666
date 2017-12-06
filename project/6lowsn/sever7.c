#include <unistd.h>  
#include <stdio.h>  
#include <arpa/inet.h>  
#include <sys/socket.h>  
#include <string.h>  
#define LOCALPORT 8888  
int main(int argc,char *argv[])  
{  
    int mysocket,len;  
    int i=0;  
    struct sockaddr_in6 addr;  
    int addr_len;  
    char msg[200];  
    char buf[300];  
    int j;
    if((mysocket=socket(AF_INET6,SOCK_DGRAM,0))<0)  
    {  
        perror("error:");  
        return(1);  
    }  
    else  
    {  
        printf("socket created ...\n");  
        printf("socket id :%d \n",mysocket);  
    }  
  
    addr_len=sizeof(struct sockaddr_in6);  
    bzero(&addr,sizeof(addr));  
   
    addr.sin6_family=AF_INET6;  
    addr.sin6_port=htons(LOCALPORT);  
    addr.sin6_addr=in6addr_any;  
  
    if(bind(mysocket,(struct sockaddr *)&addr,sizeof(addr))<0)  
    {  
        perror("connect");  
        return(1);  
    }  
    else  
    {  
        printf("bink ok .\n");  
        printf("local port : %d\n",LOCALPORT);  
    }  
    while(1)  
    {      
 	  j=0;
        bzero(msg,sizeof(msg));  
        len = recvfrom(mysocket,msg,sizeof(msg),0,(struct sockaddr *)&addr,(socklen_t*)&addr_len);  
        printf("%d:",i);  
        i++;  
        inet_ntop(AF_INET6,&addr.sin6_addr,buf,sizeof(buf));  
        printf("message from ip %s",buf); 
        printf("Received message : ");
        for(j=0;j<len;j++)
        printf(" 0x%x,",*(msg+j));  
		printf("\n\n");
     /*   if(sendto(mysocket,msg,len,0,(struct sockaddr *)&addr,addr_len)<0)  
        {  
            printf("error");  
            return(1);  
        }  */
    }  
}  
