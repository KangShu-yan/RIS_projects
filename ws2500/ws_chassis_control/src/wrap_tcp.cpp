//
// Created by RISLAB on 11-21-2020.
//
#include <ws_tcp/wrap_tcp.h>

/**
 * \file
* @brief Source code for this class that does tcp communication
*/

tcp_client::tcp_client()
{
	
}
tcp_client::~tcp_client()
{
    Disconnect();
}
void tcp_client::create_client(const char*ip, int port, bool async_comm)		//服务端地址192.168.1.10  服务端端口号4001  异步
{
    err=0;//Clear the error variable
	printf("create_client\n");
    connected=false;
	//创建TCP套接字
	sock = socket(AF_INET, SOCK_STREAM, 0);
//    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);	//ip地址类型AF_INET 表示 IPv4 地址，例如 127.0.0.1；AF_INET6 表示 IPv6 地址，例如 1030::C9B4:FF12:48AA:1A2B
														// SOCK_STREAM（流格式套接字/面向连接的套接字） 和 SOCK_DGRAM（数据报套接字/无连接的套接字）
														// IPPROTO_TCP 和 IPPTOTO_UDP
	//sock<0出错  sock=0连接关闭  sock>0接收到数据大小
	if(sock<0)											//
    {
        sprintf(err_msg,"Cannot open socket");	
        err=-1;
        return;
    }
    async=async_comm;

    /* Construct the server sockaddr_in structure */
    memset(&server, 0, sizeof(server));       /* Clear structure */
    server.sin_family = AF_INET;                  /* Internet/IP */
    server.sin_addr.s_addr = inet_addr(ip);  /* IP address */
    server.sin_port = htons(port);       /* server port */
}



int tcp_client::Connect(void)
{
	printf("Connect_function\n");
    err=0;
	//connect(int 套接字描述符,struct sockaddr * 指向套接字地址结构的指针，socklen_t 指向套接字地址结构的大小)
    ret=connect(sock, (struct sockaddr *) &server, sizeof(server)); 
	//返回0成功，-1时出错
    if(ret<0)
    {
        sprintf(err_msg,"Failed to connect with server");
        err=-2;
        return -1;
    }

    if(async)
    {
        fcntl(sock,F_SETOWN,getpid());	// 设置socket的拥有者
//        fcntl(sock,F_SETFL,O_ASYNC);	// F_SETFL：表示设置文件状态值。
		int temp_flag = fcntl(sock,F_GETFL,0);
		fcntl(sock,F_SETFL,temp_flag|O_NONBLOCK);	//无阻塞 异步通讯
    }

    connected=true;
//	if(connected){
//		printf("Conneted successfully!");
//	}
    return 0;
}

int tcp_client::Disconnect(void)
{
    err=0;
    ret=close(sock);
    if(ret<0)
    {
        sprintf(err_msg,"Failed to close connection");
        err=-3;
        return -1;
    }
    connected=false;

    return 0;
}

int tcp_client::Send(unsigned char*data, int size)
{
    if(!connected)
    {
        sprintf(err_msg,"Connection not established");
        err=-4;
        return -1;
    }
    err=0;
    errno=0;
    ret=send(sock,data,size,0);

    if(errno==EPIPE)
    {
        sprintf(err_msg,"Pipe broke");
        err=-4;
        return -1;
    }else if(ret<size)
    {
        sprintf(err_msg,"Mismatch in number of sent bytes");
        err=-4;
        return -1;
    }else if(errno!=0)
    {
        sprintf(err_msg,"Alarm! error not caught");
        perror("send");
        err=-3;
        return -1;
    }

    return 0;
}

int tcp_client::Receive(unsigned char*data, int size, bool peek, int*number, int flags)
{
    if(!connected)
    {
        sprintf(err_msg,"Connection not established");
        err=-4;
        return -1;
    }
    err=0;
    //Get the current time stamp
// 	timestamp=carmen_get_time();
    //If peek mode is selected we don't erase the information from the buffer
    if(peek)
    {
        ret=recv(sock,data,size, MSG_PEEK);
        if(ret<0)
        {
            sprintf(err_msg,"Failed to peek data from server");
            err=-5;
            return -1;
        }
        return 1;
    }
    //Normal read
    ret=recv(sock,data,size,flags);
    if(ret<0)
    {
        sprintf(err_msg,"Failed to receive bytes from server");
        err=-5;
        return -1;
    }else if(number!=NULL)
    {
        *number=ret;
    }
    return 1;
}

int tcp_client::perr(const char*text)
{
    if(err<0)
    {
        cout<<"Error!!   "<<text<<", "<<err_msg<<endl;
        return -1;
    }
    return 0;
}

int tcp_client::GetSocket()
{
    return sock;
}


