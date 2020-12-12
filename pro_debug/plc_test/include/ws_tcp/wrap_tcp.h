
#ifndef PROJECT_WRAP_TCP_H
#define PROJECT_WRAP_TCP_H

/** @file
* @brief Main header for tcp communications
*/

// System Includes
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>


using namespace std;

/**
@brief This class implements the TCP/IP client communication.
*/
class tcp_client
{

    public:
        /**
        @brief Constructor of the class.

        The class constructor will setup the socket but will  <b><em>NOT</em></b> open it.

        @param ip Ip of the server
        @param port Port in which to communicate
        @param async_comm Activate the asynchronous communication.

        */
        tcp_client();
       

        /**
        @brief Destructor of the class.

        It disconnects the communication.
        */
        ~tcp_client();

		void create_client(const char*ip, int port, bool async_comm=false);
        /**
        @brief Connect to server
        This function connects to the server in case of error sets the err_msg variable and the err code accordingly.
        @return 0 in case of success or -1 in error
        */
       	virtual int Connect(void);

        /**
        @brief Disconnect from server

        This function disconnects to the server in case of error sets the err_msg variable and the err code accordingly.
        @return 0 in case of success or -1 in error
        */
        int Disconnect(void);
        /**
        @brief Send data

        This function sends information to the server, in case of error sets the err_msg variable and the err code accordingly.
        @param data information to send, this is a byte array
        @param size size of the  information to send
        @return 0 in case of success or -1 in error
        */
        int Send(unsigned char *data, int size);
        /**
        @brief Read data

        This function reads information from the memory buffer, if the peek variable is true the information is not
        erased from the buffer.
        @param data where to store the data
        @param size maximum length to store
        @param peek if true data is not removed from the server
        @param number number of bytes received
        @param flags flags of the recv call
        @return 0 in case of success or -1 in error
        */
        int Receive(unsigned char*data, int size, bool peek=false, int*number=NULL, int flags=0);

        /**
        @brief Print error message

        This function prints the last error of this class, this is only valid for the last function called,
        if a function previous to that had a error it will not be printed.
        @param text additional text to print along with the error.
        @return 0 if theres no error to print or -1 in error
        */
        int perr(const char*text);

        /**
        @brief Get socket value

        Get the current socket value.
        @return Socket value.
        */
        int GetSocket();
        ///Last error code.
        int err;
        ///Last error message.
        char err_msg[1024];
        ///Status of the connection.
        bool connected;
    private:
        ///Socket identification
        int sock;
        ///Configuration structure
        struct sockaddr_in server;
        ///Return value for functions
        int ret;
        ///Asynchronous comm
        bool async;
    };
#endif //PROJECT_CLASS_TCP_H
