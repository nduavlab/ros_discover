#ifndef __SERIALRDM_H__
#define __SERIALRDM_H__



#include <iostream>


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <fstream>
#include <stdarg.h>

#include <sys/time.h>

#include <sys/stat.h>
#include <fcntl.h>

#define DebugMode 1

// Serial Class: raw data mode
class SerialRDM {
public:
    SerialRDM(void);                                        // Constructor
    virtual ~SerialRDM();                               // Destructor

    void set_values(char *);

    int init(void);

    void serialsendf(const char* buf,uint16_t len);    // Send with format
    void serialreceive(void* buff,size_t len);                 // Receive data
    void serialclose(void);                             // Finalize

    int fd_mavlink;  //for mavlink to use

private:
    int fd;                                             // port description file
    char *path;                                   // port directory
    int serialopen(void);                // Open port
    int serialconfig(void);                                // Config port
};



#endif
