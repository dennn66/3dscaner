#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include "arduino_driver.h"

/**
 *  Get Arduino device file name from command line arguments.
 */
char* getDeviceFileName(int argc, char **argv)
{
   if (argc < 2) {
      fprintf(stderr, "Argument missing: Arduino device file.\n");
      exit(EXIT_FAILURE);
   }
   return argv[1];
}

/**
 *  Open Arduino device file.
 */
int openDeviceFile(const char* fileName, const char* mode)
{
   FILE* file;

   file = fopen(fileName, mode);
   if (file == NULL) {
      perror(fileName);
      exit(EXIT_FAILURE);
   }
   return fileno(file);
}

/**
 *  Get baud rate from command line arguments.
 */
unsigned int getBaudRate(int argc, char **argv)
{
   return (argc > 2) ? atoi(argv[2]) : 57600;
}

/**
 *  Set Arduino serial port attributes.
 */
void setAttr(int fd, unsigned int baud)
{
   struct termios toptions;
   speed_t brate;

   if (tcgetattr(fd, &toptions) < 0) {
      perror("Can't get term attributes");
      exit(EXIT_FAILURE);
   }

   switch(baud) {
      case 4800:   brate=B4800;   break;
      case 9600:   brate=B9600;   break;
    #ifdef B14400
      case 14400:  brate=B14400;  break;
    #endif
      case 19200:  brate=B19200;  break;
    #ifdef B28800
      case 28800:  brate=B28800;  break;
    #endif
      case 38400:  brate=B38400;  break;
      case 57600:  brate=B57600;  break;
      case 115200: brate=B115200; break;
      case 230400: brate=B230400; break;
      default:
         fprintf(stderr, "Invalid baud rate value: %d\n", baud);
         exit(EXIT_FAILURE);
   }
   cfsetispeed(&toptions, brate);
   cfsetospeed(&toptions, brate);

   // 8N1
   toptions.c_cflag &= ~PARENB;
   toptions.c_cflag &= ~CSTOPB;
   toptions.c_cflag &= ~CSIZE;
   toptions.c_cflag |= CS8;
   // no flow control
   toptions.c_cflag &= ~CRTSCTS;
   
   toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
   toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    
   toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
   toptions.c_oflag &= ~OPOST; // make raw
    
   // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
   toptions.c_cc[VMIN]  = 0;
   toptions.c_cc[VTIME] = 20;
        
   if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
      perror("Can't set term attributes");
      exit(EXIT_FAILURE);
   }
}

/**
 *  Send command to Arduino.
 */
void sendCommand(int fd, const char *cmd)
{
   write(fd, cmd, strlen(cmd));
}

/**
 *  Display command result from Arduino.
 */
void displayResult(int fd)
{
  char buf[256];
  ssize_t sz;

  do {
    sz = read(fd, buf, 255);
    buf[sz] = 0;
    printf("%s", buf);
  }
  while (sz);
}

