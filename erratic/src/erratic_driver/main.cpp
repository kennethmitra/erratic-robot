/*
 * Test client for the robot. Prints raw data or extracts summaries.
 *
 * Author: Joakim Arfvidsson, joakim@arfvidsson.com
 *
 * Copyright: Videre Design, 2006
 */


#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <stdint.h>

#include "robot_packets.h"

/* settings for operation, set through arguments */
extern int raw_output;
extern int raw_content_output;
extern int escape_raw_output;
extern int summary_output;
extern int time_output;
extern int matlab_output;
extern uint64_t motor_time;

int serial_port_connection(char* port_path);
void send_config_packet(int serial_port);
void send_motor_commands(int serial_port); // send out current motor commands

uint64_t get_ms()
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return(tv.tv_sec*1000 + tv.tv_usec/1000);
}


int main(int argc, char *argv[]) 
{
  if (argc != 2) // Make sure comm port is specified
    {
      fprintf(stderr, "Usage: robot_client <comm port>\n");
      return -1;
    }

  // Open comm port (usually /dev/ttyUSB0)
  int serial_port = serial_port_connection(argv[1]);
  if (serial_port < 0) 
    {
      fprintf(stderr, "No serial port acquired\n");
      return -1;
    }

  // send configuration request
  send_config_packet(serial_port);


  // main reading loop
  for (;;) {
    char inbuffer[100];
    int i_buffer;
    
    fd_set read_set, error_set; //Sets of file descriptors that can be monitored
    FD_ZERO(&read_set); FD_ZERO(&error_set); // Empty the read and error sets
    FD_SET(serial_port, &read_set); FD_SET(serial_port, &error_set); // Add serial port to both sets
    FD_SET(0, &read_set); //Add stdin to read set?
    select(serial_port+1, &read_set, 0, &error_set, 0); //nfds is the highest-numbered file descriptor in any of the three sets, plus 1. (Timeout is null so blocks indefinetly) (Upon return, only fd's that became available are in sets)
    /* selecting was done on both data and errors on the serial port*/
    if (FD_ISSET(serial_port, &read_set) | FD_ISSET(serial_port, &error_set)) { //See if serial port is in active sets
      int result = read(serial_port, inbuffer, 5); //result = how many characters read
    
      if (result < 0) {
        perror("Error reading from serial port");
        return -1;
      } else if (result == 0) {
      //	fprintf(stderr, "Unexpected end of file from serial port");
      //  continue;
      } else {
        //	printf("Found %d chars\n", result);
        for (i_buffer = 0; i_buffer < result ;i_buffer++) {
          if (raw_output) {
            printf("%x ", (unsigned char)inbuffer[i_buffer]);
            //	    if (escape_raw_output &&
            //		!is_printable((unsigned char)inbuffer[i_buffer])) {
            //	      printf("(%x)", (unsigned char)inbuffer[i_buffer]);
            //	    } else
            //	      putchar(inbuffer[i_buffer]);
            }
          process_char(inbuffer[i_buffer]);
        }
      }
    } else { //delay
      usleep(10000);
    }
    /* data from keyboard waiting */
    if (FD_ISSET(0, &read_set)) {
      char key;
      //	read(0, &key, 1);
      key = getchar();
      process_keyboard_char(key, serial_port);
    }
    // check for motor control at 10 Hz
    if (get_ms() > motor_time + 100) {// ok, do motor control
      motor_time = get_ms();
      send_motor_commands(serial_port);
    }
  }
}

int serial_port_connection(char *port_path) 
{
  /* Opening the serial port */
  int port;
  port = open(port_path, O_RDWR);
  if (port < 0) {
    perror("Error opening serial port");
    return -1;
  }
  printf("Opened comm port %s\n", port_path);

  /* setting baud rate */
  struct termios attributes;
  tcgetattr(port, &attributes);
  
  // I have replaced cfmakeraw with what I think it does
  //cfmakeraw(&attributes);
  attributes.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
			  |INLCR|IGNCR|ICRNL|IXON);
  attributes.c_oflag &= ~OPOST;
  attributes.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
  attributes.c_cflag &= ~(CSIZE|PARENB);
  attributes.c_cflag |= CS8;  
  
  cfsetispeed(&attributes, B38400);
  cfsetospeed(&attributes, B38400);
  if (tcsetattr(port, TCSANOW, &attributes) < 0)
    perror("Error on tcsetattr");
  
  return port;
}

