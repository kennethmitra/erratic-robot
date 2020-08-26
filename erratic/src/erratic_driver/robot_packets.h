#ifndef __ROBOT_PACKETS_H__
#define __ROBOT_PACKETS_H__
int process_char(unsigned char);
void process_keyboard_char(char c, int serial_port);
void send_motor_commands(int serial_port, int motor_vel, int motor_rvel);
int serial_port_connection(char* port_path);
void send_config_packet(int serial_port);
int is_printable(unsigned char c);


/* Constants used by the robot to generate packet frames */
#define SYNC0 0xFA
#define SYNC1 0xFB
#define ENDchar 0xFE
#define ARGchar 0x3B
#define NARGchar 0x1B
#define SARGchar 0x2B


/* from the robot controller code */
#define DEBUGpac   0x15
#define CONFIGpac  0x20
#define MOTORpac   0x80
#define ENCODERpac 0x90
#define DEBUG2pac  0x98
#define ANALOGpac  0x9A
#define SONARpac   0x9B

/* command numbers */
#define COMpulse  0
#define COMopen   1
#define COMclose  2
#define COMpoll   3             /* set sonar polling sequence */
#define COMenable 4             /* enable motors */
#define COMseta   5             /* acceleration for translation */
#define COMsetv   6             /* max velocity during pos moves, mm/sec */
#define COMorigin 7             /* reset integration origin */
#define COMmove   8             /* translational move, signed mm */
#define COMrotate 9
#define COMsetrv 10             /* max rot velocity during heading moves, deg/sec */
#define COMvel   11
#define COMhead  12
#define COMdhead 13
#define COMsay   15
#define COMconfig  18           /* send back one CONFIG packet */
#define COMencoder 19           /* send back encoder packets, continuously */
#define COMrvel 21
#define COMdchead 22
#define COMsetra 23             /* acceleration for rotation */
#define COMsonars 28
#define COMstop 29              /* Stop translation and rotation */
#define COMdigout 30
#define COMtimer  31
#define COMvel2   32            /* independent wheel velocities */
#define COMadsel 35             /* A-to-D pin selection */
#define COMptupos 41
#define COMtty2   42            /* transmit on alternate tty out */
#define COMgetaux 43
#define COMdebug 70
#define COManalog      71	/* enable or disable analog inputs */
#define COMservo 75		/* set servo position */
#define COMrevcount    86	/* encoder counts per revolution / 8 */
#define COMdriftbias   87	/* drift bias, in ??? */

struct traj {
  long int distance;    /* <cnt> total distance left to cover in this move */
  long int desired_pos; /* <cnt> where we think we should be */
  long int pos;         /* <cnt> actual position */
  int desired_vel;      /* <cnt/PERIODms> how fast we think we're going */
  int limit_vel;        /* <cnt/PERIODms> how fast we should go given distance */
  int vel_cmax;         /* current velocity limit in force */
  int err;              /* <cnt> desired pos - pos */
  int int_err;          /* <cnt> integrated error for this move */
  int cval;             /* <pwm> calculated control value */
};

#endif