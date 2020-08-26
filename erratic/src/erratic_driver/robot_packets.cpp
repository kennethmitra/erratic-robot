#include "robot_packets.h"
#include "robot_params.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <termios.h> /* For tcsetattr */
#include <atomic>

// Additional constants for packet parsing states
#define LENGTH 0x01
#define DATA 0x02

// debugging
int debug_packets = 0;

// Robot data collected through packets
std::atomic<int> robot_x(0);
std::atomic<int> robot_y(0);
std::atomic<int> heading(0);
std::atomic<int> vel_cnt_left(0); 
std::atomic<int> vel_cnt_right(0);
std::atomic<int> battery_voltage(0);
std::atomic<int> stall_right(0); 
std::atomic<int> stall_left(0);
std::atomic<int> control(0);
volatile char name[256] = {0}, type[256] = {0}, subtype[256] = {0}, sn[256] = {0};
std::atomic<int> version(0);
std::atomic<int> subversion(0);

//Config settings
int raw_output = 0;
int raw_content_output = 0;
int escape_raw_output = 1;
int summary_output = 0;
int time_output = 0;
int matlab_output = 0;
uint64_t motor_time = 0;


// motor control
int motor_vel = 0;  // signed mm/sec
int motor_rvel = 0; // signed deg/sec

// servo control
int servo0 = 1700; // servo units
int servo1 = 1700;
int servo2 = 1700;

// drift
int drift = 0;

// parameters
struct parameters params;
#define revBytes(x) x = (0xff & ((x) >> 8)) | (0xff00 & ((x) << 8));

// forward declarations
void process_contents(unsigned char *data, int length);
short calc_chksum(int n, char *b);
void send_packet(char *buf, unsigned char length, int serial_port);

void fill_traj(struct traj *t, unsigned char *s);
short read_little_short(unsigned char *c);
long read_little_int(unsigned char *c);

/* for time stats */
struct timeval last_time;

/* Packet buffer */
unsigned char packet[256];

/*
 * Calculates checksum, counts bytes. Triggers packet parsing when whole packet
 * is received. At first sign of error, returns -1 and resets state to wait for
 * synchronization.
 */
int process_char(unsigned char c) {
  /* Packet processing state */
  static int state = SYNC0;
  static int packet_length;
  static int current_byte;

  switch (state) {
  case SYNC0:
    if ((unsigned char)c == SYNC0)
      state = SYNC1;
    else {
      if (debug_packets)
        printf("Expected SYNC1, got %d\n", (unsigned)c);
      return -1;
    }
    break;

  case SYNC1:
    if ((unsigned char)c == SYNC1)
      state = LENGTH;
    else {
      if (debug_packets)
        printf("Expected SYNC2, got %d\n", (unsigned)c);
      state = SYNC0;
      return -1;
    }
    break;

  case LENGTH:
    debug_packets = 1;
    packet_length = c;
    current_byte = 0;
    if (packet_length < 2) { /* malformed length, must have checksum */
      if (debug_packets)
        printf("Expected length, but it was too short (%d)\n", (unsigned)c);
      state = SYNC0;
      return -1;
    } else if (packet_length == 2) { /* packet is empty */
      if (debug_packets)
        printf("Empty packets aren't all that useful (%d)\n", (unsigned)c);
      process_contents(packet, 0);
      state = SYNC0;
    } else
      state = DATA;
    break;

  case DATA:
    packet[current_byte++] = c;
    if (current_byte == packet_length) {
      short sent_checksum =
          ((int)packet[current_byte - 2] << 8) + packet[current_byte - 1];
      short correct_checksum = calc_chksum(packet_length - 2, (char *)packet);

      if (sent_checksum != correct_checksum) {
        if (debug_packets)
          printf(" BAAAAD checksum (should be %x): \n",
                 (unsigned short)correct_checksum);
      } else {
        if (raw_output)
          printf(" [OK]\n");
        process_contents(packet, packet_length - 2);
      }

      state = SYNC0;
    }
    break;
  }
}

void process_keyboard_char(char c, int serial_port) {
  unsigned char content[256];
  int content_length = 0;
  int done = 1;
  int dd;

  // check for print command
  switch (c) {
  case 'R': // turn on raw output
    raw_output = 1;
    break;
  case 'r':
    raw_output = 0; // turn off raw output
    break;
  case 'z':
    summary_output = 1; // print summary output
    break;
  default:
    done = 0; // not handled
  }

  if (done)
    return;

  // check for controller command
  switch (c) {
  case 'A': // request analog packets
    content[0] = COManalog;
    content[1] = ARGchar;
    content[2] = 0x01;
    content[3] = 0x00;
    content_length = 4;
    break;

  case 'a': // stop analog packets
    content[0] = COManalog;
    content[1] = ARGchar;
    content[2] = 0x00;
    content[3] = 0x00;
    content_length = 4;
    break;

  case 'S': // request sonar packets
    content[0] = COMsonars;
    content[1] = ARGchar;
    content[2] = 0x01;
    content[3] = 0x00;
    content_length = 4;
    break;

  case 's': // stop sonar packets
    content[0] = COMsonars;
    content[1] = ARGchar;
    content[2] = 0x00;
    content[3] = 0x00;
    content_length = 4;
    break;

  case 'w': // servo 0 increment
    servo0 += 100;
    printf("Servo 0: %d\n", servo0);
    content[0] = COMservo;
    content[1] = SARGchar;
    content[2] = 0x03;
    content[3] = 0x00;
    content[4] = servo0 & 0xff;
    content[5] = (servo0 & 0xff00) >> 8;
    content_length = 6;
    break;

  case 'W': // servo 0 decrement
    servo0 -= 100;
    printf("Servo 0: %d\n", servo0);
    content[0] = COMservo;
    content[1] = SARGchar;
    content[2] = 0x03;
    content[3] = 0x00;
    content[4] = servo0 & 0xff;
    content[5] = (servo0 & 0xff00) >> 8;
    content_length = 6;
    break;

  case 'x': // servo 1 increment
    servo1 += 100;
    printf("Servo 1: %d\n", servo1);
    content[0] = COMservo;
    content[1] = SARGchar;
    content[2] = 0x03;
    content[3] = 0x01;
    content[4] = servo1 & 0xff;
    content[5] = (servo1 & 0xff00) >> 8;
    content_length = 6;
    break;

  case 'X': // servo 1 decrement
    servo1 -= 100;
    printf("Servo 1: %d\n", servo1);
    content[0] = COMservo;
    content[1] = SARGchar;
    content[2] = 0x03;
    content[3] = 0x01;
    content[4] = servo1 & 0xff;
    content[5] = (servo1 & 0xff00) >> 8;
    content_length = 6;
    break;

  case 'y': // servo 2 increment
    servo2 += 100;
    printf("Servo 2: %d\n", servo2);
    content[0] = COMservo;
    content[1] = SARGchar;
    content[2] = 0x03;
    content[3] = 0x02;
    content[4] = servo2 & 0xff;
    content[5] = (servo2 & 0xff00) >> 8;
    content_length = 6;
    break;

  case 'Y': // servo 2 decrement
    servo2 -= 100;
    printf("Servo 2: %d\n", servo2);
    content[0] = COMservo;
    content[1] = SARGchar;
    content[2] = 0x03;
    content[3] = 0x02;
    content[4] = servo2 & 0xff;
    content[5] = (servo2 & 0xff00) >> 8;
    content_length = 6;
    break;

  case 'B': // decrement drift bias
    drift -= 40;
  case 'b': // increment drift bias
    drift += 20;
    printf("Drift: %d\n", drift);
    content[0] = COMdriftbias;
    if (drift < 0) {
      dd = -drift;
      content[1] = NARGchar;
    } else {
      dd = drift;
      content[1] = ARGchar;
    }
    content[2] = dd & 0xff;
    content[3] = (dd & 0xff00) >> 8;
    content_length = 4;
    break;

  case 'i':
    motor_vel += 100; // increment trans velocity by 100 mm/sec
    printf("Motor vel increased\n");
    break;
  case 'm':
    motor_vel -= 100; // decrement trans velocity by 100 mm/sec
    printf("Motor vel decreased\n");
    break;

  case 'j':
    motor_rvel += 10; // increment rot velocity by 10 deg/sec
    printf("Motor rotation increased\n");
    break;
  case 'l':
    motor_rvel -= 10; // decrement rot velocity by 10 deg/sec
    printf("Motor rotation decreased\n");
    break;

  case ' ':
  case 'k':
    motor_vel = motor_rvel = 0; // stop motors
    break;
  }

  if (content_length > 0)
    send_packet((char *)content, content_length, serial_port);
}

// command motors to move according to velocity variables
void send_motor_commands(int serial_port) {
  unsigned char content[256];
  int content_length = 0;
  static int enabled = 0;

  // check motor status
  if (motor_vel == 0 && motor_rvel == 0) {
    if (enabled) {
      content[0] = COMenable;
      content[1] = ARGchar;
      content[2] = 0x0;
      content[3] = 0x0;
      content_length = 4;
    }
    enabled = 0;
  }

  if (motor_vel != 0 || motor_rvel != 0) {
    if (!enabled) {
      content[0] = COMenable;
      content[1] = ARGchar;
      content[2] = 0x1;
      content[3] = 0x0;
      content_length = 4;
    }
    enabled = 1;
  }

  // NOTE: if enabling motors, don't send motor control immediately
  //   after, wait 100 ms
  if (content_length > 0) {
    send_packet((char *)content, content_length, serial_port);
    return;
  }
  content_length = 0;

  // send translation velocity
  if (motor_vel != 0) {
    int vel = motor_vel;
    if (vel < 0)
      vel = -motor_vel;
    content[0] = COMvel;
    if (motor_vel < 0)
      content[1] = NARGchar;
    else
      content[1] = ARGchar;

    content[2] = 0xff & vel;
    content[3] = 0xff & (vel >> 8);
    content_length = 4;
  }

  if (content_length > 0)
    send_packet((char *)content, content_length, serial_port);
  content_length = 0;

  // send rotation velocity
  if (motor_rvel != 0) {
    int vel = motor_rvel;
    if (vel < 0)
      vel = -motor_rvel;
    content[0] = COMrvel;
    if (motor_rvel < 0)
      content[1] = NARGchar;
    else
      content[1] = ARGchar;

    content[2] = 0xff & vel;
    content[3] = 0xff & (vel >> 8);
    content_length = 4;
  }

  if (content_length > 0)
    send_packet((char *)content, content_length, serial_port);
}

/**
* Send drive command 
* motor_vel mm/sec
* motor_rvel deg/sec
*/
void send_motor_commands(int serial_port, int motor_vel, int motor_rvel) {
  unsigned char content[256];
  int content_length = 0;
  static int enabled = 0;

  // check motor status
  if (motor_vel == 0 && motor_rvel == 0) {
    if (enabled) {
      content[0] = COMenable;
      content[1] = ARGchar;
      content[2] = 0x0;
      content[3] = 0x0;
      content_length = 4;
    }
    enabled = 0;
  }

  if (motor_vel != 0 || motor_rvel != 0) {
    if (!enabled) {
      content[0] = COMenable;
      content[1] = ARGchar;
      content[2] = 0x1;
      content[3] = 0x0;
      content_length = 4;
    }
    enabled = 1;
  }

  // NOTE: if enabling motors, don't send motor control immediately
  //   after, wait 100 ms
  if (content_length > 0) {
    printf("robot_packets.cpp: Enabling/Disabling motors.../n");
    send_packet((char *)content, content_length, serial_port);
    return;
  }
  content_length = 0;

  // send translation velocity
  if (motor_vel != 0) {
    int vel = motor_vel;
    if (vel < 0)
      vel = -motor_vel;
    content[0] = COMvel;
    if (motor_vel < 0)
      content[1] = NARGchar;
    else
      content[1] = ARGchar;

    content[2] = 0xff & vel;
    content[3] = 0xff & (vel >> 8);
    content_length = 4;
  }

  if (content_length > 0)
    send_packet((char *)content, content_length, serial_port);
  content_length = 0;

  // send rotation velocity
  if (motor_rvel != 0) {
    int vel = motor_rvel;
    if (vel < 0)
      vel = -motor_rvel;
    content[0] = COMrvel;
    if (motor_rvel < 0)
      content[1] = NARGchar;
    else
      content[1] = ARGchar;

    content[2] = 0xff & vel;
    content[3] = 0xff & (vel >> 8);
    content_length = 4;
  }

  if (content_length > 0)
    send_packet((char *)content, content_length, serial_port);
}

// open the device and get configuration
void send_config_packet(int serial_port) {
  unsigned char content[256];
  content[0] = COMopen;
  send_packet((char *)content, 1, serial_port);
  content[0] = COMconfig;
  send_packet((char *)content, 1, serial_port);
}

void send_packet(char *buf, unsigned char length, int serial_port) {
  char packet[260];

  packet[0] = SYNC0;
  packet[1] = SYNC1;
  packet[2] = length + 2;
  for (int i_buf = 0; i_buf < length; i_buf++)
    packet[i_buf + 3] = buf[i_buf];

  short checksum = calc_chksum(length, packet + 3);
  packet[length + 3] = checksum >> 8;
  packet[length + 4] = checksum & 0x00FF;

  #if 0
    printf("Sending packet '");
    for (int i_buf = 0; i_buf < length+5 ;i_buf++)
      printf("(%x)",(unsigned char)packet[i_buf]);
    printf("'\n");
  #endif

  write(serial_port, packet, length + 5);
}

// print robot state
void put_robot_state() {
  printf("\n---------robot--------\n  x:       %6i\n  y:       %6i\n", robot_x.load(),
         robot_y.load());
  printf("  vel:     %6i\n  rot:     %6i\n  heading: %6i\n", vel_cnt_left.load(),
         vel_cnt_right.load(), heading.load());
  printf("  control: %6i\n", control.load());
  printf("  voltage:  %5.1f\n  stall:      %1i,%1i\n",
         (float)battery_voltage.load() / 10.0, stall_left.load(), stall_right.load());
  printf("  name:    %s\n  type:    %s\n  subtype: %s\n  version: %d.%d\n  "
         "serial:  %s\n\n",
         name, type, subtype, version.load(), subversion.load(), sn);
}

void process_contents(unsigned char *data, int length) {
  /* some code for confirming packet frequencies */
  struct timeval new_time;
  gettimeofday(&new_time, 0);
  float time_diff = (new_time.tv_sec - last_time.tv_sec) +
                    (new_time.tv_usec - last_time.tv_usec) * 0.000001;
  float frequency = 1.0 / time_diff;
  last_time.tv_sec = new_time.tv_sec;
  last_time.tv_usec = new_time.tv_usec;
  if (time_output)
    printf("Frequency of packets: %g\n", frequency);

  if (raw_content_output) {
    putchar('\n');
    for (int i_data = 0; i_data < length; i_data++) {
      if (!escape_raw_output && is_printable((unsigned char)data[i_data]))
        putchar(data[i_data]);
      else
        printf("(%02x)", data[i_data]);
    }
    putchar('\n');
  }

  int i, count;
  unsigned char *p;

  switch (packet[0]) {
    case MOTORpac ... MOTORpac + 3:
      robot_x.store(packet[1] + 0x100 * packet[2] + 0x10000 * packet[3]);
      if (robot_x >= 0x800000)
        robot_x -= 0x1000000;
      robot_y = packet[4] + 0x100 * packet[5] + 0x10000 * packet[6];
      if (robot_y >= 0x800000)
        robot_y -= 0x1000000;
      heading = packet[7] + 0x100 * packet[8];
      vel_cnt_left.store((short)(packet[9] + 0x100 * packet[10]));
      vel_cnt_right.store((short)(packet[11] + 0x100 * packet[12]));
      battery_voltage.store(packet[13]);
      stall_right.store((0 != (packet[14] & 0x02)));
      stall_left.store((0 != (packet[14] & 0x01)));
      control.store(packet[15] + 0x100 * packet[16]);
      // r = ...
      break;

      // parse configuration
    case CONFIGpac:
      p = (unsigned char *)&params;
      for (i = 0; i < sizeof(struct parameters); i++) // copy over info
        p[i] = 0x0;
      for (i = 0; i < length - 2; i++) // copy over info
      {
        p[i] = packet[i + 2];
        printf(" %02x ", p[i]);
      }
      printf("\n\n");

      printf("Got configuration packet from controller, version %d, length %d\n",
            packet[1], length);
      for (int i = 0; i < 20; i++)
        name[i] = params.pName[i];
      for (int i = 0; i < 20; i++)
        type[i] = params.pType[i];
      for (int i = 0; i < 20; i++)
        subtype[i] = params.pSubtype[i];
      for (int i = 0; i < 10; i++)
        sn[i] = params.pSN[i];
      version = params.pVersion;
      subversion = params.pSubVersion;
      printf("  name:    %s\n  type:    %s\n  subtype: %s\n  version: %d.%d\n  "
            "serial:  %s\n\n",
            name, type, subtype, version.load(), subversion.load(), sn);

      // shorts are wrong byte order, change
      revBytes(params.pRevCount);
      revBytes(params.pWatchDog);
      revBytes(params.pLowBattery);
      revBytes(params.pStallVal);
      revBytes(params.pStallCount);
      revBytes(params.pPwmMax);
      revBytes(params.pPwmWrap);
      revBytes(params.pRotVelMax);
      revBytes(params.pTransVelMax);
      revBytes(params.pRotAcc);
      revBytes(params.pTransAcc);
      revBytes(params.pRotDecel);
      revBytes(params.pTransDecel);
      revBytes(params.pTransKp);
      revBytes(params.pTransKv);
      revBytes(params.pTransKi);
      revBytes(params.pRotKp);
      revBytes(params.pRotKv);
      revBytes(params.pRotKi);

      printf("  host baud: %d\n  aux baud: %d\n  rev count: %d\n\n",
            params.pBaudHost, params.pBaudAux, params.pRevCount);

      printf(
          "  watchdog: %d\n  low bat: %d\n  stall val: %d\n  stall cnt: %d\n\n",
          params.pWatchDog, params.pLowBattery, params.pStallVal,
          params.pStallCount);

      printf("  pwm max: %d\n  pwm wrap: %d\n\n", params.pPwmMax,
            params.pPwmWrap);

      printf("  Rot max vel: %d\n  Rot acc: %d\n  Rot decel: %d\n\n",
            params.pRotVelMax, params.pRotAcc, params.pRotDecel);

      printf("  Trans max vel: %d\n  Trans acc: %d\n  Trans decel: %d\n\n",
            params.pTransVelMax, params.pTransAcc, params.pTransDecel);

      printf("  Rot Kp: %d\n  Rot Kv: %d\n  Rot Ki: %d\n\n", params.pRotKp,
            params.pRotKv, params.pRotKi);

      printf("  Trans Kp: %d\n  Trans Kv: %d\n  Trans Ki: %d\n\n",
            params.pTransKp, params.pTransKv, params.pTransKi);

      break;

    case ANALOGpac:
      printf("Analog packet:");
      count = packet[1];
      for (i = 0; i < count; i++) {
        int ain = (short)(packet[2 * i + 2] + 0x100 * packet[2 * i + 3]);
        printf("  %04x", ain);
      }
      printf("   Dig: %04x",
            (short)(packet[2 * i + 2] + 0x100 * packet[2 * i + 3]));
      printf("\n");
      break;

    case SONARpac:
      count = packet[1];
      for (i = 0; i < count; i++) {
        int num = packet[3 * i + 2];
        if (num == 0)
          printf("\nSonar packet:");
        int val = (short)(packet[3 * i + 3] + 0x100 * packet[3 * i + 4]);
        printf("  %d:%04d", num, val);
      }
      break;

    case DEBUGpac: // debug message
      printf("Debug message: %s\n", packet + 1);
      break;

    case DEBUG2pac: // motor control debugging
      struct traj rot, trans;

      fill_traj(&rot, &packet[1]);
      fill_traj(&trans, &packet[29]);

      long headcount = read_little_int(&packet[29 + 28]);

      if (matlab_output) {
        printf("%lu, %lu, %lu, ", rot.distance, rot.desired_pos, rot.pos);
        printf("%d, %d, %d, ", rot.desired_vel, rot.limit_vel, rot.vel_cmax);
        printf("%d, %d, %d, ", rot.err, rot.int_err, rot.cval);

        printf("%lu, %lu, %lu, ", trans.distance, trans.desired_pos, trans.pos);
        printf("%d, %d, %d, ", trans.desired_vel, trans.limit_vel,
              trans.vel_cmax);
        printf("%d, %d, %d\n", trans.err, trans.int_err, trans.cval);
      } else {
        printf("ROT traj. distance:%lu desired_pos:%lu pos:%lu\n", rot.distance,
              rot.desired_pos, rot.pos);
        printf("desired_vel:%d limit_vel:%d vel_cmax:%d\n", rot.desired_vel,
              rot.limit_vel, rot.vel_cmax);
        printf("err:%d int_err:%d cval:%d\n", rot.err, rot.int_err, rot.cval);

        printf("TRANS traj. distance:%lu desired_pos:%lu pos:%lu\n", trans.distance,
              trans.desired_pos, trans.pos);
        printf("desired_vel:%d limit_vel:%d vel_cmax:%d\n", trans.desired_vel,
              trans.limit_vel, trans.vel_cmax);
        printf("err:%d int_err:%d cval:%d\n", trans.err, trans.int_err,
              trans.cval);

        printf("headcount: %lu\n", headcount);
      }
      break;
  }
  if (summary_output) // one-shot
  {
    put_robot_state();
    summary_output = 0;
  }
}

short calc_chksum(int n, char *b) {
  /*unsigned char *buffer = (unsigned char *)b;
  int c = 0;

  while (n > 1) {
    c+= (*(buffer)<<8) | *(buffer+1);
    c = c & 0xffff;
    n -= 2;
    buffer += 2;
  }
  if (n>0) c = c^ (int)*(buffer++);

  return(c);*/

  short c = 0;
  while (n > 1) {
    c += (*((unsigned char *)b) << 8) | *((unsigned char *)b + 1);
    n -= 2;
    b += 2;
  }
  unsigned int test;
  if (n > 0) {
    unsigned short p = (unsigned char)*b;
    c = c ^ p;
    test = (unsigned char)*b;
  } else
    test = (unsigned char)*(b - 1);

  // printf("last for checksum: %x\n", test);
  return c;
}

int serial_port_connection(char *port_path) {
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

//-----------------------------
//      Helper Functions
//-----------------------------
int is_printable(unsigned char c) {
  return ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) && !escape_raw_output;
}
long read_little_int(unsigned char *c) {
  return (long)*c | (long)*(c + 1) << 8 | (long)*(c + 2) << 16 |
         (long)*(c + 3) << 24;
}
short read_little_short(unsigned char *c) {
  return (long)*c | (long)*(c + 1) << 8;
}
void fill_traj(struct traj *t, unsigned char *s) {
  t->distance = read_little_int(s);
  t->desired_pos = read_little_int(s + 4);
  t->pos = read_little_int(s + 8);
  t->desired_vel = read_little_short(s + 12);
  t->limit_vel = read_little_short(s + 14);
  t->vel_cmax = read_little_short(s + 16);
  t->err = read_little_int(s + 18);
  t->int_err = read_little_int(s + 22);
  t->cval = read_little_short(s + 26);
}