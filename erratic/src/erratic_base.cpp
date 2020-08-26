

#include <functional>
#include <memory>
#include <chrono>
#include <atomic>

#include <boost/thread.hpp>

#include "erratic_driver/robot_packets.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* settings for operation, set through arguments */
extern int raw_output;
extern int raw_content_output;
extern int escape_raw_output;
extern int summary_output;
extern int time_output;
extern int matlab_output;
extern uint64_t motor_time;

extern std::atomic<int> robot_x;
extern std::atomic<int> robot_y;
extern std::atomic<int> heading;
extern std::atomic<int> vel_cnt_left; 
extern std::atomic<int> vel_cnt_right;
extern std::atomic<int> battery_voltage;
extern std::atomic<int> stall_right; 
extern std::atomic<int> stall_left;
extern std::atomic<int> control;
extern volatile char name[256], type[256], subtype[256], sn[256];
extern std::atomic<int> version;
extern std::atomic<int> subversion;

class Erratic_Base : public rclcpp::Node {
  public:
    int serial_port_fd;

    Erratic_Base() : Node("erratic_base") {
      RCLCPP_INFO(this->get_logger(), "In Erratic_Base constructor");
      std::string port = "/dev/ttyUSB0";


      // Open comm port to robot
      // Open comm port (usually /dev/ttyUSB0)
      serial_port_fd = serial_port_connection(const_cast<char*>(port.c_str()));
      if (serial_port_fd < 0) {
        RCLCPP_WARN(this->get_logger(), "COULD NOT OPEN SERIAL PORT");
        fprintf(stderr, "No serial port acquired\n");
        exit(-1);
      }

      RCLCPP_INFO(this->get_logger(), "Opened serial port fd: %d", serial_port_fd);

      // send configuration request
      send_config_packet(serial_port_fd);

      RCLCPP_INFO(this->get_logger(), "Serial port open, sent config packet");

      //Initialize subscribers
      read_incoming_thread = boost::thread(boost::bind(&Erratic_Base::read_incoming, this));
      cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Erratic_Base::cmd_vel_callback, this, std::placeholders::_1));

      //Initialize publishers

      update_timer = create_wall_timer(100ms, std::bind(&Erratic_Base::update, this));
      bat_state_pub = this->create_publisher<sensor_msgs::msg::BatteryState>("/sensors/battery", 10);
      encoder_pos_pub = this->create_publisher<geometry_msgs::msg::Point>("/sensors/odometry", 10);
      RCLCPP_INFO(this->get_logger(), "Finished constructor");
    }

  private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::TimerBase::SharedPtr update_timer;
    boost::thread read_incoming_thread; //Reads incomming data from serial port and updates variables in robot_packets.cpp
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr bat_state_pub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr encoder_pos_pub;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
      double forward = msg->linear.x * 100.0; //m/s => cm/s
      double rotation = msg->angular.z * 180.0 / 3.14159265359; //rad/s => deg/s
      RCLCPP_INFO(this->get_logger(), "I heard: '%f' @'%f'", forward, rotation);
      //robot move
      send_motor_commands(serial_port_fd, int(forward*10), int(rotation)); //cm/s => mm/s, deg/s=>deg/s
    }

    void update(){
      auto battery_msg = sensor_msgs::msg::BatteryState();
      battery_msg.voltage = ((float) battery_voltage.load()) / 10.0;
      bat_state_pub->publish(battery_msg);

      auto encoder_pos_msg = geometry_msgs::msg::Point();
      encoder_pos_msg.x = robot_x;
      encoder_pos_msg.y = robot_y;
      encoder_pos_msg.z = 0.0;
      encoder_pos_pub->publish(encoder_pos_msg);
    }

    void read_incoming(){
        RCLCPP_INFO(this->get_logger(), "Start Read incoming loop");
        // main reading loop
        for (;;) {
          char inbuffer[100];
          int i_buffer;
          
          fd_set read_set, error_set; //Sets of file descriptors that can be monitored
          FD_ZERO(&read_set); FD_ZERO(&error_set); // Empty the read and error sets
          FD_SET(serial_port_fd, &read_set); FD_SET(serial_port_fd, &error_set); // Add serial port to both sets
          select(serial_port_fd+1, &read_set, 0, &error_set, 0); //nfds is the highest-numbered file descriptor in any of the three sets, plus 1. (Timeout is null so blocks indefinetly) (Upon return, only fd's that became available are in sets)
          /* selecting was done on both data and errors on the serial port*/
          if (FD_ISSET(serial_port_fd, &read_set) | FD_ISSET(serial_port_fd, &error_set)) { //See if serial port is in active sets
            int result = read(serial_port_fd, inbuffer, 5); //result = how many characters read
          
            if (result < 0) {
              perror("Error reading from serial port");
            } else if (result == 0) {
              fprintf(stderr, "Unexpected end of file from serial port");
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
        }
    }
    
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  printf("Done Initializing, Spinning...\n");
  // Launch spin thread for 
  rclcpp::spin(std::make_shared<Erratic_Base>());
  printf("Done spinning, Shutting down...\n");
  rclcpp::shutdown();
  return 0;
}
