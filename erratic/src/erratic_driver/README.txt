================================================================
Test program
Uses low-level packet interface through the controller comm port

*****
***** NOTE: if you use the motor control commands, make sure there is
*****  no tether to the robot, or put the robot on blocks so the 
*****  wheels do not touch the floor.
*****

================================================================

This is a simple testing client that is easily expandable to send any
packet. 

To invoke, connect the PC to the USB port of the controller, and use:
  
  ./robot_client <comm port>

The <comm port> argument is 

    /dev/erratic or /dev/ttyUSBn  for Linux
    comN			  for MS Windows


NOTE: for MS Windows, the baud rate must be set using the Device
Manager.  Set the baud rate for the COM port to 38400.

If the controller is alive, the program will print a confirming
message, and also some information about the robot (type, subtype,
name, serial number, battery voltage).

Single-character commands send command packets to the controller.  The
character must be followed by <CR> to take effect.

Commands:

   R - turns on raw packet printing.  The bytes of all packets sent
       from the controller are printed. 
   r - turn off raw packet printing

   z - print a summary of the robot state, once

   A - turn on analog packet printing
   a - turn off analog packet printing

   S - turn on sonar packet printing
   s - turn off sonar packet printing

   i - increment forward velocity
   m - decrement forward velocity
   j - increment left turn velocity
   l - decrement left turn velocity
   space, k - stop motors

   w, W - increment (decrement) servo 0
   x, X - increment (decrement) servo 1
   y, Y - increment (decrement) servo 2

   b, B - increment (decrement) drift bias
