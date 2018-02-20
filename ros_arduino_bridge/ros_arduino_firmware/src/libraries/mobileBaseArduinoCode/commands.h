/* Define single-letter commands that will be sent by the PC over the
   serial link.定义由PC发送的单字母命令
   串行连接
   该文件定义了上位机与移动底盘通信的所有命令
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define READ_PIDIN     'i'
#define READ_PIDOUT    'o'

#define A_WHEEL        1
#define B_WHEEL        2
#define C_WHEEL        3

#define FORWARDS     true        //轮子正转
#define BACKWARDS   false        //轮子反转
#endif


