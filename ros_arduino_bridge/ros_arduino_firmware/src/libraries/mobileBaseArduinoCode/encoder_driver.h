/* *************************************************************
   Encoder driver function definitions - by James Nugen
   该文件定义了各直流电机的编码器与Arduino Mega2560连接引脚，
   同时也定义了读取和重置各编码器计数的函数
   ************************************************************ */
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  //AB编码器连接引脚
  //A wheel encode pin
  #define ENC_A_PIN_A  2  //pin 2-----interrupt 0
  #define ENC_A_PIN_B  3  //pin 3-----interrupt 1

  //B wheel encode pin
  #define ENC_B_PIN_A  21  //pin 21-----interrupt 2
  #define ENC_B_PIN_B  20  //pin 20-----interrupt 3

  //C wheel encode pin
  #define ENC_C_PIN_A  19  //pin 19-----interrupt 4
  #define ENC_C_PIN_B  18  //pin 18-----interrupt 5

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

