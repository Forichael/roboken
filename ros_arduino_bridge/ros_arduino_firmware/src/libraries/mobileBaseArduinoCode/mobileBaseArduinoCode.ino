
/* Serial port baud rate 波特率*/
#define BAUDRATE     57600  

/* Maximum PWM signal 最大脉冲宽带调制*/
#define MAX_PWM        255

/* Include definition of Serial commands包括对串行命令的定义 */
#include "commands.h"

/* Sensor functions传感器的功能 */
#include "sensors.h"

/* Motor driver function definitions电机驱动程序的功能定义 */
#include "motor_driver.h"

/* Encoder driver function definitions 编码器的功能定义*/
#include "encoder_driver.h"

/* PID parameters and functions PID的参数和功能 */
#include "omniWheel_controller.h"

/* Run the PID loop at 30 times per second以每秒30次运行PID循环 */
#define PID_RATE           30     // Hz

/* Convert the rate into an interval将速率转换为一个区间 */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a  PID calculation 下次我们做一个PID计算器*/
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
/*如果多长时间没有接受到时间就让机器人停止，默认是2000  2秒 ，提高电机响应速度*/
#define AUTO_STOP_INTERVAL 200

long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Variable initialization 变量初始化*/
// A pair of varibles to help parse Serial commands (thanks Fergs)帮助解析串行命令的两种变量
int arg = 0;
int index = 0;

// Variable to hold an input character保存输入字符的变量
char chr;

// Variable to hold the current single-character command保存当前单字符命令的变量
char cmd;

// Character arrays to hold the first 、 second 、third arguments用于保存第一个到第三个轮子的字符数组，里面参数为字符
char argv1[48];
char argv2[48];
char argv3[48];

// The arguments converted to integers转换为整数的参数
long arg1 = 0;
long arg2 = 0;
long arg3 = 0;



/* Clear the current command parameters清除当前的命令参数 */
void resetCommand() {     //重置命令
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));   //把缓冲区置零
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h 运行一个命令。命令在commands.h中定义。*/
int runCommand() {     //接受从上位机传送过来的解释的命令 
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[12];   //三个轮子参数不共用，所以需要三路
  
  /*转化了整型*/
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  
  switch(cmd) {          //反馈参数给上位机
  case GET_BAUDRATE:       //获取波特率
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:           //模拟，读
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:           //数字，读
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:         //模拟，写
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:        //数字，写
    if (arg2 == 0)
      digitalWrite(arg1, LOW);
    else if (arg2 == 1) 
      digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) 
      pinMode(arg1, INPUT);    //将数字输入引脚设置为INPUT
    else if (arg2 == 1) 
      pinMode(arg1, OUTPUT);   //将数字输出引脚设置为OUTPUT
    Serial.println("OK");
    break;
  case READ_ENCODERS:   //'e',定义三个轮ABC
    Serial.print(readEncoder(A_WHEEL));
    Serial.print(" ");
    Serial.print(readEncoder(B_WHEEL));
    Serial.print(" ");
    Serial.println(readEncoder(C_WHEEL));
    break;
  case RESET_ENCODERS:    //reset编码器
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:      //电机转动速度
    /* Reset the auto stop timer重置自动停止计时器 */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0 && arg3 == 0) {
      setMotorSpeeds(0, 0,0);
      resetPID();
      moving = 0;
    }
    else 
      moving = 1;        
      
    AWheelPID.TargetTicksPerFrame = arg1;    //获取三个轮子的速度
    BWheelPID.TargetTicksPerFrame = arg2;
    CWheelPID.TargetTicksPerFrame = arg3;

    
      Serial.println(arg1);

      Serial.println(arg2);

      Serial.println(arg3);
      
    Serial.println("OK"); 
    break;
  case UPDATE_PID:   //'u'
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    /*三个轮子的校准，分别三个校准，四个一组，比较准确*/
    AWheel_Kp = pid_args[0];
    AWheel_Kd = pid_args[1];
    AWheel_Ki = pid_args[2];
    AWheel_Ko = pid_args[3];

    BWheel_Kp = pid_args[4];
    BWheel_Kd = pid_args[5];
    BWheel_Ki = pid_args[6];
    BWheel_Ko = pid_args[7];
    
    CWheel_Kp = pid_args[8];
    CWheel_Kd = pid_args[9];
    CWheel_Ki = pid_args[10];
    CWheel_Ko = pid_args[11];
    Serial.println("OK");
    break;
    /*读取两种PID输入输出情况，校准，看最后是否符合*/
    /*读取PID输入情况*/
  case READ_PIDIN:
    Serial.print(readPidIn(A_WHEEL));
    Serial.print(" ");
    Serial.print(readPidIn(B_WHEEL));
    Serial.print(" ");
    Serial.println(readPidIn(C_WHEEL));
    break;
    /*读取PID输出情况*/
  case READ_PIDOUT:
    Serial.print(readPidOut(A_WHEEL));
    Serial.print(" ");
    Serial.print(readPidOut(B_WHEEL));
    Serial.print(" ");
    Serial.println(readPidOut(C_WHEEL));
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup.Setup函数——在启动时运行一次。 */
void setup() {
  Serial.begin(BAUDRATE);
  Serial.println("init Car");
  //initEncoders(); //初始化编码器
  initMotorController();  //初始化电机控制器
  resetPID();     //重置PID
}


/* Enter the main loop.  Read and parse input from the Serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
   进入主循环。从串行端口读取和解析输入
   并运行任何有效的命令。在目标上运行一个PID计算
   间隔和检查自动停车条件。*/
void loop() {

  while (Serial.available() > 0) {          //循环读取窗口是否有上位机传送过来的命令
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = '\0';
      else if (arg == 2) argv2[index] = '\0';
      else if (arg == 3) argv3[index] = '\0';
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command使用空格来限制命令的部分
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = '\0';
        arg = 2;
        index = 0;
      }
      else if (arg == 2)
      {
        //这里要注意啊！上次少了这里，然后一直是错误的！
        argv2[index] = '\0';
        arg   = 3;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        argv3[index] = chr;         //把数据存到缓冲区里
        index++;
      }
    }
  }
// If we are using base control, run a PID calculation at the appropriate intervals
//如果我们使用基本控制，在适当的时间间隔内运行一个PID计算
//没有命令变化，参数变零
  if (millis() > nextPID) {   
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0,0,0);
    resetPID();
    moving = 0;
  }
}

