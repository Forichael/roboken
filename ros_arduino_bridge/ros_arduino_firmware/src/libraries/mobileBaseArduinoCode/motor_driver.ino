/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   该文件为电机驱动的实现文件，包括初始化控制器，设置各电机的转速等
   *************************************************************/
//motor pin define
const int A_IN1 = 23;
const int A_IN2 = 25;
const int A_PWM = 11;    //A wheel pwm pin

const int B_IN1 = 27;
const int B_IN2 = 29;
const int B_PWM = 10;    //B wheel pwm pin

const int C_IN1 = 31;
const int C_IN2 = 33;
const int C_PWM = 9;    //C wheel pwm pin
//三个轮子的方向
boolean directA = true;
boolean directB = true;
boolean directC = true;

boolean directionWheel(int wheel)
{
  if(wheel == A_WHEEL)
  {
    return directA;
  }
  else if(wheel == B_WHEEL)
  {
    return directB;
  }
  else
  {
    return directC;
  }
}
  /* Wrap the motor driver initialization */
  /*初始化电机控制器的设置，引脚都设置为OUTPUT*/
void initMotorController() {
  pinMode(A_IN1,OUTPUT);
  pinMode(A_IN2,OUTPUT);
  pinMode(A_PWM,OUTPUT);

  pinMode(B_IN1,OUTPUT);
  pinMode(B_IN2,OUTPUT);
  pinMode(B_PWM,OUTPUT);

  pinMode(C_IN1,OUTPUT);
  pinMode(C_IN2,OUTPUT);
  pinMode(C_PWM,OUTPUT);
  }

  /* Wrap the drive motor set speed function */
  /*设置电机的转速,两个参数wheel哪个轮子，spd转速*/
 void setMotorSpeed(int wheel, int spd) {
   if(spd > MAX_PWM)   //如果转速超过最大值，则调整
   {
    spd = MAX_PWM;
   }
   
   if(spd < -MAX_PWM)    //如果反向超过最大值，也调整
   {
    spd = -1 * MAX_PWM;
   }
   
 if (wheel == A_WHEEL)       //如果轮子是A轮
   {
    if(spd >= 0)     //转速是正数
    {
      directA = FORWARDS;    //正转
      digitalWrite(A_IN1,HIGH);    //正转，一引脚为高位，二引脚为低位
      digitalWrite(A_IN2,LOW);
      analogWrite(A_PWM,spd);    //调整
    }
    else if(spd <0)   //反转
    {
      directA = BACKWARDS;
      digitalWrite(A_IN1,LOW);     //一低二高
      digitalWrite(A_IN2,HIGH);
      analogWrite(A_PWM,-spd);
    }
   } 
else if (wheel == B_WHEEL)       //如果轮子是B轮
   {
    if(spd >= 0)     //转速是正数
    {
      directB = FORWARDS;    //正转
      digitalWrite(B_IN1,HIGH);    //正转，一引脚为高位，二引脚为低位
      digitalWrite(B_IN2,LOW);
      analogWrite(B_PWM,spd);    //调整
    }
    else if(spd <0)   //反转
    {
      directB = BACKWARDS;
      digitalWrite(B_IN1,LOW);     //一低二高
      digitalWrite(B_IN2,HIGH);
      analogWrite(B_PWM,-spd);
    }
   }
   
else       //如果轮子是C轮
   {
    if(spd >= 0)     //转速是正数
    {
      directC = FORWARDS;    //正转
      digitalWrite(C_IN1,HIGH);    //正转，一引脚为高位，二引脚为低位
      digitalWrite(C_IN2,LOW);
      analogWrite(C_PWM,spd);    //调整
    }
    else if(spd <0)   //反转
    {
      directC = BACKWARDS;
      digitalWrite(C_IN1,LOW);     //一低二高
      digitalWrite(C_IN2,HIGH);
      analogWrite(C_PWM,-spd);
    } 
  }
 }
 
// A convenience function for setting both motor speeds
void setMotorSpeeds(int ASpeed,int BSpeed, int CSpeed) {
    setMotorSpeed(A_WHEEL,ASpeed);
    setMotorSpeed(B_WHEEL,BSpeed);
    setMotorSpeed(C_WHEEL,CSpeed);
}

