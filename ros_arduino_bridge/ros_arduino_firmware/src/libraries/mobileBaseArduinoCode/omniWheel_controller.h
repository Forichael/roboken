/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   该文件是控制各全向轮的PID控制头文件
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo  AWheelPID, BWheelPID,CWheelPID;   //三个轮子的PID

/* PID Parameters PID参数*/
int AWheel_Kp = 15;
int AWheel_Kd = 28;
int AWheel_Ki = 0;
int AWheel_Ko = 50;

int BWheel_Kp = 15;
int BWheel_Kd = 28;
int BWheel_Ki = 0;
int BWheel_Ko = 50;

int CWheel_Kp = 15;
int CWheel_Kd = 28;
int CWheel_Ki = 0;
int CWheel_Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both encoder and PrevEnc the current encoder value
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){               //PID参数重置
   AWheelPID.TargetTicksPerFrame = 0.0;
   AWheelPID.encoder = readEncoder(A_WHEEL);
   AWheelPID.PrevEnc = AWheelPID.encoder;
   AWheelPID.output = 0;
   AWheelPID.PrevInput = 0;
   AWheelPID.ITerm = 0;

   BWheelPID.TargetTicksPerFrame = 0.0;
   BWheelPID.encoder = readEncoder(B_WHEEL);
   BWheelPID.PrevEnc = BWheelPID.encoder;
   BWheelPID.output = 0;
   BWheelPID.PrevInput = 0;
   BWheelPID.ITerm = 0;

   CWheelPID.TargetTicksPerFrame = 0.0;
   CWheelPID.encoder = readEncoder(C_WHEEL);
   CWheelPID.PrevEnc = CWheelPID.encoder;
   CWheelPID.output = 0;
   CWheelPID.PrevInput = 0;
   CWheelPID.ITerm = 0;
}

/* PID routine to compute the next motor commands 用于计算下一个电机命令的PID例程*/
void doAWheelPID(SetPointInfo * p) {          //A轮PID校准
  long Perror = 0;
  long output = 0;
  int input = 0;

  p->encoder = readEncoder(A_WHEEL);
  input = p->encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
  * Avoid derivative kick and allow tuning changes,
  * 避免使用派生的启动和允许调优更改
  */
  //output = (AWheel_Kp * Perror + AWheel_Kd * (Perror - p->PrevErr) + AWheel_Ki * p->Ierror) / AWheel_Ko;
  // p->PrevErr = Perror;
  output = (AWheel_Kp * Perror - AWheel_Kd * (input - p->PrevInput) + p->ITerm) / AWheel_Ko;
  p->PrevEnc = p->encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes
  */
    p->ITerm += AWheel_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* PID routine to compute the next motor commands */
void doBWheelPID(SetPointInfo * p) {          //B轮PID校准
  long Perror = 0;
  long output = 0;
  int input = 0;

  p->encoder = readEncoder(B_WHEEL);
  input = p->encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
  * Avoid derivative kick and allow tuning changes,
  */
  //output = (BWheel_Kp * Perror + BWheel_Kd * (Perror - p->PrevErr) + BWheel_Ki * p->Ierror) / BWheel_Ko;
  // p->PrevErr = Perror;
  output = (BWheel_Kp * Perror - BWheel_Kd * (input - p->PrevInput) + p->ITerm) / BWheel_Ko;
  p->PrevEnc = p->encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes
  */
    p->ITerm += BWheel_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* PID routine to compute the next motor commands */
void doCWheelPID(SetPointInfo * p) {          //C轮PID校准
  long Perror = 0;
  long output = 0;
  int input = 0;

  p->encoder = readEncoder(C_WHEEL);
  input = p->encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
  * Avoid derivative kick and allow tuning changes,
  */
  //output = (CWheel_Kp * Perror + CWheel_Kd * (Perror - p->PrevErr) + CWheel_Ki * p->Ierror) / CWheel_Ko;
  // p->PrevErr = Perror;
  output = (CWheel_Kp * Perror - CWheel_Kd * (input - p->PrevInput) + p->ITerm) / CWheel_Ko;
  p->PrevEnc = p->encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += CWheel_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  AWheelPID.encoder = readEncoder(A_WHEEL);
  BWheelPID.encoder = readEncoder(B_WHEEL);
  CWheelPID.encoder = readEncoder(C_WHEEL);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    if (AWheelPID.PrevInput != 0 || BWheelPID.PrevInput != 0|| CWheelPID.PrevInput != 0)  resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doAWheelPID(&AWheelPID);
  doBWheelPID(&BWheelPID);
  doCWheelPID(&CWheelPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(AWheelPID.output, BWheelPID.output, CWheelPID.output);
}
/*pid校准两函数*/
long readPidIn(int wheel)
{
  long pidin = 0;
  if(wheel == A_WHEEL)
  {
    pidin = AWheelPID.PrevInput;
}else if(wheel == B_WHEEL)
{
  pidin = BWheelPID.PrevInput;
}else
{
  pidin = CWheelPID.PrevInput; 
}
return pidin;
}
long readPidOut(int wheel)
{
  long pidout = 0;
  if(wheel == A_WHEEL)
  {
    pidout = AWheelPID.output;
}
else if(wheel == B_WHEEL)
{
  pidout= BWheelPID.output;
}
else
{
  pidout = CWheelPID.output; 
}
return pidout;
}

