/***************************************************************
   Motor driver function definitions - by James Nugen
   电机驱动函数定义
   *************************************************************/
void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int ASpeed, int BSpeed,int CSpeed);
boolean directionWheel(int wheel);
