/**************************************************************
   Encoder definitions
   Encoder A: connect interrupt 0, 1--[pin 2, 3];
   Encoder B: connect interrupt 2, 3--[pin 21, 20];
   Encoder C: connect intterupt 4, 5--[pin 19, 18]
   ************************************************************ */

#include<Encoder.h>

Encoder A_Wheel_encoder(ENC_A_PIN_A,ENC_A_PIN_B);
Encoder B_Wheel_encoder(ENC_B_PIN_A,ENC_B_PIN_B);
Encoder C_Wheel_encoder(ENC_C_PIN_A,ENC_C_PIN_B);

long A_Wheel_Position;
long B_Wheel_Position;
long C_Wheel_Position;

/* Wrap the encoder reading function */
long readEncoder(int i) 
{
  if (i == A_WHEEL)
  {
    return A_Wheel_encoder.read()-A_Wheel_Position;
  }
  else if (i == B_WHEEL)
  {
    return B_Wheel_encoder.read()-B_Wheel_Position;
  }
  else
  {
    return C_Wheel_encoder.read()-C_Wheel_Position;
  }
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == A_WHEEL)
  {
    A_Wheel_Position = A_Wheel_encoder.read();
  }
  else if (i == B_WHEEL)
  {
    B_Wheel_Position = B_Wheel_encoder.read();
  }
  else if (i == C_WHEEL)
  {
    C_Wheel_Position = C_Wheel_encoder.read();
  }
}
void resetEncoders()
{
  
  resetEncoder(A_WHEEL);
  resetEncoder(B_WHEEL);
  resetEncoder(C_WHEEL);
  
}

