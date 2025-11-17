#ifndef MYMISC_H
#define MYMISC_H

void resetVector(float v[3]);
int sign(float u);
void resetMatrix(float m[3][3]);
//void sensorUpdate(unsigned int counter, long timer_old, long timer, float G_Dt);
void resetVars();

enum myUMode{
  U_OFF                     =0,
  U_FREQ                    =1,
  U_CONTROL_STATE_FEEDBACK  =2,
  U_FOURIER                 =3,
  U_PRBS                    =4,
  U_OBS                =5
};



#endif