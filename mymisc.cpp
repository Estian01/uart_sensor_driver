#include "mymisc.h"
#include <Arduino.h>


void resetVector(float v[3]) {
    for (int i = 0; i < 3; ++i)
        v[i] = 0.0f;
}

int sign(float u){
  if (u>0){return 1;}
  else if(u<0){return -1;}
  else if(u==0){return 0;}
}

void resetMatrix(float m[3][3]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m[i][j] = 0.0;
}