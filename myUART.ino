#include <ODriveUART.h>
#include <Wire.h>
//#include <Ticker.h>
#include <IntervalTimer.h>

#include "mymisc.h"
// //ESP32
// #define I2C_SDA 21
// #define I2C_SCL 22

//Teensy
#define I2C_SDA 18
#define I2C_SCL 19

// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

float u=0.0, u1=0,u2=0.0, u3=0.0, u4=0.0;
float e=0.0, e1=0.0, e2=0.0, e3=0.0, e4=0.0, roll1=0.0;
// Documentation for this example can be found here: https://docs.odriverobotics.com/v/latest/guides/arduino-uart-guide.html

// Set up serial pins to the ODrive
// Below are some sample configurations.
// You can comment out the default one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive ISOVDD and ISOGND to Arduino 3.3V/5V and GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
// HardwareSerial& odrive_serial = Serial1;
// int baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)

//ESP32
HardwareSerial& odrive_serial = Serial2;

//Teensy4.1 Serial2
// pin7 RX
//pin8 TX
int baudrate = 230400; // Must match what you configure on the ODrive (see docs for details)

ODriveUART odrive(odrive_serial);

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

bool is_this_ready= false;

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -2890//-3002//-2306//-6045//-2971//-2624//-883
#define M_Y_MIN -1955//-1560//-1554//-4441//-2094//-2040//+1300
#define M_Z_MIN -3009//-2990//-2605//-5873//-2962//-2852//-2415
#define M_X_MAX +760//+756//+409//+4133//+1169//+696//-745
#define M_Y_MAX +1954//+1556//+1354//+5476//+1847//+1691//+1427
#define M_Z_MAX +738//+603//+129//+4490//+984//+416//-2192

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

//For debugging purposes/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 10
#define onofftime 0.02//0.4

#define T_LED 13
//#define PI() 3.14159265358979323846264
float G_Dt=0.01;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll=0;
float pitch=0;
float yaw=0;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
//byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

float AMPLITUD=0.5, PERIODO=0.4;
//
IntervalTimer timeri;

IntervalTimer TLyapunov;
float vel_tope=0;

bool flip = false;
bool switch_torque_mode = true;// 1 torque 0 velocidad

const float torque_lim=1.5;
const float vel_lim=30;

const unsigned long interval = 10;  // Intervalo de muestreo en milisegundos <------ min T= 4
const unsigned long intervaltimer = interval; 

unsigned long previousMillis = 0;
float currentMillis=0;

float t=0;
float tref=0;
float tseconds=0;
float trefsegundos=0;

int sat_counter=0;
int unsat_counter=0;
int signu=0;

bool pasoxcero= false; //Se activa cuando la bici pasa por cero para activar el control
bool isthemotoron=false;

//double K[]={-0.0006689369, -142.9209, -27.55614};
double K[]={-0.0005566705, -118.4083, -18.84987};
double uref=0;
int p_counter=0;
const int PRBS[]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1};

float AMP[]={0,0,0,0};
float PER[]={0,0,0,0};

float roll_offset = ToRad(-3.5); // valor inicial en radianes

enum myUMode controlMode=U_OFF;//U_OFF;

// struct OutputLog{
//   float phi;
//   float dphi;
//   float az;
//   float u;
//   float omega;
//   float torque_SP;
//   float torque_est;
// };
// struct myLog{
//   float a;
//   float b;

// };
float arr[]={0, 0, 0, 0, 0};

// const int BUFFER_SIZE=255;

float V[]={0.0, 0.0, 0.0, 0.0, 0.0};
float Vprom=0;
//float Vsorted[]={0.0, 0.0, 0.0, 0.0, 0.0};

// const double Aobs[4][4]={{    0.8199617, -0.2479938, -0.02177908,      0.8354496},
//                         {-0.0003640041,    1.00124, -0.02721174, -0.00001246759},
//                         {-0.0007582793,  0.2481058,   0.8349798,   -0.002493657},
//                         {  -0.00958802,          0, -0.01021598,            1.0}};

const double Aobs[4][4]={{     0.3701959, 0.000004326545,  0.003356073,      0.8354496},
                        {0.000004478455,        0.67032, -4.809751e-8, -0.00001246759},
                        {   0.001160211,    -1.48681e-8,    0.7557733,   -0.002493657},
                        {    -0.1166772, 0.000001684042,  0.001175949,            1.0}};
                        
 

// const double Bobs[4][3]={{     0.8354496,    0.1791327, 0.02053918},
//                           {-0.00001246759, 0.0003640176, 0.03721587},
//                           {  -0.002493657, 0.0007609824,  0.1662605},
//                           {             0,   0.00958802, 0.01021598}};

const double Bobs[4][4]={{     0.8354496,      0.6288984,      -0.2479982, -0.004595973},
                          {-0.00001246759, -0.00000446494,       0.3309202,   0.01000418},
                          {  -0.002493657,   -0.001157508,       0.2481059,     0.245467},
                          {             0,      0.1166772, -0.000001684042, -0.001175949}};

float xo[]={0.0, 0.0, 0.0, 0.0};
float xoa[]={0.0, 0.0, 0.0, 0.0};

float xoe[]={0.0,0.0};
float xoea[]={0.0,0.0};

const float Aoe[2][2]={{ 0.276851, 0.882845},
                      {-0.146933,      1.0}};

const float Boe[2][2]={{0.882845, 0.722923},
                      {       0, 0.146933}}; 

float torque_estimateOD=0.0, u_OD=0.0;

float Vfilt=0.0, Vfilt1=0.0;

void Iglesias(){
   digitalWrite(T_LED,flip);
   flip = !flip;
}

void timerCallback(){
  digitalWrite(STATUS_LED,HIGH);
  t=t+intervaltimer;
  tseconds=t/1000.0;

  //tiempo para la referencia 
  tref=tref+intervaltimer;
  trefsegundos=tref/1000.0;
    
  currentMillis=t;

  ODriveFeedback feedback = odrive.getFeedback();

  for(int n=0; n<=4; n++)
  {
    arr[n]=V[n];
  }
  
  for (int i=0; i<4; i++){
    for (int j=i+1;j<5;j++){
      if(arr[j]<arr[i]){
        float temporal=arr[i];
        arr[i]=arr[j];
        arr[j]=temporal;
      }
    }
  }
  Vprom=arr[2];

  //Vprom=(V[0]+V[1]+V[2]+V[3])/4;

if((millis()-timer)>=10)  // Main loop runs at 50Hz
  {
    sensorUpdate(counter, timer_old, timer, G_Dt);
  }

  e=roll;
  //
  

  //u=-(-K[0]*feedback.vel*2*PI +K[1]*roll+K[2]*(Gyro_Vector[0]));   
  //u= 0.2*sin(2*PI/0.4*trefsegundos);
  switch(controlMode){
    case U_OFF:{
      u=0;
      break;
    }
    case U_FREQ:{
      if(PERIODO==0){
      u= AMPLITUD;
      }else{
      u= AMPLITUD * sin(2*PI/PERIODO*trefsegundos);
      }
      break;
    }
    case U_CONTROL_STATE_FEEDBACK:{
      //u=-(K[0]*feedback.vel*2*PI +K[1]*roll+K[2]*(Gyro_Vector[0]));
      u=-(K[0]*feedback.vel*2*PI +K[1]*roll+K[2]*xo[1]);
      //u=-(K[0]*feedback.vel*2*PI +K[1]*roll+K[2]*(Vfilt));
      break;
    }
    case U_FOURIER:{
      u=AMP[0]* sin(2*PI/PER[0]*trefsegundos)+AMP[1]* sin(2*PI/PER[1]*trefsegundos)+AMP[2]* sin(2*PI/PER[2]*trefsegundos)+AMP[3]* sin(2*PI/PER[3]*trefsegundos);
      break;
    }
    case U_PRBS:{
      if (p_counter<1024){
      u=0.2*PRBS[p_counter];
      }
      else if (p_counter>=1024){
        u=-0.2*PRBS[p_counter-1024];
      }
      p_counter++;
      if (p_counter==2048){p_counter=0;}
      //Serial.println("PRBS");
      break;
    }
    case U_LYAPUNOV:{
      u=0.01*sign(u-u1)*p_counter+u1;
      p_counter++;
      if (feedback.vel==vel_tope){p_counter=0;}
      //Serial.println("PRBS");
      break;
    }
    default:{
      u=0;
      Serial.println("Default Control Mode");
      break;
    }      
  }

  if(!pasoxcero){//u definition MUST BE BEFORE this conditional
    if(sign(roll)!=sign(roll1))
      if(abs(roll)>ToRad(0.0002))
      {pasoxcero=true;
      //controlMode= U_FREQ;
      }
    u=0;
    tref=0;
    //u=0.1;
  }

 // Vfilt= Vfilt1*0.8464817+Gyro_Vector[0]*0.1535183; Original
  Vfilt= Vfilt1*0.7777778+Gyro_Vector[0]*0.1111111;

  if(switch_torque_mode){
    if(u>torque_lim){u=torque_lim;}if(u<-torque_lim){u=-torque_lim;}//limite para que no se desborde u
    odrive.setTorque(u);//u//u+uref
  }else{
    if(u>vel_lim){u=vel_lim;}if(u<-vel_lim){u=-vel_lim;}//limite para que no se desborde u
    odrive.setVelocity(u);
  }
  
  
  //Serial.print(sign(pasoxcero));
  //Serial.print(", ");
  
//OBSERVADOR
  // xo[0]=Aobs[0][0]*xoa[0] + Aobs[0][1]*xoa[1] +Aobs[0][2]*xoa[2] + Aobs[0][3]*xoa[3]  +  Bobs[0][0]*torque_estimateOD + Bobs[0][1]*feedback.vel*2*PI + Bobs[0][2]*roll;
  // xo[1]=Aobs[1][0]*xoa[0] + Aobs[1][1]*xoa[1] +Aobs[1][2]*xoa[2] + Aobs[1][3]*xoa[3]  +  Bobs[1][0]*torque_estimateOD + Bobs[1][1]*feedback.vel*2*PI + Bobs[1][2]*roll;
  // xo[2]=Aobs[2][0]*xoa[0] + Aobs[2][1]*xoa[1] +Aobs[2][2]*xoa[2] + Aobs[2][3]*xoa[3]  +  Bobs[2][0]*torque_estimateOD + Bobs[2][1]*feedback.vel*2*PI + Bobs[2][2]*roll;
  // xo[3]=Aobs[3][0]*xoa[0] + Aobs[3][1]*xoa[1] +Aobs[3][2]*xoa[2] + Aobs[3][3]*xoa[3]  +  Bobs[3][0]*torque_estimateOD + Bobs[3][1]*feedback.vel*2*PI + Bobs[3][2]*roll;

  xo[0]=Aobs[0][0]*xoa[0] + Aobs[0][1]*xoa[1] +Aobs[0][2]*xoa[2] + Aobs[0][3]*xoa[3]  +  Bobs[0][0]*torque_estimateOD + Bobs[0][1]*feedback.vel*2*PI + Bobs[0][2]*Gyro_Vector[0]+Bobs[0][3]*roll;
  xo[1]=Aobs[1][0]*xoa[0] + Aobs[1][1]*xoa[1] +Aobs[1][2]*xoa[2] + Aobs[1][3]*xoa[3]  +  Bobs[1][0]*torque_estimateOD + Bobs[1][1]*feedback.vel*2*PI + Bobs[1][2]*Gyro_Vector[0]+Bobs[1][3]*roll;
  xo[2]=Aobs[2][0]*xoa[0] + Aobs[2][1]*xoa[1] +Aobs[2][2]*xoa[2] + Aobs[2][3]*xoa[3]  +  Bobs[2][0]*torque_estimateOD + Bobs[2][1]*feedback.vel*2*PI + Bobs[2][2]*Gyro_Vector[0]+Bobs[2][3]*roll;
  xo[3]=Aobs[3][0]*xoa[0] + Aobs[3][1]*xoa[1] +Aobs[3][2]*xoa[2] + Aobs[3][3]*xoa[3]  +  Bobs[3][0]*torque_estimateOD + Bobs[3][1]*feedback.vel*2*PI + Bobs[3][2]*Gyro_Vector[0]+Bobs[3][3]*roll;


  Serial.print(feedback.vel);
  Serial.print(", ");
  Serial.print(xo[3]);
  Serial.print(", ");

  if (odrive.getState() == AXIS_STATE_CLOSED_LOOP_CONTROL) {
  //   //Serial.print(feedback.pos);
  //   //Serial.print(", ");
    
    torque_estimateOD=odrive.getParameterAsFloat("axis0.controller.effective_torque_setpoint");
    // Serial.print(torque_estimateOD);
    // Serial.print(", ");
    Serial.print(odrive.getParameterAsFloat("axis0.motor.torque_estimate"));
    Serial.print(", ");
  }else {
    //Serial.print(0); Serial.print(", ");
    //Serial.print(0); Serial.print(", ");
    Serial.print(0); Serial.print(", ");
  }

  //Serial.print(xo[0]/(2*PI));
  Serial.print(u);
  Serial.print(", ");
  printdata();
  //Serial.print(V[0]);Serial.print(", ");Serial.print(V[1]);Serial.print(", ");Serial.print(V[2]);Serial.print(", ");Serial.print(V[3]);Serial.print(", ");Serial.print(V[4]);Serial.print(", ");
  Serial.print(ToDeg(xo[1]));
  //Serial.print()
  // Serial.print(Serial.availableForWrite());
  // //Serial.print(pasoxcero); Serial.print(", ");
  // Serial.println();
  // //u=odrive.getParameterAsFloat("axis0.controller.effective_torque_setpoint");
  Serial.println();
  
  if(pasoxcero){
    u4=u3;
    u3=u2;
    u2=u1;
    u1=u;
    e4=e3;
    e3=e2;
    e2=e1;
    e1=e;
  }
  roll1=roll;
  Vfilt1=Vfilt;
  // V[4]=V[3];
  // V[3]=V[2];
  // V[2]=V[1];
  // V[1]=V[0];
  // V[0]=Gyro_Vector[0];
  
  for (int n=0; n<4; n++){
    xoa[n]=xo[n];
  }

  
  
  digitalWrite(STATUS_LED,LOW);
}

void resetSensor(){
  is_this_ready=false;
      pasoxcero=false;
      u=0;
      //timeri.detach();
      timeri.end();
      odrive.setState(AXIS_STATE_IDLE);
      //delay(500);
      //resetMatrix(DCM_Matrix);
      resetMatrix(Temporary_Matrix);
      resetVector(Accel_Vector);
      resetVector(Gyro_Vector); 
      resetVector(Omega_Vector);
      resetVector(Omega_P);
      resetVector(Omega_I);
      resetVector(Omega);
      resetVector(errorRollPitch);
      resetVector(errorYaw);
      roll=0; pitch=0; yaw=0;
      resetVars();
}

void setup() {
  
  Serial.begin(230400); // Serial to PC
  I2C_Init();
  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  setupVars();
  //resetVars();
  TLyapunov.begin(Iglesias,1000000);
}

void loop() {

  //  A PARTIR DE AQUÍ: SIEMPRE ESCUCHA EL SERIAL
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    //Serial.println(input);

    if (input.startsWith("OFFSET=")) {
      float newOffsetDeg = input.substring(7).toFloat();
      roll_offset = ToRad(newOffsetDeg);
      u=0;
      resetSensor();
      // for (int n=0; n<=4; n++){
      //   xoe[n]=0;
      // }
    }else if(input.startsWith("M=")) {
      if(isthemotoron){
        u=0;
        odrive.setState(AXIS_STATE_IDLE);
        controlMode=U_OFF;
      }else{
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      }
      isthemotoron=!isthemotoron;
    }else if (input.startsWith("K=")) {
      input = input.substring(2);
      int idx1 = input.indexOf(',');
      int idx2 = input.indexOf(',', idx1 + 1);

      if (idx1 != -1 && idx2 != -1) {
        K[0] = input.substring(0, idx1).toFloat();
        K[1] = input.substring(idx1 + 1, idx2).toFloat();
        K[2] = input.substring(idx2 + 1).toFloat();
        controlMode=U_CONTROL_STATE_FEEDBACK;
        pasoxcero= false;
        //resetSensor();
      } else {
        Serial.println("Error en el formato de K. Usa: K=x,y,z");
      }
    }else if(input.startsWith("U=")) {
      input = input.substring(2);
      int idx1 = input.indexOf(',');
      if (idx1!=-1){
        //u=0;
        //delay(2500);
        //odrive.setState(AXIS_STATE_IDLE);

        
        AMPLITUD= input.substring(0, idx1).toFloat();
        PERIODO = input.substring(idx1 + 1).toFloat();
        controlMode=U_FREQ;
        //odrive.clearErrors();
        //xoe[0]=0; xoe[1]=0;
        //odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        //delay(10);
      }
    }else if(input.startsWith("P=")) {
      if(pasoxcero){
        pasoxcero=false;
        odrive.setState(AXIS_STATE_IDLE);
        isthemotoron=false;
      }else{
        pasoxcero=true;
        tref=0;
      }
    /////////////////////
    }else if(input.startsWith("F=")){
      input = input.substring(2);
      int idx[7];
      int previdx=-1;
      int i=0;
      for (i=0; i<7; i++){
        idx[i]=input.indexOf(',',previdx+1);
        previdx=idx[i];
      }

      bool check=true;
      
      for (i=0; i<7; i++){
        check= check && (idx[i])!=-1;//revisa que todo idx!=-1
      }
      if(check){
        AMP[0]=input.substring(0,idx[0]).toFloat();
        AMP[1]=input.substring(idx[0]+1,idx[1]).toFloat();
        AMP[2]=input.substring(idx[1]+1,idx[2]).toFloat();
        AMP[3]=input.substring(idx[2]+1,idx[3]).toFloat();
        PER[0]=input.substring(idx[3]+1,idx[4]).toFloat();
        PER[1]=input.substring(idx[4]+1,idx[5]).toFloat();
        PER[2]=input.substring(idx[5]+1,idx[6]).toFloat();
        PER[3]=input.substring(idx[6]+1).toFloat();
        
      }
        controlMode=U_FOURIER;
        //resetSensor();
      }
      else if(input.startsWith("PRBS=")){
        p_counter=0;
        controlMode=U_PRBS;
      }
      else if(input.startsWith("L=")){
        p_counter=0;
        controlMode=U_LYAPUNOV;
        vel_tope= input.substring(2).toFloat();
      }
      // else if(input.startsWith("VT")) {
      //   u=0;
      //   odrive.setState(AXIS_STATE_IDLE);
      //   controlMode=U_OFF;

      //   delay(500);
      //   switch_torque_mode= !switch_torque_mode;//modo torque

      //   odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      // }
  //*/
  }

  if (!is_this_ready) {
    //Serial.println(odrive.getState());
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
      delay(100);
      is_this_ready = true;
    }

    // while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    //   odrive.clearErrors();
    //   odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    //   delay(10);
    // }
  }
}
