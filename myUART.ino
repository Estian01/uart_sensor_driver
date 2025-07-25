#include <ODriveUART.h>
#include <Wire.h>
#include <Ticker.h>

#define I2C_SDA 21
#define I2C_SCL 22
// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

#define VDD_sensor_pin 19

float u=0.0, u1=0,u2=0.0, u3=0.0, u4=0.0;
float e=0.0, e1=0.0, e2=0.0, e3=0.0, e4=0.0, roll1=0.0;
// Documentation for this example can be found here: https://docs.odriverobotics.com/v/latest/guides/arduino-uart-guide.html

// Set up serial pins to the ODrive
// Below are some sample configurations.
// You can comment out the default one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive ISOVDD and ISOGND to Arduino 3.3V/5V and GND.

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
//SoftwareSerial odrive_serial(8, 9);onof
//unsigned long baudrate = 19200; // Must match what you configure on the ODrive (see docs for details)

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
// HardwareSerial& odrive_serial = Serial1;
// int baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)

//ESP32
HardwareSerial& odrive_serial = Serial2;
int baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)

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

#define STATUS_LED 13
#define onofftime 0.02//0.4
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

void resetVector(float v[3]) {
    for (int i = 0; i < 3; ++i)
        v[i] = 0.0f;
}

//
Ticker timeri;
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

//#define ROLL_OFFSET ToRad(-3)//-2.78  //Initial roll offset in deg/radians


bool pasoxcero= false; //Se activa cuando la bici pasa por cero para activar el control

double K[]={-0.001862943, -155.4314, -33.66148};
double uref=0;

 
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


float roll_offset = ToRad(-3.4); //2.78 valor inicial en radianes

void timerCallback(){
  t=t+intervaltimer;
  tseconds=t/1000.0;

  //tiempo para la referencia 
  tref=tref+intervaltimer;
  trefsegundos=tref/1000.0;
    
  currentMillis=t;

  ODriveFeedback feedback = odrive.getFeedback();

if((millis()-timer)>=10)  // Main loop runs at 50Hz
  {
    digitalWrite(STATUS_LED,HIGH);
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
    {
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 100 ms   //200 ms
    }
    else
      G_Dt = 0;

    // * DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    
    digitalWrite(STATUS_LED,LOW);
  }

  e=0.0-roll;

  //u=43.503246932384755041*e1 - 21.95437663765436298*e - 21.512803502947924983*e2 + 1.8535052807369114536*u1 - 0.83265300708364831106*u2;

  //u=-(-K[0]*feedback.vel*2*3.14159265358979323846264 +K[1]*roll+K[2]*(Gyro_Vector[0]));   
   u=211.45722526770180139*e1 - 107.16805330958501941*e - 104.28926993053794092*e2 + 1.9703962490402568974*u1 - 0.97039319502697318764*u2;
  
  //u=0.06875;//0.0765625;
  if (feedback.vel>=7.6||feedback.vel<=-7.6){
    sat_counter++;
  }else{
    sat_counter=0;
  }
  if (sat_counter>=5){
    unsat_counter=10;
  }
  //lyapunov
  //u=((26.6*0.365*9.81*sin(roll)-5*(roll-roll1)/0.2-100*abs((e-e1)/0.2+e)))/16.64;

  
/*
  if (unsat_counter>0){
    if(u>0){signu=1;}
    else if (u<0){signu=-1;}
    else if (u==0){signu=0;}
    u=-signu*1;
    unsat_counter--;
  }
  */


 /* 
 if(trefsegundos>=0 && trefsegundos<onofftime){
    uref=1.2; 
    }
    if(trefsegundos>=onofftime && trefsegundos<2*onofftime){
    uref=0; 
    }
    if(trefsegundos>=2*onofftime && trefsegundos<3*onofftime){
    uref=-1.2;
    }
    if(trefsegundos>=3*onofftime && trefsegundos<4*onofftime){
    uref=-0; 
    }
    if(trefsegundos>=4*onofftime){
    tref=0;
    }
    
  u=uref;
  */

  //onoff control
  /*
  if(roll>ToRad(0.06)){u=1;}
  else if(roll<ToRad(-0.06)){u=-1;}
  else{u=0;}*/
  

  if(!pasoxcero){//u definition MUST BE BEFORE this conditional
    //if(sign(roll)!=sign(roll1))
    if(roll>ToRad(0.02))
      {pasoxcero=true;}
    u=0;
    tref=0;
    //u=0.1;
  }

  if(u>1.2){u=1.2;}if(u<-1.2){u=-1.2;}//limite para que no se desborde u
  odrive.setTorque(u);//u//u+uref
  //odrive.setTorque(roll*1.6/0.5235987756);
  
  //Serial.print(sign(pasoxcero));
  //Serial.print(", ");
  printdata();
  Serial.print(u);
  Serial.print(", ");
  
  

  if (odrive.getState() == AXIS_STATE_CLOSED_LOOP_CONTROL) {
    //Serial.print(feedback.pos);
    //Serial.print(", ");
    Serial.print(feedback.vel);
    Serial.print(", ");
    Serial.print(odrive.getParameterAsFloat("axis0.controller.effective_torque_setpoint"));
    Serial.print(", ");
    Serial.print(odrive.getParameterAsString("axis0.motor.torque_estimate"));
    Serial.print(", ");
  }else {
    Serial.print(0); Serial.print(", ");
    Serial.print(0); Serial.print(", ");
    Serial.print(0); Serial.print(", ");
  }
  Serial.println();
  //u=odrive.getParameterAsFloat("axis0.controller.effective_torque_setpoint");
  //Serial.println();
  
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
}

void resetVars(){
  delay(10);

  //Serial.println("The device started, now you can pair it with bluetooth!");

  pinMode (STATUS_LED,OUTPUT);  // Status LED
  pinMode (VDD_sensor_pin,OUTPUT);
  digitalWrite(VDD_sensor_pin,HIGH);

  digitalWrite(STATUS_LED,LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5]*cos(roll_offset);
  AN_OFFSET[4]-=GRAVITY*SENSOR_SIGN[5]*sin(roll_offset);

  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  counter=0;

  odrive_serial.begin(baudrate,SERIAL_8N1,16,17);

  Serial.println("Waiting for ODrive...");
  
  //timer
   timeri.attach_ms(intervaltimer, timerCallback);

}

void resetSensor(){
  is_this_ready=false;
      pasoxcero=false;
      u=0;
      timeri.detach();
      odrive.setState(AXIS_STATE_IDLE);
      delay(500);
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
  
  Serial.begin(115200); // Serial to PC
  I2C_Init();
  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  resetVars();
}

// void loop() {
//   if (is_this_ready==false){
//       while (odrive.getState() == AXIS_STATE_UNDEFINED) {
//       delay(100);
//       //Serial.println("found ODrive");
//     //Serial.print("DC voltage: ");
//     //Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

//     is_this_ready=true;

//     }

//     while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
//       odrive.clearErrors();
//       odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
//       delay(10);
//     }

//   }

//     if (Serial.available()) {
//     String input = Serial.readStringUntil('\n');
//     input.trim();

//     if (input.startsWith("OFFSET=")) {
//       float newOffsetDeg = input.substring(7).toFloat();
//       roll_offset = ToRad(newOffsetDeg);

//     }

//     if (input.startsWith("K=")) {
//       // Formato: K=-0.01,-123,-45.6
//       input = input.substring(2); // quitar "K="
//       int idx1 = input.indexOf(',');
//       int idx2 = input.indexOf(',', idx1 + 1);

//       if (idx1 != -1 && idx2 != -1) {
//         K[0] = input.substring(0, idx1).toFloat();
//         K[1] = input.substring(idx1 + 1, idx2).toFloat();
//         K[2] = input.substring(idx2 + 1).toFloat();

//       }
//     }
//   }
// }

void loop() {

  //  A PARTIR DE AQUÍ: SIEMPRE ESCUCHA EL SERIAL
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("OFFSET=")) {
      float newOffsetDeg = input.substring(7).toFloat();
      roll_offset = ToRad(newOffsetDeg);

      resetSensor();
    }

    if (input.startsWith("K=")) {
      input = input.substring(2);
      int idx1 = input.indexOf(',');
      int idx2 = input.indexOf(',', idx1 + 1);

      if (idx1 != -1 && idx2 != -1) {
        K[0] = input.substring(0, idx1).toFloat();
        K[1] = input.substring(idx1 + 1, idx2).toFloat();
        K[2] = input.substring(idx2 + 1).toFloat();

        resetSensor();

      } else {
        Serial.println("Error en el formato de K. Usa: K=x,y,z");
      }
    }
  }

  if (!is_this_ready) {
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
      delay(100);
      is_this_ready = true;
    }

    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrive.clearErrors();
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      delay(10);
    }
  }
}

