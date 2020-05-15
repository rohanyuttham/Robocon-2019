#include <PID_v1.h>

#include <hcsr04.h>


#include <Kangaroo.h>


#include <Encoder.h>


//--------- Libraries for IMU --------------


#include <Adafruit_Sensor.h>


#include <Adafruit_BNO055.h>


#include <utility/imumaths.h>


////----------Libraries for PS4 --------

#include <PS4BT.h>

#include <usbhub.h>

#ifdef dobogusinclude

#include <spi4teensy3.h>

#endif

#include <SPI.h>

#include <Wire.h>


USB Usb;

BTD Btd(&Usb);

PS4BT PS4(&Btd, PAIR);
//PS4BT PS4(&Btd);

int geregepin=8;

KangarooSerial  kang2(Serial2);

KangarooSerial  kang1(Serial3);


KangarooChannel J1(kang2, '1');

KangarooChannel J2(kang2, '2');

KangarooChannel K1(kang1, '1');

KangarooChannel K2(kang1, '2');


Encoder myEnc(18, 19);

double Setpoint_1, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=20, aggKi=0.05, aggKd=5;
double consKp=4, consKi=0.005, consKd=1;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint_1, consKp, consKi, consKd, REVERSE);



int state = 0;

int j1_fact = -1, k1_fact = 1, j2_fact = -1, k2_fact = 1;   //for controlling the direction of wheel rotation

int base_speed = 0;

int max_speed = 3500;

int max_speedM = 3500;

int accM = 250;

int acc = 250;

int dif_1;

static float setpoint = 0.0;
float y;

boolean check1=false;

sensors_event_t event;


#define BNO055_SAMPLERATE_DELAY_MS (100);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float current_time, elapsed_time, deltaT;


#define kang_ramp 10000

#define kang_deramp 6000

int kp1 = 50;
int kd1 = 8000;

float x, dif;

float kp = 100;

float ki = 0.01 ;

float kd = 6000;

float output = 0;

float sum = 0;

float diff = 0;

float prev_err = 0;

float err;

int a = 45;

long distance = 0;

long d = 14255;

int temp = 0;
int temp1 = 0;

int dist = 700;

int dist1 = 700;

//int max_speed1 = 1500;

//int min_speed1 = 200;

int acc1 = 300;
int acc2 = 200;

int ang_speed = 4000;
int ang_speed1 = 4000;

float angle_l = -55.0;

float angle_r = 55.0;


boolean stop_motor = false;

boolean check = false;

#define TRIG_PIN 35

#define ECHO_PIN 37

#define TRIG_PINr 31

#define ECHO_PINr 33


HCSR04 hcsr04l(TRIG_PIN, ECHO_PIN, 20, 4000);

HCSR04 hcsr04r(TRIG_PINr, ECHO_PINr, 20, 4000);

int throwing=9;

void setup() {

  Serial.begin(115200);
  pinMode(geregepin,OUTPUT);
  pinMode(throwing,OUTPUT);
  Serial.println("setup started");

#if !defined(__MIPSEL__)

  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection

#endif

  if (Usb.Init() == -1) {

    Serial.print(F("\r\nOSC did not start"));

    while (1); // Halt

  }

  Serial.println(F("\r\nPS4 USB Library Started"));


  Serial.println("Orientation Sensor Test"); Serial.println("");

  Serial2.begin(115200);

  Serial3.begin(115200);

  Serial.println("Serial communications tested");
  if (!bno.begin())

  {

    /* There was a problem detecting the BNO055 ... check your connections */

    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");

  }

  J1.start();

  J2.start();

  K1.start();

  K2.start();


  Serial.println("motors started");

  bno.setExtCrystalUse(true);

  bno.getEvent(&event);

  delay(100);


  bno.getEvent(&event);


  setpoint = event.orientation.x;

  Setpoint_1 = 12000;
  myPID.SetMode(AUTOMATIC);
   //myPID.SetTunings(aggKp, aggKi, aggKd);
Serial.println("setup complete");
}




void loop() {
sensors_event_t event;
bno.getEvent(&event);
bno.getEvent(&event);
x = event.orientation.x;
dif = x - setpoint;

  Input = abs(myEnc.read());
  double gap = (Setpoint_1 - Input); //distance away from setpoint
  if (abs(gap) < 500)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
//  while(abs(myEnc.read())<=12000) {
//  sensors_event_t event;
//  bno.getEvent(&event);
//  bno.getEvent(&event);
//  x = event.orientation.x;
//  dif = x - setpoint;
//  
//  int* p4 = straight(2000);
//  int* p5 = straight_1(pid1(dif_1));
//  int* p2 = rotate(pid(e_val(dif))) ;
//  int* v;
//  v = add_vels(p4,p2);  // straight + rotate
//  //write_motor_speeds(v);
//}
  myPID.Compute();
  delay(100);
  int* p4 = straight(Output*10);
  int* p2 = rotate(pid(e_val(dif))); 
  int* v;
  v = add_vels(p4,p2);
  //write_motor_speeds(v);
  Serial.println(Output);
  Serial.print("Encoder reading..........");
  Serial.println(Input);
}

int* rotate(int v) {

  int static vels[4];

  vels[0] = -v;

  vels[1] = v;

  vels[2] = v;

  vels[3] = -v;

  return vels;

}


int* manual_rotate(int v) {

  int static vels[4];

  vels[0] = -v;

  vels[1] = v;

  vels[2] = v;

  vels[3] = -v;

  return vels;

}


int* straight(int v) {

  //Serial.print("in straight  ");

  //Serial.println(v);

  int static vels[4];

  vels[0] = v;

  vels[1] = v;

  vels[2] = v;

  vels[3] = v;

  return vels;

}
int* straight_1(int v) {

  //Serial.print("in straight  ");

  //Serial.println(v);

  int static vels[4];

  vels[0] = v;

  vels[1] = v;

  vels[2] = v;

  vels[3] = v;

  return vels;

}



int* right(int v) {

  int static vels[4];

  vels[0] = -v;

  vels[1] = v;

  vels[2] = -v;

  vels[3] = v;

  return vels;

}


int* move_angl(long v, float ang) {

  int static vels[4];

  ang = ang * 3.14 / 180;

  vels[0] = v * (cos(ang) - sin(ang));

  vels[1] = v * (cos(ang) + sin(ang));

  vels[2] = v * (cos(ang) - sin(ang));

  vels[3] = v * (cos(ang) + sin(ang));

  return vels;

}



int* move_angr(long v, float ang) {

  int static vels[4];

  ang = ang * 3.14 / 180;

  vels[0] = v * (cos(ang) - sin(ang));

  vels[1] = v * (cos(ang) + sin(ang));

  vels[2] = v * (cos(ang) - sin(ang));

  vels[3] = v * (cos(ang) + sin(ang));

  return vels;

}




void write_motor_speeds(int* p) {

  J1.s(j1_fact * (*p), kang_ramp);

//  Serial.print("hi----------------------------------------------------------");
//  Serial.println(j1_fact * (*p));

  J2.s(j2_fact * (*(p + 1)), kang_ramp);

  K1.s(k1_fact * (*(p + 2)), kang_ramp);

  K2.s(k2_fact * (*(p + 3)), kang_ramp);

}



int* add_vels(int* p1, int* p2) {

  int static vels[4];

  vels[0] = *p1 + *p2;

  vels[1] = *(p1 + 1) + *(p2 + 1);

  vels[2] = *(p1 + 2) + *(p2 + 2);

  vels[3] = *(p1 + 3) + *(p2 + 3);

  return vels;

}

int* add_3vels(int* p1, int* p2,int* p3) {

  int static vels[4];

  vels[0] = *p1 + *p2 + *p3;

  vels[1] = *(p1 + 1) + *(p2 + 1) + *(p3 + 1);

  vels[2] = *(p1 + 2) + *(p2 + 2) + *(p3 + 2);

  vels[3] = *(p1 + 3) + *(p2 + 3) + *(p3 + 3);

  return vels;

}



float e_val(float(dif)) {                                   // x = dif

  if (setpoint <= 360 && setpoint > 355 ) {

    if (dif == 0) {

      return float(dif);

    }

    else if (dif < 0 && dif > -180) {

      return float((-1) * dif);

    }

    else if (dif < -180 && dif > -360) {

      return float((-1) * (360 + dif));

    }

  }

  else {

    if (dif == 0) {

      return float(dif);

    }

    else if (dif > 0 && dif < 180) {

      return float(-dif);

    }

    else if (dif > 180 && dif < 360) {

      return float((360 - dif));

    }

  }

}



float pid(float(dif)) {

  deltaT = (millis() - elapsed_time);

  elapsed_time = millis();

  sum = sum + dif * deltaT;

  diff = (dif - prev_err) / deltaT;

  prev_err = dif;

  output = (kp * dif + ki * sum + kd * diff);

  //  Serial.print("dt:  ");

  //  Serial.print(deltaT);

  //  Serial.print("  sum:  ");

  //  Serial.print(sum);

  //  Serial.print("  diff:  ");

  //  Serial.print(diff);

  //  Serial.print("  output:  ");

  //  Serial.println(output);

  return (output);

}


int* stop_motors() {

  int static vels[4];

  vels[0] = 0;

  vels[1] = 0;

  vels[2] = 0;

  vels[3] = 0;

  return vels;

}


int* reverse(int v) {

  int static vels[4];

  vels[0] = -v;

  vels[1] = -v;

  vels[2] = -v;

  vels[3] = -v;

  return vels;

}

int pid1(int(dif)) {
//if (abs(dif) <= 4000) {
  deltaT = (millis() - elapsed_time);
  elapsed_time = millis();
  sum = sum + dif * deltaT;
  diff = (dif - prev_err) / deltaT;
  prev_err = dif;
  output = (kp1 * dif + kd1 * diff)/50;
//}
//else {
//  output = 0 ;
//}
  //  Serial.print("dt:  ");

  //  Serial.print(deltaT);

  //  Serial.print("  sum:  ");

  //  Serial.print(sum);

  //  Serial.print("  diff:  ");

  //  Serial.print(diff);

  //  Serial.print("  output:  ");

  //  Serial.println(output);

  return (output);

}
