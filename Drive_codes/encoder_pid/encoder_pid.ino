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

long geregepin=8;

KangarooSerial  kang2(Serial2);

KangarooSerial  kang1(Serial3);


KangarooChannel J1(kang2, '1');

KangarooChannel J2(kang2, '2');

KangarooChannel K1(kang1, '1');

KangarooChannel K2(kang1, '2');


Encoder myEnc(18, 19);

long kp1 = 30;
long ki1 = 0.5;
long kd1 = 10000;

long sum1 = 0;
long diff1 = 0;
float elapsed_time1 = 0;
float output1 = 0;
long prev1 = 0;
float deltaT1;
long setpoint_1;

long state = 0;

long j1_fact = -1, k1_fact = 1, j2_fact = -1, k2_fact = 1;   //for controlling the direction of wheel rotation

long base_speed = 0;

long max_speed = 3500;

long max_speedM = 3500;

long accM = 250;

long acc = 250;

long dif_1;

static float setpoint = 0.0;
float y;

boolean check1=false;

sensors_event_t event;


#define BNO055_SAMPLERATE_DELAY_MS (100);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float current_time, elapsed_time, deltaT;


#define kang_ramp 3000

#define kang_deramp 6000



float x, dif;

float kp = 100;

float ki = 0.01 ;

float kd = 6000;

float output = 0;

float sum = 0;

float diff = 0;

float prev_err = 0;

float err;

long a = 45;

long distance = 0;

long d = 14255;

long temp = 0;
long temp1 = 0;

long dist = 700;

long dist1 = 700;

//long max_speed1 = 1500;

//long min_speed1 = 200;

long acc1 = 300;
long acc2 = 200;

long ang_speed = 4000;
long ang_speed1 = 4000;

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

long throwing=9;

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
  setpoint_1 = 12000;
Serial.println("setup complete");
}




void loop() {
  
while(abs(myEnc.read())<=12000 && check== false) {
  sensors_event_t event;
  bno.getEvent(&event);
  bno.getEvent(&event);
  x = event.orientation.x;
  dif = x - setpoint;
  
 long* p4 = straight(4000);
 long* p5 = straight_1(pid1(dif_1));
 long* p2 = rotate(pid(e_val(dif))) ;
 long* v;
  v = add_vels(p4,p2);  // straight + rotate
  write_motor_speeds(v);
}
 write_motor_speeds_stop(stop_motors()); 
delay(5000);

check = true;

//sensors_event_t event;
//bno.getEvent(&event);
//bno.getEvent(&event);
//x = event.orientation.x;
//dif = x - setpoint;
//y = abs(myEnc.read());
//dif_1 = setpoint_1 - y;
//if (abs(dif_1) <= 500) {
//  elapsed_time=millis();
//  Serial.print("\ndifference...................................");
//  Serial.println(dif_1);
//  float tempe=pid1(dif_1);
//  Serial.println(tempe);
//  Serial.print("Encoder reading..........");
//  Serial.println(myEnc.read());
// long* p5 = straight(tempe);
// long* p2 = rotate(pid(e_val(dif))) ;
// long* u;
//  u = add_vels(p5,p2);  // straight + rotate
//  write_motor_speeds(u);
//  
//}
//else{
//  elapsed_time=millis();
//    
// long* p5 = straight(pid1(dif_1));
// long* p2 = rotate(pid(e_val(dif))) ;
// long* u;
//  u = add_vels(p5,p2);  // straight + rotate
//   write_motor_speeds(u);
//  Serial.print("\ndifference...................................");
//  Serial.println(dif_1);
//  Serial.println(pid1(dif_1));
//  Serial.print("Encoder reading..........");
//  Serial.println(myEnc.read());
//}

//long* p0 = stop_motors();
// long* p2 = rotate(pid(e_val(dif))) ;
// long* u;
//  u = add_vels(p0,p2);  // straight + rotate
//  write_motor_speeds(u);
}

long* rotate(long v) {

  long static vels[4];

  vels[0] = -v;

  vels[1] = v;

  vels[2] = v;

  vels[3] = -v;

  return vels;

}


long* manual_rotate(long v) {

  long static vels[4];

  vels[0] = -v;

  vels[1] = v;

  vels[2] = v;

  vels[3] = -v;

  return vels;

}


long* straight(long v) {
  Serial.print("\nthe uotput for straight:::::::::::::::::"); 
  Serial.print(v);

  //Serial.print("in straight  ");

  //Serial.println(v);

  long static vels[4];

  vels[0] = v;

  vels[1] = v;

  vels[2] = v;

  vels[3] = v;

  return vels;

}
long* straight_1(long v) {

  //Serial.print("in straight  ");

  //Serial.println(v);

  long static vels[4];

  vels[0] = v;

  vels[1] = v;

  vels[2] = v;

  vels[3] = v;

  return vels;

}



long* right(long v) {

  long static vels[4];

  vels[0] = -v;

  vels[1] = v;

  vels[2] = -v;

  vels[3] = v;

  return vels;

}


long* move_angl(long v, float ang) {

  long static vels[4];

  ang = ang * 3.14 / 180;

  vels[0] = v * (cos(ang) - sin(ang));

  vels[1] = v * (cos(ang) + sin(ang));

  vels[2] = v * (cos(ang) - sin(ang));

  vels[3] = v * (cos(ang) + sin(ang));

  return vels;

}



long* move_angr(long v, float ang) {

  long static vels[4];

  ang = ang * 3.14 / 180;

  vels[0] = v * (cos(ang) - sin(ang));

  vels[1] = v * (cos(ang) + sin(ang));

  vels[2] = v * (cos(ang) - sin(ang));

  vels[3] = v * (cos(ang) + sin(ang));

  return vels;

}




void write_motor_speeds(long* p) {

  J1.s(j1_fact * (*p), kang_ramp);

//  Serial.print("hi----------------------------------------------------------");
//  Serial.println(j1_fact * (*p));

  J2.s(j2_fact * (*(p + 1)), kang_ramp);

  K1.s(k1_fact * (*(p + 2)), kang_ramp);

  K2.s(k2_fact * (*(p + 3)), kang_ramp);

}



long* add_vels(long* p1,long* p2) {

  long static vels[4];

  vels[0] = *p1 + *p2;

  vels[1] = *(p1 + 1) + *(p2 + 1);

  vels[2] = *(p1 + 2) + *(p2 + 2);

  vels[3] = *(p1 + 3) + *(p2 + 3);

  return vels;

}

long* add_3vels(long* p1,long* p2,long* p3) {

  long static vels[4];

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


long* stop_motors() {

  long static vels[4];

  vels[0] = 0;

  vels[1] = 0;

  vels[2] = 0;

  vels[3] = 0;

  return vels;

}


long* reverse(long v) {

  long static vels[4];

  vels[0] = -v;

  vels[1] = -v;

  vels[2] = -v;

  vels[3] = -v;

  return vels;

}

float pid1(float(dif1)) {
if (abs(dif1) <= 4000) {
  deltaT1 = (millis() - elapsed_time1);
  elapsed_time1 = millis();
  sum1 = sum1 + dif1 * deltaT1;
  diff1 = (dif1 - prev1) / deltaT1;
  prev1 = dif1;
  output1 = float(kp1*dif1 + ki1*sum1 + kd1*diff1);
}
else {
  output1 = 0 ;
}
  //  Serial.print("dt:  ");

  //  Serial.print(deltaT);

  //  Serial.print("  sum:  ");

  //  Serial.print(sum);

  //  Serial.print("  diff:  ");

  //  Serial.print(diff);

  //  Serial.print("  output:  ");

  //  Serial.println(output);

  return (output1);

}

void write_motor_speeds_stop(long* p) {

  J1.s(j1_fact * (*p), 20000);

//  Serial.print("hi----------------------------------------------------------");
//  Serial.println(j1_fact * (*p));

  J2.s(j2_fact * (*(p + 1)), 20000);

  K1.s(k1_fact * (*(p + 2)), 20000);

  K2.s(k2_fact * (*(p + 3)), 20000);

}
