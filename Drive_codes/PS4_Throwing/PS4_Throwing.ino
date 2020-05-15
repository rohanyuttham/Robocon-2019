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

KangarooSerial  kang2(Serial2);
KangarooSerial  kang1(Serial3);

KangarooChannel J1(kang1, '1');
KangarooChannel J2(kang1, '2');
KangarooChannel K1(kang2, '1');
KangarooChannel K2(kang2, '2');

Encoder myEnc(18, 19);
#define geregepin 8
#define throwing 9



int state = 0;
int j1_fact = -1, k1_fact = 1, j2_fact = -1, k2_fact = 1;   //for controlling the direction of wheel rotation
int base_speed = 0;
int max_speed = 2500;
int max_speedM = 3500;
int accM = 250;
int acc = 150;
float setpoint = 0.0;

sensors_event_t event;

#define BNO055_SAMPLERATE_DELAY_MS (100);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float current_time, elapsed_time, deltaT;
#define kang_ramp 1500
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
int a = 45;
long distance = 0;
long d = 14255;
int temp = 0;
int dist = 700;
int dist1 = 700;
//int max_speed1 = 1500;
//int min_speed1 = 200;
int acc1 = 100;
int ang_speed = 5000;

boolean stop_motor = false;
boolean check = false;

#define TRIG_PIN 35
#define ECHO_PIN 37
#define TRIG_PINr 31
#define ECHO_PINr 33

HCSR04 hcsr04l(TRIG_PIN, ECHO_PIN, 20, 4000);
HCSR04 hcsr04r(TRIG_PINr, ECHO_PINr, 20, 4000);

void setup() {
  Serial.begin(115200);
#if !defined(_MIPSEL_)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 USB Library Started"));

  Serial.println("Orientation Sensor Test"); Serial.println("");
  Serial2.begin(115200);
  Serial3.begin(115200);

  pinMode(geregepin,OUTPUT);
  pinMode(throwing, OUTPUT);

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  J1.start();
  J2.start();
  K1.start();
  K2.start();



  bno.setExtCrystalUse(true);
  bno.getEvent(&event);
  delay(100);

  bno.getEvent(&event);

  setpoint = event.orientation.x;

}



void loop() {
  Usb.Task();
  if (PS4.connected()) {

    if (PS4.getButtonClick(CIRCLE)) {
      sensors_event_t event;
      bno.getEvent(&event);
      x = event.orientation.x;
      dif = x - setpoint;
      if (e_val(dif) < 1.0 && e_val(dif) > -1.0) {
        write_motor_speeds(stop_motors());
      }
      else {
        int* p2 = rotate(pid(e_val(dif))) ;
        int* p3 = stop_motors();
        int v1[4];
        int* v;
        v = add_vels(p3, p2);
        write_motor_speeds(v);

      }
    }



    base_speed = constrain(base_speed + acc, 0, 2000); //max and min value of speed

    int psw = (PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2));
    if (psw != 0) {
      bno.getEvent(&event);

      setpoint = event.orientation.x;

      int* p3 = manual_rotate(base_speed * psw);
      write_motor_speeds(p3);
    }

    int ps3y = PS4.getAnalogHat(RightHatX) - 127;
    //    int ax = map(x,0,117,max_speed,0);
    //    int bx = map(x,138,255,0,max_speed);
    int ps3x = 127 - PS4.getAnalogHat(LeftHatY);
    //    int ay = map(x,0,117,max_speed,0);
    //    int by = map(x,138,255,0,max_speed);

    sensors_event_t event;
    bno.getEvent(&event);

    bno.getEvent(&event);
    x = event.orientation.x;
    dif = x - setpoint;

    int r = base_speed * sqrt(ps3x * ps3x + ps3y * ps3y) / 127;
    float theta = 180 * atan(ps3y / ps3x) / 3.14;
    if (ps3x < 0) theta = theta + 180;
    if (abs(r) < 50) r = 0;

    int* p4 = move_angr(r, theta);
    int* p2 = rotate(pid(e_val(dif))) ;
    int v1[4];
    int* v;
    v = add_vels(p4, p2);  // straight + rotate
    write_motor_speeds(v);
   
  

  
    
  


      if (PS4.getButtonClick(CROSS)){
        if (digitalRead(geregepin)==HIGH) {
        digitalWrite(geregepin,LOW);
        Serial.println("gerege - LOW");
      }      else{
        digitalWrite(geregepin,HIGH);
        Serial.println("gerege - HIGH");
      }
      }

      if (PS4.getButtonClick(SQUARE)){
        if (digitalRead(throwing)== HIGH){
          digitalWrite(throwing,LOW);
          Serial.println("throwing - LOW");
      }        else{
          digitalWrite(throwing,HIGH);
          Serial.println("throwing - HIGH");
      }
  }
  }
  
      
    



else {
  Serial.println("PS4 not connected");
  }

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
  //Serial.println(j1_fact*(*p));
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
