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

int geregepin=5;

KangarooSerial  kang2(Serial2);

KangarooSerial  kang1(Serial3);


KangarooChannel J1(kang2, '1');

KangarooChannel J2(kang2, '2');

KangarooChannel K1(kang1, '1');

KangarooChannel K2(kang1, '2');


Encoder myEnc(18, 19);


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
boolean check = true;
boolean check0 = false;
sensors_event_t event;


#define BNO055_SAMPLERATE_DELAY_MS (100);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float current_time, elapsed_time, deltaT;


#define kang_ramp 10000

#define kang_deramp 10000


float x, dif;

float kp = 100;

float ki = 0.01 ;

float kd = 6100;

float kp1 = 100;

float ki1 = 0.01 ;

float kd1 = 6000;

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

int acc1 = 100;
int acc2 = 200;
int ch =0;
int ang_speed = 2000;
int ang_speed1 = 4000;

float angle_l = -55.0;

float angle_r = 55.0;

int i;
int h_speed = 2000;

boolean stop_motor = false;


#define TRIG_PIN 35

#define ECHO_PIN 37

#define TRIG_PINr 31

#define ECHO_PINr 33


HCSR04 hcsr04l(TRIG_PIN, ECHO_PIN, 20, 4000);

HCSR04 hcsr04r(TRIG_PINr, ECHO_PINr, 20, 4000);

int throwing=6;
int picking = 4;

void setup() {

  Serial.begin(115200);
  pinMode(geregepin,OUTPUT);
  pinMode(throwing,OUTPUT);
  pinMode(picking,OUTPUT);
  digitalWrite(geregepin,LOW);
  digitalWrite(throwing,LOW);
  digitalWrite(picking,LOW);
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

Serial.println("setup complete");
}




void loop() {
  phase_1();
  phase_2();
  rest(2000);
//  phase_3();
//  phase_4();

//  Usb.Task();
//  if (PS4.connected()) {
//    
//while (check == true) {
//  //Usb.Task();
//    khagai();
//    rest(500);
//    check = false;
//    check0 = true;
//}
//while (check0 == true) {
//  Usb.Task();
//    bridge();
//    ch =1;
//        //***********************************************************************************************************
//}
//    Usb.Task();
//    manual();
//    
//}
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
int* left(int v) {

  int static vels[4];

  vels[0] = v;

  vels[1] = -v;

  vels[2] = v;

  vels[3] = -v;

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

float pid1(float(dif)) {
if (abs(dif) <= 300) {
  deltaT = (millis() - elapsed_time);
  elapsed_time = millis();
  sum = sum + dif * deltaT;
  diff = (dif - prev_err) / deltaT;
  prev_err = dif;
  output = (kp1 * dif + ki1 * sum + kd1 * diff);
}
else {
  output = 0 ;
}
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

int rest(int t) {
  sensors_event_t event;
    bno.getEvent(&event);
    bno.getEvent(&event);
    x = event.orientation.x;
    dif = x - setpoint;
            
            int* p0 = stop_motors();
            int* p2 = rotate(pid(e_val(dif))) ;
            unsigned long initt = millis();
            unsigned long currt = millis();

while(currt - initt < t){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p0 = stop_motors();
            int* p2 = rotate(pid(e_val(dif))) ;

            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p0, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }
}

void phase_1() {
    sensors_event_t event;
    bno.getEvent(&event);
    bno.getEvent(&event);
    x = event.orientation.x;
    dif = x - setpoint;
            
            unsigned long initt = millis();
            unsigned long currt = millis();
            while(currt - initt < 1250){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(7000);
            int* p0 = stop_motors();
            int* p4 = move_angr(10000, 50);
            int* p2 = rotate(pid(e_val(dif))) ;

            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p4, p2);  // straight + rotate
            write_motor_speeds_lowRamp(v);
            currt = millis(); 
              
            }
            
            rest(150);
            initt = millis();
            currt = millis();
            while(currt - initt < 495){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(7000);
            int* p0 = stop_motors();
            int* p4 = move_angl(10000, 50);
            int* p2 = rotate(pid(e_val(dif))) ;

            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p1, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }

            rest(150);
            }
void phase_2() {
   sensors_event_t event;
    bno.getEvent(&event);
    bno.getEvent(&event);
    x = event.orientation.x;
    dif = x - setpoint;
            
            unsigned long initt = millis();
            unsigned long currt = millis();
            while(currt - initt < 930){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(6000);
            int* p0 = stop_motors();
            int* p3 = move_angr(10000, -115);
            int* p2 = rotate(pid(e_val(dif))) ;

            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p3, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }
            
            rest(150);
            initt = millis();
            currt = millis();
            while(currt - initt < 650){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(6000);
            int* p0 = stop_motors();
            int* p3 = move_angr(10000, 115);
            int* p2 = rotate(pid(e_val(dif))) ;


            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p1, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }

            rest(150);
}
void phase_3() {
  sensors_event_t event;
    bno.getEvent(&event);
    bno.getEvent(&event);
    x = event.orientation.x;
    dif = x - setpoint;
            
            unsigned long initt = millis();
            unsigned long currt = millis();
            while(currt - initt < 740){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(6000);
            int* p0 = stop_motors();
            int* p3 = move_angl(10000, 110);
            int* p2 = rotate(pid(e_val(dif))) ;

            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p3, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }
            
            rest(150);
            initt = millis();
            currt = millis();
            while(currt - initt < 550){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(7000);
            int* p0 = stop_motors();
            int* p3 = move_angl(1000, -60);
            int* p2 = rotate(pid(e_val(dif))) ;



            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p1, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }

            rest(150);
}
void phase_4() {
            unsigned long initt = millis();
            unsigned long currt = millis();
            while(currt - initt < 470){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(6000);
            int* p0 = stop_motors();
            int* p3 = move_angr(6000, -60);
            int* p2 = rotate(pid(e_val(dif))) ;

            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p3, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }
            rest(750);
}
void phase_5(){
  unsigned long initt = millis();
            unsigned long currt = millis();
            while(currt - initt < 1300){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(6000);
            int* p0 = stop_motors();
            int* p3 = move_angr(6000, 60);
            int* p2 = rotate(pid(e_val(dif))) ;

            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p1, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }
            rest(1000);
}
void phase_6() {
  unsigned long initt = millis();
            unsigned long currt = millis();
            while(currt - initt < 2000){
                  sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;
            int* p1 = straight(6000);
            int* p0 = stop_motors();
            int* p3 = move_angl(7000, -85);
            int* p2 = rotate(pid(e_val(dif))) ;

            int* v;
            int* u;
            int* w;
            int* z;
            v = add_vels(p3, p2);  // straight + rotate
            write_motor_speeds(v);
            currt = millis(); 
              
            }
            rest(300);
}



void manual() {


    while (PS4.getButtonClick(CIRCLE)) {

      sensors_event_t event;

      bno.getEvent(&event);

      x = event.orientation.x;

      dif = x - setpoint;
        write_motor_speeds(stop_motors());
        Serial.println(*stop_motors());

    }




    base_speed = constrain(base_speed + acc, 0, 2000); //max and min value of speed


    int psw = PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2);
    int multfac=map(psw, -255, 255, -2000, 2000);
    int randvar=0;

    while ((psw >20 || psw<-20)&&check1==false) {
      Usb.Task();
      
       PS4.setLed(Red);
      int* p6 = manual_rotate(multfac);
      Serial.println(multfac);
      write_motor_speeds(p6);
      sensors_event_t event;
      bno.getEvent(&event);   
      setpoint = event.orientation.x;
      //Serial.println(setpoint);
      psw = PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2);
      randvar=1;
    }
    if (randvar==1){
        sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
       setpoint = event.orientation.x;
        randvar=0;
    }
        

//    if(check1==true){
//       write_motor_speeds(stop_motors());
//       check1=true;
//    }
    
 PS4.setLed(Blue);
 if (PS4.getAnalogHat(RightHatX) > 137 || PS4.getAnalogHat(RightHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117) {

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


    int r = base_speed * sqrt(ps3x * ps3x + ps3y * ps3y) /100 ;

    float theta = 180 * atan(ps3y / ps3x) / 3.14;

    if (ps3x < 0) theta = theta + 180;

    if (abs(r) < 20) r = 0;


    int* p4 = move_angr(r, theta);

    int* p2 = rotate(pid(e_val(dif))) ;

    int v1[4];

    int* v;

    v = add_vels(p4, p2);  // straight + rotate

    write_motor_speeds(p4);
 }
 if (PS4.getAnalogHat(RightHatX) < 137 && PS4.getAnalogHat(RightHatX) > 117 && PS4.getAnalogHat(LeftHatY) < 137 && PS4.getAnalogHat(LeftHatY) > 117) {
  write_motor_speeds(stop_motors()
  );
 }


    if (PS4.getButtonClick(CROSS)){
        if (digitalRead(geregepin)==HIGH) {
        digitalWrite(geregepin,LOW);
        //Serial.println("gerege - LOW");
      }      else{
        digitalWrite(geregepin,HIGH);
        //Serial.println("gerege - HIGH");
      }
      }

      if (PS4.getButtonClick(SQUARE)){
        if (digitalRead(throwing)== HIGH){
          digitalWrite(throwing,LOW);
          //Serial.println("throwing - LOW");
      }        else{
          digitalWrite(throwing,HIGH);
          //Serial.println("throwing - HIGH");
      }
  }
  if (PS4.getButtonClick(UP)){
        if (digitalRead(picking)== HIGH){
          digitalWrite(picking,LOW);
          //Serial.println("throwing - LOW");
      }        else{
          digitalWrite(picking,HIGH);
          //Serial.println("throwing - HIGH");
      }
  }
}

void bridge() {
 // Usb.Task();
    base_speed = constrain(base_speed + acc, 0, 2000); //max and min value of speed


    int psw = PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2);
    int multfac=map(psw, -255, 255, -2000, 2000);
    int randvar=0;

    while ((psw >20 || psw<-20)&&check1==false) {
      Usb.Task();
      
       PS4.setLed(Red);
      int* p6 = manual_rotate(multfac);
      Serial.println(multfac);
      write_motor_speeds(p6);
      sensors_event_t event;
      bno.getEvent(&event);   
      setpoint = event.orientation.x;
      //Serial.println(setpoint);
      psw = PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2);
      randvar=1;
    }
    if (randvar==1){
        sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
       setpoint = event.orientation.x;
        randvar=0;
    }
        

//    if(check1==true){
//       write_motor_speeds(stop_motors());
//       check1=true;
//    }
    
 
  if (PS4.getAnalogHat(RightHatX) > 137 || PS4.getAnalogHat(RightHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117) {

    int ps3y = PS4.getAnalogHat(RightHatX) - 127;
    int ps3x = 127 - PS4.getAnalogHat(LeftHatY);

    sensors_event_t event;

    bno.getEvent(&event);


    bno.getEvent(&event);

    x = event.orientation.x;

    dif = x - setpoint;


    int r = base_speed * sqrt(ps3x * ps3x + ps3y * ps3y) / 100;
    

    float theta = 180 * atan(ps3y / ps3x) / 3.14;

    if (ps3x < 0) theta = theta + 180;

    if (abs(r) < 100) r = 0;


    int* p4 = move_angr(r, theta);

//    int* p2 = rotate(pid(e_val(dif))) ;
//
//    int v1[4];
//
//    int* v;
//
//    v = add_vels(p4, p2);  // straight + rotate

    write_motor_speeds(p4);
 }
 if (PS4.getAnalogHat(RightHatX) < 137 && PS4.getAnalogHat(RightHatX) > 117 && PS4.getAnalogHat(LeftHatY) < 137 && PS4.getAnalogHat(LeftHatY) > 117) {
  write_motor_speeds(stop_motors());
 }
 if (PS4.getButtonClick(TRIANGLE)) {
  phase_5();
  phase_6();
  rest(300);
  //manual();
 }
}
void khagai() {
  //if (PS4.getButtonClick(TRIANGLE)) {
    phase_1();
    phase_2();
    phase_3();
    phase_4();
  //}
}

void write_motor_speeds_lowRamp(int* p) {

  J1.s(j1_fact * (*p), 5000);

//  Serial.print("hi----------------------------------------------------------");
//  Serial.println(j1_fact * (*p));

  J2.s(j2_fact * (*(p + 1)), 5000);

  K1.s(k1_fact * (*(p + 2)), 5000);

  K2.s(k2_fact * (*(p + 3)), 5000);

}
