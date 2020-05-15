#include <hcsr04.h>
#include <Kangaroo.h>
#include <Encoder.h>
//--------- Libraries for PS4 --------------
#include <PS4BT.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <usbhub.h>
#include <SPI.h>
#include <Wire.h>
//--------- Libraries for IMU --------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

KangarooSerial  kang2(Serial2);
KangarooSerial  kang1(Serial3);
KangarooChannel J1(kang1, '1');
KangarooChannel J2(kang1, '2');
KangarooChannel K1(kang2, '1');
KangarooChannel K2(kang2, '2');

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd, PAIR);


bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

Encoder myEnc(18, 19);
int state = 0;
int j1_fact = -1, k1_fact = 1, j2_fact = -1, k2_fact = 1;   //for controlling the direction of wheel rotation
int base_speed = 0;
int max_speed = 2000;
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

int dist = 550;

int max_speed1 = 1500;
int min_speed1 = 200;
int acc1 = 100;
int ang_speed = 5000;

boolean check = false;

#define TRIG_PIN 35
#define ECHO_PIN 37
#define TRIG_PINr 31
#define ECHO_PINr 33
HCSR04 hcsr04l(TRIG_PIN, ECHO_PIN, 20, 4000);
HCSR04 hcsr04r(TRIG_PINr, ECHO_PINr, 20, 4000);


void setup() {

  Serial.begin(115200);
  #if !defined(__MIPSEL__)
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

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  J1.start();
  J2.start();
  K1.start();
  K2.start();


  bno.setExtCrystalUse(true);

  bno.getEvent(&event);
  delay(1000);
  bno.getEvent(&event);
  setpoint = event.orientation.x;
}

void loop() {
  Usb.Task();

//  if (PS4.connected()) {
//    if (PS4.getButtonClick(CROSS)) {
//      Serial.print(F("\r\nCross"));
//      PS4.setLedFlash(10, 10); // Set it to blink rapidly
//    }
//  }
  
  base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of speed
  delay(50);
  sensors_event_t event;
  bno.getEvent(&event);

  bno.getEvent(&event);
  x = event.orientation.x;
  dif = x - setpoint;
  int* s1 = straight(0);
  int* q = right(-base_speed);
  int* q1 = right(base_speed);
  int* q2 = right(0);
  int* p0 = straight(base_speed);
  int* p4 = move_angl(base_speed, -55.0);
  int* p5 = move_angr(base_speed, 55.0);
  //Serial.println(*p0);
  int* p1 = reverse(base_speed);
  int* p2 = rotate(pid(e_val(dif))) ;
  int* p3 = stop_motors();
  int v1[4];
  int* v;

if (check == false) {  
  if (myEnc.read() < 60340) {
    if (hcsr04l.distanceInMillimeters() > dist) {
      while (hcsr04l.distanceInMillimeters() > dist) {
        Serial.println("1");
        base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of spee
        if (ang_speed>300){
          ang_speed = ang_speed - acc1;
        }
        else{
          ang_speed = 300;
        }

        delay(50);
        sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
        //Serial.println("i'm here");
        int* p4 = move_angl(ang_speed, -55.0);
        int* p5 = move_angr(ang_speed, 55.0);
        
        //Serial.println(hcsr04l.distanceInMillimeters());
        int* p2 = rotate(pid(e_val(dif))) ;
        Serial.println("left loop");
        Serial.println(ang_speed);
        int v1[4];
        int* v;
//        Serial.println("base speed:              ");
//        Serial.println(base_speed*((hcsr04l.distanceInMillimeters()-150))/100);
       
        v = add_vels(p4, p2);  // straight + rotate
        write_motor_speeds(v);
        Serial.println(*v);
        
      }
      
      while (myEnc.read() < d) {
        
        Serial.println(2);
        base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of speed
        delay(50);
        sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
        //Serial.println(*p0);
        int* p0 = straight(base_speed);
        int* p2 = rotate(pid(e_val(dif))) ;
        int v1[4];
        int* v;
        v = add_vels(p0, p2);  // straight + rotate
        write_motor_speeds(v);
      }
      ang_speed = 5000;
      d = d + 19278;             // 1st turn

      write_motor_speeds(stop_motors());
      delay(50);
    }
    
    else {
      while (hcsr04r.distanceInMillimeters() > dist) {
        if (temp != 0) {
        sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
        int* p2 = rotate(pid(e_val(dif))) ;
        int* p3 = stop_motors();
          int v1[4];
          int* v;
          v = add_vels(p3, p2);
          write_motor_speeds(v);
        break;
        }
        else{
          temp++;
        }
        if (myEnc.read()<60340){
          Serial.println("3");
          base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of speed
          if (ang_speed>200){
          ang_speed = ang_speed - acc1;
        }
        else{
          ang_speed = 200;
        }
          delay(50);
          sensors_event_t event;
          bno.getEvent(&event);
  
          bno.getEvent(&event);
          x = event.orientation.x;
          dif = x - setpoint;
          int k1 = hcsr04l.distanceInMillimeters()-130;
          int k2 = hcsr04r.distanceInMillimeters()-130;
          int* p4 = move_angl(ang_speed, -55.0);
          int* p5 = move_angr(ang_speed, 55.0);
          int* p2 = rotate(pid(e_val(dif))) ;
          int v1[4];
          int* v;
          v = add_vels(p5, p2);  // straight + rotate
          Serial.println("right loop");
          Serial.println(ang_speed);
          
          write_motor_speeds(v);
        }
        else{
          break;
        }
      }
      //Serial.print("d :     ");
      //Serial.println(d);
      //Serial.println(myEnc.read());
      while (myEnc.read() < d) {
        Serial.println(4);
        base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of speed
        delay(50);
        sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
        //Serial.println(*p0);
        int* p0 = straight(base_speed);
        int* p2 = rotate(pid(e_val(dif))) ;
        int v1[4];
        int* v;
        v = add_vels(p0, p2);  // straight + rotate
        write_motor_speeds(v);
      }
      ang_speed = 5000;
      d = d + 19278;         //2nd turn
      
      write_motor_speeds(stop_motors());
      delay(500);
    }
  }
  else {
    sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
    int* p2 = rotate(pid(e_val(dif))) ;
        int* p3 = stop_motors();
          int v1[4];
          int* v;
          v = add_vels(p3, p2);
          write_motor_speeds(v);
  }
  check = true;
}

else {
  if (PS4.connected()) {
    int x = PS4.getAnalogHat(LeftHatX);
    int ax = map(x,0,117,max_speed,0);
    int bx = map(x,138,255,0,max_speed); 
    int y = PS4.getAnalogHat(LeftHatY);
    int ay = map(x,0,117,max_speed,0);
    int by = map(x,138,255,0,max_speed);
    if (ay<117) {
        sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
        int* p0 = straight(ay);
        int* p2 = rotate(pid(e_val(dif))) ;
        int v1[4];
        int* v;
        v = add_vels(p0, p2);  // straight + rotate
        write_motor_speeds(v);
    }
    else if (by>138) {
        sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
        int* p1 = reverse(by);
        int* p2 = rotate(pid(e_val(dif))) ;
        int v1[4];
        int* v;
        v = add_vels(p1, p2);  // straight + rotate
        write_motor_speeds(v);
    }
    if (ax<117){
      sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
        int* q1 = right(ax);
        int* p2 = rotate(pid(e_val(dif))) ;
        int v1[4];
        int* v;
        v = add_vels(q1, p2);  // straight + rotate
        write_motor_speeds(v);
    }
    else  if (bx>138) {
      sensors_event_t event;
        bno.getEvent(&event);

        bno.getEvent(&event);
        x = event.orientation.x;
        dif = x - setpoint;
        int* q = right(-bx);
        int* p2 = rotate(pid(e_val(dif))) ;
        int v1[4];
        int* v;
        v = add_vels(q, p2);  // straight + rotate
        write_motor_speeds(v);
    }
  }
  else{
    Serial.println("PS4 not connected");
  }
}

  
}







//  write_motor_speeds(v);
//  delay(2000);
//  write_motor_speeds(w);
//  delay(100);
//  write_motor_speeds(u);
//  delay(2000);
//  write_motor_speeds(w);
//  delay(100);




int* rotate(int v) {
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
