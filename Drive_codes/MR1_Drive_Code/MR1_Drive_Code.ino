#include <hcsr04.h>
#include <Kangaroo.h>
#include <Encoder.h>
//--------- Libraries for PS4 --------------
#include <PS4BT.h>
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

  // Encoder myEnc(3, 2);

  int j1_fact = 1, k1_fact = 1, j2_fact = -1, k2_fact = 1;   //for controlling the direction of wheel rotation
  int base_speed=0;
  int max_speed=4000;
  int acc=150;
  float setpoint = 0.0;
  sensors_event_t event;
  #define BNO055_SAMPLERATE_DELAY_MS (100);
  Adafruit_BNO055 bno = Adafruit_BNO055(55);
  float current_time, elapsed_time, deltaT;
  #define kang_ramp 1500
  #define kang_deramp 6000
  float x, dif;
  float kp=100;
  float ki = 0.01 ;
  float kd = 6000;
  float output = 0;
  float sum = 0;
  float diff = 0;
  float prev_err = 0;
  float err;
  int a = 45;

  #define TRIG_PIN 31
  #define ECHO_PIN 33
  HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);
  
void setup() {

  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  Serial2.begin(115200);
  Serial3.begin(115200);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
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
  base_speed = constrain(base_speed + acc,0,max_speed);   //max and min value of speed
  delay(50);
  sensors_event_t event;
  bno.getEvent(&event);
 
  bno.getEvent(&event);
  x = event.orientation.x;
  dif = x-setpoint;
  //rotate(700);
//  Serial.print(F("Orientation: "));
//  Serial.println("dif is   ");
//  Serial.println(dif);
//  Serial.println("setpoint:  ");
//  Serial.println(setpoint);
//  Serial.println(deltaT);
//  Serial.print(F("Difference :" ));
//  Serial.println(dif);
//  Serial.println(pid(e_val(dif)));
  //pid(e_val(dif));
  //rotate(pid(e_val(dif)));
  int* s = straight(0);
  int* q = right(-base_speed); 
  int* q1 = right(base_speed);
  int* q2 = right(0);
  int* p = straight(base_speed);
  int* p1 = reverse(base_speed);
  int* p2 = rotate(pid(e_val(dif))) ;
  int* p3 = stop_motors();
  int v1[4];
  int* v = v1;
  int* u = v1;
  int* w = v1;
  int* t = v1;
  int* l = v1;
  int* r = v1;
  int* k = v1;
  v = add_vels(p,p2);   // straight + rotate
  Serial.println(*v);
  u = add_vels(p1,p2);  // reverse + rotate
  w = add_vels(p3,p2);  // stop + rotate
  t = add_vels(s,p2);  // stop in straight direction + rotate
  l = add_vels(q,p2);  // left + rotate
  r = add_vels(q1,p2); // right + rotate
  k = add_vels(q2,p2); // stop in lateral direction + rotate

//  while (dist<1500) {
//    write_motor_speeds(v);
//  }
//  if (dist = 1500) {
//    write_motor_speeds(t);
//  }
//  while (ultraL < 130) {
//    write_motor_speeds(l);
//  }
//  if (ultraL>= 130) {
//    write_motor_speeds(k);
//  }
  write_motor_speeds(v);
  Serial.println(base_speed);
  Serial.println(*(v+1));
//  Serial.println(hcsr04.ToString());
//
//  delay(250);


  

//  write_motor_speeds(v);
//  delay(2000);
//  write_motor_speeds(w);
//  delay(100);
//  write_motor_speeds(u);
//  delay(2000);
//  write_motor_speeds(w);
//  delay(100);

}


int* rotate(int v){
    int static vels[4];
    vels[0] = -v;
    vels[1] = v;
    vels[2] = v;
    vels[3] = -v;
    return vels;
}



int* straight(int v){
    Serial.print("in straight  ");
    Serial.println(v);
    int static vels[4];
    vels[0] = v;
    vels[1] = v;
    vels[2] = v;
    vels[3] = v;
    return vels;
}

int* right(int v){
    int static vels[4];
    vels[0] = -v;
    vels[1] = v;
    vels[2] = -v;
    vels[3] = v;
    return vels;
}



int* move_ang(int v, int ang){
   int static vels[4];
    vels[0] = v*(cos(ang) - sin(ang));
    vels[1] = v*(cos(ang) + sin(ang));
    vels[2] = v*(cos(ang) - sin(ang));
    vels[3] = v*(cos(ang) + sin(ang));
    return vels;
}

void write_motor_speeds(int* p){
  J1.s(j1_fact*(*p),kang_ramp);
  Serial.println(j1_fact*(*p));
  J2.s(j2_fact*(*(p+1)),kang_ramp);
  K1.s(k1_fact*(*(p+2)),kang_ramp);
  K2.s(k2_fact*(*(p+3)),kang_ramp);
}

int* add_vels(int* p1,int* p2){
    int static vels[4];
    vels[0] = *p1 + *p2;
    vels[1] = *(p1+1) + *(p2+1);
    vels[2] = *(p1+2) + *(p2+2);
    vels[3] = *(p1+3) + *(p2+3);
    return vels;
}

float e_val(float(dif)){                                    // x = dif
  if (setpoint<=360 && setpoint> 355 ){
    if (dif==0){
      return float(dif);
     }
     else if (dif<0 && dif>-180){
      return float((-1)*dif);
     }
     else if (dif<-180 && dif>-360){
      return float((-1)*(360+dif));
     }
  }
  else {
    if (dif==0){
      return float(dif);
     }
     else if (dif>0 && dif<180){
      return float(-dif);
     }
     else if (dif>180 && dif<360){
      return float((360-dif));
     }
  }
}

float pid(float(dif)) {
  deltaT = (millis()-elapsed_time);
  elapsed_time = millis();
  sum = sum + dif*deltaT;
  diff = (dif-prev_err)/deltaT;
  prev_err = dif;
  output = (kp*dif + ki*sum + kd*diff);
  Serial.print("dt:  ");
  Serial.print(deltaT);
  Serial.print("  sum:  ");
  Serial.print(sum);
  Serial.print("  diff:  ");
  Serial.print(diff);
  Serial.print("  output:  ");
  Serial.println(output);
  return (output);
}  

int* stop_motors(){
    int static vels[4];
    vels[0] = 0;
    vels[1] = 0;
    vels[2] = 0;
    vels[3] = 0;
    return vels;
}

int* reverse(int v){
  int static vels[4];
    vels[0] = -v;
    vels[1] = -v;
    vels[2] = -v;
    vels[3] = -v;
    return vels;
}
 
