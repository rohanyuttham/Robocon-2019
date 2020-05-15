#include <hcsr04.h>

//--------- Libraries for Motor Controller --------------
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Kangaroo.h>
#include <Encoder.h>
//--------- Libraries for PS4 --------------
#include <PS4BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include <Wire.h>
//--------- Libraries for IMU --------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//------ Libraries for Ulrasonic Sensor ---------
//#include "Ultrasonic.h"
// ----
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd, PAIR);
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Ultrasonic Ul(2, 3);
Ultrasonic Ur(4, 5);
//Ultrasonic U3(6, 7);
//Ultrasonic U4(8, 9);
//----
// ----
KangarooSerial  J(Serial3);
KangarooSerial  K(Serial2);           //       FRONT
KangarooChannel J1 (J, '1');           //    K2-------K1
KangarooChannel J2 (J, '2');           //     |       |
                                       //LEFT |       | RIGHT
KangarooChannel K1 (K, '1');           //     |       |
KangarooChannel K2 (K, '2');           //    J1-------J2
//----imu
float x,a;
float tfm=0.2; //tfm=0.1 for v = 20-50
float kp=0.6;
float sumr=0;
float suml=0;
int state=0;
long time1=0,time2=0;
long time3;
//----
//imu::Vector<3> euler;
//imu::Vector<3> gyro;
#define kang_ramp 1500
#define kang_deramp 6000
#define v 2000
#define vd 50
boolean check = true;
sensors_event_t event;
//----
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  /* ----------------------------------------------------------- */
  /*  Initialise the PS4 */
#if !defined(MIPSEL)
  while (!Serial); // Wait for serial port to connect - used on Leonardo,
#endif             // Teensy and other boards with built-in USB CDC serial connection
//  if (Usb.Init() == -1) {
//    Serial.print(F("\r\nOSC did not start"));
//    while (1); //halt
//  }
  Serial.print(F("\r\nPS4 USB Library Started"));
  /* ----------------------------------------------------------- */
  /* Initialise the Kangaroos */
  K1.start();
  K1.home().wait();
  K2.start();
  K2.home().wait();
  J1.start();
  J1.home().wait();
  J2.start();
  J2.home().wait();
  /* ----------------------------------------------------------- */
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  for(int i=0;i<10;i++){
  sumr=sumr+(Ur.Ranging(CM));
  suml=suml+(Ul.Ranging(CM));
 }

}

void loop() {
  
  //K1.s(2000,1500);
 // if(check==true){

suml=0;sumr=0;
 for(int i=0;i<3;i++){
  sumr=sumr+(Ur.Ranging(CM));
  suml=suml+(Ul.Ranging(CM));
 }
 sumr=sumr/3.0;
 suml=suml/3.0;
// Serial.print(sumr);
// Serial.print("   u  ");
// Serial.println(suml);
// time3=millis;
// Serial.println(time3);
// Serial.print(time1);
// Serial.print("   h  ");
// Serial.println(time2);
 //delay(1000);
  bno.getEvent(&event);
  x = event.orientation.x;
  check=false;
//}
 // a=event.orientation.x;
  int u =1000;
//      K1.s(v, kang_ramp);
//      K2.s(v, kang_ramp);
//      J1.s(v, kang_ramp);
//      J2.s(v, kang_ramp);


//if(check==true){
//  time1=millis;
//  check=false;
//}
if(suml>sumr){
  while(suml>70){
    
// Serial.print(sumr);
// Serial.print("   u  ");
// Serial.println(suml);
 
      K1.s(v, kang_ramp);
      K2.s(-0.5*v, kang_ramp);
      J1.s(v, kang_ramp);
      J2.s(-0.5*v, kang_ramp);
  float dif = x-a;
     // Serial.println(dif);
      if (dif>1 && dif <180){dif=1;
      u=50;
      K1.s((1+tfm*dif) * kp*vd * u, kang_deramp);
      K2.s((1+tfm*dif) * vd * u, kang_deramp);
      J1.s((1+tfm*dif) * vd * u, kang_deramp);
      J2.s((1+tfm*dif) * kp*vd * u, kang_deramp);
    }
    if (dif<359 && dif>180){dif=-1;
      u=50;
      K1.s((1+tfm*dif) * vd * u, kang_deramp);
      K2.s((1-tfm*dif) * kp*vd * u, kang_deramp);
      J1.s((1-tfm*dif) * kp*vd * u, kang_deramp);
      J2.s((1+tfm*dif) * vd * u, kang_deramp);
    }
//suml=0;
//    for(int i=0;i<3;i++){
//  
  suml=(Ul.Ranging(CM));
// }suml=suml/3.0;
  }
}else{

//  if(suml <180 && sumr>100 && millis-time1<5){
//      K1.s(v, kang_ramp);
//      K2.s(v, kang_ramp);
//      J1.s(v, kang_ramp);
//      J2.s(v, kang_ramp);
//}
  while(sumr>70 ){
//     Serial.print(sumr);
// Serial.print("   u  ");
// Serial.println(suml);
      K1.s(-0.5*v, kang_ramp);
      K2.s(v, kang_ramp);
      J1.s(-0.5*v, kang_ramp);
      J2.s(v, kang_ramp);

      float dif = x-a;
     // Serial.println(dif);
      if (dif>1 && dif <180){dif=1;
      u=50;
      K1.s((1+tfm*dif) * kp*vd * u, kang_deramp);
      K2.s((1+tfm*dif) * vd * u, kang_deramp);
      J1.s((1+tfm*dif) * vd * u, kang_deramp);
      J2.s((1+tfm*dif) * kp*vd * u, kang_deramp);
    }
    if (dif<359 && dif>180){dif=-1;
      u=50;
      K1.s((1+tfm*dif) * vd * u, kang_deramp);
      K2.s((1-tfm*dif) * kp*vd * u, kang_deramp);
      J1.s((1-tfm*dif) * kp*vd * u, kang_deramp);
      J2.s((1+tfm*dif) * vd * u, kang_deramp);
    }
// sumr=0;
//    for(int i=0;i<3;i++){
 
  sumr=(Ur.Ranging(CM));
 //}sumr=sumr/3.0;
  }
}





//if(check==true){
//  time2=millis;
//  check=false;
//}
//if(sumr<100 && suml>100 && millis-time2<5){
//      K1.s(v, kang_ramp);
//      K2.s(v, kang_ramp);
//      J1.s(v, kang_ramp);
//      J2.s(v, kang_ramp);
//}

 // Usb.Task();
  /*if (Ul.Ranging(CM) < 15 || Ur.Ranging(CM) < 15 ) {
    K1.s(0, kang_deramp);
    K2.s(0, kang_deramp);
    J1.s(0, kang_deramp);
    J2.s(0, kang_deramp);
  }
  else{
     if (Ul.Ranging(CM) < 30 || Ur.Ranging(CM) < 30 ) {
      PS4.setLed(Red);
     }
    else{PS4.setLed(Blue);}*/
     
     //if (PS4.getButtonClick(CIRCLE)) {
   //  a=x;
    // if(a<10 || a>350)
     //{PS4.setLed(Red);}
   //}
      
//    if (PS4.getButtonClick(CROSS)) {
//      K1.s(0, kang_deramp);
//      K2.s(0, kang_deramp);
//      J1.s(0, kang_deramp);
//      J2.s(0, kang_deramp);
//    }
//    if (PS4.getAnalogButton(R2) > 70) //ClockWise Rotation
//    {
//      K1.s(-v, kang_ramp);
//      K2.s(v, kang_ramp);
//      J1.s(v, kang_ramp);
//      J2.s(-v, kang_ramp);
//    }
//    if (PS4.getAnalogButton(L2) > 70) //Anti-ClockWise Rotation
//    {
      float dif = x-a;
      Serial.println(dif);
      if (dif>1 && dif <180){dif=1;
      u=50;
      K1.s((1+tfm*dif) * kp*vd * u, kang_deramp);
      K2.s((1+tfm*dif) * vd * u, kang_deramp);
      J1.s((1+tfm*dif) * vd * u, kang_deramp);
      J2.s((1+tfm*dif) * kp*vd * u, kang_deramp);
    }
    if (dif<359 && dif>180){dif=-1;
      u=50;
      K1.s((1+tfm*dif) * vd * u, kang_deramp);
      K2.s((1-tfm*dif) * kp*vd * u, kang_deramp);
      J1.s((1-tfm*dif) * kp*vd * u, kang_deramp);
      J2.s((1+tfm*dif) * vd * u, kang_deramp);
    }
    
//    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117) {
//      int  lhx = PS4.getAnalogHat(LeftHatX) - 133;
//      int  lhy = PS4.getAnalogHat(LeftHatY) - 127;
//      K1.s(vd * ((lhy) + (1.5 * (lhx))), kang_ramp);
//      K2.s(vd * ((lhy) - (1.5 * (lhx))), kang_ramp);
//      J1.s(vd * ((lhy) + (1.5 * (lhx))), kang_ramp);
//      J2.s(vd * ((lhy) - (1.5 * (lhx))), kang_ramp);
//    }
//    if (PS4.getAnalogHat(LeftHatX) < 137 && PS4.getAnalogHat(LeftHatX) > 117 && PS4.getAnalogHat(LeftHatY) < 137 && PS4.getAnalogHat(LeftHatY) > 117 && PS4.getAnalogButton(R2) < 70 && PS4.getAnalogButton(L2) < 70) {
//      K1.s(0, kang_deramp);
//      K2.s(0, kang_deramp);
//      J1.s(0, kang_deramp);
//      J2.s(0, kang_deramp);
//     }
}
