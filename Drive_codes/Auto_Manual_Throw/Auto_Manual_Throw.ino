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


KangarooChannel J1(kang1, '1');

KangarooChannel J2(kang1, '2');

KangarooChannel K1(kang2, '1');

KangarooChannel K2(kang2, '2');


Encoder myEnc(18, 19);


int state = 0;

int j1_fact = -1, k1_fact = 1, j2_fact = -1, k2_fact = 1;   //for controlling the direction of wheel rotation

int base_speed = 0;

int max_speed = 2500;

int max_speedM = 3500;

int accM = 250;

int acc = 150;

float setpoint = 0.0;

boolean check1=false;

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
int temp1 = 0;

int dist = 700;

int dist1 = 700;

//int max_speed1 = 1500;

//int min_speed1 = 200;

int acc1 = 100;

int ang_speed = 5000;

float angle_l = -55.0;

float angle_r = 55.0;


boolean stop_motor = false;

boolean check = true;

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

  //  Serial.println(hcsr04l.distanceInMillimeters());

  //  Serial.println(hcsr04l.distanceInMillimeters());

  if (PS4.connected()) {
  
    while (check == true) {

      PS4.setLed(Yellow);
      

      if (myEnc.read() < 57524 && stop_motor == false) {
        Serial.println(myEnc.read());
        ang_speed = 5000;
        if (hcsr04l.distanceInMillimeters() > dist || (myEnc.read()>12000 && myEnc.read()<49000)) {
         Serial.println("in loop");
          while (hcsr04l.distanceInMillimeters() > dist1 || (myEnc.read()>12000 && myEnc.read()<49000)){


            //Serial.println("1");

            //Serial.println(myEnc.read());


            base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of spee

            delay(50);

            if (ang_speed > 300) {

              ang_speed = ang_speed - acc1;

            }

            else {

              ang_speed = 300;

            }

            //Serial.print("base speed is:         ");

            //Serial.println(base_speed);

            sensors_event_t event;

            bno.getEvent(&event);


            bno.getEvent(&event);

            x = event.orientation.x;

            dif = x - setpoint;

            int* p4 = move_angl(ang_speed, angle_l);

            int* p5 = move_angr(ang_speed, angle_r);

            int* p2 = rotate(pid(e_val(dif))) ;

            int v1[4];

            int* v;

            v = add_vels(p4, p2);  // straight + rotate

            write_motor_speeds(v);

          }


          if (temp == 0) {

            while (myEnc.read() <10756) {

              //Serial.println(myEnc.read());

              base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of spee

              delay(50);

              sensors_event_t event;

              bno.getEvent(&event);


              bno.getEvent(&event);

              x = event.orientation.x;

              dif = x - setpoint;

              int* p0 = straight(base_speed);

              int* p2 = rotate(pid(e_val(dif))) ;

              int v1[4];

              int* v;

              v = add_vels(p0, p2);  // straight + rotate

              write_motor_speeds(v);

            }

            temp++;

          }

          else {

            while (myEnc.read() < 57524) {

              //Serial.println(myEnc.read());

             base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of spee

              delay(50);

              sensors_event_t event;

              bno.getEvent(&event);


              bno.getEvent(&event);

              x = event.orientation.x;

              dif = x - setpoint;

              int* p0 = straight(base_speed);

              int* p2 = rotate(pid(e_val(dif))) ;

              int v1[4];

              int* v;

              v = add_vels(p0, p2);  // straight + rotate

              write_motor_speeds(v);

            }

          }


        }


        else {

          while (hcsr04r.distanceInMillimeters() > 250) {

            //Serial.println("2");


            if (myEnc.read() < 57524) {

              //Serial.println(myEnc.read());


              base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of spee

              delay(50);

              if (ang_speed > 300) {

                ang_speed = ang_speed - acc1;

              }

              else {

                ang_speed = 300;

              }

              //Serial.print("base speed is:         ");

              //Serial.println(base_speed);


              sensors_event_t event;

              bno.getEvent(&event);


              bno.getEvent(&event);

              x = event.orientation.x;

              dif = x - setpoint;

              //Serial.println("i'm here");

              int* p4 = move_angl(ang_speed, angle_l);

              int* p5 = move_angr(ang_speed, angle_r);

              //int* p6 = move_angr(ang_speed, 40.0);

              int* p2 = rotate(pid(e_val(dif))) ;

              int v1[4];

              int* v;

              //if (myEnc.read() < 59524) {

              v = add_vels(p5, p2);  // straight + rotate

              //}

              //else {

              //v = add_vels(p6, p2);

              //}

              write_motor_speeds(v);
              

            }

           else {

              break;

            }

          }



          if (temp1 == 0) {

            while (myEnc.read() < 36233) {

              //Serial.println(myEnc.read());

              base_speed = constrain(base_speed + acc, 0, 1500); //max and min value of spee

              delay(50);

              sensors_event_t event;

              bno.getEvent(&event);


              bno.getEvent(&event);

              x = event.orientation.x;

              dif = x - setpoint;

              int* p0 = straight(base_speed);

              int* p2 = rotate(pid(e_val(dif))) ;

              int v1[4];

              int* v;

              v = add_vels(p0, p2);  // straight + rotate

              write_motor_speeds(v);

            }

            temp1++;

          }

          else {

            //        while (myEnc.read() < 60386) {

            //          //Serial.println(myEnc.read());

            //          base_speed = constrain(base_speed + acc, 0, max_speed); //max and min value of spee

            //          delay(50);

            //          sensors_event_t event;

            //          bno.getEvent(&event);

            //

            //          bno.getEvent(&event);

            //          x = event.orientation.x;

            //          dif = x - setpoint;

            //          int* p0 = straight(base_speed);

            //          int* p2 = rotate(pid(e_val(dif))) ;

            //          int v1[4];

            //          int* v;

            //          v = add_vels(p0, p2);  // straight + rotate

            //          write_motor_speeds(v);

            stop_motor = true;

          }

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
      for(int h=0;h<10;h++){
        write_motor_speeds(v);
      }
        stop_motor = true;

        check = false;

        break;

      }

    }

   




   while (check == false) {

    Usb.Task();

    

    while (PS4.getButtonClick(CIRCLE)) {

      sensors_event_t event;

      bno.getEvent(&event);

      x = event.orientation.x;

      dif = x - setpoint;

      if (e_val(dif) < 1.0 && e_val(dif) > -1.0) {

        write_motor_speeds(stop_motors());
        Serial.println(*stop_motors());

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


    int psw = PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2);
    int multfac=map(psw, -255, 255, -2000, 2000);
    int randvar=0;

    while ((psw >20 || psw<-20)&&check1==false) {
      Usb.Task();
      if (PS4.getButtonClick(TRIANGLE)){
        check1=true;
        write_motor_speeds(stop_motors());
        sensors_event_t event;
      bno.getEvent(&event); 
      setpoint = event.orientation.x;
      PS4.setLed(Green);
      break;       
      }
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


    int r = base_speed * sqrt(ps3x * ps3x + ps3y * ps3y) / 75;
    

    float theta = 180 * atan(ps3y / ps3x) / 3.14;

    if (ps3x < 0) theta = theta + 180;

    if (abs(r) < 100) r = 0;


    int* p4 = move_angr(r, theta);

    int* p2 = rotate(pid(e_val(dif))) ;

    int v1[4];

    int* v;

    v = add_vels(p4, p2);  // straight + rotate

    write_motor_speeds(v);


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
