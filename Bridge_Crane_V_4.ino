#include <AccelStepper.h>
#include "PinChangeInterrupt.h"

const int pul1 = 2;
const int dir1 = 5;

const int pul2 = 3;
const int dir2 = 6;

const int pul3 = 4;
const int dir3 = 7;

const int pul4 = 12;
const int dir4 = 13;

AccelStepper stepperX(1, pul1, dir1);
AccelStepper stepperY(1, pul2, dir2);
AccelStepper stepperZ(1, pul3, dir3);
AccelStepper stepperA(1, pul4, dir4);

const float pi = 3.14159;

const int yIn = A0;
const int xIn = A2;
const int buttonPin = A1;

const int limX1 = 9;
const int limX2 = 10;
const int limY1 = 11;
const int limY2 = A3;

const int lock = A4;

const int max_speed = 200;

bool buttonState = true;
bool locked = false;
long lastDebounceTime = 0;
const int debounceDelay = 50;

int x;
int y;
int button;

int speed_x;
int speed_y;
int dir_x;
int dir_y;
int v;
float angle;
float magnitude;
float mag;


void setup() {
  //Serial.begin(115200);
  stepperX.setMaxSpeed(max_speed);
  stepperY.setMaxSpeed(max_speed);
  stepperZ.setMaxSpeed(max_speed);
  stepperA.setMaxSpeed(max_speed);

  pinMode(limX1, INPUT_PULLUP);
  pinMode(limX2, INPUT_PULLUP);
  pinMode(limY1, INPUT_PULLUP);
  pinMode(limY2, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(lock, OUTPUT);
  digitalWrite(lock, locked);

  attachPCINT(digitalPinToPCINT(buttonPin), blinkLed, CHANGE);
}


void blinkLed()
{

  buttonState = !buttonState;
  
  if (buttonState && (millis() - lastDebounceTime) > debounceDelay)
  {
    locked = !locked;               // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    digitalWrite(lock, locked);     // !!!!!! LOCK CODE HERE !!!!!!
  }                                 // !!!!!!!!!!!!!!!!!!!!!!!!!!!!

  lastDebounceTime = millis();

}


void loop() {

                    // Read joystick control values
  long y = -(analogRead(xIn) - 512);
  long x = analogRead(yIn) - 512;
  button = digitalRead(buttonPin);


                    // Convert joystick readings to radial coordinates
  angle = atan2(y, x);
  magnitude = sqrt(x*x + y*y);

  float limitAngle = atan2(y, x);
  float magnitudeLimit = sqrt(x*x + y*y);


                    // Set maximum magnitude, x and y to 511
  if (magnitude > 500) {
    magnitude = 500;
  }
  if (x > 500) {
    x = 500;
  }
  if (x < -500) {
    x = -500;
  }
  if (y > 500) {
    y = 500;
  }
  if (y < -500) {
    y = -500;
  }

                                              // Move hook up and down
  if (locked) {
    if (abs(x) >= 100) {
      v = map(abs(x), 100, 500, 0, max_speed/2);
      if (x < 0) {
        v = -v;
        
      }
      lift_lower();
    }
    else {
      v = 0;
    }
  }

  else {

    bool limitX1 = digitalRead(limX1);
    bool limitX2 = digitalRead(limX2);
    bool limitY1 = digitalRead(limY1);
    bool limitY2 = digitalRead(limY2);

    if (!limitX1 && abs(angle) < (pi/2+0.05)) {                  // >>> |X1| <<<
      magnitude = abs(y);
      if (angle > 0.05) {
        angle = pi/2;
      }
      else if (angle < -0.05) {
        angle = -pi/2;
      }
      else {
        magnitude = 0;
      }
    }

    if (!limitX2 && abs(angle) > (pi/2-0.05)) {                  // >>> |X2| <<<
      magnitude = abs(y);
      if (angle > 0.05) {
        angle = pi/2;
      }
      else if (angle < 0.05) {
        angle = -pi/2;
      }
      else {
        magnitude = 0;
      }
    }

    if (!limitY1 && angle > -0.05) {                          // >>> |Y1| <<<
      magnitude = abs(x);
      if (angle > pi/2+0.05) {
        angle = pi;
      }
      else if (angle < pi/2-0.05) {
        angle = 0;
      }
      else {
        magnitude = 0;
      }
    }

    if (!limitY2 && angle < 0.05) {                          // >>> |Y2| <<<
      magnitude = abs(x);
      if (angle < -pi/2+0.05) {
        angle = pi;
      }
      else if (angle > -pi/2-0.05) {
        angle = 0;
      }
      else {
        magnitude = 0;
      }
    }

    normal_controls();

  }
                      // Mapping for python visualisation    >>> UNUSED <<<
  /*
  x = map(x, -512, 512, -127, 128);
  y = map(y, -512, 512, -127, 128);
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y)
  */
  
  //Serial.println(v);
}


void spin_axis() {            // Spin motors x-y-plane
  int vx = speed_x * mag;
  int vy = speed_y * mag;

  stepperX.setSpeed(vx);
  stepperY.setSpeed(vy);
  stepperZ.setSpeed(-vy);
  stepperA.setSpeed(vx);

  stepperX.runSpeed();
  stepperY.runSpeed();
  stepperZ.runSpeed();
  stepperA.runSpeed();
}


void lift_lower() {           // Spin motors z-axis
  stepperX.setSpeed(-v);
  stepperY.setSpeed(v);
  stepperZ.setSpeed(v);
  stepperA.setSpeed(v);

  stepperX.runSpeed();
  stepperY.runSpeed();
  stepperZ.runSpeed();
  stepperA.runSpeed();
}


void limit_control() {       // Controls for limit situations

}


void normal_controls() {      // Controls for normal situations in x-y-plane

                              // Rotate control readings axis by 45 degrees
  if ((-(3*pi)/4) < angle) {
    angle -= pi/4;
  } else {
    angle = pi + (angle + (3*pi)/4);
  }

                              // Get x and y values from controls inside r=511 circle
  x = magnitude * cos(angle);
  y = magnitude * sin(angle);


                              // Get speed for both axis motors
  if (magnitude >= 100) {
    speed_x = cos(angle) * max_speed;
    speed_y = sin(angle) * max_speed;
    if (abs(speed_x) < 10) {
      speed_x = 0;
    }
    if (abs(speed_y) < 10) {
      speed_y = 0;
    }
    //mag = ( log(magnitude-80) / (log(2) * 4.43) ) - 0.975;      //  Alternate mapping for
    //mag = pow(5, (magnitude / 198) - 2.554);                    //  magnitude fine control
    float mag1 = map(magnitude, 100, 500, 0, 1000);
    mag = mag1 / 1000;
  } else {
    speed_x = 0;
    speed_y = 0;
    mag = 0;
  }

                // > > > > > > > > > >  Spin the motors < < < < < < < < < <
  if (magnitude >= 100) {
                              // Set directions for x-axis motors
    if (x >= 0) {
      dir_x = 1;
    } else {
      dir_x = -1;
    }
                              // Set directions for y-axis motors
    if (y >= 0) {
      dir_y = 1;
    } else {
      dir_y = -1;
    }

    spin_axis();
  }
}

