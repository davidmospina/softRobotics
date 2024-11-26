#include <Servo.h>

#define SERVO_1 (9) // RIGHT
#define SERVO_2 (10) // LEFT
#define INPUT_PIN (13) // Using pin 13 for input from 3.3V

// ______________________________________________________Variables_________________________________________________________________________________//
int time = 1500;
int pos_servo_1 = 50; // in degree between 0 and 180
int pos_servo_2 = 50; // in degree between 0 and 180

// ________________________________________________Arduino PWM Speed Control_______________________________________________________________________//

const int E1 = 3; // Speed
const int M1 = 4; // Direction

const int E2 = 11;   // Enable
const int M2 = 12;   // State

const int E3 = 5;
const int M3 = 8;

const int E4 = 6;
const int M4 = 7;

//___________________________________________________________Class________________________________________________________________________________//

class Valve {
  private:
    int enablePin;
    int statePin;

  public:
    bool state = false;

    Valve(int enPin, int stPin) {
      enablePin = enPin;
      statePin = stPin;
      pinMode(enablePin, OUTPUT);
      pinMode(statePin, OUTPUT);
    }

    void off() {
      analogWrite(enablePin, 255);
      digitalWrite(statePin, HIGH);
      state = false;
    }
      
    void on() {
      analogWrite(enablePin, 0);
      digitalWrite(statePin, HIGH);
      state = true;
    }
};

// Valve instances
Valve valv_inside_deflate(E1, M1);
Valve valv_outside_deflate(E3, M3);
Valve valv_inside_inflate(E4, M4);

Servo servo1;
Servo servo2;

// ______________________________________________________Functions_________________________________________________________________________________//

void goForward() {
    valv_inside_deflate.on();
    valv_inside_inflate.off();
    valv_outside_deflate.off();
    delay(time);

    valv_inside_deflate.off();
    valv_inside_inflate.on();
    valv_outside_deflate.on();
    delay(time);

    valv_inside_deflate.off();
    valv_inside_inflate.on();
    valv_outside_deflate.off();
    delay(time);
}

// ________________________________________________________Setup_________________________________________________________________________________//
void setup() {
  Serial.begin(115200);
  pinMode(INPUT_PIN, INPUT_PULLUP); // Set pin 13 as input with pull-down
  valv_inside_deflate.off();
  valv_inside_inflate.off();

  valv_outside_deflate.off();
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  // motor1.off();
  // motor2.off();
}

// __________________________________________________________Loop__________________________________________________________________________________//
void loop() {

  // Check if 3.3V is connected to INPUT_PIN

  // goForward();
  // valv_inside_deflate.on();
  // valv_inside_inflate.off();
  // valv_outside_deflate.off();
  // delay(time*0.75);

  // valv_inside_deflate.on();
  // valv_inside_inflate.off();
  // valv_outside_deflate.on();
  // delay(time)*0.25;


  // valv_inside_deflate.off();
  // valv_inside_inflate.on();
  // valv_outside_deflate.on();
  // delay(time);

  // valv_inside_deflate.off();
  // valv_inside_inflate.on();
  // valv_outside_deflate.off();
  // delay(time*0.75);


  valv_inside_deflate.on();
  valv_inside_inflate.off();
  valv_outside_deflate.off();
  delay(time/4);

  valv_outside_deflate.on();
  delay(time/20);

  valv_outside_deflate.on();
  valv_inside_inflate.on();
  delay(time/12);
  
  valv_inside_deflate.off();
  valv_inside_inflate.on();
  delay(time*1/4);

  valv_inside_deflate.off();
  valv_inside_inflate.on();
  valv_outside_deflate.off();
  delay(time/6);

  

  

// Preserving all your commented code:
  // while (digitalRead(SWITCH_1_RIGHT)==0 || digitalRead(SWITCH_1_LEFT)==0){
  //   delay(100);
  // }
  // while (digitalRead(13)==0){
  //   delay(100);
  // }

  // if (digitalRead(SWITCH_1_RIGHT) == 0){
  //   valv_in_deflate.off();
  //   valv_in_inflate.off();
  //   valv_out_inflate.off();
  //   valv_out_deflate.off();
  //   servo2.write(pos_servo_2);
  // } else if (digitalRead(SWITCH_1_LEFT) == 0){
  //   valv_in_deflate.off();
  //   valv_in_inflate.off();
  //   valv_out_inflate.off();
  //   valv_out_deflate.off();
  //   servo1.write(pos_servo_1);
  // } else{
  //     servo1.write(0);
  //     servo2.write(0);
  //     goForward();
  // }

  // if (digitalRead(SWITCH_2)==0){
  //   goForward();

  //   if (digitalRead(SWITCH_1_RIGHT) == 0){
  //     servo2.write(pos_servo_2);
  //   } else if (digitalRead(SWITCH_1_LEFT) == 0){
  //     servo1.write(pos_servo_1);
  //   } else{
  //     servo1.write(0);
  //     servo2.write(0);
  //   }
  // } else{
  //   valv_in_deflate.off();
  //   valv_in_inflate.off();
  //   valv_out_inflate.off();
  //   valv_out_deflate.off();
  //   // motor1.off();
  //   // motor2.off();
  // }

  //_____Test servo___//
  // servo1.write(pos_servo_1);
  // Serial.println("1");
  // delay(1000);
  // servo1.write(0);
  // Serial.println("2");
  // delay(1000);

  // servo2.write(pos_servo_2);
  // Serial.println("1");
  // delay(1000);
  // servo2.write(0);
  // Serial.println("2");
  // delay(1000);

  //____old code____//
  // Code removed for brevity as requested
}
