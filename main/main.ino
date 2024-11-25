#include <Servo.h>

#define SERVO_1 (9) //RIGHT
#define SERVO_2 (10) //LEFT
#define SWITCH_1_RIGHT (1)
#define SWITCH_1_LEFT (2)
#define SWITCH_2 (13)

// ______________________________________________________Variables_________________________________________________________________________________//
//int motorspeed = 250;
int time = 500;
int pos_servo_1 = 50; //in degree between 0 and 180
int pos_servo_2 = 50; //in degree between 0 and 180


// ________________________________________________Arduino PWM Speed Control_______________________________________________________________________//

const int E1 = 3; //Speed
const int M1 = 4; //Direction

const int E2 = 11;   //Enable
const int M2 = 12;  //State

const int E3 = 5;
const int M3 = 8;

const int E4 = 6;
const int M4 = 7;

//___________________________________________________________Class________________________________________________________________________________//

class Motor {
  private:
    int speedPin;
    int directionPin;

  public:
    bool state = false;

    Motor(int spdPin, int dirPin) {
      speedPin = spdPin;
      directionPin = dirPin;
      pinMode(speedPin, OUTPUT);
      pinMode(directionPin, OUTPUT);
    }

    void on(int motorspeed) {
      analogWrite(speedPin, motorspeed);
      digitalWrite(directionPin, HIGH);
      state = true;
    }
      
    void off()
    {
      analogWrite(speedPin, 0);
      digitalWrite(directionPin, HIGH);
      state = false;
    }
};

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

    void inflate() {
      analogWrite(enablePin, 255);
      digitalWrite(statePin, HIGH);
      state = false;
      //Serial.println("try to on valve");
    }
      
    void deflate() {
      analogWrite(enablePin, 0);
      digitalWrite(statePin, HIGH);
      state = true;
    }
};

// Motor motor1(E2, M2); // M2 = Pump1
// Motor motor2(E4, M4); // M4 = Pump2

Valve valv_outside(E1, M1); 
Valve valv_inside(E3, M3);

Servo servo1;
Servo servo2;

// ______________________________________________________Fonctions_________________________________________________________________________________//

void goForward() {
  
    // if (!motor1.state){
    //   motor1.on(motorspeed);
    // }
    // if (!motor2.state){
    //   motor2.on(motorspeed);
    // }

    valv_inside.deflate();
    valv_outside.inflate();
    delay(time);

    valv_inside.deflate();
    valv_outside.deflate();
    delay(time);

    valv_inside.inflate();
    valv_outside.deflate();
    delay(time);

    valv_inside.inflate();
    valv_outside.inflate();
    delay(time);
}


// ________________________________________________________Set up_________________________________________________________________________________//
void setup() {
  //Serial.begin(115200);
  valv_outside.deflate();
  valv_inside.deflate();
  // motor1.off();
  // motor2.off();
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  pinMode(SWITCH_1_RIGHT, INPUT_PULLUP); //right position : digitalRead(2) = 1 & digitalRead(3) = 0
  pinMode(SWITCH_1_LEFT, INPUT_PULLUP); //left position : digitalRead(3) = 1 & digitalRead(2) = 0
                                       // middle position : digitalRead(2) = digitalRead(3) = 1
  pinMode(SWITCH_2, INPUT_PULLUP);
  while (digitalRead(SWITCH_1_RIGHT)==0){
    delay(100);
  }

}

// __________________________________________________________Loop__________________________________________________________________________________//
void loop() {

  goForward();
  

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
    // switch (state_forward) {

    //   case DEFLATE_1:     // Inflate chanel 1, deflate chanel 2
  
    //     if (!valve1.state){
    //       valve1.on();
    //     }
    //     if (valve2.state){
    //       valve2.off();
    //     }
    //     // if (motor2.state) {
    //     //   motor2.off();
    //     // }
    //     // if (!lock_1 && !motor1.state) {
    //     //   motor1.on(motorspeed);
    //     // }

    //     // if (sensor1.readFiltered() <= setpoint) {  // If pressure is close to Setpoint, stop motor and lock
    //     //   // motor1.off();
    //     //   // lock_1 = true;
    //     //   state_forward = DEFLATE_1_2;
    //     // }

    //     if (millis() - timer >= 500) { // After 50 ms, change state
    //       state_forward = DEFLATE_1_2;
    //       timer = millis();
    //     }

        
    //   case DEFLATE_1_2: // Inflate both chanels

    //     if (!valve1.state){
    //       valve1.on();
    //     }
    //     // if (!lock_1 && !motor1.state ) {
    //     //   motor1.on(motorspeed);
    //     // }
    //     if (!valve2.state){
    //       valve2.on();
    //     }
    //     // }
    //     // if (!lock_2 && !motor2.state) {
    //     //   motor2.on(motorspeed);
    //     // }

    //     // if (sensor1.readFiltered() <= setpoint ) {  // If pressure is close to Setpoint, stop motor and lock
    //     //   motor1.off();
    //     //   lock_1 = true;
    //     // }

    //     // if (sensor2.readFiltered() <= setpoint) {  // If pressure is close to Setpoint, stop motor and lock
    //     //   motor2.off();
    //     //   lock_2 = true;
    //     // }

    //     if (millis() - timer >= 500) { // After 50 ms, change state
    //      // lock_1 = false;
    //       state_forward = DEFLATE_2;
    //       timer = millis();
    //     }

    //   case DEFLATE_2: // Inflate chanel 2, deflate chanel 1

    //     if (!valve2.state){
    //       valve2.on();
    //     }
    //     if (valve1.state){
    //       valve1.off();
    //     }
    //     // if (motor1.state){
    //     //   motor1.off();
    //     // }
    //     // if (!lock_2 && !motor2.state) {
    //     //   motor2.on(motorspeed);
    //     // }

    //     // if (sensor2.readFiltered() >= setpoint - 1) {  // If pressure is close to Setpoint, stop motor and lock
    //     //   motor2.off();
    //     //   lock_2 = true;
    //     // }

    //     if (millis() - timer >= 500) { // After 50 ms, change state
    //       state_forward = INFLATE_1_2;
    //      // lock_2 = false;
    //       timer = millis();
    //     }

    //   case INFLATE_1_2:  // Deflate both chanels
    //     if (valve1.state){
    //       valve1.off();
    //     }
    //     if (valve2.state){
    //       valve2.off();
    //     }
    //     // if (motor1.state){
    //     //   motor1.off();
    //     // }
    //     // if (motor2.state){
    //     //   motor2.off();
    //     // }

    //     if (millis() - timer >= 500) { // After 50 ms, change state
    //       state_forward = DEFLATE_1;
    //       timer = millis();
    //     }

    // }



}