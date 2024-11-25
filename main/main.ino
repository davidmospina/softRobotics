#include <Servo.h>

#define SERVO_1 (9) //RIGHT
#define SERVO_2 (10) //LEFT
#define SWITCH_1_RIGHT (1)
#define SWITCH_1_LEFT (2)
#define SWITCH_2 (13)

// ________________________________________________Arduino PWM Speed Control_______________________________________________________________________//

// M1 = Pump1
int E1 = 3; //Speed
int M1 = 4; //Direction

// M2 = Valv1
int E2 = 11;   //Enable
int M2 = 12;  //State

// M3 = Pump2
const int E3 = 5;
const int M3 = 8;

// M4 = Valv2
const int E4 = 6;
const int M4 = 7;

// ______________________________________________________Variables_________________________________________________________________________________//
int motorspeed = 150;
int time = 500;
int pos_servo_1 = 50; //in degree between 0 and 180
int pos_servo_2 = 50; //in degree between 0 and 180

//___________________________________________________________States________________________________________________________________________________//
enum State {
  DEFLATE_1,
  DEFLATE_1_2,
  DEFLATE_2,
  INFLATE_1_2,
};

State state_forward = DEFLATE_1;

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

    void off() {
      analogWrite(enablePin, 255);
      digitalWrite(statePin, HIGH);
      state = false;
      //Serial.println("try to on valve");
    }
      
    void on() {
      analogWrite(enablePin, 0);
      digitalWrite(statePin, HIGH);
      state = true;
    }
};

Motor motor1(E1, M1); // M1 = Pump1
Motor motor2(E3, M3); // M3 = Pump2

Valve valve1(E2, M2); // M2 = Valv1
Valve valve2(E4, M4); // M4 = Valv2

Servo servo1;
Servo servo2;

// ______________________________________________________Fonctions_________________________________________________________________________________//

void goForward() {
  
    if (!motor1.state){
      motor1.on(motorspeed);
    }
    if (!motor2.state){
      motor2.on(motorspeed);
    }

    valve1.on();
    delay(time);
    valve2.on();
    delay(time);
    valve1.off();
    delay(time);
    valve2.off();
    delay(time);
}


// ________________________________________________________Set up_________________________________________________________________________________//
void setup() {
  //Serial.begin(115200);
  valve1.off();
  valve2.off();
  motor1.off();
  motor2.off();
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  pinMode(SWITCH_1_RIGHT, INPUT_PULLUP); //right position : digitalRead(2) = 1 & digitalRead(3) = 0
  pinMode(SWITCH_1_LEFT, INPUT_PULLUP); //left position : digitalRead(3) = 1 & digitalRead(2) = 0
                                       // middle position : digitalRead(2) = digitalRead(3) = 1
  pinMode(SWITCH_2, INPUT_PULLUP);

}

// __________________________________________________________Loop__________________________________________________________________________________//
void loop() {
  if (digitalRead(SWITCH_2)==0){
    goForward();

    if (digitalRead(SWITCH_1_RIGHT) == 0){
      servo2.write(pos_servo_2);
    } else if (digitalRead(SWITCH_1_LEFT) == 0){
      servo1.write(pos_servo_1);
    } else{
      servo1.write(0);
      servo2.write(0);
    }
  } else{
    valve1.off();
    valve2.off();
    motor1.off();
    motor2.off();
  }
  

  

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