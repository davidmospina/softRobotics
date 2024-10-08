#include "Arduino.h"

#define PRESSURE_SENSOR (A1)             //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)

//Arduino PWM Speed Control：
int E1 = 3;    ///<Pump 1 Speed
int M1 = 4;    ///<Pump 1 Direction

int E2 = 11;   ///<Valve 1 Enable
int M2 = 12;  ///<Valve  1 State

const int E3 = 5; ///<Pump 2 Speed
const int M3 = 8; ///<Pump 2 Direction

const int E4 = 6; ///<Valve 2 Enable
const int M4 = 7; ///<Valve 1 Direction

int timecounter = 1;  // Auxiliary variable for controlling the time of the process
bool lock = false;
int stateprocess = 0;

// Motor class definition
class Motor {
  private:
    int speedPin;     // Pin for PWM speed control
    int directionPin; // Pin for motor direction control

  public:
    // Constructor: takes speed pin and direction pin as arguments
    Motor(int spdPin, int dirPin) {
      speedPin = spdPin;
      directionPin = dirPin;
      pinMode(speedPin, OUTPUT);
      pinMode(directionPin, OUTPUT);
    }

    // Turn the motor on at a given speed
    void on(int motorspeed) {
      analogWrite(speedPin, motorspeed);   // PWM Speed Control value
      digitalWrite(directionPin, HIGH);    // Set direction (HIGH or LOW can represent forward/reverse)
    }

    // Turn the motor off
    void off() {
      analogWrite(speedPin, 0);   // Stop the motor by setting speed to 0
      digitalWrite(directionPin, HIGH);    // Keep direction set (optional)
    }
    // comment Pauline
};

// Valve class definition
class Valve {
  private:
    int enablePin;  // Pin for enabling the valve
    int statePin;   // Pin for controlling the valve state (on/off)

  public:
    // Constructor: takes enable pin and state pin as arguments
    Valve(int enPin, int stPin) {
      enablePin = enPin;
      statePin = stPin;
      pinMode(enablePin, OUTPUT);
      pinMode(statePin, OUTPUT);
    }

    // Open the valve
    void on() {
      analogWrite(enablePin, 255);   // Fully open the valve (PWM at max)
      digitalWrite(statePin, HIGH);  // Set state to open
    }

    // Close the valve
    void off() {
      analogWrite(enablePin, 0);     // Close the valve (PWM at 0)
      digitalWrite(statePin, HIGH);  // Keep state set (optional)
    }
};

// PressureSensor class definition
class PressureSensor {
  private:
    int sensorPin;            // Pin for analog sensor input
    float sensorOffset;       // Calibration offset
    float sensorGain;         // Calibration gain
    float alpha;              // Filter coefficient
    float pressure_f = 0;     // Filtered pressure
    float pressure_a = 0;     // Auxiliary filtered pressure

  public:
    // Constructor: takes the pin and calibration parameters as arguments
    PressureSensor(int pin, float offset, float gain, float filterAlpha) {
      sensorPin = pin;
      sensorOffset = offset;
      sensorGain = gain;
      alpha = filterAlpha;
      pinMode(sensorPin, INPUT);  // Define sensor input for ADC
    }

    // Read and calibrate the raw pressure value from the sensor
    float readRaw() {
      return (analogRead(sensorPin) * sensorGain - sensorOffset);
    }

    // Read the filtered pressure value using a low-pass filter
    float readFiltered() {
      float pressure = readRaw();  // Get the raw pressure value
      pressure_f = pressure_f + alpha * (pressure - pressure_a);  // Apply filtering
      pressure_a = pressure_f;
      return pressure_f;
    }
};

// Create instances of Motor, Valve, and PressureSensor classes
Motor motor1(E1, M1);  // Motor 1, speed on pin 3, direction on pin 4
Motor motor2(E3, M3);  // Motor 2, speed on pin 5, direction on pin 8

Valve valve1(E2, M2);  // Valve 1, enable on pin 11, state on pin 12
Valve valve2(E4, M4);    // Valve 2, enable on pin 6, state on pin 7

PressureSensor pressureSensor(PRESSURE_SENSOR_PIN, 4.44, 0.109, 0.2);  // Pressure sensor with calibration

void setup() {
  Serial.begin(115200);  // Start Serial communication

  // Print legend of system parameters
  Serial.print("Process_Status");
  Serial.print(",");
  Serial.print("Pressure_sensor_Value");
  Serial.print(",");
  Serial.print("Filtered_Pressure");
  Serial.println(",");
}

void loop() {
  timecounter++;
  int motorspeed = 100;
  float Setpoint = 20;  // Desired pressure in kPa

  // Read pressure values from the pressure sensor
  float pressure_sensorValue = pressureSensor.readRaw();      // Raw pressure reading
  float pressure_f = pressureSensor.readFiltered();           // Filtered pressure reading

  // Control logic using Setpoint
  if (!lock) {  // Inflation process until reaching the Setpoint (20 kPa)
    motor1.on(250); // Control motor 1
    valve1.on();    // Open valve 1
    stateprocess = 1;
  }

  if (pressure_f >= Setpoint - 1) {  // Once pressure is close to Setpoint, stop motor and lock
    lock = true;
    valve1.on();
    motor1.off();
    stateprocess = 2;
  }

  if (timecounter > 70 && timecounter <= 120) {
    motor1.off();
    valve1.off();
    stateprocess = 3;
  }

  if (timecounter >= 120) {
    timecounter = 0;
    lock = false;
    stateprocess = 4;
  }

  // Print system parameters to serial monitor
  Serial.print(stateprocess);
  Serial.print(",");
  Serial.print(pressure_sensorValue);
  Serial.print(",");
  Serial.print(pressure_f);
  Serial.println(",");

  delay(100);  // Define sample time = 100 milliseconds
}
