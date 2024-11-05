#define PRESSURE_SENSOR_1 (A1) //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)
#define PRESSURE_SENSOR_2 (A2)

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

//___________________________________________________________States________________________________________________________________________________//
enum State {
  INFLATE_1,
  INFLATE_1_2,
  INFLATE_2,
  DEFLATE_1_2,
};

State state = INFLATE_1;
State previousState;

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

class PressureSensor {
  private:
    int sensorPin;
    float sensorOffset;
    float sensorGain;
    float alpha;
    float pressure_f = 0;
    float pressure_a = 0;

  public:
    PressureSensor(int pin, float offset, float gain, float filterAlpha){
      sensorPin = pin;
      sensorOffset = offset;
      sensorGain = gain;
      alpha = filterAlpha;
      pinMode(sensorPin, INPUT);
    }

    float readRaw() {
      return(analogRead(sensorPin) * sensorGain - sensorOffset);
    }

    float readFiltered() {
      float pressure = readRaw();
      pressure_f = pressure_f + alpha * (pressure - pressure_a);
      pressure_a = pressure_f;
      return pressure_f;
    }
};


// ______________________________________________________Variables_________________________________________________________________________________//
int timer;
bool lock_1 = false;
bool lock_2 = false;
float setpoint = -3;
int motorspeed = 100;

Motor motor1(E1, M1); // M1 = Pump1
Motor motor2(E3, M3); // M3 = Pump2

Valve valve1(E2, M2); // M2 = Valv1
Valve valve2(E4, M4); // M4 = Valv2

PressureSensor sensor1(PRESSURE_SENSOR_1, 4.44, 0.109, 0.2);
PressureSensor sensor2(PRESSURE_SENSOR_2, 4.44, 0.109, 0.2);


void printStatus() {
  Serial.print("Control state : ");
  Serial.print(state);
  Serial.print(" | Valve 1 state : ");
  Serial.print(valve1.state);
  Serial.print(" | Pressure 1 : ");
  Serial.print(sensor1.readFiltered());
  Serial.print(" | Pressure 2 : ");
  Serial.print(sensor2.readFiltered());
  Serial.print(" | Valve 2 state : ");
  Serial.println(valve2.state);
}

// ________________________________________________________Set up_________________________________________________________________________________//
void setup() {
  Serial.begin(115200);
  valve1.off();
  valve2.off();
  motor1.off();
  motor2.off();
  timer = millis();

}

// __________________________________________________________Loop__________________________________________________________________________________//
void loop() {
    delay(1000);
    // printStatus();

  switch (state) {

    
    
    case INFLATE_1:     // Inflate chanel 1, deflate chanel 2
      if (!valve1.state){
        valve1.on();
      }
      if (valve2.state){
        motor2.off();
        valve2.off();
      }
        
      if (!lock_1 && !motor1.state) {
        motor1.on(motorspeed);
      }

      if (sensor1.readFiltered() <= setpoint) {  
        // If pressure is close to Setpoint, stop motor and lock
        // printStatus();
        motor1.off();
        lock_1 = true;
        printStatus();

      }

      if (millis() - timer >= 10000) { // After 50 ms, change state
        state = INFLATE_1_2;
        timer = millis();
        Serial.println("timeout reached");

      }

      
    case INFLATE_1_2: // Inflate both chanels

      if (!valve1.state){
        valve1.on();
      }
      if (!lock_1 && !motor1.state ) {
        motor1.on(motorspeed);
      }
      if (!valve2.state){
        valve2.on();
      }
      if (!lock_2 && !motor2.state) {
        motor2.on(motorspeed);
      }


      if (sensor1.readFiltered() <= setpoint ) {  
        // IF pressure is close to Setpoint, stop motor and lock
        motor1.off();
        lock_1 = true;
      }

      if (sensor2.readFiltered() <= setpoint) {  
        // If pressure is close to Setpoint, stop motor and lock
        motor2.off();
        lock_2 = true;
      }

      // if (millis() - timer >= 10000) { // After 50 ms, change state
      //   lock_1 = false;
      //   state = INFLATE_2;
      //   timer = millis();
      // }

    //   case INFLATE_2: // Inflate chanel 2, deflate chanel 1
    //     if (!valve2.state){
    //       valve2.on();
    //     }

    //     if (valve1.state){
    //     valve1.off();
    //     }

    //     motor1.off();

    //     if (!lock_2) {
    //       motor2.on(motorspeed);
    //     }

    //   if (sensor2.readFiltered() >= setpoint - 1) {  
    //     // If pressure is close to Setpoint, stop motor and lock
    //     motor2.off();
    //     lock_2 = true;
    //   }

    //   if (millis() - timer >= 5000) { // After 50 ms, change state
    //     state = DEFLATE_1_2;
    //     lock_2 = false;
    //     timer = millis();
    //   }

    //   case DEFLATE_1_2:  // Deflate both chanels
    //   if (valve1.state){
    //     valve1.off();
    //     }

    //   if (valve2.state){
    //     valve2.off();
    //     }
    //     motor2.off(); 
    //     valve2.off();

    //   if (millis() - timer >= 5000) { // After 50 ms, change state
    //     state = INFLATE_1;
    //     timer = millis();
    //   }

  }

}