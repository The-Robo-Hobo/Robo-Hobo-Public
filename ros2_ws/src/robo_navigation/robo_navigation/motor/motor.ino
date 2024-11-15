#include <NewPing.h>

// DEFINE GLOBAL VARIABLES
String serialData;
int data;

// motor pin configurations
// LEFT MOTOR
#define IN1 7
#define IN2 6
// RIGHT MOTOR
#define IN3 8
#define IN4 9

// Define the pins for the ultrasonic sensors
#define LEFT_TRIG_PIN 2
#define LEFT_ECHO_PIN 3
#define FRONT_TRIG_PIN 4
#define FRONT_ECHO_PIN 5
#define RIGHT_TRIG_PIN 11
#define RIGHT_ECHO_PIN 10

// Define the maximum and minimum distances for obstacle detection (in centimeters)
#define MIN_DISTANCE 70

// Define variables to be used for battery monitoring
#define ANALOG_PIN A0
#define MAX_VOLTAGE 13.5
#define REF_VOLTAGE 10.0
float voltage, percent, voltage_diff;
unsigned long previousMillis = 0;  // Stores last time voltage was sent
const long interval = 1000; 

// Create NewPing objects for each ultrasonic sensor
NewPing leftSensor(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
NewPing frontSensor(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
NewPing rightSensor(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);


void setup() {
  Serial.begin(115200);       // initializes Serial communication
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ANALOG_PIN, INPUT_PULLUP);

  // clear any initial data in serial buffer
  while (Serial.available() > 0){ 
    Serial.read();
  }
}


void loop() {
  unsigned long currentMillis = millis();
  sendVoltagePercentToSerial(currentMillis);

  if (Serial.available() > 0){    // receives the number of bytes
    // read string until new line
    serialData = Serial.readStringUntil('\n');    // 1,2,3,4,5 = left, front, right, stop, listen to ult

    // convert data from serial to int
    data = serialData.toInt();
    // data = Serial.parseInt();
    
    if ((data < 6) && (data > 0)){

      // Read ultrasonic sensors and return sensor distance
      int objectDistanceInLeft = leftSensor.ping_cm();
      int objectDistanceInFront = frontSensor.ping_cm();
      int objectDistanceInRight = rightSensor.ping_cm();

      // main loop
      moveFunction(data, objectDistanceInLeft, objectDistanceInFront, objectDistanceInRight);
    }
  }
}


void sendVoltagePercentToSerial(unsigned long currentMillis){
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // update prev time with current time

    // Read the voltage from the analog pin
    voltage = analogRead(ANALOG_PIN) * (MAX_VOLTAGE / 1023.0);  // convert to voltage
    voltage_diff = voltage - REF_VOLTAGE; // take voltage diff from reference
    percent = (voltage_diff / 3.5) * 100;  // convert to percent
    
    if (percent > 100){
      percent = 100.00;
    } else if (percent < 0.00){
      percent = 0.0;
    }

    // Send the batter percen reading to the serial port every 10 seconds
    Serial.print("Percent: ");
    Serial.println(percent, 0);
  }
}


// using the concept of FUNCTION OVERLOADING to get the data type of contents in serial
void types(String a) { Serial.println("String"); }
void types(int a) { Serial.println("int"); }
void types(float a) { Serial.println("float"); }
void types(bool a) { Serial.println("bool"); }


// /*
void debugUltrasonicReadings(int left, int front, int right){
    Serial.print(F("Left sensor: "));
    Serial.print(left);
    Serial.print(F(" cm\t"));

    Serial.print(F("Front sensor: "));
    Serial.print(front);
    Serial.print(F(" cm\t"));

    Serial.print(F("Right sensor: "));
    Serial.print(right);
    Serial.println(F(" cm"));
}
// */

void moveFunction(int datum, int left, int front, int right){

  switch (datum){
    case 1:
      if ((objectIsCloseAt(front)) || (objectIsCloseAt(left)) || (objectIsCloseAt(right))){roamAround(left, front, right);}
      move_forward();
      break;

    case 2:
      if ((objectIsCloseAt(front)) || (objectIsCloseAt(left)) || (objectIsCloseAt(right))){roamAround(left, front, right);}
      move_forward_left();
      break;

    case 3:
      if ((objectIsCloseAt(front)) || (objectIsCloseAt(left)) || (objectIsCloseAt(right))){roamAround(left, front, right);}
      move_forward_right();
      break;

    case 4:
      stop();
      break;

    default:
      roamAround(left, front, right);
      break;
  }
}


void roamAround(int left, int front, int right){
  // listens to ult, default in switch case, heat orientation is false
  if (objectIsCloseAt(front) && objectIsCloseAt(left) && objectIsCloseAt(right)){
    turn_right();
  }

  else if (objectIsCloseAt(front) && objectIsCloseAt(left)){
    turn_right();
  }

  else if (objectIsCloseAt(front) && objectIsCloseAt(right)){
    turn_left();
  }

  else if (objectIsCloseAt(front)){
    turn_right();
  }

  else{
    move_forward();
  }
}


bool objectIsCloseAt(int sensorReading){
  if ((sensorReading < MIN_DISTANCE) && (sensorReading != 0)){
    return true;
  } else {
    return false;
  }
}


void move_forward() {
  // Move forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void move_forward_right() {
  // Move forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


void move_forward_left() {
  // Move forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void move_backward() {
  // Move backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}


void turn_left() {
  // Turn left
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void turn_right() {
  // Turn right
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}


void stop() {
  // Stop
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}
