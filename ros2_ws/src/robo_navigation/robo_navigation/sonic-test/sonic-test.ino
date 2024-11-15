
#include <NewPing.h>

// Define the pins for the ultrasonic sensors
#define LEFT_TRIG_PIN 2
#define LEFT_ECHO_PIN 3
#define FRONT_TRIG_PIN 4
#define FRONT_ECHO_PIN 5
#define RIGHT_TRIG_PIN 11
#define RIGHT_ECHO_PIN 10

// Define the maximum distance for the sensors
// #define MAX_DISTANCE 200

// Create NewPing objects for each ultrasonic sensor
NewPing leftSensor(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
NewPing frontSensor(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
NewPing rightSensor(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read distances from ultrasonic sensors
  int distance1 = leftSensor.ping_cm();
  int distance2 = frontSensor.ping_cm();
  int distance3 = rightSensor.ping_cm();

  // Print the distances to the Serial Monitor
  Serial.print("Sensor 1: ");
  Serial.print(distance1);
  Serial.print(" cm\t");

  Serial.print("Sensor 2: ");
  Serial.print(distance2);
  Serial.print(" cm\t");

  Serial.print("Sensor 3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  // Add a delay between readings
  delay(500);
}
// */

