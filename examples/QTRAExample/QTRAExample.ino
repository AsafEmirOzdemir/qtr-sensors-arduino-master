#include <QTRSensors.h>

// This example is designed for use with six analog QTR sensors. These
// reflectance sensors should be connected to analog pins A0 to A5. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is. Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 5000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}


-------------------------------------

#include <DRV8835MotorShield.h>


#include <QTRSensors.h>


#define Kp 0.05 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 2 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2


DRV8835MotorShield motors; //don't forget this line


QTRSensorsAnalog qtra((unsigned char[]) {  0, 1, 2, 3, 4, 5} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19

unsigned int sensorValues[NUM_SENSORS];

void setup()
{

motors.flipM1(true);
motors.flipM2(true);
  
  
  

  /* comment this part out for automatic calibration 
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right();  
   else
     turn_left(); */ 
   qtra.calibrate(QTR_EMITTERS_ON);   
   delay(20);
  
delay(10000); // wait for 10s to position the bot before entering the main loop 
    
    /* comment out for serial printing
    
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    */
  } 

int lastError = 0;

void loop()
{
  unsigned int sensors[6];
  int position = qtra.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2500;

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
    if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
   {
  // move forward with appropriate speeds

  motors.setM1Speed(rightMotorSpeed); //only works if "DRV8835MotorShield motors;" is defined in the beginning
  motors.setM2Speed(leftMotorSpeed);
 
}
}

-----------------------------------------------------

#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];

void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);
  delay(500);

  for(uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  Serial.println(qtr.calibrationOn.maximum[0]);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  /*for(int i = 0; i < sensorCount; i++) {
    Serial.print(sensorValues[i] > 700 ? 1 : 0);
  }
  Serial.println();*/
  Serial.println(position);
}

---------------------------------------------------

int ss[4] = {3, 5, 6, 9};
int out = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  for(int i = 0; i < 4; i++) {
    pinMode(ss[i], OUTPUT);
  }
  pinMode(out, INPUT);

  digitalWrite(ss[0], HIGH);
  digitalWrite(ss[1], LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ss[2], LOW);
  digitalWrite(ss[3], LOW);
  Serial.print(pulseIn(out, LOW));
  Serial.print(" - ");

  delay(50);

  digitalWrite(ss[2], HIGH);
  digitalWrite(ss[3], HIGH);
  Serial.print(pulseIn(out, LOW));
  Serial.print(" - ");

  delay(50);

  digitalWrite(ss[2], LOW);
  digitalWrite(ss[3], HIGH);
  Serial.print(pulseIn(out, LOW));
  Serial.print(" - ");

  Serial.println();
}

---------------------------------------------

float* getDistance() {
  static float values[3] = {0.0, 0.0, 0.0};

  for(int i = 0; i < 3; i++) {
    digitalWrite(trigPin[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[i], LOW);

    duration = pulseIn(echoPin[i], HIGH);
    distance = (duration*.0343)/2;
    values[i] = distance;
  }

  return values;
}

-----------------------------------------------

int trigPin = 13; 
int echoPin = 12;  
long zaman;
long mesafe;

void setup() {ö
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin,INPUT); 
  Serial.begin(9600); 
}

void loop()
{
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);  
  zaman = pulseIn(echoPin, HIGH);
  mesafe= (zaman /29.1)/2;    
  Serial.print("Uzaklik ");  
  Serial.print(mesafe);
  Serial.println(" cm");  
  delay(500); 
}
