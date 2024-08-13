// Arduino sketch that powers my self-balancing robot

// I2Cdev, MPU6050, and PID must be installed as libraries
#include "I2Cdev.h" // I2Cdev
// #include "MPU6050.h" // MPU6050
#include "MPU6050_6Axis_MotionApps20.h" // MPU6050
#include <PID_v1.h> // Arduino PID Library

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

//////////// MPU ////////////
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer/Users/daryatavvafi/Library/CloudStorage/OneDrive-UniversityofToronto/Summer (2024)/Personal Project/sketch_jun23a/MPU6050_Offset_Calibration.ino

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//////////// PID ////////////
double setpoint= 179.75; // set the value when the bot is perpendicular to ground using serial monitor
// read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 20; // set this first
double Kd = 0.8; // set this secound
double Ki = 80; // set this last
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// interrupt
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

//////////// SETUP ////////////
void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  Serial.begin(115200); // or 38400
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  // prints with carriage return (/r) and newline (/n)
  // F() tells compiler to keep a string inside of PROGMEM and not allow it to consume RAM at a small performance cost
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(112);
  mpu.setYGyroOffset(27);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1308);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // initialise the motor output pins
    pinMode (9, OUTPUT); // IN1
    pinMode (8, OUTPUT); // IN2
    pinMode (7, OUTPUT); // IN3
    pinMode (6, OUTPUT); // IN4
    pinMode(10, OUTPUT); // ENA
    pinMode(5, OUTPUT); // ENB


  // by default turn off both the motors
    // clockwise
    analogWrite(9,LOW);
    analogWrite(8,LOW);
    analogWrite(7,LOW);
    analogWrite(6,LOW);
    analogWrite(10, 0);
    analogWrite(5, 0);
}

// define wheel control functions
void Forward() // code to rotate the wheel forward 
{
  // clockwise
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);
  digitalWrite(7, HIGH);
  digitalWrite(6, LOW);
  analogWrite(10, output*0.8);
  analogWrite(5, output);
  Serial.print("F"); // debugging information 
}

void Reverse() // code to rotate the wheel Backward  
{
  // anti-clockwise
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(7, LOW);
  digitalWrite(6, HIGH);
  analogWrite(10, output*-0.8);
  analogWrite(5, output*-1);
  Serial.print("R");
}

void Stop() // code to stop both the wheels
{
  // clockwise
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);
  digitalWrite(7, HIGH);
  digitalWrite(6, LOW);
  analogWrite(10, 0);
  analogWrite(5, 0);
  Serial.print("S");
}

//////////// LOOP ////////////
void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // no mpu data - performing PID calculations and output to motors     
    pid.Compute();

    // print the value of Input and Output on serial monitor to check how it is working
    Serial.print(input); Serial.print(" =>"); Serial.println(output);
  
    // if the robot is falling
    if (input>150 && input<210)
    {
      if (output>0) // falling towards front
      {
        Forward(); // rotate the wheels forward 
      }
    else if (output<0) // falling towards back
      {
        Reverse(); // rotate the wheels backward 
      }
    else // if robot not falling
      {
        Stop(); // hold the wheels still
      }
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
    mpu.dmpGetGravity(&gravity, &q); //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

    input = ypr[1] * 180/M_PI + 180;
  }
}