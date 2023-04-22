#include "I2Cdev.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

Servo ESC;
Servo direccion;
MPU6050 mpu;

#define PIN 8
#define NUMPIXELS 21
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
char command;
char NumberColor;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

int dato = 0;
String cadena = "";
String cadena2 = "";
String cadena3 = "";
float dat1 = 90;  // direccion
float dat2;
int dat3 = 0;
bool Bdato = true;
bool Bdatoo = true;
int data_vel;
int entrada = 0;
int data_dir = 0;
int max_izquierda = 55;  // valor minimo de la izquierda dir
int max_derecha = 125;   // valor maximo de la derecha dir
int centro = 90;         // valor del centro dir
int dat22 = 0;
int sw = 1;
int var = 1;

int val;
volatile uint16_t T1Ovs2;  //
volatile uint16_t T2OverFlow;
int16_t Encoder;  //Counter of rising edges
int16_t last_Encoder;
volatile uint16_t deltatime = (volatile uint16_t)0;
String inputLight = "";          // a string to hold incoming data
String inputServo = "";          // a string to hold incoming data
String inputSpeed = "";          // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean servoComplete = false;
boolean lightComplete = false;
boolean reset_T1Ovs2 = false;
boolean rising_edge = false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;                          // wait for Serial
  ESC.attach(12, 1000, 2000);  // Attach the ESC on pin 9 (pin, min pulse width, max pulse width in microseconds)
  ESC.write(90);
  direccion.attach(10);
  direccion.write(centro);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(1963);
  mpu.setYAccelOffset(3691);
  mpu.setZAccelOffset(1321);  // 1688 factory default for my test chip
  mpu.setXGyroOffset(-7);
  mpu.setYGyroOffset(20);
  mpu.setZGyroOffset(17);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pixels.begin();  // This initializes the NeoPixel library.
}

void loop() {

  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (Serial.available() > 0) {
      if (Bdato == true && Bdatoo == true) {
        dato = Serial.read();
        if (dato != '*') {
          cadena += (char)dato;
        } else {
          dat1 = (cadena.toFloat());  //Direccion
          Bdato = false;
          cadena = "";
        }
      }
      if (Bdato == false && Bdatoo == true) {
        dato = Serial.read();
        if (dato != '$' and dato != NULL) {  //NULL=Vacio
          cadena2 += (char)dato;
        } else if (dato == '$') {
          dat2 = (cadena2.toFloat());  //Velocidad
          Bdato = false;
          Bdatoo = false;
          cadena2 = "";
        }
      }
      if (Bdato == false && Bdatoo == false) {
        dato = Serial.read();
        if (dato != '\n' and dato != NULL) {  //NULL=Vacio
          cadena3 += (char)dato;
        } else if (dato == '\n') {
          dat3 = (cadena3.toInt());  //luces
          //Serial.println(dat3);
          //lucesLED(dat3);
          Bdato = true;
          Bdatoo = true;
          cadena3 = "";
        }
      }
    }

    lightControl(dat3);  // control lights of the car, this function should call befor control motor
    servoControl(dat1);  // control servo motor
    speedControl(dat2);
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //            Serial.print("y");
    //            Serial.println(ypr[0] * 180/M_PI);
    //            Serial.print("\t");
    //            Serial.print(ypr[1] * 180/M_PI);
    //            Serial.print("\t");
    //            Serial.println(ypr[2] * 180/M_PI);
#endif
    /*
      Serial.print("y");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("s");
      Serial.print(deltatime);  ///*0.0000000625 second
      Serial.print("e");
      Serial.print((int)Encoder);
      Serial.print("\n");
*/
    if (rising_edge == true) {
      if (((T1Ovs2)*25 + (T1Ovs2)*5 / 10 + TCNT2 / 10) > 40000) {
        deltatime = (volatile uint16_t)0;
        rising_edge = false;
      }
    }
    last_Encoder = Encoder;
  }
}

/* Control servo */
void servoControl(int dat1) {
  direccion.attach(10);
  if (dat1 >= max_izquierda && dat1 <= max_derecha) {  // condicion para no pasar los limites del servo y evitar forzarlo
    direccion.write(dat1);                             // envia el angulo al servo
  }
}
/* Control lights */
/*L20C32+16+8+4+2+1, 32+16/16=2+1 -> R , 8+4/4=2+1 -> G, 2+1 -> B : WHITE=63, RED=48, YELLOW=56,OR 60*/
void lightControl(int inputLight) {

  int LF[] = {0,1,2,3};
  int RF[] = {6,7,8,9};

  int RB[] = {10,11,12,13,14};
  int LB[] = {16,17,18,19,20};

  if (inputLight == 0){// off
    for (int i = 0; i < 21; i++)
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));  //disable
  } else if (inputLight == 1) {// left
    for (int i = 0; i < 21; i++)
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));  //off
    //lights left on
    pixels.setPixelColor(LF[0], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LF[1], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LF[2], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LF[3], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[0], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[1], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[2], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[3], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[4], pixels.Color(255, 80, 0));  //yellow

  } else if (inputLight == 2) {// right
    for (int i = 0; i < 21; i++)
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));  //off
    //lights right on
    pixels.setPixelColor(RF[0], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RF[1], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RF[2], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RF[3], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[0], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[1], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[2], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[3], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[4], pixels.Color(255, 80, 0));  //yellow

  } else if (inputLight == 3) {// stop
    for (int i = 0; i < 21; i++)
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));  //off
    //lights back
    pixels.setPixelColor(RB[0], pixels.Color(255, 0, 0));  //red
    pixels.setPixelColor(RB[1], pixels.Color(255, 0, 0));  //red
    pixels.setPixelColor(RB[2], pixels.Color(255, 0, 0));  //red
    pixels.setPixelColor(RB[3], pixels.Color(255, 0, 0));  //red
    pixels.setPixelColor(RB[4], pixels.Color(255, 0, 0));  //yellow
    pixels.setPixelColor(LB[0], pixels.Color(255, 0, 0));  //red
    pixels.setPixelColor(LB[1], pixels.Color(255, 0, 0));  //red
    pixels.setPixelColor(LB[2], pixels.Color(255, 0, 0));  //red
    pixels.setPixelColor(LB[3], pixels.Color(255, 0, 0));  //red
    pixels.setPixelColor(LB[4], pixels.Color(255, 0, 0));  //red

  } else if (inputLight == 4) {
    
    for (int i = 0; i < 21; i++)
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));  //red
    
    pixels.setPixelColor(LF[0], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LF[1], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LF[2], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LF[3], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[0], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[1], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[2], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[3], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(LB[4], pixels.Color(255, 80, 0));  //yellow

    pixels.setPixelColor(RF[0], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RF[1], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RF[2], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RF[3], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[0], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[1], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[2], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[3], pixels.Color(255, 80, 0));  //yellow
    pixels.setPixelColor(RB[4], pixels.Color(255, 80, 0));  //yellow


  }
  pixels.show();  // This sends the updated pixel color to the hardware.
}

void speedControl(int dat2) {
  dat22 = dat2 + 90;
  if (dat22 >= (0) && dat22 <= 180) {  // condicion para no pasar los limites del motor y evitar forzarlo
    ESC.write(dat22);                  // envia la velocidad al motor
  }
}
