#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

Servo ESC;
Servo direccion;
MPU6050 sensor;

int dato = 0;
String cadena = "";
String cadena2 = "";
float dat1 = 90;  // direccion
float dat2;
bool Bdato = true;
int data_vel;
int entrada = 0;
int data_dir = 0;
int max_izquierda = 70;  // valor minimo de la izquierda dir
int max_derecha = 110;   // valor maximo de la derecha dir
int centro = 90;         // valor del centro dir
int dat22 = 0;
int sw=1;
int var=1;
int ax, ay, az;
int gx, gy, gz;

String ax1, ay1, az1, gx1, gy1, gz1, vS, distS = "";

int encoder = 3;         // pin de conexiòn del encoder
int N = 14;              // nùmero de ranuras del encoder
float diametro = 6.732;  //diametro de la llanta (cm)

volatile int vel = 0;
volatile unsigned muestreoActualInterrupcion = 0;  // variables para definiciòn del tiempo de interrupciòn y calculo de la velocidad motor
volatile unsigned muestreoAnteriorInterrupcion = 0;
volatile unsigned deltaMuestreoInterrupcion = 0;

double frecuencia = 0;
double W = 0;                                                    // Velocidad angular
double V = 0;                                                    // velocidad lineal
double distancia = 0;                                            //Distancia recorrida total.
double p_llanta = 3.1415926535897932384626433832795 * diametro;  //Perimetro llanta
double d_llanta = p_llanta / N;

void setup() {
  Serial.begin(115200);
  ESC.attach(12, 1000, 2000);  // Attach the ESC on pin 9 (pin, min pulse width, max pulse width in microseconds)
  ESC.write(90);              // Motor en posición de stop.
  direccion.attach(10);
  direccion.write(centro);  // Iniciar servo en posicion centro.
  Wire.begin();
  sensor.initialize();

  attachInterrupt(digitalPinToInterrupt(encoder), Encoder, FALLING);
}

void Encoder() {
  distancia += d_llanta / 100;
  deltaMuestreoInterrupcion = muestreoActualInterrupcion - muestreoAnteriorInterrupcion;  // diferencia tiempos de interruciones de ticks del motor                                                                             // funciòn de interrupciòn del enconder llanta derecha
  muestreoAnteriorInterrupcion = muestreoActualInterrupcion;                              // se actualiza el tiempo de interrupciòn anterior
  frecuencia = (1000) / (double)deltaMuestreoInterrupcion;                                // frecuencia de interrupciòn llanta
  W = ((2 * 3.141516) / N) * frecuencia;                                                  // frecuencia angular (Rad/s)
  V = (W * (diametro / 2)) / 100;                                                         // velocidad lineal (m/s)
}

void loop() {
  while (Serial.available() > 0) {
    if (Bdato == true) {
      dato = Serial.read();
      if (dato != '*') {
        cadena += (char)dato;
      } else {
        dat1 = (cadena.toFloat());  //Direccion
        Bdato = false;
        cadena = "";
      }
    }
    if (Bdato == false) {
      dato = Serial.read();
      if (dato != '\n' and dato != NULL) {  //NULL=Vacio
        cadena2 += (char)dato;
      } else if (dato == '\n') {
        dat2 = (cadena2.toFloat());  //Velocidad
        Bdato = true;
        cadena2 = "";
      }
    }
  }

  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  float ax_m_s2 = ax * (9.81 / 16384.0);
  float ay_m_s2 = ay * (9.81 / 16384.0);
  float az_m_s2 = az * (9.81 / 16384.0);
  float gx_deg_s = gx * (250.0 / 32768.0);
  float gy_deg_s = gy * (250.0 / 32768.0);
  float gz_deg_s = gz * (250.0 / 32768.0);

  if (dat1 >= max_izquierda && dat1 <= max_derecha) {  // condicion para no pasar los limites del servo y evitar forzarlo
    direccion.write(dat1);                             // envia el angulo al servo
  }

  dat22 = dat2 + 90;
  if(var!=dat2){
    sw=1;
  }
  var=dat2;
  
  if(var)
  if (dat22 >= (0) && dat22 <= 180) {  // condicion para no pasar los limites del motor y evitar forzarlo
    if(sw==1){
      if(dat2<=18&&dat2>0){
        ESC.write(dat22+10);
        delay(250);
        sw=0;
        ESC.write(dat22-10);
      
      }
    }
    
    ESC.write(dat22);                  // envia la velocidad al motor
  }

  //Tacometro
  muestreoActualInterrupcion = millis();  // se asigna el tiempo de ejecuciòn a el muestreo actual

  if (V <= 8) {
    ax1 = String(ax_m_s2);
    ay1 = String(ay_m_s2);
    az1 = String(az_m_s2);
    gx1 = String(gx_deg_s);
    gy1 = String(gy_deg_s);
    gz1 = String(gz_deg_s);
    vS = String(V);
    distS = String(distancia);
    Serial.print(ax1 + "," + ay1 + "," + az1 + "," + gx1 + "," + gy1 + "," + gz1 + "," + vS + "," + distS + "\n");
  }

  if ((muestreoActualInterrupcion - muestreoAnteriorInterrupcion) > 50) {
    V = 0;
  }
  delay(50);
}


// De -90 a +90 es la velocidad que se interpretara en e codigo como de 0 a 180(Máxima).

//(Dirección)*(Velocidad)
//(70-110)*(-90  +90)               VELOCIDAD: -90 es hacia atras, +90 es hacia adelante. En el código: -90 es 0(Máxima hacia atrás) y 90 es 180 (Máxima)
//(90 centro)*(0 es stop)

// conf. Dirección, conf. velocidad, ax, ay, az, gx, gy, gz, velocidad_rpm, distancia