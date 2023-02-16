#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

Servo ESC;
Servo direccion;

bool Bdato = true;
int dato;
int max_left = 70;
int max_right = 110;
float dir;
float vel;
String cadena1 = "";
String cadena2 = "";


void setup() {
  Serial.begin(115200);
  ESC.attach(12, 1000, 2000);
  direccion.attach(10);
}

void loop() {
  while (Serial.available() > 0) {
    if (Bdato == true) {
      dato = Serial.read();
      if (dato != '*') {
        cadena1 += (char)dato;
      } else {
        dir = (cadena1.toFloat());  //Direccion
        Bdato = false;
        cadena1 = "";
      }
    }

    if (Bdato == false) {
      dato = Serial.read();
      if (dato != '\n' and dato != NULL) {  //NULL=Vacio
        cadena2 += (char)dato;
      } else if (dato == '\n') {
        vel = (cadena2.toFloat());  //Velocidad
        Bdato = true;
        cadena2 = "";
      }
    }

  }

  if (dir >= max_left && dir <= max_right) {  // condicion para no pasar los limites del servo y evitar forzarlo
    direccion.write(dir);                     // envia el angulo al servo
  }

  vel=vel+90;

  if (vel >= (0) && vel <= 180) {  // condicion para no pasar los limites del motor y evitar forzarlo
    ESC.write(vel);              // envia la velocidad al motor
    delay(100);
  }

}
