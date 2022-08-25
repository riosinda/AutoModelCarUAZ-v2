#include <Servo.h>

Servo servoMotor;

int data;
int max_izquierda = 70; // valor maximo de la izquierda
int max_derecha = 110;  // valor maximo de la derecha
int centro = 90;      // valor del centro

void setup() {
  Serial.begin(9600);
  servoMotor.attach(10);     //pin de salida
  servoMotor.write(centro);  //Inicializar posición de servomotor.
}

void loop() {

  if (Serial.available() > 0){
      String str = Serial.readStringUntil('\n');// leer lo que se recibe por el puerto serial
      data = str.toInt(); // convertir a entero lo que se lee en el puerto serial
      Serial.println(data); // imprimir el entero en el puerto serial
      if(data>=max_izquierda && data<=max_derecha){ // condicion para no pasar los limites del servo y evitar forzarlo
        servoMotor.write(data); // envia el angulo al servo
      } 
   }
}
