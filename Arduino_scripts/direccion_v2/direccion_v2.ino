#include <Servo.h>

Servo servoMotor;

int data;
int max_izquierda = 70; // valor maximo de la izquierda
int max_derecha = 110;  // valor maximo de la derecha
int centro = 90;      // valor del centro
int i = 90;

void setup() {
  Serial.begin(9600);
  servoMotor.attach(10);     //pin de salida
  servoMotor.write(centro);  //Inicializar posición de servomotor.
}

void loop() {

  if (Serial.available() > 0){
      String str = Serial.readStringUntil('\n');
      data = str.toInt();
      Serial.println(data);
      if(data>=70 && data<=110){
        servoMotor.write(data);
      } 
   }
}
