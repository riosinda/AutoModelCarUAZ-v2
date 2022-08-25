#include <Servo.h>

Servo ESC;     // create servo object to control the ESC
int data;

void setup() {
  
  Serial.begin(9600);
  ESC.attach(9,1000,2000); // Attach the ESC on pin 9 (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(0);
}

void loop() {
  
  if (Serial.available() > 0)
  {
      String str = Serial.readStringUntil('\n');// leer lo que se recibe por el puerto serial
      data = str.toInt(); // convertir a entero lo que se lee en el puerto serial
      Serial.println(data); // imprimir el entero en el puerto serial
      if(data>=0 && data<=180) // condicion para no pasar los limites del motor y evitar forzarlo
        ESC.write(data); // envia la velocidad al motor
  }     
}

//90 es alto
//máximo derecho: 180
//>90 adelante
//<90 atras 87 reversa controlada
