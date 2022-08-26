#include <Servo.h>
 
Servo ESC;     // create servo object to control the ESC
Servo servoMotor;

int data_vel;
int entrada=0;
int data_dir=0;
int max_izquierda = 70; // valor maximo de la izquierda
int max_derecha = 110;  // valor maximo de la derecha
int centro = 90;      // valor del centro

char c;

void setup() {
  
  Serial.begin(9600);
  ESC.attach(9,1000,2000); // Attach the ESC on pin 9 (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(90);

  servoMotor.attach(10);     //pin de salida
  servoMotor.write(centro);  //Inicializar posición de servomotor.
}

void loop() {

  while (Serial.available() > 0){
    c=Serial.read();

    if(c=='V'){
      entrada = Serial.parseInt();
    }
    if(c=='D'){
      data_dir = Serial.parseInt();
    }
  }
  
  
      //String str_vel = Serial.readStringUntil('\n');// leer lo que se recibe por el puerto serial
      //entrada = str_vel.toInt(); // convertir a entero lo que se lee en el puerto serial
      data_vel = entrada + 90;
      Serial.println(data_vel); // imprimir el entero en el puerto serial
      if(data_vel>=0 && data_vel<=180){ // condicion para no pasar los limites del motor y evitar forzarlo
        ESC.write(data_vel); // envia la velocidad al motor
      }

      //String str_dir = Serial.readStringUntil('\n');// leer lo que se recibe por el puerto serial
      //data_dir = str_dir.toInt(); // convertir a entero lo que se lee en el puerto serial
      Serial.println(data_dir); // imprimir el entero en el puerto serial
      if(data_dir>=max_izquierda && data_dir<=max_derecha){ // condicion para no pasar los limites del servo y evitar forzarlo
        servoMotor.write(data_dir); // envia el angulo al servo
  }     
}
