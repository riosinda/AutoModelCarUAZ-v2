#include <Servo.h>
 
Servo ESC;     // create servo object to control the ESC
Servo direccion;        //servoMotor
int dato=0;
String cadena = "";
String cadena2 = "";
float dat1 = 90;  // dir
float dat2;
bool Bdato= true;
int data_vel;
int entrada=0;
int data_dir=0; 
int max_izquierda = 70; // valor minimo de la izquierda dir
int max_derecha = 110;  // valor maximo de la derecha dir
int centro = 90;      // valor del centro dir
int dat22=0;

void setup() {
  Serial.begin(115200); 
  ESC.attach(9,1000,2000); // Attach the ESC on pin 9 (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(90);
  direccion.attach(10);
  direccion.write(centro);
}

void loop() {
   //si existe información pendiente
  while (Serial.available()>0){
    
    if(Bdato == true){
     dato = Serial.read(); 
     if (dato != '*') { 
      cadena += (char)dato;
    }else{
      dat1 = (cadena.toFloat());
     Bdato = false; 
     cadena="";
     }
  }  
    if (Bdato == false){
      dato = Serial.read();
      if (dato != '\n' and dato != NULL) { 
      cadena2 += (char)dato;
    }else if(dato == '\n'){
      dat2 = (cadena2.toFloat());
      Bdato = true; 
      cadena2=""; 
     }
    } 
  }
  //////////////////////////////////////////
  dat22=dat2+90;
  Serial.println(dat22); // imprimir el entero en el puerto serial
  if(dat22>=(0) && dat22<=180){ // condicion para no pasar los limites del motor y evitar forzarlo
    ESC.write(dat22); // envia la velocidad al motor 
  }
  Serial.println(dat1); // imprimir el entero en el puerto serial
      if(dat1>=max_izquierda && dat1<=max_derecha){ // condicion para no pasar los limites del servo y evitar forzarlo
        direccion.write(dat1); // envia el angulo al servo
      }
  }

 
////////
//70-110 dir
//0>79 reversa
//>90 frente
