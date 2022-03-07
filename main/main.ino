#include "cuadricula.h"

int error;
int integral = 0;
int derivado = 0;
int lastError = 0;

Robot robot(2,3,30,32,34,36);
//Cuadricula cuadricula(9,9,1,1);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  robot.QTRcalibration();
  Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:
  // robot.derecha(2);
  // robot.adelante(200, 200);
  robot.siguelineas(&integral, &lastError);
}
