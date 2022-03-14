#include "cuadricula.h"
#include <QTRSensors.h>
int error;
int integral = 0;
int derivado = 0;
int lastError = 0;

Robot robot(2, 3, 30, 32, 34, 36);
//Cuadricula cuadricula(9,9,1,1);
MPU6050 mpu(Wire);


void IMUcalibration() {
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}


float giro_z() { // saca el ángulo de la IMU en z
  mpu.update();
  return mpu.getAngleZ();
}


void giro_imu(bool sentido, int valor_giro) { // false = giro izquierda, true = giro derecha
  float giro_inicial, dif;
  dif = 0;
  giro_inicial = giro_z();
  if (sentido) {
    while (dif < valor_giro)
    {
      dif = abs(giro_z() - giro_inicial);

      //Serial.println(dif);
      robot.derecha(80);
    }
    robot.brake();
    return;
  }
  else {
    while (dif < valor_giro )
    {
      dif = abs(giro_z() - giro_inicial);
      //Serial.println(dif);
      robot.izquierda(80);
    }
    robot.brake();
    return ;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  //robot.QTRcalibration();
  //IMUcalibration(); // Falla la calibración porque xdddddddddddddddddddddddddddddddddddddd
  Casilla* Recorrido;
  int robs[] = {1,2,4};
  int cobs[] = {1,2,4};
  Cuadricula cuadricula(5, 5, robs, cobs, 3); // FUNCIONA
  //cuadricula.printTablero();
  Recorrido = cuadricula.Planner(0, 0, 4, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
  // robot.derecha(2);
  // robot.adelante(200, 200);
  // robot.siguelineas(&integral, &lastError);
  // robot.derecha(80);
//  while (!robot.checkIntersection()) {
//    
////    Serial.print(robot.checkIntersection());
////    Serial.println();
//  }
//  robot.brake();
//  delay(500);
//  giro_imu(false, 69-4);
//  delay(500);
//  if (robot.checkIntersection()){
//    robot.brake();
//    giro_imu(false, 69-4);
//    robot.brake();
//  }
//  robot.siguelineas(&integral, &lastError);
}
