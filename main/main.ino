#include "cuadricula.h"

int error;
int integral = 0;
int derivado = 0;
int lastError = 0;

MPU6050 mpu(Wire);

// Robot
Robot robot(2, 3, 30, 32, 34, 36);

// Cuadricula
int robs[] = {1,2,4};
int cobs[] = {1,2,4};
Cuadricula cuadricula(6, 6, robs, cobs, 3);

// Planificador
int* Recorrido = (int*)malloc(36*sizeof(int));
int* Movimientos = (int*)malloc(36*sizeof(int));
int* Orientacion = (int*)malloc(36*sizeof(int));
int nRecorrido;
int count = 1;

void IMUcalibration() {
  digitalWrite(LED_BUILTIN, HIGH); //Encender el led al empezar la calibración
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  digitalWrite(LED_BUILTIN, LOW); //Apagar el led al acabar la calibración
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


void giro(bool sentido, int tgiro) { // false = giro izquierda, true = giro derecha
  if (sentido) {
    robot.derecha(80);
    delay(tgiro);
    robot.brake();
    return;
  }
  else {
    robot.izquierda(80);
    delay(tgiro);
    robot.brake();
    return ;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  robot.QTRcalibration();
  //IMUcalibration(); // La IMU ha muerto, oremos
  nRecorrido = cuadricula.Planner(4, 0, 2, 5, Recorrido);
  cuadricula.MovGenerator(nRecorrido, Recorrido, Movimientos, Orientacion);
  Serial.print("nRecorrido:");
  Serial.print(nRecorrido);
  Serial.print("\t Resultados:");
  Serial.print(Recorrido[0]);
  Serial.print('\t');
  Serial.print(Recorrido[1]);
  Serial.print('\t');
  Serial.print(Recorrido[2]);
  Serial.print('\t');
  Serial.print(Recorrido[3]);
  Serial.print('\t');
  Serial.print(Recorrido[4]);
  Serial.print('\t');
  Serial.println(Recorrido[5]);
  Serial.print('\t');
  Serial.println(Recorrido[6]);
  
  Serial.print("\t Movimientos:");
  Serial.print(Movimientos[0]);
  Serial.print('\t');
  Serial.print(Movimientos[1]);
  Serial.print('\t');
  Serial.print(Movimientos[2]);
  Serial.print('\t');
  Serial.print(Movimientos[3]);
  Serial.print('\t');
  Serial.print(Movimientos[4]);
  Serial.print('\t');
  Serial.println(Movimientos[5]);
  Serial.print('\t');
  Serial.println(Movimientos[6]);
}

void loop() {
  while (!robot.checkIntersection()) {
    robot.siguelineas(&integral, &lastError);
    /*if (robot detecta obstaculo) {
      ori_inicio = Orientacion[count-1];
      posr = cuadricula.getRow();
      posc = cuadricula.getCol();
      nRecorrido = cuadricula.Planner(posr, posc, 2, 5, Recorrido);
      cuadricula.MovGenerator(nRecorrido, Recorrido, Movimientos, Orientacion, ori_inicio);
    }*/
  }
  digitalWrite(LED_BUILTIN, HIGH); // Encender al detectar intersección
  robot.brake();
  delay(200);
  switch (Movimientos[count]){
    case 0:
      robot.brake();
      break;
    case 1:
      giro(false, 750);
      break;
    case 2:
      giro(true, 750);
      break;
    case 3:
      giro(true, 2000);
      break;
    case 4:
      robot.adelante(50,50);
      delay(500);
      robot.brake();
      delay(5000);
      break;
    default:
      break;
  }
  count++;
  delay(100);
  digitalWrite(LED_BUILTIN, LOW); //Apagar el led al acabar la intersección
}
