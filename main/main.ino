#include "cuadricula.h"

int error;
int integral = 0;
int derivado = 0;
int lastError = 0;

Robot robot(2,3,30,32,34,36);
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


float giro_z(){ // saca el ángulo de la IMU en z
 mpu.update();
 return mpu.getAngleZ(); 
}


void giro_imu(bool sentido,int valor_giro){
  float giro_inicial,dif;
  dif=0;
  giro_inicial=giro_z();
  if (sentido){
   while(dif < valor_giro)
   {
    dif=abs(giro_z()-giro_inicial);
    
    //Serial.println(dif);
    robot.derecha(80);
   }
   robot.brake();
   return;
  }
  else{
   while(dif < valor_giro )
   {
    dif=abs(giro_z()-giro_inicial);
    //Serial.println(dif);
    robot.izquierda(80);
   } 
   robot.brake();
   return ;     
  } 
}

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
