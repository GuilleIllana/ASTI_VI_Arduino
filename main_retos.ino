// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <QTRSensors.h>


//Sensor siguelineas
#define Kp 0.5 // experimentar (valores bajos)
#define Ki 0 // no se ni lo que estoy haciendo
#define Kd 30 // experimentar con valores bajos (Ki < Kp < Kd)
#define NUM_SENSORS  8     // number of sensors used

//Velocidades
#define rightMaxSpeed 150 // max speed of the robot (0-255)
#define leftMaxSpeed 150 // max speed of the robot (0-255)
#define rightBaseSpeed 100 // this is the speed at which the motors should spin when the robot is perfectly on the line (0-255)
#define leftBaseSpeed 100  // this is the speed at which the motors should spin when the robot is perfectly on the line (0-255)

//Pines
#define ENA 2
#define IN1 31
#define IN2 32
#define IN3 33
#define IN4 34
#define ENB 3

//Sensor QTR
QTRSensors qtr;
const uint8_t SensorCount = NUM_SENSORS;
uint16_t sensorValues[SensorCount];
int error;
int integral = 0;
int derivado = 0;
int lastError = 0;
int position;
bool inter = false;

// IMU
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

//Variables usadas por el filtro pasa bajos
long f_ax,f_ay, f_az;
int p_ax, p_ay, p_az;
long f_gx,f_gy, f_gz;
int p_gx, p_gy, p_gz;
int counter=0;

//Valor de los offsets
int ax_o,ay_o,az_o;
int gx_o,gy_o,gz_o;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z


long tiempo_prev, dt;
float girosc_ang_x, girosc_ang_y, girosc_ang_z;
float girosc_ang_x_prev, girosc_ang_y_prev,girosc_ang_z_prev;

float giro;


void wait() {
 digitalWrite(ENA, 0);
 digitalWrite(ENB, 0);
  }


void brake(int ret) {
  // Direccion motor A
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, HIGH);
  analogWrite (ENA,0); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB,0); //Velocidad motor B Rueda de la derecha

  if (ret > 0){
  delay(ret);
  }
}


void adelante(int DPWM, int IPWM, int ret) {
  if (DPWM > rightMaxSpeed) DPWM = rightMaxSpeed; //Prevencion
  if (IPWM > leftMaxSpeed) IPWM = leftMaxSpeed; //Prevencion
  if (DPWM < 0) DPWM = 0; 
  if (IPWM < 0) IPWM = 0;

  // Direccion motor A
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA,DPWM); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  analogWrite (ENB,IPWM); //Velocidad motor B Rueda de la derecha

  if (ret > 0){
  delay(ret);
  }
}


void izquierda(int DPWM) {
  // Direccion motor A
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, HIGH);
  analogWrite (ENA,0); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, DPWM); //Velocidad motor B Rueda de la derecha

}

void derecha(int IPWM) {

  // Direccion motor A
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA,IPWM); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB,0); //Velocidad motor B Rueda de la derecha

}

bool checkIntersection() {
  int position = qtr.readLineBlack(sensorValues); //Lectura de la posición de la linea con respecto al robot
  if (sensorValues[1] > 800 && sensorValues[7] > 800) return true;
  else return false;
}


 void siguelineas() {
  position = qtr.readLineBlack(sensorValues); //Lectura de la posición de la linea con respecto al robot
  error =  position - 3500; //El error irá desde +3500 a -3500 si es > 0 linea a izq del sensor, si es < 0 linea a la dch del sensor
   for (uint8_t i = 0; i < SensorCount; i++)
   {
     Serial.print(sensorValues[i]);                                                        
     Serial.print('\t');
   }
   Serial.println();
   Serial.println(); 
  // Serial.println(position); //Posición del robot en la linea (debugging)
  // Serial.println(error); //Errir del robot (debugging)

  error =  position - 3500; //El error irá desde +3500 a -3500 si es >0 linea a izq del sensor, si es <0 linea a la dch del sensor
  integral = integral + error; 
  derivado = error - lastError; 
  int vel = Kp * error + Kd * derivado + Ki * integral;
  lastError = error; 

  // Asignación de velocidades
  int MotorDPWM = rightBaseSpeed - vel; //Base, modificar según parámetros PID
  int MotorIPWM = leftBaseSpeed + vel; //Base, modificar según parámetros PID
  // Serial.print(MotorIPWM);
  // Serial.print('\t');
  // Serial.print(MotorDPWM);
  // Serial.println();

  adelante(MotorDPWM, MotorIPWM,0);
}


float giro_z(){
    // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Filtrar las lecturas
  f_ax = f_ax-(f_ax>>5)+ax;
  p_ax = f_ax>>5;

  f_ay = f_ay-(f_ay>>5)+ay;
  p_ay = f_ay>>5;

  f_az = f_az-(f_az>>5)+az;
  p_az = f_az>>5;

  f_gx = f_gx-(f_gx>>3)+gx;
  p_gx = f_gx>>3;

  f_gy = f_gy-(f_gy>>3)+gy;
  p_gy = f_gy>>3;

  f_gz = f_gz-(f_gz>>3)+gz;
  p_gz = f_gz>>3;

  //Cada 100 lecturas corregir el offset
  if (counter==100){
    //Mostrar las lecturas separadas por un [tab]
    Serial.print("promedio:"); Serial.print("t");
    Serial.print(p_ax); Serial.print("\t");
    Serial.print(p_ay); Serial.print("\t");
    Serial.print(p_az); Serial.print("\t");
    Serial.print(p_gx); Serial.print("\t");
    Serial.print(p_gy); Serial.print("\t");
    Serial.println(p_gz);

    //Calibrar el acelerometro a 1g en el eje z (ajustar el offset)
    if (p_ax>0) ax_o--;
    else {ax_o++;}
    if (p_ay>0) ay_o--;
    else {ay_o++;}
    if (p_az-16384>0) az_o--;
    else {az_o++;}
    
    sensor.setXAccelOffset(ax_o);
    sensor.setYAccelOffset(ay_o);
    sensor.setZAccelOffset(az_o);

    //Calibrar el giroscopio a 0º/s en todos los ejes (ajustar el offset)
    if (p_gx>0) gx_o--;
    else {gx_o++;}
    if (p_gy>0) gy_o--;
    else {gy_o++;}
    if (p_gz>0) gz_o--;
    else {gz_o++;}
    
    sensor.setXGyroOffset(gx_o);
    sensor.setYGyroOffset(gy_o);
    sensor.setZGyroOffset(gz_o);    

    counter=0;
  }
  counter++;
//
//   siguelineas();
//  
//   if (checkIntersection()){
//     brake(0);
//     delay(5000000000);
//   }

  dt = millis()-tiempo_prev;
  tiempo_prev=millis();
  
  girosc_ang_x = (gx/131)*dt/1000.0 + girosc_ang_x_prev;
  girosc_ang_y = (gy/131)*dt/1000.0 + girosc_ang_y_prev;
  girosc_ang_z = (gz/131)*dt/1000.0 + girosc_ang_z_prev;

  girosc_ang_x_prev=girosc_ang_x;
  girosc_ang_y_prev=girosc_ang_y;
  girosc_ang_z_prev=girosc_ang_z;

  //Mostrar los angulos separadas por un [tab]
  Serial.print("Rotacion en X:  ");
  Serial.print(girosc_ang_x); 
  Serial.print("tRotacion en Y: ");
  Serial.print(girosc_ang_y);
  Serial.print("tRotacion en Z: ");
  Serial.println(girosc_ang_z);


  return girosc_ang_z;
}
void giro_90(bool sentido){
  if (sentido){
   while(giro_z()<90)
   {
    derecha(150);
   }
   brake(0);
  }
  else{
   while(giro_z()<90)
   {
    izquierda(150);
   } 
   brake(0);      
  } 
}


void setup() {
  delay(500);
  pinMode(13, OUTPUT);

  //Ultrasonidos
  // for (int k = 0; k < 3; k++){
  //   pinMode(Trigger[k], OUTPUT);
  //   pinMode(Echo[k], INPUT);
  // }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT); 
  pinMode(6, INPUT); 
  pinMode(7, INPUT); 
  pinMode(8, INPUT); 
  pinMode(9, INPUT); 
  pinMode(10, INPUT); 
  pinMode(11, INPUT); 
  pinMode(12, INPUT); 
  pinMode(15, INPUT); 

  // Configure the QTR
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){23, 24, 25, 26, 27, 28, 29, 30}, SensorCount);
  
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH); //Encender el led al empezar la calibración

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // ~25 ms per calibrate() call.
  // Call calibrate() 200 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); //Apagar el led al acabar la calibración


  // Valores de calibración
  Serial.begin(9600);
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(qtr.calibrationOn.maximum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  // Serial.println();
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
//
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");

//  // Leer los offset los offsets anteriores
  ax_o=sensor.getXAccelOffset();
  ay_o=sensor.getYAccelOffset();
  az_o=sensor.getZAccelOffset();
  gx_o=sensor.getXGyroOffset();
  gy_o=sensor.getYGyroOffset();
  gz_o=sensor.getZGyroOffset();
//  
  Serial.println("Offsets:");
  Serial.print(ax_o); Serial.print("\t"); 
  Serial.print(ay_o); Serial.print("\t"); 
  Serial.print(az_o); Serial.print("\t"); 
  Serial.print(gx_o); Serial.print("\t"); 
  Serial.print(gy_o); Serial.print("\t");
  Serial.print(gz_o); Serial.print("\t");
//  Serial.println("nnEnvie cualquier caracter para empezar la calibracionnn");  
//  // Espera un caracter para empezar a calibrar
//  while (true){if (Serial.available()) break;}  
  Serial.println("Calibrando, no mover IMU");

  
  tiempo_prev=millis();
}


void loop() {

 giro_90(true);
 
}
