#include <QTRSensors.h>
/*CONSTANTES*/
//Sensor siguelineas
#define Kp 0.2 // experimentar (valores bajos)
#define Ki 0.02 // no se ni lo que estoy haciendo
#define Kd 0.5 // experimentar con valores bajos (Ki < Kp < Kd)
#define NUM_SENSORS  8     // number of sensors used

//Velocidades
#define rightMaxSpeed 255 // max speed of the robot (0-255)
#define leftMaxSpeed 255 // max speed of the robot (0-255)
#define rightBaseSpeed 200 // this is the speed at which the motors should spin when the robot is perfectly on the line (0-255)
#define leftBaseSpeed 200  // this is the speed at which the motors should spin when the robot is perfectly on the line (0-255)

//Pines
#define ENA 12
#define IN1 11
#define IN2 16
#define IN3 13
#define IN4 15
#define ENB 10

/*VARIABLES*/
//Sensor QTR
QTRSensors qtr;
const uint8_t SensorCount = NUM_SENSORS;
uint16_t sensorValues[SensorCount];
int error;
int integral = 0;
int derivado = 0;
int lastError = 0;
int position;

//Ultrasonidos
// const int Echo[3] = {52, 36, 23}; // de izquierda a derecha (L, F, R)
// const int Trigger[3] = {53, 38, 22}; // de izquierda a derecha (L, F, R)
// int distancia[3];


void wait() {
 digitalWrite(ENA, 0);
 digitalWrite(ENB, 0);
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


int ping(int Echo, int Trigger) {
  long duration;
  digitalWrite(Trigger, LOW); //Generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(2);
  digitalWrite(Trigger, HIGH);  //Generar Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW); 
  duration = pulseIn(Echo, HIGH);  //Medir el tiempo entre pulsos, en microsegundos
  return duration * 10 / 292/ 2; 
}


 void siguelineas() {
  position = qtr.readLineBlack(sensorValues); //Lectura de la posición de la linea con respecto al robot
  error =  position - 3500; //El error irá desde +3500 a -3500 si es > 0 linea a izq del sensor, si es < 0 linea a la dch del sensor
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position); //Posición del robot en la linea (debugging)
  // Serial.println(error); //Errir del robot (debugging)

  error =  position - 3500; //El error irá desde +3500 a -3500 si es >0 linea a izq del sensor, si es <0 linea a la dch del sensor
  integral = integral + error; 
  derivado = error - lastError; 
  int vel = Kp * error + Kd * derivado + Ki * integral;
  lastError = error; 

  // Asignación de velocidades
  int MotorDPWM = rightBaseSpeed + vel; //Base, modificar según parámetros PID
  int MotorIPWM = leftBaseSpeed - vel; //Base, modificar según parámetros PID
  Serial.print(MotorIPWM);
  Serial.print('\t');
  Serial.print(MotorDPWM);
  Serial.println();
  adelante(MotorDPWM, MotorIPWM,0);
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

  // Configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){3, 4, 5, 6, 7, 8, 9, 10}, SensorCount);
  
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
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

  // Ultrasonidos
  // while (ping(Echo[1], Trigger[1]) < 10) {
  //   wait();
  // }
}


void loop() {
 siguelineas();
}
