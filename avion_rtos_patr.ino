
#include <Arduino_FreeRTOS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <semphr.h>
#include <SimpleTimer.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;
SimpleTimer timer;  // Un singur obiect SimpleTimer poate stoca mai multe timere

SemaphoreHandle_t mutexIntA;
SemaphoreHandle_t mutexCalcA;
SemaphoreHandle_t mutexIntG;
SemaphoreHandle_t mutexCalcG;
SemaphoreHandle_t mutexCitireA;
SemaphoreHandle_t mutexCitireG;
// Obiecte pentru senzori MPU-6050
Adafruit_MPU6050 mpu1;  // Senzor 1
Adafruit_MPU6050 mpu2;  // Senzor 2

// Structuri pentru stocarea datelor senzorilor
typedef struct {
  float ax, ay, az;
} AccelerometerData;

typedef struct {
  float roll, pitch, yaw;
} GyroscopeData;

// Variabile globale protejate
AccelerometerData accelData = { 0, 0, 0 };
GyroscopeData gyroData = { 0, 0, 0 };
float vx = 0, vy = 0, vz = 0;

// Funcții de integrare
float integrate(float value, float deltaT) {
  return value * deltaT;  // Simplă integrare folosind valoarea curentă și timpul
}
float a = 0, b = 0, c = 0, count = 0;
// Task 1: Citire accelerometru de la senzor 1
void TaskReadAccelerometer(void *pvParameters) {
  (void)pvParameters;
  sensors_event_t accel, gyro, temp;
  mpu1.getEvent(&accel, &gyro, &temp);
  Serial.println("Accelerometer a început...");
  float axBias = 0, ayBias = 0, azBias = 0;
  Serial.println("Calibrare... ține dispozitivul nemișcat.");
  // Variabile pentru bias și calibrare

  const float g = 9.81;  // Acceleratia gravitațională
  for (int i = 0; i < 1000; i++) {

    axBias += accel.acceleration.x;
    ayBias += accel.acceleration.y;
    azBias += accel.acceleration.z;
  }
  axBias /= 1000;
  ayBias /= 1000;
  azBias /= 1000 + g;  // Adaugam gravitația (pe axa verticală)

  Serial.println("Calibrare completă.");
  while (1) {

    sensors_event_t accel, gyro, temp;
    mpu1.getEvent(&accel, &gyro, &temp);

    // Protejăm accesul la accelData
    if (xSemaphoreTake(mutexCitireA, portMAX_DELAY))  // primeste mutex de la functia de integrare, care trimite mutex dupa ce termina de integrat
    {
      accelData.ax = accel.acceleration.x - axBias;  //- 1.7096;
      accelData.ay = accel.acceleration.y - ayBias;  // 0.191407;
      accelData.az = accel.acceleration.z - azBias;  //- 11.2816;

      xSemaphoreGive(mutexIntA);
      xSemaphoreGive(mutexIntA);
    }  // Activeaza functia de integrare aceleratie

    vTaskDelay(pdMS_TO_TICKS(5));  // Rulează la fiecare 5 ms
  }
}


float rollBias, yawBias, pitchBias;

// Task 2: Citire giroscop de la senzor 2
void TaskReadGyroscope(void *pvParameters) {
  (void)pvParameters;
  sensors_event_t accel, gyro, temp;
  mpu2.getEvent(&accel, &gyro, &temp);

  Serial.println("Gyroscope a început...");
  Serial.println("Calibrare... ține dispozitivul nemișcat.");
  // Variabile pentru bias și calibrare
  rollBias = 0;
  yawBias = 0;
  pitchBias = 0;
  const float g = 9.8;  // Acceleratia gravitațională
  for (int i = 0; i < 1000; i++) {

    rollBias += gyro.gyro.x;
    pitchBias += gyro.gyro.y;
    yawBias += gyro.gyro.z;
  }
  rollBias /= 1000;
  pitchBias /= 1000;
  yawBias /= 1000; 

  Serial.println("Calibrare completă.");
  while (1) {

    sensors_event_t accel, gyro, temp;
    mpu2.getEvent(&accel, &gyro, &temp);

    // Calculăm unghiurile de ruliu, tangaj și girație
    if (xSemaphoreTake(mutexCitireG, portMAX_DELAY))  // primeste mutex de la functia de integrare, care trimite mutex dupa ce termina de integrat
    {

      gyroData.roll = gyro.gyro.x - rollBias;
      gyroData.pitch = gyro.gyro.y - pitchBias;
      gyroData.yaw = gyro.gyro.z - yawBias;
      xSemaphoreGive(mutexIntG);  // Activeaza functia de integrare giroscop
    }

    vTaskDelay(pdMS_TO_TICKS(40));  // Rulează la fiecare 40 ms
  }
}

// Parametrii filtrului complementar
const float alpha = 0.98;  // Pondere pentru giroscop (ajustabil între 0 și 1)
float g = 9.81;
// Variabile pentru filtrul complementar
float pitchFiltered = 0;
float rollFiltered = 0;
float yaw;

// Task 4: Integrare giroscop senzor 2 (cu filtru complementar)
void TaskIntegrateGyroscope(void *pvParameters) {
  (void)pvParameters;

  Serial.println("TaskIntegrateGyroscope a început...");

  while (1) {
    float rollRate, pitchRate, yawRate, ax, ay, az, pitch, roll;

    if (xSemaphoreTake(mutexIntG, portMAX_DELAY)) {
      rollRate = gyroData.roll;
      pitchRate = gyroData.pitch;
      yawRate = gyroData.yaw;
      xSemaphoreGive(mutexCitireG);  // Activează funcția de citire gyro

      // Integrare giroscop (angulare)
      float pitchAcc, rollAcc;  // Calcul din accelerometru
      if (xSemaphoreTake(mutexIntA, portMAX_DELAY)) {
        ax = accelData.ax;
        ay = accelData.ay;
        az = accelData.az;
      }

      pitch += integrate(pitchRate, 0.04);
      roll += integrate(rollRate, 0.04);

      // Filtrul complementar
      pitchFiltered = alpha * pitch + (1 - alpha) * ax;
      rollFiltered = alpha * roll + (1 - alpha) * ay;

      // Integrare yaw fără corecții
      yaw += integrate(yawRate, 0.04);
      xSemaphoreGive(mutexCalcG);     // Activeaza functia de adunare acceleratie
      vTaskDelay(pdMS_TO_TICKS(40));  // Rulează sincronizat cu citirea giroscopului
    }
  }
}

void TaskIntegrateAcceleration(void *pvParameters) {
  (void)pvParameters;

  Serial.println("TaskIntegrateAcceleration a început...");
  while (1) {
    float ax, ay, az;

    if (xSemaphoreTake(mutexIntA, portMAX_DELAY))  // primeste mutex de la functia de citire, care trimite mutex dupa ce termina de citit
    {

      ax = accelData.ax;
      ay = accelData.ay;
      az = accelData.az;
      xSemaphoreGive(mutexCitireA);  // Activeaza functia de citire acceleratie
      // ax -= sin(pitchFiltered) * g;
      // ay -= sin(rollFiltered) * g;
      // az -= cos(pitchFiltered) * cos(rollFiltered) * g;
      if (abs(ax) < 0.1) ax = 0;
      if (abs(ay) < 0.1) ay = 0;
      if (abs(az) < 0.1) az = 0;

      vx += integrate(ax, 0.005);  // Integrare accelerație
      vy += integrate(ay, 0.005);
      vz += integrate(az, 0.005);
      if (abs(vx) < 0.1) vx = 0;
      if (abs(vy) < 0.1) vy = 0;
      if (abs(vz) < 0.1) vz = 0;


      xSemaphoreGive(mutexCalcA);  // Activeaza functia de adunare acceleratie
      //Serial.println("TaskIntegrateAcceleration a terminat...");
      vTaskDelay(pdMS_TO_TICKS(5));  // Rulează sincronizat cu citirea accelerometrului
    }
  }
}

float vTotal, vxC, vyC, vzC;
// Task 5: Calculare și afișare viteză față de Pământ (corectată)
void TaskCalculateVelocity(void *pvParameters) {
  (void)pvParameters;
  const float g = 9.81;
  float gint = integrate(g, 0.005);

  Serial.println("TaskCalculateVelocity a început...");
  while (1) {
    float ax, ay, az, aroll, ayaw, apitch;

    if (xSemaphoreTake(mutexCalcA, portMAX_DELAY)) {
      ax = vx;
      ay = vy;
      az = vz;
    }
    if (xSemaphoreTake(mutexCalcG, portMAX_DELAY)) {
      aroll = rollFiltered;
      ayaw = yaw;
      apitch = pitchFiltered;
    }

    float cosTheta = cos(aroll);
    float sinTheta = sin(aroll);
    float cosPhi = cos(ayaw);
    float sinPhi = sin(ayaw);
    float cosPsi = cos(apitch);
    float sinPsi = sin(apitch);

    // Transformare în coordonate globale
    vxC = cosTheta * cosPsi * ax + (sinPhi * sinTheta * cosPsi - cosPhi * sinPsi) * ay + (cosPhi * sinTheta * cosPsi + sinPhi * sinPsi) * az - (cosPhi * sinTheta * cosPsi + sinPhi * sinPsi) * gint;

    vyC = cosTheta * sinPsi * ax + (sinPhi * sinTheta * sinPsi + cosPhi * cosPsi) * ay + (cosPhi * sinTheta * sinPsi - sinPhi * cosPsi) * az - (cosPhi * sinTheta * sinPsi - sinPhi * cosPsi) * gint;

    vzC = -sinTheta * ax + cosTheta * sinPhi * ay + cosTheta * cosPhi * az - cosPsi * cosTheta * gint;

    // Calculul vitezei totale
    vTotal = sqrt(vxC * vxC + vyC * vyC + vzC * vzC);
    timer.run();
    vTaskDelay(pdMS_TO_TICKS(5));  // Rulează la fiecare 5 ms
  }
}


void TaskPrint() {
  // Afișare viteze
  Serial.print("Viteza: vx = ");
  Serial.print(vxC, 3);
  Serial.print(", vy = ");
  Serial.print(vyC, 3);
  Serial.print(", vz = ");
  Serial.print(vzC, 3);
  Serial.print(", vTotal = ");
  Serial.println(vTotal, 3);
  Serial.print("Roll = ");
  Serial.print(rollFiltered, 3);
  Serial.print(", yaw = ");
  Serial.print(yaw, 3);
  Serial.print(", [pitch] = ");
  Serial.println(pitchFiltered, 3);
}
extern unsigned int __heap_start, *__brkval;
int freeMemory() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void setup() {
  Serial.begin(9600);

  // Mesaje de debug pentru inițializarea senzorilor
  if (!mpu1.begin(0x69)) {
    Serial.println("Eroare la senzor 1");
    // Opriți programul dacă senzorul 1 nu poate fi inițializat
  } else {
    Serial.println("Senzor 1 inițializat cu succes");
  }

  if (!mpu2.begin(0x68)) {
    Serial.println("Eroare la senzor 2");
    while (1)
      ;  // Opriți programul dacă senzorul 2 nu poate fi inițializat
  } else {
    Serial.println("Senzor 2 inițializat cu succes");
  }

  // Inițializare mutex
  mutexIntA = xSemaphoreCreateCounting(2, 0);
  mutexCalcA = xSemaphoreCreateMutex();
  mutexIntG = xSemaphoreCreateMutex();
  mutexCalcG = xSemaphoreCreateMutex();
  mutexCitireA = xSemaphoreCreateMutex();
  mutexCitireG = xSemaphoreCreateMutex();

  // Creare task-uri
  if (xTaskCreate(TaskReadAccelerometer, "ReadAccel", 512, NULL, 2, &Task1) != pdPASS) {
    Serial.println("Failed to create TaskReadAccelerometer");
  }

  if (xTaskCreate(TaskReadGyroscope, "ReadGyro", 512, NULL, 2, &Task2) != pdPASS) {
    Serial.println("Failed to create TaskReadGyroscope");
  }

  if (xTaskCreate(TaskIntegrateAcceleration, "IntegrateAccel", 512, NULL, 2, &Task3) != pdPASS) {
    Serial.println("Failed to create TaskIntegrateAcceleration");
  }

  if (xTaskCreate(TaskIntegrateGyroscope, "IntegrateGyro", 512, NULL, 2, &Task4) != pdPASS) {
    Serial.println("Failed to create TaskIntegrateGyroscope");
  }

  if (xTaskCreate(TaskCalculateVelocity, "CalculateVelocity", 512, NULL, 2, &Task5) != pdPASS) {
    Serial.println("Failed to create TaskCalculateVelocity");
  }
  timer.setInterval(500, TaskPrint);
  Serial.println(freeMemory());

  // Pornire scheduler
  vTaskStartScheduler();
}

void loop() {

  // Bucla principală este goală - rularea este gestionată de FreeRTOS
}