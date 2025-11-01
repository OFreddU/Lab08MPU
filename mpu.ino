#include "I2Cdev.h"
#include "MPU9250.h" // <-- 1. USA LA LIBRERÍA PARA TU SENSOR
#include "Wire.h"

MPU9250 IMU; // <-- 2. USA EL OBJETO MPU9250

// Variables para los datos RAW
int ax_raw, ay_raw, az_raw;
int gx_raw, gy_raw, gz_raw;

// ================================================================
// ¡PEGA TUS OFFSETS AQUÍ! (Paso 3.2)
// Reemplaza estos ceros con los valores que obtuviste
// ================================================================
int ax_o = -5420;
int ay_o = -6256;
int az_o = 10014;
int gx_o = 5;
int gy_o = 9;
int gz_o = 11;
// ================================================================

// Variables para datos escalados (Paso 3.3)
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

// Constantes de escalado (según la guía)
// Acelerómetro: Rango +-2g / 16 bits = 32767. 1g = 16384 (aprox)
const float ACCEL_SCALE = 16384.0;
// Giroscopio: Rango +-250°/s / 16 bits = 32767. Factor = 32767 / 250 = 131.0
const float GYRO_SCALE = 131.0;

// Variables para ángulos del Acelerómetro (Paso 3.4)
float angle_pitch_accel, angle_roll_accel;

// Variables para el Filtro Complementario (Paso 3.6)
float angle_pitch = 0;
float angle_roll = 0;

// Variables para el tiempo (dt) (Paso 3.5)
unsigned long t_prev;
float dt;

void setup() {
  Serial.begin(57600);
  Wire.begin();       // En el Mega, usa pines 20 (SDA) y 21 (SCL)
  IMU.initialize();

  // if (!IMU.testConnection()) { // <-- Comentado
  //   Serial.println("Fallo al iniciar MPU9250");
  //   while (1);
  // }
  Serial.println("MPU9250 Conectado (forzadamente)."); // <-- Mensaje modificado
  // Aplicar los offsets que encontraste
  IMU.setXAccelOffset(ax_o);
  IMU.setYAccelOffset(ay_o);
  IMU.setZAccelOffset(az_o);
  IMU.setXGyroOffset(gx_o);
  IMU.setYGyroOffset(gy_o);
  IMU.setZGyroOffset(gz_o);

  // Iniciar el temporizador
  t_prev = millis();
}

void loop() {
  // --- Cálculo de dt (Paso 3.5) ---
  // Reemplazamos delay() con un temporizador para un 'dt' preciso
  unsigned long t_now = millis();
  dt = (t_now - t_prev) / 1000.0; // dt en segundos
  t_prev = t_now;

  // --- Leer Datos ---
  IMU.getAcceleration(&ax_raw, &ay_raw, &az_raw);
  IMU.getRotation(&gx_raw, &gy_raw, &gz_raw);

  // --- Escalado de Lecturas (Paso 3.3) ---
  // Convertir valores RAW a unidades físicas
  accel_x = ax_raw / ACCEL_SCALE; // en 'g'
  accel_y = ay_raw / ACCEL_SCALE; // en 'g'
  accel_z = az_raw / ACCEL_SCALE; // en 'g'
  
  gyro_x = gx_raw / GYRO_SCALE; // en °/s (grados por segundo)
  gyro_y = gy_raw / GYRO_SCALE; // en °/s
  gyro_z = gz_raw / GYRO_SCALE; // en °/s

  // --- Cálculo de Ángulos con Acelerómetro (Paso 3.4) ---
  // Usamos atan2 para más estabilidad que atan.
  // La guía usa 'theta_x' para el ángulo de pitch y 'theta_y' para el de roll.
  // Convertimos de radianes a grados (* 180.0 / M_PI)
  
  // Pitch (Theta_X en la guía): Inclinación usando 'ax'
  angle_pitch_accel = atan2(accel_x, sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * 180.0 / M_PI;
  // Roll (Theta_Y en la guía): Inclinación usando 'ay'
  angle_roll_accel = atan2(accel_y, sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * 180.0 / M_PI;


  // --- Filtro Complementario (Paso 3.5 y 3.6) ---
  // Fórmula: angulo = 0.98 * (angulo_prev + gyro * dt) + 0.02 * (ang_acelerometro)
  
  // Fusión para Pitch (Theta_X)
  // Nota: La guía fusiona theta_x (acel) con omega_x (giro).
  angle_pitch = 0.98 * (angle_pitch + gyro_x * dt) + 0.02 * (angle_pitch_accel);
  
  // Fusión para Roll (Theta_Y)
  // Nota: La guía fusiona theta_y (acel) con omega_y (giro).
  angle_roll = 0.98 * (angle_roll + gyro_y * dt) + 0.02 * (angle_roll_accel);
  

  // --- Imprimir Resultados ---
  // Imprime los datos listos para el "Serial Plotter" (el "scope")
  
  // Imprime: [Ruido (Acel)] [Filtrado (Filtro)]
  Serial.print(angle_pitch_accel); Serial.print("\t");
  Serial.print(angle_pitch);       Serial.print("\t");
  
  Serial.print(angle_roll_accel);  Serial.print("\t");
  Serial.println(angle_roll);

  // Un pequeño delay para no saturar el Serial, pero el 'dt' se encarga del cálculo
  delay(10); 
}
