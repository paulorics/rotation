#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#define G 9.806;

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float acX, acY, acZ;
float giroX, giroY, giroZ; // VELOCIDADES ANGULARES
float roll_a, pitch_a; // ÂNGULOS OBTIDOS PELO ACELERÔMETRO
float angleXgiro = 0, angleYgiro = 0; // ÂNGULOS OBTIDOS ATRAVÉS DO GIRO
float roll = 0, pitch = 0, yaw = 0; // ÂNGULO FILTRADO

const float alpha = 0.97; // PARÂMETRO DO FILTRO COMPLEMENTAR

unsigned long currentTime = 0; // VARIÁVEL PARA REALIZAR AMOSTRAGEM
float dt = 5000; //0,005s  // INTERVALO DE AMOSTRAGEM
unsigned long previoustime = 0; // VARIÁVEL PARA REALIZAR AMOSTRAGEM

void setup() {
  Wire.begin();
  
  // INICIALIZA A COMUNICAÇÃO SERIAL
  Serial.begin(115200);
  
  accelgyro.initialize();
  // CALIBRAÇÃO (OFFSETS)
  accelgyro.setXGyroOffset(-14);
  accelgyro.setYGyroOffset(41);
  accelgyro.setZGyroOffset(16);
  accelgyro.setXAccelOffset(308);
  accelgyro.setYAccelOffset(-2011);
  accelgyro.setZAccelOffset(1861);
  
  // INICIALIZAÇÃO DA DO SENSOR
  Serial.println("Initializing I2C devices...");
 
  // VERIFICAÇÃO DA CONEXÃO
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  previoustime = micros();
}

void loop() {
    currentTime = micros();
    if(currentTime - previoustime >= dt){
    previoustime = currentTime; // ATUALIZA O TEMPO ANTERIOR
    
    // LEITURA DAS ACELERAÇÕES E DAS VELOCIDADES ÂNGULARES E ARMAZENAMENTO EM (ax, ay, az, gx, gy, gz)
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    acX = (ax / 16384.0) * G;
    acY = (ay / 16384.0) * G;
    acZ = (az / 16384.0) * G;
    giroX = (gx / 131.0);
    giroY = (gy / 131.0);
    giroZ = (gz / 131.0);

    ///*
      Serial.print("ax ay az: ");
      Serial.print(acX); Serial.print(" ");
      Serial.print(acY); Serial.print(" ");
      Serial.print(acZ);
      Serial.print("\t");
      Serial.print(giroX);Serial.print(" ");
      Serial.print(giroY); Serial.print(" ");
      Serial.println(giroZ); Serial.print(" ");
    //*/
    // ÂNGULOS OBTIDOS POR TRIGONOMETRIA PELO ACELERÔMETRO
    roll_a = (atan(acY / sqrt(pow(acX, 2) + pow(acZ, 2))) * 180 / PI);
    pitch_a = (atan(-1 * acX / sqrt(pow(acY, 2) + pow(acZ, 2))) * 180 / PI);
    
    //INTEGRAÇÃO DAS VELOCIDADES ANGULARES
    angleXgiro = (angleXgiro + giroX * (dt/1000000)); //Divide por 1000000 para obter em segundos
    angleYgiro = (angleYgiro + giroY * (dt/1000000));
    
    //FILTRO COMPLEMENTAR
    roll = alpha * angleXgiro + (1 - alpha) * (roll_a); 
    pitch = alpha * angleYgiro + (1 - alpha) * (pitch_a);

    ///*
    //Serial.print(roll); Serial.print(" ");
    //Serial.println(pitch);
    //Serial.println(dt);
    //*/
    }
}