#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#define G 9.806;

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float acX, acY, acZ;
float giroX, giroY, giroZ; // VELOCIDADES ANGULARES
float ga[3] = {0, 0, 0}; // VELOCIDADES ÂNGULARES ANTERIORES gi-1

unsigned long currentTime = 0; // VARIÁVEL PARA REALIZAR AMOSTRAGEM
float dT = 5000; // INTERVALO DE AMOSTRAGEM EM MICROS
float dt = 0.005; //0,005s  // INTERVALO DE AMOSTRAGEM (200Hz)
unsigned long previoustime = 0; // VARIÁVEL PARA REALIZAR AMOSTRAGEM

// ------- VARIÁVEIS PARA ENCONTRAR qf --------- //
float qa[4]; // Quaternion anterior qi-1
float qf[4]; // QUATERNION ATUAL qi
// PRODUTO ENTRE qi-1 * gi-1 (QuaternionAnterior x giroanterior)
float QiG0; // Linha 1 do produto entre qi-1 x gi-1 (Matriz)
float QiGi; // Linha 2 do produto entre qi-1 x gi-1 (Matriz)
float QiGj; // Linha 3 do produto entre qi-1 x gi-1 (Matriz)
float QiGk; // Linha 4 do produto entre qi-1 x gi-1 (Matriz)
float as[3]; // Acelerações do sensor
float gf[3]; // VELOCIDADES ÂNGULARES ATUAIS Gi
float gm[3]; // MÉDIA DAS VELOCIDADES ANGULARES EM DOIS INTERVALOS DE TEMPO (gi-1 + gi)/2
float K1, K1i, K1j, K1k; // K1 (K1 = 1/2 * Qi x Gi) Derivada do Quaternion no instante i
float q0k2, q1k2, q2k2, q3k2;  // K2 (K2 = 1/2 * [Qi + K1/2 * dt] x Gi+0.5  Derivada do Quaternion no instante i+0.5
float K2, K2i, K2j, K2k; 
float q0k3, q1k3, q2k3, q3k3; // K3 (K3 = 1/2 * [Qi + K2/2 * dt] x Gi+0.5  Derivada do Quaternion no instante i+0.5
double K3, K3i, K3j, K3k;
float q0k4, q1k4, q2k4, q3k4; // K4 (K4 = 1/2 * [Qi + K3 * dt] x Gi+1  Derivada do Quaternion no instante i+1
double K4, K4i, K4j, K4k;
float qfg[4]; //Quaternion para encontrar as componentes da gravidade no BODY FRAME
float M[9]; //Matriz de rotação 
float Gcomp[3]; //Componentes gx', gy', gz'
float acterra[3];

void setup() {
  Wire.begin();
  
  // INICIALIZA A COMUNICAÇÃO SERIAL
  Serial.begin(115200);
  
  accelgyro.initialize();
  // CALIBRAÇÃO (OFFSETS)
  accelgyro.setXGyroOffset(-16);
  accelgyro.setYGyroOffset(41);
  accelgyro.setZGyroOffset(22);
  accelgyro.setXAccelOffset(334);
  accelgyro.setYAccelOffset(-1979);
  accelgyro.setZAccelOffset(1869);
  
  // INICIALIZAÇÃO DA DO SENSOR
  Serial.println("Initializing I2C devices...");
 
  // VERIFICAÇÃO DA CONEXÃO
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  previoustime = micros();
}

void loop() {
    currentTime = micros();
    if(currentTime - previoustime >= dT){
    
    previoustime = currentTime; // ATUALIZA O TEMPO ANTERIOR
    
    // LEITURA DAS ACELERAÇÕES E DAS VELOCIDADES ÂNGULARES E ARMAZENAMENTO EM (ax, ay, az, gx, gy, gz)
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    acX = (ax / 16384.0) * G;
    acY = (ay / 16384.0) * G;
    acZ = (az / 16384.0) * G;
    giroX = (gx / 131.0);
    giroY = (gy / 131.0);
    giroZ = (gz / 131.0);

    /*
      Serial.print("ax ->");
      Serial.print("\t");
      Serial.print(acX);
      Serial.print("\t");
      Serial.print("ay ->");
      Serial.print("\t");
      Serial.print(acY);
      Serial.print("\t");
      Serial.print("az ->");
      Serial.print("\t");
      Serial.print(acZ);
      Serial.print("\t");
      Serial.print("gx ->");
      Serial.print("\t");
      Serial.print(giroX);
      Serial.print("\t");
      Serial.print("gy ->");
      Serial.print("\t");
      Serial.print(giroY);
      Serial.print("\t");
      Serial.print("gz ->");
      Serial.print("\t");
      Serial.println(giroZ);
    */
    
    qa[0] = 1; qa[1] = 0; qa[2] = 0; qa[3] = 0; // Quaternion inicial (ângulo 0)
    
    QiG0 = - qa[1] * ga[0] - qa[2] * ga[1] - qa[3] * ga[2];
    QiGi = qa[0] * ga[0] - qa[3] * ga[1] + qa[2] * ga[2];
    QiGj = qa[3] * ga[0] + qa[0] * ga[1] - qa[1] * ga[2];
    QiGk = - qa[2] * ga[0] + qa[1] * ga[1] + qa[0] * ga[2];

    // VELOCIDADES ÂNGULARES ATUAIS Gi
    gf[0] = giroX * PI / 180; 
    gf[1] = giroY * PI / 180; 
    gf[2] = giroZ * PI / 180;
    
    // ACELERAÇÕES OBTIDAS PELO SENSOR (ATUAIS) 
    as[0] = acX; as[1] = acY; as[2] = acZ;

    // MÉDIA DAS VELOCIDADES ANGULARES EM DOIS INTERVALOS DE TEMPO (gi-1 + gi)/2
    gm[0] = (ga[0] + gf[0]) / 2;
    gm[1] = (ga[1] + gf[1]) / 2;
    gm[2] = (ga[2] + gf[2]) / 2;

    //----- IMPLEMENTANDO O ALGORITMO DE RUNGE KUTTA 4 --------
    // K1 (K1 = 1/2 * Qi x Gi) Derivada do Quaternion no instante i
    K1 = 0.5 * QiG0;
    K1i = 0.5 * QiGi;
    K1j = 0.5 * QiGj;
    K1k = 0.5 * QiGk;
   
    // K2 (K2 = 1/2 * [Qi + K1/2 * dt] x Gi+0.5  Derivada do Quaternion no instante i+0.5
    // Equação ---> 1/2*(Qi+K1/2*dt)
    
    q0k2 = 0.5 * (qa[0] + (K1 / 2) * dt);
    q1k2 = 0.5 * (qa[1] + (K1i / 2) * dt);
    q2k2 = 0.5 * (qa[2] + (K1j / 2) * dt);
    q3k2 = 0.5 * (qa[3] + (K1k / 2) * dt);

    K2 = - q1k2 * gm[0] - q2k2 * gm[1] - q3k2 * gm[2];
    K2i = q0k2 * gm[0] - q3k2 * gm[1] + q2k2 * gm[2];
    K2j = q3k2 * gm[0] + q0k2 * gm[1] - q1k2 * gm[2];
    K2k = - q2k2 * gm[0] + q1k2 * gm[1] + q0k2 * gm[2];

    // K3 (K3 = 1/2 * [Qi + K2/2 * dt] x Gi+0.5  Derivada do Quaternion no instante i+0.5
    // Equação ---> 1/2*(Qi+K2/2*dt)
    
    q0k3 = 0.5 * (qa[0] + (K2 / 2) * dt);
    q1k3 = 0.5 * (qa[1] + (K2i / 2) * dt);
    q2k3 = 0.5 * (qa[2] + (K2j / 2) * dt);
    q3k3 = 0.5 * (qa[3] + (K2k / 2) * dt);

    K3 = - q1k3 * gm[0] - q2k3 * gm[1] - q3k3 * gm[2];
    K3i = q0k3 * gm[0] - q3k3 * gm[1] + q2k3 * gm[2];
    K3j = q3k3 * gm[0] + q0k3 * gm[1] - q1k3 * gm[2];
    K3k = - q2k3 * gm[0] + q1k3 * gm[1] + q0k3 * gm[2];

    // K4 (K4 = 1/2 * [Qi + K3 * dt] x Gi+1  Derivada do Quaternion no instante i+1

    q0k4 = 0.5 * (qa[0] + K3 * dt);
    q1k4 = 0.5 * (qa[1] + K3i * dt);
    q2k4 = 0.5 * (qa[2] + K3j * dt);
    q3k4 = 0.5 * (qa[3] + K3k * dt);

    K4 = - q1k4 * gm[0] - q2k4 * gm[1] - q3k4 * gm[2];
    K4i = q0k4 * gm[0] - q3k4 * gm[1] + q2k4 * gm[2];
    K4j = q3k4 * gm[0] + q0k4 * gm[1] - q1k4 * gm[2];
    K4k = - q2k4 * gm[0] + q1k4 * gm[1] + q0k4 * gm[2];

    // --------- QUATERNION ATUAL ----- //
    qf[0] = qa[0] + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
    qf[1] = qa[1] + dt / 6 * (K1i + 2 * K2i + 2 * K3i + K4i);
    qf[2] = qa[2] + dt / 6 * (K1j + 2 * K2j + 2 * K3j + K4j);
    qf[3] = qa[3] + dt / 6 * (K1k + 2 * K2k + 2 * K3k + K4k);

     // --------- MATRIZ DE ROTAÇÃO ---------
    M[0] = pow(qf[0], 2) + pow(qf[1], 2) - pow(qf[2], 2) - pow(qf[3], 2);
    M[1] = 2 * (qf[1] * qf[2] - qf[3] * qf[0]); 
    M[2] = 2 * (qf[1] * qf[3] + qf[2] * qf[0]);
    M[3] = 2 * (qf[1] * qf[2] + qf[3] * qf[0]);
    M[4] = pow(qf[0], 2) - pow(qf[1], 2) + pow(qf[2], 2) - pow(qf[3], 2);
    M[5] = 2 * (qf[2] * qf[3] - qf[0] * qf[1]);
    M[6] = 2 * (qf[1] * qf[3] - qf[2] * qf[0]);
    M[7] = 2 * (qf[0] * qf[1] + qf[2] * qf[3]); 
    M[8] = pow(qf[0], 2) - pow(qf[1], 2) - pow(qf[2], 2) + pow(qf[3], 2);
    
    // REMOÇÃO DAS COMPONENTES DA GRAVIDADE DO BODY FRAME
    qfg[0] = qf[0]; 
    qfg[1] = - qf[1]; 
    qfg[2] = - qf[2]; 
    qfg[3] = - qf[3];
    
    Gcomp[0] = (2 * (qfg[0] * qfg[2] + qfg[1] * qfg[3])) * G; //Componente gx'
    Gcomp[1] = (2 * (qfg[2] * qfg[3] - qfg[0] * qfg[1])) * G; //Componente gy'
    Gcomp[2] = (pow(qfg[0], 2) - pow(qfg[1], 2) - pow(qfg[2], 2) + pow(qfg[3], 2)) * G; //Componente gz'

    as[0] = as[0] - Gcomp[0]; // Aceleração gx' no bodyframe sem a gravidade
    as[1] = as[1] - Gcomp[1]; // Aceleração gy' no bodyframe sem a gravidade
    as[2] = as[2] - Gcomp[2]; // Aceleração gz' no bodyframe sem a gravidade

    // --------- ACELERAÇÕES NO SISTEMA GLOBAL ------//
    acterra[0] = M[0] * as[0] + M[1] * as[1] + M[2] * as[2];
    acterra[1] = M[3] * as[0] + M[4] * as[1] + M[5] * as[2];
    acterra[2] = M[6] * as[0] + M[7] * as[1] + M[8] * as[2];

    ///*
    Serial.print(acterra[0]); Serial.print(" ");
    Serial.print(acterra[1]); Serial.print(" ");
    Serial.println(acterra[2]);
    //*/
    ga[0] = gf[0]; ga[1] = gf[1]; ga[2] = gf[2];
    }
}
