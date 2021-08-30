#include <iostream>
#include<math.h>
#define PI 3.14159265
#define G 9.8;
using namespace std;

int main(){
    //----------- DECLARANDO VARIÁVEIS -----------------//
    float dt; dt = 1; // Intervalo de amostragem de 1s
    float qa[4]; // Quaternion anterior qi-1
    qa[0] = 1; qa[1] = 0; qa[2] = 0; qa[3] = 0; // Quaternion inicial (ângulo 0)

    float qf[4]; // QUATERNION ATUAL qi

    float ga[3]; // VELOCIDADES ÂNGULARES ANTERIORES gi-1
    ga[0] = 0; ga[1] = 0; ga[2] = 0;

    // VELOCIDADE ÂNGULAR EM GRAUS/S (PARA TESTES)............................
    // ENTRADA DE DADOS
    float graus[3];
    cout << "Insira o valor da velocidade angular inicial gx (EM GRAUS/S):\n ";
    cin >> graus[0];
    cout << "Insira o valor da velocidade angular inicial gy (EM GRAUS/S):\n ";
    cin >> graus[1];
    cout << "Insira o valor da velocidade angular inicial gz (EM GRAUS/S):\n ";
    cin >> graus[2];

    ga[0] = graus[0] * PI / 180;
    ga[1] = graus[1] * PI / 180;
    ga[2] = graus[2] * PI / 180;
    //..................................................

    // PRODUTO ENTRE qi-1 * gi-1 (QuaternionAnterior x giroanterior)
    float QiG0; // Linha 1 do produto entre qi-1 x gi-1 (Matriz)
    float QiGi; // Linha 2 do produto entre qi-1 x gi-1 (Matriz)
    float QiGj; // Linha 3 do produto entre qi-1 x gi-1 (Matriz)
    float QiGk; // Linha 4 do produto entre qi-1 x gi-1 (Matriz)
    
    QiG0 = - qa[1] * ga[0] - qa[2] * ga[1] - qa[3] * ga[2];
    QiGi = qa[0] * ga[0] - qa[3] * ga[1] + qa[2] * ga[2];
    QiGj = qa[3] * ga[0] + qa[0] * ga[1] - qa[1] * ga[2];
    QiGk = - qa[2] * ga[0] + qa[1] * ga[1] + qa[0] * ga[2];

    // VELOCIDADES ÂNGULARES ATUAIS Gi
    float gf[3]; gf[0] = 0; gf[1] = 0; gf[2] = 0;
    // ENTRADA DE DADOS
    // VELOCIDADE ÂNGULAR EM GRAUS/s (PARA TESTES)...............................
    float grausf[3];
    cout << "Insira o valor da velocidade angular ATUAL gx medida pelo SENSOR (EM GRAUS/S):\n ";
    cin >> grausf[0];
    cout << "Insira o valor da velocidade angular ATUAL gy medida pelo SENSOR (EM GRAUS/S):\n ";
    cin >> grausf[1];
    cout << "Insira o valor da velocidade angular ATUAL gz medida pelo SENSOR (EM GRAUS/S):\n ";
    cin >> grausf[2];

    gf[0] = grausf[0] * PI / 180;
    gf[1] = grausf[1] * PI / 180;
    gf[2] = grausf[2] * PI / 180;

    //.................................
    
    // ACELERAÇÕES OBTIDAS PELO SENSOR (ATUAIS)
    float as[3]; 
    as[0] = 0; as[1] = 0; as[2] = 0;
    
    // ENTRADA DE DADOS
    cout << "Insira o valor da aceleração atual medida pelo sensor (axs):\n ";
    cin >> as[0];
    cout << "Insira o valor da aaceleração atual medida pelo sensor (ays):\n ";
    cin >> as[1];
    cout << "Insira o valor da aceleração atual medida pelo sensor (azs):\n ";
    cin >> as[2];

    // MÉDIA DAS VELOCIDADES ANGULARES EM DOIS INTERVALOS DE TEMPO (gi-1 + gi)/2
    float gm[3]; gm[0] = 0; gm[1] = 0; gm[2] = 0;
    gm[0] = (ga[0] + gf[0]) / 2;
    gm[1] = (ga[1] + gf[1]) / 2;
    gm[2] = (ga[2] + gf[2]) / 2;

    //----- IMPLEMENTANDO O ALGORITMO DE RUNGE KUTTA 4 --------

    // K1 (K1 = 1/2 * Qi x Gi) Derivada do Quaternion no instante i
    float K1, K1i, K1j, K1k;

    K1 = 0.5 * QiG0;
    K1i = 0.5 * QiGi;
    K1j = 0.5 * QiGj;
    K1k = 0.5 * QiGk;
   
    // K2 (K2 = 1/2 * [Qi + K1/2 * dt] x Gi+0.5  Derivada do Quaternion no instante i+0.5
    // Equação ---> 1/2*(Qi+K1/2*dt)
    float q0k2, q1k2, q2k2, q3k2;

    q0k2 = 0.5*(qa[0] + (K1/2)*dt);
    q1k2 = 0.5*(qa[1] + (K1i/2)*dt);
    q2k2 = 0.5*(qa[2] + (K1j/2)*dt);
    q3k2 = 0.5*(qa[3] + (K1k/2)*dt);

    float K2, K2i, K2j, K2k;

    K2 = -q1k2*gm[0]-q2k2*gm[1]-q3k2*gm[2];
    K2i = q0k2*gm[0]-q3k2*gm[1]+q2k2*gm[2];
    K2j = q3k2*gm[0]+q0k2*gm[1]-q1k2*gm[2];
    K2k = -q2k2*gm[0]+q1k2*gm[1]+q0k2*gm[2];

    // K3 (K3 = 1/2 * [Qi + K2/2 * dt] x Gi+0.5  Derivada do Quaternion no instante i+0.5
    // Equação ---> 1/2*(Qi+K2/2*dt)
    float q0k3, q1k3, q2k3, q3k3;
    
    q0k3 = 0.5*(qa[0] + (K2/2)*dt);
    q1k3 = 0.5*(qa[1] + (K2i/2)*dt);
    q2k3 = 0.5*(qa[2] + (K2j/2)*dt);
    q3k3 = 0.5*(qa[3] + (K2k/2)*dt);

    double K3, K3i, K3j, K3k;

    K3 = -q1k3*gm[0]-q2k3*gm[1]-q3k3*gm[2];
    K3i = q0k3*gm[0]-q3k3*gm[1]+q2k3*gm[2];
    K3j = q3k3*gm[0]+q0k3*gm[1]-q1k3*gm[2];
    K3k = -q2k3*gm[0]+q1k3*gm[1]+q0k3*gm[2];

    // K4 (K4 = 1/2 * [Qi + K3 * dt] x Gi+1  Derivada do Quaternion no instante i+1
    float q0k4, q1k4, q2k4, q3k4;

    q0k4 = 0.5*(qa[0] + K3*dt);
    q1k4 = 0.5*(qa[1] + K3i*dt);
    q2k4 = 0.5*(qa[2] + K3j*dt);
    q3k4 = 0.5*(qa[3] + K3k*dt);

    double K4, K4i, K4j, K4k;

    K4 = -q1k4*gm[0]-q2k4*gm[1]-q3k4*gm[2];
    K4i = q0k4*gm[0]-q3k4*gm[1]+q2k4*gm[2];
    K4j = q3k4*gm[0]+q0k4*gm[1]-q1k4*gm[2];
    K4k = -q2k4*gm[0]+q1k4*gm[1]+q0k4*gm[2];

    // --------- QUATERNION ATUAL ----- //
    qf[0] = qa[0]+dt/6*(K1+2*K2+2*K3+K4);
    qf[1] = qa[1]+dt/6*(K1i+2*K2i+2*K3i+K4i);
    qf[2] = qa[2]+dt/6*(K1j+2*K2j+2*K3j+K4j);
    qf[3] = qa[3]+dt/6*(K1k+2*K2k+2*K3k+K4k);

    // --------- MATRIZ DE ROTAÇÃO ---------
    float MR[3][3];
    MR[0][0] = pow(qf[0], 2) + pow(qf[1], 2) - pow(qf[2], 2) - pow(qf[3], 2);
    MR[0][1] = 2 * (qf[1] * qf[2] - qf[3] * qf[0]); 
    MR[0][2] = 2 * (qf[1] * qf[3] + qf[2] * qf[0]);
    MR[1][0] = 2 * (qf[1] * qf[2] + qf[3] * qf[0]);
    MR[1][1] = pow(qf[0], 2) - pow(qf[1], 2) + pow(qf[2], 2) - pow(qf[3], 2);
    MR[1][2] = 2 * (qf[2] * qf[3] - qf[0] * qf[1]);
    MR[2][0] = 2 * (qf[1] * qf[3] - qf[2] * qf[0]);
    MR[2][1] = 2 * (qf[0] * qf[1] + qf[2] * qf[3]); 
    MR[2][2] = pow(qf[0], 2) - pow(qf[1], 2) - pow(qf[2], 2) + pow(qf[3], 2);

    // REMOÇÃO DAS COMPONENTES DA GRAVIDADE DO BODY FRAME

    float qfg[4]; //Quaternion para encontrar as componentes da gravidade no BODY FRAME
    qfg[0] = qf[0]; 
    qfg[1] = - qf[1]; 
    qfg[2] = - qf[2]; 
    qfg[3] = - qf[3];

    float Gcomp[3]; //Componentes gx', gy', gz'
    Gcomp[0] = (2 * (qfg[0] * qfg[2] + qfg[1] * qfg[3])) * G; //Componente gx'
    Gcomp[1] = (2 * (qfg[2] * qfg[3] - qfg[0] * qfg[1])) * G; //Componente gy'
    Gcomp[2] = (pow(qfg[0], 2) - pow(qfg[1], 2) - pow(qfg[2], 2) + pow(qfg[3], 2)) * G; //Componente gz'

    as[0] = as[0] - Gcomp[0]; // Aceleração gx' no bodyframe sem a gravidade
    as[1] = as[1] - Gcomp[1]; // Aceleração gy' no bodyframe sem a gravidade
    as[2] = as[2] - Gcomp[2]; // Aceleração gz' no bodyframe sem a gravidade

    // --------- ACELERAÇÕES NO SISTEMA GLOBAL ------//
    float acterra[3];
    acterra[0] = MR[0][0] * as[0] + MR[0][1] * as[1] + MR[0][2] * as[2];
    acterra[1] = MR[1][0] * as[0] + MR[1][1] * as[1] + MR[1][2] * as[2];
    acterra[2] = MR[2][0] * as[0] + MR[2][1] * as[1] + MR[2][2] * as[2];

    cout << "Aceleracoes no sistema de coordenadas global" << endl;
    cout << "A aceleracao ax e: \n" << acterra[0] << endl;
    cout << "A aceleracao ay e: \n" << acterra[1] << endl;
    cout << "A aceleracao az e: \n" << acterra[2] << endl;

    system("pause");
    return(0);
}