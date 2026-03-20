#ifndef DRONE_CODEGEN_H
#define DRONE_CODEGEN_H

#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

class Drone_codegen {
public:
    // O PositionCtrl agora é nosso calculador de setpoints para o ROS
    Drone_codegen *PositionCtrl(const double Xd[3], const double Vd[3], const double Ad[3]);
    Drone_codegen *init();

    // --- ESTADOS REAIS (Serão preenchidos pelo Subscriber do ROS) ---
    double r[3];     // Posição: x, y, z
    double dr[3];    // Velocidade: vx, vy, vz
    double euler[3]; // Atitude atual: roll, pitch, yaw
    double w[3];     // Velocidade angular: p, q, r

    // --- PARÂMETROS FÍSICOS ---
    double g;
    double m;
    double dt;

    // --- COMANDOS CALCULADOS (O main.cpp vai ler daqui para enviar ao drone) ---
    double phi_des;   // Roll desejado
    double theta_des; // Pitch desejado
    double psi_des;   // Yaw desejado (normalmente fixo ou vindo do planner)
    double zdot_des;  // Velocidade vertical desejada

    // --- MEMÓRIA DOS CONTROLADORES (Para os termos Integrais e Derivativos) ---
    double r_err_sum[3];
    double phi_err_sum;
    double theta_err_sum;
    double psi_err_sum;
    double zdot_err_sum;
    double zdot_err_prev;

    // --- GANHOS (Já inicializados no .cpp) ---
    double kP_pos[3], kI_pos[3], kD_pos[3];
    double kP_phi, kD_phi;
    double kP_theta, kD_theta;
    double kP_zdot, kI_zdot, kD_zdot;

    // Variáveis auxiliares de referência
    double r_des[3];
    double dr_des[3];
    double r_err[3];

private:
    Drone_codegen *handle_init();
};

#endif