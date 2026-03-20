#include "Drone_codegen.h"
#include "cos.h"
#include "minOrMax.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include <algorithm> // <--- CORREÇÃO 2: Biblioteca adicionada para std::max e std::min

// Inicializador simplificado para ROS 2
Drone_codegen *Drone_codegen::init() {
    Drone_codegen *obj = this; // Mudado de handle_init() para 'this' direto para evitar dependências extras
    
    obj->g = 9.81;
    obj->dt = 0.01; // Manteremos o passo de 100Hz
    obj->m = 1.25;
    
    // <--- CORREÇÃO 1: Zerando as variáveis que REALMENTE existem no .h
    for (int i = 0; i < 3; i++) {
        obj->r[i] = 0.0;
        obj->dr[i] = 0.0;
        obj->euler[i] = 0.0;
        obj->w[i] = 0.0;
        obj->r_err_sum[i] = 0.0;
    }
    obj->phi_err_sum = 0.0;
    obj->theta_err_sum = 0.0;
    obj->psi_err_sum = 0.0;
    obj->zdot_err_sum = 0.0;
    obj->zdot_err_prev = 0.0;
    
    // Inicializa ganhos (conforme seu original)
    obj->kP_phi = 4.5; obj->kD_phi = 0.4;
    obj->kP_theta = 4.5; obj->kD_theta = 0.4;
    obj->kP_zdot = 6.0; obj->kI_zdot = 0.1;
    
    // Ganhos de Posição
    obj->kP_pos[0] = 4.2; obj->kI_pos[0] = 0.93; obj->kD_pos[0] = 3.0;
    obj->kP_pos[1] = 4.2; obj->kI_pos[1] = 0.93; obj->kD_pos[1] = 3.0;
    obj->kP_pos[2] = 5.0; obj->kI_pos[2] = 1.0;  obj->kD_pos[2] = 3.0;

    return obj;
}

// O PositionCtrl agora funciona como um "Cérebro" puro
Drone_codegen *Drone_codegen::PositionCtrl(const double Xd[3], const double Vd[3], const double Ad[3]) {
    Drone_codegen *obj = this;
    double p_cmd = obj->dt;
    double dr_err[3];
    
    // Não usamos mais 'dv', removi para não dar warning de variável não usada

    // --- CÁLCULO DO ERRO (Usando a posição que o ROS nos der em obj->r) ---
    for(int i=0; i<3; i++) {
        obj->r_des[i] = Xd[i];
        obj->dr_des[i] = Vd[i];
        obj->r_err[i] = obj->r_des[i] - obj->r[i];
        
        double vel_error = obj->dr_des[i] - obj->dr[i];
        obj->r_err_sum[i] += obj->r_err[i] * p_cmd;
        
        // PID de Posição -> Gera Aceleração Desejada
        dr_err[i] = ((obj->kP_pos[i] * obj->r_err[i] + obj->kI_pos[i] * obj->r_err_sum[i]) +
                     obj->kD_pos[i] * vel_error) + Ad[i];
    }

    // --- MAPEAMENTO PARA ATITUDE (Roll/Pitch) ---
    // theta_des ~ -ax/g | phi_des ~ ay/g
    obj->theta_des = -dr_err[0] / obj->g;
    obj->phi_des = dr_err[1] / obj->g;
    
    // Comando de Velocidade Vertical (Z-dot)
    obj->zdot_des = Vd[2] + obj->kP_pos[2] * obj->r_err[2];

    // Saturação de segurança (máximo 45 graus) usando a biblioteca <algorithm>
    obj->phi_des = std::max(std::min(obj->phi_des, 0.785), -0.785);
    obj->theta_des = std::max(std::min(obj->theta_des, 0.785), -0.785);

    return obj;
}