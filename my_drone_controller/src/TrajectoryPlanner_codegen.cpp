#include "TrajectoryPlanner_codegen.h"
#include <cmath>

TrajectoryPlanner_codegen *TrajectoryPlanner_codegen::init() {
    this->currentSegment = 0;
    this->tAccum = 0;
    this->inHover = false;
    
    // Pegamos o 'n' dinâmico que o main() configurou
    int n = this->numSegments; 

    // Aloca a matriz dinamicamente: [n Segmentos x 3 Eixos x 6 Coeficientes]
    this->coefficients.set_size(n, 3, 6);

    // ========================================================
    // A MATEMÁTICA: Dinâmica para 'n' segmentos
    // ========================================================
    for (int seg = 0; seg < n; seg++) {
        double T = this->segmentTimes[seg]; 
        
        for (int dim = 0; dim < 3; dim++) { 
            // O número total de pontos é sempre n + 1
            int total_pontos = n + 1;
            
            // Pega o Ponto A e Ponto B dinamicamente
            double p_start = this->waypoints[seg + dim * total_pontos];
            double p_end   = this->waypoints[(seg + 1) + dim * total_pontos];
            
            double a0 = p_start;               
            double a1 = (p_end - p_start) / T; 
            
            // Salva na matriz do MATLAB calculando o índice dinâmico [n x 3 x 6]
            this->coefficients[seg + dim * n + 0 * (n * 3)] = a0;
            this->coefficients[seg + dim * n + 1 * (n * 3)] = a1;
            this->coefficients[seg + dim * n + 2 * (n * 3)] = 0.0;
            this->coefficients[seg + dim * n + 3 * (n * 3)] = 0.0;
            this->coefficients[seg + dim * n + 4 * (n * 3)] = 0.0;
            this->coefficients[seg + dim * n + 5 * (n * 3)] = 0.0;
        }
    }

    // Posição final de segurança (O último waypoint de cada eixo)
    this->X_final[0] = this->waypoints[n];                 // Último X
    this->X_final[1] = this->waypoints[n + (n + 1)];       // Último Y
    this->X_final[2] = this->waypoints[n + 2 * (n + 1)];   // Último Z

    return this;
}

void TrajectoryPlanner_codegen::getNextSetpoint(double t, double Xd[3], double Vd[3], double Ad[3]) {
    int n = this->numSegments;
    
    // 1. Descobre em qual segmento estamos baseado no tempo atual 't'
    int seg = 0;
    double t_start = 0.0;
    for (int i = 0; i < n; i++) {
        // Usa 0.001 de margem para evitar bugs de arredondamento de float
        if (t <= t_start + this->segmentTimes[i] + 0.001) { 
            seg = i;
            break;
        }
        t_start += this->segmentTimes[i];
        
        // Se passar do último, trava no último segmento
        if (i == n - 1) { 
            seg = n - 1;
        }
    }

    // 2. Trava na posição final se o tempo total da missão estourar
    double total_time = 0;
    for(int i = 0; i < n; i++) {
        total_time += this->segmentTimes[i];
    }
    
    if (t >= total_time) {
        Xd[0] = this->X_final[0]; Xd[1] = this->X_final[1]; Xd[2] = this->X_final[2];
        Vd[0] = 0.0; Vd[1] = 0.0; Vd[2] = 0.0;
        Ad[0] = 0.0; Ad[1] = 0.0; Ad[2] = 0.0;
        return;
    }

    // 3. Tempo local dentro do segmento atual (vai de 0.0 a 5.0)
    double t_seg = t - t_start;

    // 4. Calcula Posição, Velocidade e Aceleração dinamicamente para X(0), Y(1) e Z(2)
    for (int dim = 0; dim < 3; dim++) {
        // Acessa a matriz dinâmica do MATLAB com a matemática certa: [seg + dim*n + coef*(n*3)]
        double a0 = this->coefficients[seg + dim * n + 0 * (n * 3)];
        double a1 = this->coefficients[seg + dim * n + 1 * (n * 3)];
        double a2 = this->coefficients[seg + dim * n + 2 * (n * 3)];
        double a3 = this->coefficients[seg + dim * n + 3 * (n * 3)];
        double a4 = this->coefficients[seg + dim * n + 4 * (n * 3)];
        double a5 = this->coefficients[seg + dim * n + 5 * (n * 3)];

        // Equação da Posição (Xd)
        Xd[dim] = a0 + a1*t_seg + a2*pow(t_seg, 2) + a3*pow(t_seg, 3) + a4*pow(t_seg, 4) + a5*pow(t_seg, 5);
        
        // Equação da Velocidade (Vd) - Derivada 1
        Vd[dim] = a1 + 2*a2*t_seg + 3*a3*pow(t_seg, 2) + 4*a4*pow(t_seg, 3) + 5*a5*pow(t_seg, 4);
        
        // Equação da Aceleração (Ad) - Derivada 2
        Ad[dim] = 2*a2 + 6*a3*t_seg + 12*a4*pow(t_seg, 2) + 20*a5*pow(t_seg, 3);
    }
}