#ifndef TRAJECTORYPLANNER_CODEGEN_H
#define TRAJECTORYPLANNER_CODEGEN_H

#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <vector>

class TrajectoryPlanner_codegen {
public:
    TrajectoryPlanner_codegen *init();
    
    // Esta função será o coração do movimento no ROS 2
    void getNextSetpoint(double t_atual, double Xd[3], double Vd[3], double Ad[3]);

    std::vector<double> waypoints;
    std::vector<double> segmentTimes;
    
    coder::array<double, 3U> coefficients; // Matriz de coeficientes do polinômio
    double numSegments;
    double X_final[3];
    double hoverTime[4];
    boolean_T inHover;
    double hoverStartReal;
    double currentSegment;
    double tAccum;
};

#endif