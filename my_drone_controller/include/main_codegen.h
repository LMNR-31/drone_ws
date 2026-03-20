#ifndef MAIN_CODEGEN_H
#define MAIN_CODEGEN_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
// Mudamos para extern para que o compilador saiba que a implementação 
// está nos arquivos gerados pelo MATLAB, mas não chamaremos a "main" do MATLAB, 
// e sim as classes Drone e Planner.
extern void main_codegen(coder::array<double, 2U> &drone1_state_out);

#endif