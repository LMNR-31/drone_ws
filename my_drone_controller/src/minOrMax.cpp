//
// File: minOrMax.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[3]
//                double ex[3]
// Return Type  : void
//
namespace coder {
namespace internal {
void maximum2(const double x[3], double ex[3])
{
  ex[0] = std::fmax(x[0], -12.0);
  ex[1] = std::fmax(x[1], -12.0);
  ex[2] = std::fmax(x[2], -12.0);
}

//
// Arguments    : double x
//                double y
// Return Type  : double
//
double maximum2(double x, double y)
{
  return std::fmax(x, y);
}

//
// Arguments    : const double x[3]
//                double ex[3]
// Return Type  : void
//
void minimum2(const double x[3], double ex[3])
{
  ex[0] = std::fmin(x[0], 12.0);
  ex[1] = std::fmin(x[1], 12.0);
  ex[2] = std::fmin(x[2], 12.0);
}

//
// Arguments    : double x
//                double y
// Return Type  : double
//
double minimum2(double x, double y)
{
  return std::fmin(x, y);
}

} // namespace internal
} // namespace coder

//
// File trailer for minOrMax.cpp
//
// [EOF]
//