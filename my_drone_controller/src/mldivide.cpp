//
// File: mldivide.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>

// Function Definitions
//
// Arguments    : const double A[16]
//                double B[4]
// Return Type  : void
//
namespace coder {
void b_mldivide(const double A[16], double B[4])
{
  double b_A[16];
  double smax;
  int a;
  int i;
  int jA;
  signed char ipiv[4];
  std::copy(&A[0], &A[16], &b_A[0]);
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (int j{0}; j < 3; j++) {
    int b_tmp;
    int jp1j;
    int mmj_tmp;
    signed char i1;
    mmj_tmp = 2 - j;
    b_tmp = j * 5;
    jp1j = b_tmp + 2;
    jA = 4 - j;
    a = 0;
    smax = std::abs(b_A[b_tmp]);
    for (int k{2}; k <= jA; k++) {
      double s;
      s = std::abs(b_A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (b_A[b_tmp + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = static_cast<signed char>(jA + 1);
        smax = b_A[j];
        b_A[j] = b_A[jA];
        b_A[jA] = smax;
        smax = b_A[j + 4];
        b_A[j + 4] = b_A[jA + 4];
        b_A[jA + 4] = smax;
        smax = b_A[j + 8];
        b_A[j + 8] = b_A[jA + 8];
        b_A[jA + 8] = smax;
        smax = b_A[j + 12];
        b_A[j + 12] = b_A[jA + 12];
        b_A[jA + 12] = smax;
      }
      i = (b_tmp - j) + 4;
      for (a = jp1j; a <= i; a++) {
        b_A[a - 1] /= b_A[b_tmp];
      }
    }
    jA = b_tmp;
    for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
      smax = b_A[(b_tmp + (jp1j << 2)) + 4];
      if (smax != 0.0) {
        i = jA + 6;
        a = (jA - j) + 8;
        for (int k{i}; k <= a; k++) {
          b_A[k - 1] += b_A[((b_tmp + k) - jA) - 5] * -smax;
        }
      }
      jA += 4;
    }
    i1 = ipiv[j];
    if (i1 != j + 1) {
      smax = B[j];
      B[j] = B[i1 - 1];
      B[i1 - 1] = smax;
    }
  }
  for (int k{0}; k < 4; k++) {
    jA = k << 2;
    if (B[k] != 0.0) {
      i = k + 2;
      for (a = i; a < 5; a++) {
        B[a - 1] -= B[k] * b_A[(a + jA) - 1];
      }
    }
  }
  for (int k{3}; k >= 0; k--) {
    jA = k << 2;
    smax = B[k];
    if (smax != 0.0) {
      smax /= b_A[k + jA];
      B[k] = smax;
      for (a = 0; a < k; a++) {
        B[a] -= B[k] * b_A[a + jA];
      }
    }
  }
}

//
// Arguments    : const double A[36]
//                double B[6]
// Return Type  : void
//
void mldivide(const double A[36], double B[6])
{
  double b_A[36];
  double smax;
  int a;
  int i;
  int jA;
  signed char ipiv[6];
  std::copy(&A[0], &A[36], &b_A[0]);
  for (i = 0; i < 6; i++) {
    ipiv[i] = static_cast<signed char>(i + 1);
  }
  for (int j{0}; j < 5; j++) {
    int A_tmp;
    int b_tmp;
    int jp1j;
    int mmj_tmp;
    signed char i1;
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    jA = 6 - j;
    a = 0;
    smax = std::abs(b_A[b_tmp]);
    for (int k{2}; k <= jA; k++) {
      double s;
      s = std::abs(b_A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (b_A[b_tmp + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = static_cast<signed char>(jA + 1);
        for (int k{0}; k < 6; k++) {
          a = j + k * 6;
          smax = b_A[a];
          A_tmp = jA + k * 6;
          b_A[a] = b_A[A_tmp];
          b_A[A_tmp] = smax;
        }
      }
      i = (b_tmp - j) + 6;
      for (a = jp1j; a <= i; a++) {
        b_A[a - 1] /= b_A[b_tmp];
      }
    }
    jA = b_tmp;
    for (A_tmp = 0; A_tmp <= mmj_tmp; A_tmp++) {
      smax = b_A[(b_tmp + A_tmp * 6) + 6];
      if (smax != 0.0) {
        i = jA + 8;
        a = (jA - j) + 12;
        for (jp1j = i; jp1j <= a; jp1j++) {
          b_A[jp1j - 1] += b_A[((b_tmp + jp1j) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
    i1 = ipiv[j];
    if (i1 != j + 1) {
      smax = B[j];
      B[j] = B[i1 - 1];
      B[i1 - 1] = smax;
    }
  }
  for (int k{0}; k < 6; k++) {
    jA = 6 * k;
    if (B[k] != 0.0) {
      i = k + 2;
      for (a = i; a < 7; a++) {
        B[a - 1] -= B[k] * b_A[(a + jA) - 1];
      }
    }
  }
  for (int k{5}; k >= 0; k--) {
    jA = 6 * k;
    smax = B[k];
    if (smax != 0.0) {
      smax /= b_A[k + jA];
      B[k] = smax;
      for (a = 0; a < k; a++) {
        B[a] -= B[k] * b_A[a + jA];
      }
    }
  }
}

//
// Arguments    : const double A[9]
//                const double B[3]
//                double Y[3]
// Return Type  : void
//
void mldivide(const double A[9], const double B[3], double Y[3])
{
  double b_A[9];
  double a21;
  double maxval;
  int r1;
  int r2;
  int r3;
  std::copy(&A[0], &A[9], &b_A[0]);
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(A[0]);
  a21 = std::abs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (std::abs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (std::abs(b_A[r3 + 3]) > std::abs(b_A[r2 + 3])) {
    int rtemp;
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  Y[1] = B[r2] - B[r1] * b_A[r2];
  Y[2] = (B[r3] - B[r1] * b_A[r3]) - Y[1] * b_A[r3 + 3];
  Y[2] /= b_A[r3 + 6];
  Y[0] = B[r1] - Y[2] * b_A[r1 + 6];
  Y[1] -= Y[2] * b_A[r2 + 6];
  Y[1] /= b_A[r2 + 3];
  Y[0] -= Y[1] * b_A[r1 + 3];
  Y[0] /= b_A[r1];
}

} // namespace coder

//
// File trailer for mldivide.cpp
//
// [EOF]
//