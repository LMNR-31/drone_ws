/*
 * File: rtwtypes.h
 * Updated for ROS 2 Integration (Independent of tmwtypes.h)
 */

#ifndef RTWTYPES_H
#define RTWTYPES_H

/*=======================================================================*
 * Fixed width word size data types (Standard C++ Definitions)           *
 *=======================================================================*/

#include <stddef.h>

/* Tipos Básicos do MATLAB Coder */
typedef double real_T;
typedef float real32_T;
typedef double real64_T;
typedef unsigned char boolean_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef char char_T;
typedef int int_T;
typedef unsigned int uint_T;

/* Definições de 64 bits para sistemas Linux/x64 (Padrão ROS 2) */
#ifndef INT64_T
typedef long int64_T;
#endif

#ifndef UINT64_T
typedef unsigned long uint64_T;
#endif

/* Definições de Inteiros de Tamanho Nativo */
#define MAX_int8_T      ((int8_T)(127))
#define MIN_int8_T      ((int8_T)(-128))
#define MAX_uint8_T     ((uint8_T)(255))
#define MAX_int16_T     ((int16_T)(32767))
#define MIN_int16_T     ((int16_T)(-32768))
#define MAX_uint16_T    ((uint16_T)(65535))
#define MAX_int32_T     ((int32_T)(2147483647))
#define MIN_int32_T     ((int32_T)(-2147483647-1))
#define MAX_uint32_T    ((uint32_T)(4294967295U))

/* Definição de True/False se não existirem */
#ifndef FALSE
#define FALSE (0U)
#endif
#ifndef TRUE
#define TRUE (1U)
#endif

#endif /* RTWTYPES_H */