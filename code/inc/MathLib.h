#ifndef _MATHFUNC_H
#define _MATHFUNC_H

#include "zf_common_typedef.h"

/*-----------------数学宏定义----------------------------*/

#ifndef PI
#define PI 3.1415926535897
#endif

#define DEG2RAD (PI / 180)
#define RAD2DEG (180 / PI)

#define ABS(x)	  ((x) > 0 ? (x) : (-(x)))	   // 取绝对值
#define SIG(x)	  (((x) < 0) ? (-1) : (1))	   // 取符号
#define MIN(x, y) (((x) > (y)) ? (y) : (x))	   // 取最小
#define MAX(x, y) (((x) > (y)) ? (x) : (y))	   // 取最大

// 交换
#define SWAP(x, y, _type) \
	{					  \
		_type z;		  \
		z = x;			  \
		x = y;			  \
		y = z;			  \
	}	 // 指定类型交换两变量

// 限制峰值
#define PEAK(A, B)				  \
	{						  \
		if (ABS(A) > ABS(B))	\
			(A) = SIG(A) * (B); \
	}

// 限幅
#define Limit(A, min, max) \
	{					   \
		if (A > max)	   \
			A = max;	   \
		if (A < min)	   \
			A = min;	   \
	}

// STM32标准库
/*! STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t	s8;

typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t  sc8;

typedef __IO int32_t vs32;
typedef __IO int16_t vs16;
typedef __IO int8_t  vs8;

typedef __I int32_t vsc32;
typedef __I int16_t vsc16;
typedef __I int8_t  vsc8;

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t  uc8;

typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t  vuc8;

// 数学函数声明
float Lerp(float start, float end, float t);
float InvSqrt(float x);
float FastSin(float x);
float FastCos(float x);
float FastAtan2(float y, float x);
float Constrain(float value, float min, float max);
float Map(float value, float fromLow, float fromHigh, float toLow, float toHigh);
int Sign(float value);
float DeadZone(float value, float threshold);

#endif
