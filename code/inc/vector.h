#ifndef _POINT_H_
#define _POINT_H_

#include "zf_common_typedef.h"

typedef struct _Vector3fType	// 三维向量及其角度，贝塞尔Vector有一个角度和贝塞尔插值t∈[0,1]，但没有用到
{
	float x;
	float y;
	float z;
} Vector3f;

typedef struct _Vector2fType	// 二维向量及其角度，贝塞尔Vector有一个角度和贝塞尔插值t∈[0,1]，但没有用到
{
	float x;
	float y;
} Vector2f;

float	 Vector2fInnerProduct(Vector2f a, Vector2f b);	  // 向量内积
float	 Vector2fMagnitude(Vector2f a);					  // 向量取模
Vector2f Vector2fDivideNum(Vector2f a, float num);		  // 向量/数
Vector2f Vector2fXNum(Vector2f a, float num);			  // 向量*数
Vector2f Vector2fAdd(Vector2f a, Vector2f b);			  // 向量加
Vector2f Vector2fSub(Vector2f a, Vector2f b);			  // 向量减

Vector2f BezierCalculate(uint8 degree, Vector2f coeff[], float t);
void	 Rotate(Vector2f* point, Vector2f point0, float angle);

#endif
