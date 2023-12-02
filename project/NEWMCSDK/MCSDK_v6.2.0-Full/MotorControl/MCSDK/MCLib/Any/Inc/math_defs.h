/**
  ******************************************************************************
  * @file    math_defs.h
  * @author  Piak Electronic Design B.V.
  * @brief   This file is part of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Piak Electronic Design B.V.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#ifndef _math_defs_h
#define	_math_defs_h

//	System includes.
#include	<stdlib.h>
#include	<math.h>

//	Max Cos table _MTL = 2^k.
//	Actual length = _MTL+1.
#define		_MTL	128

//	Position of binary point.
#define		_MNB	30

//	Cos table.
#include	"cos_tab_128.h"

//	Maximum bits of precision.
#define		_BITS	24

//	Maximum number of Cordic iterations.
#define		_MAXI	_BITS+1

//	Float DQ structure prototype.
typedef struct _DQF {
	//	z = {x + j*y} complex pair.
	float	x, y;

	//	r = |z|, th = atan2(y, x).
	float	r, th, thr;

	//	Cordic consts.
	float	g, K;

	//	pi/2, pi, and 2*pi.
	float	hpi, pi, pi2;

	//	Cordic tables.
	float	atan[_MAXI], gain[_MAXI], conv[_MAXI];

	//	Cordic precision.
	int		n;
} dqf, *DQF;

//	Int DQ structure prototype.
typedef struct _DQI {
	//	z = {x + j*y} complex pair.
	int		x, y;

	//	r = |z|, th = atan2(y, x).
	int		r, th, thr;
	float	rf, thf;

	//	Cordic consts.
	int		g, K;

	//	pi/2, pi, 2*pi and 1.
	int		hpi, pi, pi2, one;

	//	Cordic tables.
	int		atan[_MAXI], gain[_MAXI], conv[_MAXI];

	//	Cordic & bit precision.
	int		n, b;
} dqi, *DQI;

//	Long DQ structure prototype.
typedef struct _DQL {
	//	z = {x + j*y} complex pair.
	long		x, y;

	//	r = |z|, th = atan2(y, x).
	long long	r, th, thr;
	float		rf, thf;

	//	Cordic consts.
	long long	g, K;

	//	pi/2, pi, 2*pi and 1.
	long long	hpi, pi, pi2, one;

	//	Cordic tables.
	long long	atan[_MAXI], gain[_MAXI], conv[_MAXI];

	//	Cordic & bit precision.
	int		n, b;
} dql, *DQL;

//	Cos, Sin poly size.
#define		_CSSN	6

//	Atan poly size.
#define		_ATAN	7

//	MNB: 2^_MNB.
#define		_ONEI	1073741824L
#define		_ZERO	0L

//	Maximum limit.
#define		_MPL	_ONEI

//	Float 1.0/6.0 and 2.0/pi.
#define		_SIXIF	0.166666666f
#define		_HPIIF	0.636619772f

//	_MNB: 1.0/6.0 and 2.0/pi.
#define		_SIXII	178956971L
#define		_HPIII	683565276L

//	Float pi/2, pi, and 2*pi.
#define		_HPIF	1.570796326f
#define		_PIF	3.141592653f
#define		_PIF2	6.283185307f

//	_MNB: pi/2, pi, and 2*pi.
#define		_HPII	1686629714L
#define		_PII	3373259427L
#define		_PII2	6746518853L

//	Float Cordic convergence & gain factors.
#define		_CCNF	0.607252935f
#define		_CGNF	0.858785336f

//	_MNB: Cordic convergence & gain factors.
#define		_CCNI	652032874L
#define		_CGNI	922113734L

//	cstl[k] = _RNDL(csti[k]*((float) _MPL)).
static	long 	_cstl[4*_MTL+2];


//	ccsl[k] = _RNDL(ccsf[k]*((float) _MPL)).
static	long _ccsl[_CSSN] = { 1073741824, -536870908,
			44739216, -1491255, 26587, -280 };

//	csnl[k] = _RNDL(csnf[k]*((float) _MPL)).
static	long _csnl[_CSSN] = { 1073741824, -178956970,
			8947847, -213040, 2956, -26 };


#define	_MAX(x, y)	((x) > (y) ? (x) : (y))
#define	_MIN(x, y)  ((x) < (y) ? (x) : (y))
#define _SGNF(x)	((float) (((x) >= 0.0f) - ((x) < 0.0f)))
#define _SGNI(x)	((int) (((x) >= 0) - ((x) < 0)))

//	Inline: (float) round((float) x).
static inline	float	_RNDF(float x) {
	//	Round from zero.
	if (x >= 0.0f)
		return	(float) ((int) (x + 0.5f));
	else
		return	(float) ((int) (x - 0.5f));
}

//	Inline: (int) round((float) x).
static inline	int	_RNDI(float x) {
	//	Round from zero.
	if (x >= 0.0f)
		return	(int) (x + 0.5f);
	else
		return	(int) (x - 0.5f);
}

//	Inline: (long) round((float) x).
static inline	long _RNDL(float x) {
	//	Round from zero.
	if (x >= 0.0f)
		return	(long) (x + 0.5f);
	else
		return	(long) (x - 0.5f);
}

//	Inline: (long long) round((float) x).
static inline	long long _RNDLL(float x) {
	//	Round from zero.
	if (x >= 0.0f)
		return	(long long) (x + 0.5f);
	else
		return	(long long) (x - 0.5f);
}

//	Inline: (int) round(log2f((float) m)).
static inline int log2i(int m) {
	//	Initialize.
	int p2 = 0, k = 1;

	//	Find p2, m <= 2^p2.
	while (k < m) {
		p2++;
		k <<= 1;
	}

	//	Return p2.
	return p2;
}

//
//	Approximate 1/(x-1) for x = 2^k, e.g. for 
//	k = 2, 1/(4-1) = 1/3 = 1/4 + 1/16 + 1/64 + ...
//	to b bits of precision.
//
static inline	long fracgen(int k, int b) {
	//
	long e, z;

	//	Initialize.
	e = 1L << b;
	z = 0;

	//	e = 2^-k, z = e + e*e + e*e*e + ...
	while (e >>= k)
		z += e;

	return z;
}

//	Function prototypes.
void	cos_sin_poly_L(DQL zi);

void	cos_sin_tabl_L(DQL zi, int m);

void	cos_sin_tabl_LX(DQL zi, int m);

//
#endif	//	End math_defs.h

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
