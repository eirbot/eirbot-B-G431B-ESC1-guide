/**
  ******************************************************************************
  * @file    mathlib.c
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
/* mathlib.c */

#include "mathlib.h"

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/* defines */

#define CST_LEN				128
#define _MTL				(CST_LEN)	//	Max Cos table _MTL = 2^k, Actual length = _MTL+1.
#define COSSIN_TABLESIZE	_MTL
#define COSSIN_FORMAT		15
#define _MNB				30			//	Position of binary point.
#define _BITS				24			//	Maximum bits of precision.
#define _MAXI				_BITS+1		//	Maximum number of Cordic iterations.

#ifndef M_TWOPI
#define M_TWOPI 6.28318530717958647f
#endif

/* Fixed point macros */

#define __CFIXPn(A, n)			(int32_t) ((A) * powf(2.0f, n))
#define __CFIXPnmpy(a, b, n)	( (int32_t)(((int64_t) (a) * (b)) >> (n)) )
#define __CFIXPntoF(A, n)		((float_t)(((float_t) (A)) / powf(2.0f, n)))

/* types */

typedef struct _DQL
{
	int		b;			// bit precision.
	int		tpk;		// number of bits required to index length m table (m=128 -> s_tpk=7)
	int		tablesize;	// source table size (RAM table is (4xtablesize)+2 long)
} dql, *DQL;

//	Int DQ structure prototype.
typedef struct _DQI
{
	//	Cordic consts.
	int		g;

	//	pi/2, pi and 1.
	int		hpi, pi, one;

	//	Cordic tables.
	int		atan[_MAXI];

	//	Cordic & bit precision.
	int		n, b;
} dqi, *DQI;

/* variables */

/* RAM table */
//	_cstl[k] = _RNDL(csti[k]*((float) _MPL)).
//	Extra point need for sin interpolation.
long _cstl[4*_MTL+2];

dql cossin_instance =
{
	.b = COSSIN_FORMAT	/* fixp_t<n> format angle and output */
};

dqi cordic_instance =
{
	.n = 15,		/* cordic_iterations */
	.b = 30			/* cordic_bits */
};

//	Cos table: _RNDL(cos(i*pi/(2*_MTL))*(2 << _MNB))
long _cst_128[CST_LEN+1] = {
  1073741824, 1073660973, 1073418433, 1073014240, 1072448455,
  1071721163, 1070832474, 1069782521, 1068571464, 1067199483,
  1065666786, 1063973603, 1062120190, 1060106826, 1057933813,
  1055601479, 1053110176, 1050460278, 1047652185, 1044686319,
  1041563127, 1038283080, 1034846671, 1031254418, 1027506862,
  1023604567, 1019548121, 1015338134, 1010975242, 1006460100,
  1001793390, 996975812, 992008094, 986890984, 981625251,
  976211688, 970651112, 964944360, 959092290, 953095785,
  946955747, 940673101, 934248793, 927683790, 920979082,
  914135678, 907154608, 900036924, 892783698, 885396022,
  877875009, 870221790, 862437520, 854523370, 846480531,
  838310216, 830013654, 821592095, 813046808, 804379079,
  795590213, 786681534, 777654384, 768510122, 759250125,
  749875788, 740388522, 730789757, 721080937, 711263525,
  701339000, 691308855, 681174602, 670937767, 660599890,
  650162530, 639627258, 628995660, 618269338, 607449906,
  596538995, 585538248, 574449320, 563273883, 552013618,
  540670223, 529245404, 517740883, 506158392, 494499676,
  482766489, 470960600, 459083786, 447137835, 435124548,
  423045732, 410903207, 398698801, 386434353, 374111709,
  361732726, 349299266, 336813204, 324276419, 311690799,
  299058239, 286380643, 273659918, 260897982, 248096755,
  235258165, 222384147, 209476638, 196537583, 183568930,
  170572633, 157550647, 144504935, 131437462, 118350194,
  105245103, 92124163, 78989349, 65842639, 52686014,
  39521455, 26350943, 13176464, 0 };

/* function declarations */
void cos_sin_tabl_init(void);
void cordic_initi(DQI zi);

/* functions */

static inline int log2i(int m)
{
	int p2 = 0, k = 1;

	//	Find p2, m <= 2^p2.
	while (k < m)
	{
		p2++;
		k <<= 1;
	}

	return p2;
} /* end of log2i() */

void cos_sin_tabl_init()
{
	/* Not optimized, since we will be converting to a static table, pre-compiled */

	DQL zi = &cossin_instance;
	int tablesize = COSSIN_TABLESIZE;

	int b = zi->b;
	int m4 = tablesize << 2;

	zi->tablesize = tablesize;

	//	Find tpk, m = 2^tpk.
	zi->tpk = log2i(tablesize);	/* number of bits required to index length m table. m=128 -> 7 */

	//	Difference bits.
	int bb = _MNB - b;

	//	Decimation factor.
	int mk;
	mk = _MTL >> zi->tpk;

	//	Quadrant = 0 cos.
	int i;
	int k;
	for (i = 0, k = 0; k <= _MTL; k += mk, i++)
	{
		_cstl[i] = _cst_128[k] >> bb;

		//	Next lower 2 bits.
		long ths;
		ths = _cst_128[k] >> (bb - 2);

		//	Next 2 lower bits on?
		if ((ths & 3) == 3)
		{
			//	Round up.
			_cstl[i] += 1;
		}
	}

	//	Quadrant = 1 cos.
	for (k = i, i = 0; i <= tablesize; i++)
	{
		_cstl[i+tablesize] = -_cstl[--k];
	}

	//	Quadrants = 2, 3 cos.
	for (i += tablesize, k = i-1; i <= m4; i++)
	{
		_cstl[i] = _cstl[--k];
	}

	//	Extra point.
	_cstl[i] = _cstl[i-1];
} /* end of cos_sin_tabl_init() */

// Assumes a normalized angle: 0 <= th/(2*pi) < 1.
// Fast table z = cos(th) + j*sin(th) (long) precision,
// for a value of th using 4*m+2 entry cos table + Taylor
// extrapolation or linear interpolation, absolute.

Vector_cossin_s MATHLIB_cossin(int32_t angle_pu)
{
	/* scale of data is fixp_t<m> format, 15 is working, 30 is not.
	 *
	 * inputs:
	 * 		thr			angle theta in per unit (0 = 0 degrees, 1 = 360 degrees)
	 *
	 * outputs:
	 * 		x			cosine
	 *		y			sine
	 *
	 * tables:
	 * 		_cst_128	128 long cosine table, source data, could be in rom
	 * 		_cstl		4*_MTL+2 = up to 514 long, derived table, in ram
	 *
	 */

	DQL zi = &cossin_instance;

	//	Number of bits.
	int b = zi->b;

	// table size
	int tablesize = zi->tablesize;

	//	ths = |th|*m*4.
	/* shift left (multiply by 2^n) by tpk (7) + 2 = 9 */
	/* ths = fixp15_t(0.999f) << 9, giving fixp24_t(0.999f) */
	/* This causes fixp30_t angle to fail */
	/* tpk comes from needing log2(m) bits to index the table */
	/* the 2 comes from having 1/4 of a rotation in the table */
	long ths = labs(angle_pu) << (zi->tpk + 2);

	//	k = ths/m, 0 <= k < m.
	/* shift right by b=15, k is fixp9_t (for tablesize = 128), i.e. k becomes the index for the cosine value but ... */
	/* fixp9_t(1.0f) = 512, the table is 1/4 of that. The 2 extra bits */
	int k = (int) (ths >> b);

	/* k could be calculated in one step; */
	/* k = _iq<b>toiq<s_tpk+2> = _iq15toiq9 */
	/* k = th >> () */ // _q15 << (s_tkp + 2) >> b == _q15 << (7 + 2) >> 15 == _q15 >> 6 == conversion to _q9

	//	thf = (ths - k*m).
	//  Note use of bit ops.
	long thf = ths & ~(k << b);
		/* used in linear interpolation / extrapolation */
		/* k = index, top bits of angle */
		/* k << b = top bits back in top bits place */
		/* ~ is bitwise NOT, turning those bits off */
		/* & masks the index bits off */

		/* remainder after removing the k*m part */

	//	Cos value.
	long cs = _cstl[k];

	//	Modulo m4 index.
	/* sine table is cosine table + offset, values inverted? */
	int mk = tablesize + k;		/* sine index = tablesize + cosine index */
	int	m4 = tablesize << 2; //	Table length * 4.
	if (mk > m4)			/* wrap sine index (could be bit operation?) */
	{
		mk -= m4;
	}

	//	Sin value.
	long sn = -_cstl[mk];		/* mk is sine index, value is negated because sine table is -cosine table, with an offset? */

	//	Linear interpolation.
	cs += (((_cstl[k + 1] - cs) * thf) >> b);
	sn -= (((_cstl[mk + 1] + sn) * thf) >> b);

	/* linear interpolation for better, smoother result */
	/* k+1 and mk+1 point to the next entry in the table
	 * (_cstl[k+1]-cs) = delta to next		(sine variant uses inverse values)
	 * * thf multiplies with a fixp_t<b> format number (?), giving fixp_t<2*b> intermediate, >> b shifts back to fixp_t<b> format
	 * (this limits the function to (1/2 * fixp32_t format), approx)
	 *
	 * cs += delta_next * thf
	 *
	 * thf must be linear factor dx/step
	 */

	Vector_cossin_s result;
	result.cos = cs;
	result.sin = sn;
	return result;
} /* end of cos_sin_tabl_LX() */

//
//	zi->K factor & atanti for zi->n iterations.
//	Note: Should be called first for init.
//
void cordic_initi(DQI zi)
{
	int		k;		/* Loop variable */
	int		g;		/* term for gain[] calculation */

	//	Convert pi/2, pi, 3*pi/2, 2*pi and one.
	zi->one = __CFIXPn(1.0f, zi->b);
	/* Per unit angles */
	zi->hpi = __CFIXPn(0.25f, zi->b);	/* 1/2 PI, 90 degrees, 0.25f per unit */
	zi->pi = __CFIXPn(0.5f, zi->b);		/* PI, 180 degrees, 0.5f per unit */

	// Initialize.
	g = zi->one;

	// Cordic tables & constants.
	for (k = 0; k < zi->n; k++)
	{
		//	Cordic atan tables.

		/* floating point calculated variant */
		{
			float_t slope = powf(2, -k);
			zi->atan[k] = __CFIXPn((atanf(slope) / M_TWOPI), zi->b);
		}

		//	Gain table.
		if (k < 7)
		{
			g += (g >> 2*(k + 1));
		}
	}

	//	nth constants.
	float_t g_flt = __CFIXPntoF(g, zi->b);		/* Convert g to floating point */
	float_t gain_flt = 1.0f / sqrtf(g_flt);		/* Calculate 1/sqrt(g) */
	zi->g = __CFIXPn(gain_flt, zi->b);			/* Convert result back to fixed point */
} /* end of cordic_initi() */

//	Integer precision (int) version
// to b bits of precision, b = zi->b.
// Cordic algorithm to convert complex
// cartesian to polar form. The input
// is complex pair {x, y} where w = x + j*y
// and the phase angle th = atan2(y, x).
// Applies n iterations of Cordic algorithm
// to rotate w to w' such that imag(w') = 0
// and thus |w'| = real(w'). The result is
// multiplied by the Cordic gain g. Pass
// address &zi of dqi data structure.
//
// Input: x = zi->x, y = zi->y, n = zi->n,
//		  b = zi->b, and g = zi->g.
// Output: th = zi->th, r = zi->r.
//
// cordic_cpoli(&zi);
//
void MATHLIB_polar(const int32_t x, const int32_t y, int32_t *angle_pu, int32_t *magnitude)
{
	DQI zi = &cordic_instance;

	// Get |components|.
	int ax = abs(x);
	int ay = abs(y);

	// Swap x & y?
	bool swapped = false;
	if (ay > ax)
	{
		int temp = ax;
		ax = ay;
		ay = temp;
		swapped = true;
	}

	// Rotate y to zero.
	int sx;
	int sy;
	int th = 0;
	int	k;
	for (k = 1; k < zi->n; k++)
	{
		// Cordic increment.
		sy = ay >> k;
		sx = ax >> k;

		// Cordic rotation.
		if (ay >= 0)
		{
			ax += sy;
			ay -= sx;
			th += zi->atan[k];
		}
		else
		{
			ax -= sy;
			ay += sx;
			th -= zi->atan[k];
		}
	}

	if (swapped) th = zi->hpi - th;			/* x/y swapped */
	if (x < 0) th = zi->pi - th;			/* left half plane */
	if (y < 0) th = zi->one - th;			/* lower half plane */

	*angle_pu = th;

	//	Normalized magnitude.
	*magnitude = __CFIXPnmpy(ax, zi->g, zi->b); /* r = x*g */
} /* end of MATHLIB_polar() */

void MATHLIB_init()
{
	/* Initialize cosine/sine table/object */
	cos_sin_tabl_init();

	/* Initialize cordic table/object */
	cordic_initi(&cordic_instance);
} /* end of MATHLIB_init() */

/* end of mathlib.c */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
