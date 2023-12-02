/**
  ******************************************************************************
  * @file    cos_sin.c
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
//
#include	"math_defs.h"

//	Needed for printf(), etc.
// #include	<stdio.h>

//
//	 Assumes a normalized angle: 0 <= th/(2*pi) <= 1.
// Fast poly z = cos(th) + j*sin(th) int32 precision,
// for any value of th using 10th order polynomial
// approximations, absolute error = |e| < 10^-5. See:
// Abramowitz and Stegun, "Handbook of Mathematical
// Functions", p. 76. Polynomial coeffs. in cordicx.h
//
// cos_sin_poly_L(&zi);
//
void cos_sin_poly_L(DQL zi) {
	//
	// Cos, Sin coeffs.
	static	long long	ccsi[_CSSN], csni[_CSSN];
	static	long long	pi2, pi, hpi, one;
	static	int			ob, n = 0;
	
	//	Misc. variables.
	int			b, bb, hth, i, quad;
	long long	th, th2, cs, sn;
	
	//	Number of bits.
	b = zi->b;

	//	Convert coefficients?
	if (b != ob) {
		//	Difference bits.
		bb = _MNB - b;
		ob = b;

		//	Move & scale cos,sin coeffs.
		for (i = 0; i < _CSSN; i++) {
			//	Scale coeffs.
			ccsi[i] = _ccsl[i] >> bb;
			csni[i] = _csnl[i] >> bb;
			
			//	Both coefs. zero?
			if (ccsi[i] == 0LL && csni[i] == 0LL)
				break;
		}

		//	Last non-zero coefficients.
		n = i;

		//	Constants.
		pi2 = _PII2 >> bb;
		pi = _PII >> bb;
		hpi = _HPII >> bb;
		one = _ONEI >> bb;
	}

	//	th = thr/(2*pi).
	th = (zi->thr*pi) >> (zi->b - 1);

	//	|th| > pi?
	if (th > pi)
		th -= pi2;
	else if (th < -pi)
		th += pi2;

	//	Get quadrant.
	if (th > _ZERO) {
		if (th > hpi) {
			th -= hpi;
			quad = 1;
		}
		else
			quad = 0;
	}
	else {
		if (th < -hpi) {
			th += hpi;
			quad = 2;
		}
		else
			quad = 3;
	}

	// 	1/2 Angle avoids overflow.
	hth = (int) (llabs(th) > one) ? 1:0;
	if (hth) {
		th = th >> 1;
	}
	//	Angle squared.
	th2 = (th*th) >> b;

	//	Initialize.
	cs = ccsi[n-1];
	sn = csni[n-1];

	//	Nested polynomial evaluation.
	for (i = n-2; i >= 0; i--) {
		cs = ((cs*th2) >> b) + ccsi[i];
		sn = ((sn*th2) >> b) + csni[i];
	}

	//	Complete Sin.
	sn = (sn*th) >> b;

	//	Half angle conversion?
	if (hth) {
		//	sin(x) = 2*cos(x/2)*sin(x/2).
		sn = (cs*sn) >> (b - 1);
		
		//	cos(x) = 2*cos(x/2)*cos(x/2) - 1.
		cs = ((cs*cs) >> (b - 1)) - one;
	}

	//	Quadrant conversion.
	switch (quad) {
	case 0:
		zi->x = (long) cs;
		zi->y = (long) sn;
		return;
	case 1:
		zi->x = (long) -sn;
		zi->y = (long) cs;
		return;
	case 2:
		zi->x = (long) sn;
		zi->y = (long) -cs;
		return;
	case 3:
		zi->x = (long) cs;
		zi->y = (long) sn;
	default:
		return;
	}
}
//
//	 Assumes a normalized angle: 0 <= th/(2*pi) <= 1.
// Fast table z = cos(th) + j*sin(th) (long) precision,
// for any value of th using m+1 entry cos table + Taylor
// extrapolation or linear interpolation, absolute. 
//
// cos_sin_tabl_L(&zi, m);
//
void	cos_sin_tabl_L(DQL zi, int m) {
	//
	static long dth, dthi, sixi, pi, pi2, hpi, one;
	static int	 tpk, bb, ob = 0, om = 0;

	//
	long th, ths, thf, thr, thr2, cs, sn, dcs, dsn;
	int			i, k, b, mk, quad;

	//	Number of bits.
	b = zi->b;

	//	Fill table?
	if (_cstl[0] == 0LL || b != ob || m != om) {
		//	Find tpk, m = 2^tpk.
		tpk = log2i(m);
		om = m;

		//	Difference bits.
		bb = _MNB - b;
		ob = b;

		//	Constants scaled to b.
		one = _ONEI >> bb;
		dthi = _HPIII >> bb;
		hpi = _HPII >> bb;
		pi = _PII >> bb;
		pi2 = _PII2 >> bb;
		sixi = _SIXII >> bb;

		//	Theta increment 2^-tpk.
		dth = (long ) (one >> (tpk + 2));
		(void) dth; /* Suppress warning not used */
		th = 0LL;

		//	Save old value.
		ths = zi->thr;

		//	1st and last entries.
		_cstl[0] = one;
		_cstl[m] = _ZERO;

		mk = _MTL >> tpk;

#if 0
		//	Use poly to fill cos table.
		for (k = 1; k < m; k++) {
			//	Fraction of pi/2.
			th += dth;
			zi->thr = (long)th;

			//	Int Cos,sin poly.
			cos_sin_poly_L(zi);

			//	Cos only for table.
			_cstl[k] = zi->x;
		}
#else
		//	Scaled subset cos table entries.
		for (i = 1, k = mk; k < _MTL; k += mk, i++)
			_cstl[i] = _cst_128[k] >> bb;
#endif
		//	Restore old value.
		zi->thr = ths;
	}

	//	th = |thr|/(2*pi).
	th = (llabs(zi->thr)*hpi) >> (b - 2);

	//	|th| > pi?
	if (th > pi)
		th -= pi2;

	//	Get quadrant.
	if (th > _ZERO) {
		if (th > hpi) {
			th -= pi;
			quad = 1;
		}
		else
			quad = 0;
	}
	else {
		if (th < -hpi) {
			th += pi;
			quad = 2;
		}
		else
			quad = 3;
	}

	//	ths = |th|*(2/pi).
	ths = (labs(th)*dthi) >> (b - tpk);

	//	k = ths/m, 0 <= k < m.
	k = (int) (ths >> b);

	//	thf = (ths - k*m).
	//  Note use of bit ops.
	thf = ths & ~(k << b);

	//	Table Cos, Sin.
	cs = _cstl[k];
	mk = m - k;
	sn = _cstl[mk];

	//	Extrapolation method?
	if (m <= 16) {
		//	thr = (ths - k)*((pi/2)/m).
		thr = (thf*hpi) >> (b + tpk);

		//	thr2 = thr*thr.
		thr2 = (thr*thr) >> b;

		//	dcs = 1 - (thr*thr/2).
		dcs = one - (thr2 >> 1);

		//	dsn = thr*(1 - thr*thr/6).
		dsn = (thr*(one - ((thr2*sixi) >> b)) >> b);

		//	Rotate Cos, Sin.
		th = (dcs*cs - dsn*sn) >> b;
		sn = (dcs*sn + dsn*cs) >> b;
		cs = th;
	}
	//	Linear interpolation.
	else if (k < m) {
		//	Secant approximation.
		cs += (((_cstl[k + 1] - cs)*thf) >> b);
		sn += (((_cstl[mk - 1] - sn)*thf) >> b);
	}

	//	Quadrant conversion.
	switch (quad) {
	case 0:
		zi->x = (long) cs;
		zi->y = (long) sn;
		return;
	case 1:
		zi->x = (long) -cs;
		zi->y = (long) sn;
		return;
	case 2:
		zi->x = (long) -cs;
		zi->y = (long) -sn;
		return;
	case 3:
		zi->x = (long) cs;
		zi->y = (long) -sn;
	default:
		return;
	}
}
//
//	 Assumes a normalized angle: 0 <= th/(2*pi) < 1.
// Fast table z = cos(th) + j*sin(th) (long) precision,
// for a value of th using 4*m+2 entry cos table + Taylor
// extrapolation or linear interpolation, absolute.
//
// cos_sin_tabl_LX(&zi, m);
//
void	cos_sin_tabl_LX(DQL zi, int m) {
	//
	static	long	dth, hpi;
	static	long	one, sixth;
	static	int		tpk, bb, ob = 0, om = 0;

	(void) dth; /* Suppress warning not used */

	//
	long		th, ths, thf, thr, thr2, cs, sn, dcs, dsn;
	int			i, k, b, mk, m4;

	//	Number of bits.
	b = zi->b;

	//	Table length * 4.
	m4 = m << 2;

	//	Fill table?
	if (_cstl[0] == 0L || b != ob || m4 != om) {
		//	Find tpk, m = 2^tpk.
		tpk = log2i(m);
		om = m4;

		//	Difference bits.
		bb = _MNB - b;
		ob = b;

		//	Constants scaled to b.
		one = (long) (_ONEI >> bb);
		sixth = (long) (_SIXII >> bb);
		hpi = (long) (_HPII >> bb);

		//	Decimation factor.
		mk = _MTL >> tpk;

		//	Quadrant = 0 cos.
		for (i = 0, k = 0; k <= _MTL; k += mk, i++) {
			_cstl[i] = _cst_128[k] >> bb;

			//	Next lower 2 bits.
			ths = _cst_128[k] >> (bb - 2);

			//	Next 2 lower bits on?
			if ((ths & 3) == 3)
				//	Round up.
				_cstl[i] += 1;
		}

		//	Quadrant = 1 cos.
		for (k = i, i = 0; i <= m; i++)
			_cstl[i+m] = -_cstl[--k];

		//	Quadrants = 2, 3 cos.
		for (i += m, k = i-1; i <= m4; i++)
			_cstl[i] = _cstl[--k];

		//	Extra point.
		_cstl[i] = _cstl[i-1];

	}

	//	th = |thr|.
	th = labs(zi->thr);

	//	ths = |th|*m*4.
	ths = labs(th) << (tpk + 2);

	//	k = ths/m, 0 <= k < m.
	k = (int) (ths >> b);

	//	thf = (ths - k*m).
	//  Note use of bit ops.
	thf = ths & ~(k << b);

	//	Cos value.
	cs = _cstl[k];

	//	Modulo m4 index.
	mk = m + k;
	if (mk > m4)
		mk -= m4;

	//	Sin value.
	sn = -_cstl[mk];

	//	Extrapolation method?
	if (m <= 16) {
		//	thr = (ths - k)*((pi/2)/m).
		thr = (thf*hpi) >> (b + tpk);

		//	thr2 = thr*thr.
		thr2 = (thr*thr) >> b;

		//	dcs = 1 - (thr*thr/2).
		dcs = one - (thr2 >> 1);

		//	dsn = thr*(1 - thr*thr/6).
		dsn = (thr*(one - ((thr2*sixth) >> b)) >> b);

		//	Rotate Cos, Sin.
		th = (dcs*cs - dsn * sn) >> b;
		sn = (dcs*sn + dsn * cs) >> b;
		cs = th;
	}
	//	Linear interpolation.
	else {
		cs += (((_cstl[k + 1] - cs)*thf) >> b);
		sn -= (((_cstl[mk + 1] + sn)*thf) >> b);
	}

	//	Save results.
	zi->x = cs;
	zi->y = sn;
}

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
