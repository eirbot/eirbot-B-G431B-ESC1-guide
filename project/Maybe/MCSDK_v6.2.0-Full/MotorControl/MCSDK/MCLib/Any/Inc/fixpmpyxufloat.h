/**
  ******************************************************************************
  * @file    fixpmpyxufloat.h
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
/* fixpmpyxufloat.h */

#ifndef _FIXPMPYXUFLOAT_H_
#define _FIXPMPYXUFLOAT_H_

/* Multiply a variable of _any_ FIXP format with a floating point number
 * This is used for compile-time constant float values only
 */
#define FIXPmpy_xuFloat(var,fixed_float) \
         (((fixed_float) < 1.0)         ? (FIXP30_rsmpy((var),FIXP30(fixed_float)))\
        :(((fixed_float) < 2.0)         ? (FIXP29_rsmpy((var),FIXP29(fixed_float)))\
        :(((fixed_float) < 4.0)         ? (FIXP28_rsmpy((var),FIXP28(fixed_float)))\
        :(((fixed_float) < 8.0)         ? (FIXP27_rsmpy((var),FIXP27(fixed_float)))\
        :(((fixed_float) < 16.0)        ? (FIXP26_rsmpy((var),FIXP26(fixed_float)))\
        :(((fixed_float) < 32.0)        ? (FIXP25_rsmpy((var),FIXP25(fixed_float)))\
        :(((fixed_float) < 64.0)        ? (FIXP24_rsmpy((var),FIXP24(fixed_float)))\
        :(((fixed_float) < 128.0)       ? (FIXP23_rsmpy((var),FIXP23(fixed_float)))\
        :(((fixed_float) < 256.0)       ? (FIXP22_rsmpy((var),FIXP22(fixed_float)))\
        :(((fixed_float) < 512.0)       ? (FIXP21_rsmpy((var),FIXP21(fixed_float)))\
        :(((fixed_float) < 1024.0)      ? (FIXP20_rsmpy((var),FIXP20(fixed_float)))\
        :(((fixed_float) < 2048.0)      ? (FIXP19_rsmpy((var),FIXP19(fixed_float)))\
        :(((fixed_float) < 4096.0)      ? (FIXP18_rsmpy((var),FIXP18(fixed_float)))\
        :(((fixed_float) < 8192.0)      ? (FIXP17_rsmpy((var),FIXP17(fixed_float)))\
        :(((fixed_float) < 16384.0)     ? (FIXP16_rsmpy((var),FIXP16(fixed_float)))\
        :(((fixed_float) < 32768.0)     ? (FIXP15_rsmpy((var),FIXP15(fixed_float)))\
        :(((fixed_float) < 65536.0)     ? (FIXP14_rsmpy((var),FIXP14(fixed_float)))\
        :(((fixed_float) < 131072.0)    ? (FIXP13_rsmpy((var),FIXP13(fixed_float)))\
        :(((fixed_float) < 262144.0)    ? (FIXP12_rsmpy((var),FIXP12(fixed_float)))\
        :(((fixed_float) < 524288.0)    ? (FIXP11_rsmpy((var),FIXP11(fixed_float)))\
        :(((fixed_float) < 1048567.0)   ? (FIXP10_rsmpy((var),FIXP10(fixed_float)))\
        :(((fixed_float) < 2097152.0)   ? ( FIXP9_rsmpy((var),FIXP9(fixed_float))) \
        :(((fixed_float) < 4194304.0)   ? ( FIXP8_rsmpy((var),FIXP8(fixed_float))) \
        :(((fixed_float) < 8388608.0)   ? ( FIXP7_rsmpy((var),FIXP7(fixed_float))) \
        :(((fixed_float) < 16777216.0)  ? ( FIXP6_rsmpy((var),FIXP6(fixed_float))) \
        :(((fixed_float) < 33554432.0)  ? ( FIXP5_rsmpy((var),FIXP5(fixed_float))) \
        :(((fixed_float) < 67108864.0)  ? ( FIXP4_rsmpy((var),FIXP4(fixed_float))) \
        :(((fixed_float) < 134217728.0) ? ( FIXP3_rsmpy((var),FIXP3(fixed_float))) \
        :(((fixed_float) < 268435456.0) ? ( FIXP2_rsmpy((var), FIXP2(fixed_float)))\
        :                                 ( FIXP1_mpy((var),   FIXP1(fixed_float))))))))))))))))))))))))))))))))

#endif /* _FIXPMPYXUFLOAT_H_ */

/* end of fixpmpyxufloat.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
