/**
  ******************************************************************************
  * @file    cordic_defs.h
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
/* cordic_defs.h */

/* Reduced definition of cordic module registers, based on LL driver */

#ifndef _CORDIC_DEFS_H_
#define _CORDIC_DEFS_H_

/* Register macros from stm32g4xx.h */
/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

/* from core_cm4.h */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* CORDIC from stm32g431xx.h */

/******************************************************************************/
/*                                                                            */
/*                          CORDIC calculation unit                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CORDIC_CSR register  *****************/
#define CORDIC_CSR_FUNC_Pos      (0U)
#define CORDIC_CSR_FUNC_Msk      (0xFUL << CORDIC_CSR_FUNC_Pos)                /*!< 0x0000000F */
#define CORDIC_CSR_FUNC          CORDIC_CSR_FUNC_Msk                           /*!< Function */
#define CORDIC_CSR_FUNC_0        (0x1UL << CORDIC_CSR_FUNC_Pos)                /*!< 0x00000001 */
#define CORDIC_CSR_FUNC_1        (0x2UL << CORDIC_CSR_FUNC_Pos)                /*!< 0x00000002 */
#define CORDIC_CSR_FUNC_2        (0x4UL << CORDIC_CSR_FUNC_Pos)                /*!< 0x00000004 */
#define CORDIC_CSR_FUNC_3        (0x8UL << CORDIC_CSR_FUNC_Pos)                /*!< 0x00000008 */
#define CORDIC_CSR_PRECISION_Pos (4U)
#define CORDIC_CSR_PRECISION_Msk (0xFUL << CORDIC_CSR_PRECISION_Pos)           /*!< 0x000000F0 */
#define CORDIC_CSR_PRECISION     CORDIC_CSR_PRECISION_Msk                      /*!< Precision */
#define CORDIC_CSR_PRECISION_0   (0x1UL << CORDIC_CSR_PRECISION_Pos)           /*!< 0x00000010 */
#define CORDIC_CSR_PRECISION_1   (0x2UL << CORDIC_CSR_PRECISION_Pos)           /*!< 0x00000020 */
#define CORDIC_CSR_PRECISION_2   (0x4UL << CORDIC_CSR_PRECISION_Pos)           /*!< 0x00000040 */
#define CORDIC_CSR_PRECISION_3   (0x8UL << CORDIC_CSR_PRECISION_Pos)           /*!< 0x00000080 */
#define CORDIC_CSR_SCALE_Pos     (8U)
#define CORDIC_CSR_SCALE_Msk     (0x7UL << CORDIC_CSR_SCALE_Pos)               /*!< 0x00000700 */
#define CORDIC_CSR_SCALE         CORDIC_CSR_SCALE_Msk                          /*!< Scaling factor */
#define CORDIC_CSR_SCALE_0       (0x1UL << CORDIC_CSR_SCALE_Pos)               /*!< 0x00000100 */
#define CORDIC_CSR_SCALE_1       (0x2UL << CORDIC_CSR_SCALE_Pos)               /*!< 0x00000200 */
#define CORDIC_CSR_SCALE_2       (0x4UL << CORDIC_CSR_SCALE_Pos)               /*!< 0x00000400 */
#define CORDIC_CSR_IEN_Pos       (16U)
#define CORDIC_CSR_IEN_Msk       (0x1UL << CORDIC_CSR_IEN_Pos)                 /*!< 0x00010000 */
#define CORDIC_CSR_IEN           CORDIC_CSR_IEN_Msk                            /*!< Interrupt Enable */
#define CORDIC_CSR_DMAREN_Pos    (17U)
#define CORDIC_CSR_DMAREN_Msk    (0x1UL << CORDIC_CSR_DMAREN_Pos)              /*!< 0x00020000 */
#define CORDIC_CSR_DMAREN        CORDIC_CSR_DMAREN_Msk                         /*!< DMA Read channel Enable */
#define CORDIC_CSR_DMAWEN_Pos    (18U)
#define CORDIC_CSR_DMAWEN_Msk    (0x1UL << CORDIC_CSR_DMAWEN_Pos)              /*!< 0x00040000 */
#define CORDIC_CSR_DMAWEN        CORDIC_CSR_DMAWEN_Msk                         /*!< DMA Write channel Enable */
#define CORDIC_CSR_NRES_Pos      (19U)
#define CORDIC_CSR_NRES_Msk      (0x1UL << CORDIC_CSR_NRES_Pos)                /*!< 0x00080000 */
#define CORDIC_CSR_NRES          CORDIC_CSR_NRES_Msk                           /*!< Number of results in WDATA register */
#define CORDIC_CSR_NARGS_Pos     (20U)
#define CORDIC_CSR_NARGS_Msk     (0x1UL << CORDIC_CSR_NARGS_Pos)               /*!< 0x00100000 */
#define CORDIC_CSR_NARGS         CORDIC_CSR_NARGS_Msk                          /*!< Number of arguments in RDATA register */
#define CORDIC_CSR_RESSIZE_Pos   (21U)
#define CORDIC_CSR_RESSIZE_Msk   (0x1UL << CORDIC_CSR_RESSIZE_Pos)             /*!< 0x00200000 */
#define CORDIC_CSR_RESSIZE       CORDIC_CSR_RESSIZE_Msk                        /*!< Width of output data */
#define CORDIC_CSR_ARGSIZE_Pos   (22U)
#define CORDIC_CSR_ARGSIZE_Msk   (0x1UL << CORDIC_CSR_ARGSIZE_Pos)             /*!< 0x00400000 */
#define CORDIC_CSR_ARGSIZE       CORDIC_CSR_ARGSIZE_Msk                        /*!< Width of input data */
#define CORDIC_CSR_RRDY_Pos      (31U)
#define CORDIC_CSR_RRDY_Msk      (0x1UL << CORDIC_CSR_RRDY_Pos)                /*!< 0x80000000 */
#define CORDIC_CSR_RRDY          CORDIC_CSR_RRDY_Msk                           /*!< Result Ready Flag */

/*******************  Bit definition for CORDIC_WDATA register  ***************/
#define CORDIC_WDATA_ARG_Pos     (0U)
#define CORDIC_WDATA_ARG_Msk     (0xFFFFFFFFUL << CORDIC_WDATA_ARG_Pos)        /*!< 0xFFFFFFFF */
#define CORDIC_WDATA_ARG         CORDIC_WDATA_ARG_Msk                          /*!< Input Argument */

/*******************  Bit definition for CORDIC_RDATA register  ***************/
#define CORDIC_RDATA_RES_Pos     (0U)
#define CORDIC_RDATA_RES_Msk     (0xFFFFFFFFUL << CORDIC_RDATA_RES_Pos)        /*!< 0xFFFFFFFF */
#define CORDIC_RDATA_RES         CORDIC_RDATA_RES_Msk                          /*!< Output Result */

typedef struct
{
  __IO uint32_t CSR;          /*!< CORDIC control and status register,        Address offset: 0x00 */
  __IO uint32_t WDATA;        /*!< CORDIC argument register,                  Address offset: 0x04 */
  __IO uint32_t RDATA;        /*!< CORDIC result register,                    Address offset: 0x08 */
} CORDIC_Local_TypeDef;

#define LCORDIC              ((CORDIC_Local_TypeDef *) (0x40000000UL + 0x00020000UL + 0x0C00UL))

#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

/* -------------------------------------------- */

#define __STATIC_INLINE                        static inline

#define LL_CORDIC_FLAG_RRDY                CORDIC_CSR_RRDY

#define LL_CORDIC_IT_IEN                   CORDIC_CSR_IEN            /*!< Result Ready interrupt enable */

#define LL_CORDIC_FUNCTION_COSINE          (0x00000000U)                                                          /*!< Cosine */
#define LL_CORDIC_FUNCTION_SINE            ((uint32_t)(CORDIC_CSR_FUNC_0))                                        /*!< Sine */
#define LL_CORDIC_FUNCTION_PHASE           ((uint32_t)(CORDIC_CSR_FUNC_1))                                        /*!< Phase */
#define LL_CORDIC_FUNCTION_MODULUS         ((uint32_t)(CORDIC_CSR_FUNC_1 | CORDIC_CSR_FUNC_0))                    /*!< Modulus */
#define LL_CORDIC_FUNCTION_ARCTANGENT      ((uint32_t)(CORDIC_CSR_FUNC_2))                                        /*!< Arctangent */
#define LL_CORDIC_FUNCTION_HCOSINE         ((uint32_t)(CORDIC_CSR_FUNC_2 | CORDIC_CSR_FUNC_0))                    /*!< Hyperbolic Cosine */
#define LL_CORDIC_FUNCTION_HSINE           ((uint32_t)(CORDIC_CSR_FUNC_2 | CORDIC_CSR_FUNC_1))                    /*!< Hyperbolic Sine */
#define LL_CORDIC_FUNCTION_HARCTANGENT     ((uint32_t)(CORDIC_CSR_FUNC_2 | CORDIC_CSR_FUNC_1 | CORDIC_CSR_FUNC_0))/*!< Hyperbolic Arctangent */
#define LL_CORDIC_FUNCTION_NATURALLOG      ((uint32_t)(CORDIC_CSR_FUNC_3))                                        /*!< Natural Logarithm */
#define LL_CORDIC_FUNCTION_SQUAREROOT      ((uint32_t)(CORDIC_CSR_FUNC_3 | CORDIC_CSR_FUNC_0))                    /*!< Square Root */

#define LL_CORDIC_PRECISION_1CYCLE         ((uint32_t)(CORDIC_CSR_PRECISION_0))
#define LL_CORDIC_PRECISION_2CYCLES        ((uint32_t)(CORDIC_CSR_PRECISION_1))
#define LL_CORDIC_PRECISION_3CYCLES        ((uint32_t)(CORDIC_CSR_PRECISION_1 | CORDIC_CSR_PRECISION_0))
#define LL_CORDIC_PRECISION_4CYCLES        ((uint32_t)(CORDIC_CSR_PRECISION_2))
#define LL_CORDIC_PRECISION_5CYCLES        ((uint32_t)(CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_0))
#define LL_CORDIC_PRECISION_6CYCLES        ((uint32_t)(CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_1))
#define LL_CORDIC_PRECISION_7CYCLES        ((uint32_t)(CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_1 | CORDIC_CSR_PRECISION_0))
#define LL_CORDIC_PRECISION_8CYCLES        ((uint32_t)(CORDIC_CSR_PRECISION_3))
#define LL_CORDIC_PRECISION_9CYCLES        ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_0))
#define LL_CORDIC_PRECISION_10CYCLES       ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_1))
#define LL_CORDIC_PRECISION_11CYCLES       ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_1 | CORDIC_CSR_PRECISION_0))
#define LL_CORDIC_PRECISION_12CYCLES       ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_2))
#define LL_CORDIC_PRECISION_13CYCLES       ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_0))
#define LL_CORDIC_PRECISION_14CYCLES       ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_1))
#define LL_CORDIC_PRECISION_15CYCLES       ((uint32_t)(CORDIC_CSR_PRECISION_3 | CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_1 | CORDIC_CSR_PRECISION_0))

#define LL_CORDIC_SCALE_0                  (0x00000000U)
#define LL_CORDIC_SCALE_1                  ((uint32_t)(CORDIC_CSR_SCALE_0))
#define LL_CORDIC_SCALE_2                  ((uint32_t)(CORDIC_CSR_SCALE_1))
#define LL_CORDIC_SCALE_3                  ((uint32_t)(CORDIC_CSR_SCALE_1 | CORDIC_CSR_SCALE_0))
#define LL_CORDIC_SCALE_4                  ((uint32_t)(CORDIC_CSR_SCALE_2))
#define LL_CORDIC_SCALE_5                  ((uint32_t)(CORDIC_CSR_SCALE_2 | CORDIC_CSR_SCALE_0))
#define LL_CORDIC_SCALE_6                  ((uint32_t)(CORDIC_CSR_SCALE_2 | CORDIC_CSR_SCALE_1))
#define LL_CORDIC_SCALE_7                  ((uint32_t)(CORDIC_CSR_SCALE_2 | CORDIC_CSR_SCALE_1 | CORDIC_CSR_SCALE_0))

#define LL_CORDIC_NBWRITE_1                (0x00000000U)             /*!< One 32-bits write containing either only one
                                                                          32-bit data input (Q1.31 format), or two 16-bit
                                                                          data input (Q1.15 format) packed in one 32 bits Data */
#define LL_CORDIC_NBWRITE_2                CORDIC_CSR_NARGS          /*!< Two 32-bit write containing two 32-bits data input
                                                                          (Q1.31 format) */
#define LL_CORDIC_NBREAD_1                 (0x00000000U)             /*!< One 32-bits read containing either only one
                                                                          data output (Q1.15 format) packed in one 32 bits Data */
#define LL_CORDIC_NBREAD_2                 CORDIC_CSR_NRES           /*!< Two 32-bit Data containing two 32-bits data output */

#define LL_CORDIC_INSIZE_32BITS            (0x00000000U)             /*!< 32 bits input data size (Q1.31 format) */
#define LL_CORDIC_INSIZE_16BITS            CORDIC_CSR_ARGSIZE        /*!< 16 bits input data size (Q1.15 format) */

#define LL_CORDIC_OUTSIZE_32BITS           (0x00000000U)             /*!< 32 bits output data size (Q1.31 format) */
#define LL_CORDIC_OUTSIZE_16BITS           CORDIC_CSR_RESSIZE        /*!< 16 bits output data size (Q1.15 format) */

#define LL_CORDIC_DMA_REG_DATA_IN          (0x00000000U)             /*!< Get address of input data register */
#define LL_CORDIC_DMA_REG_DATA_OUT         (0x00000001U)             /*!< Get address of output data register */

#endif /* _CORDIC_DEFS_H_ */

/* end of cordic_defs.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
