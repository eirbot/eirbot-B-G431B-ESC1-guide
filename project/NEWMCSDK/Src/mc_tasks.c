
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/
/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

#define STOPPERMANENCY_MS              ((uint16_t)400)
#define STOPPERMANENCY_MS2             ((uint16_t)400)
#define STOPPERMANENCY_TICKS           (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)  / ((uint16_t)1000))
#define STOPPERMANENCY_TICKS2          (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / ((uint16_t)1000))
/* USER CODE END Private define */

#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)
/* Private variables----------------------------------------------------------*/

static FOCVars_t FOCVars[NBR_OF_MOTORS];

static PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];
//cstat !MISRAC2012-Rule-8.9_a
static RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                    during the start-up switch over. */

static uint16_t hMFTaskCounterM1 = 0; //cstat !MISRAC2012-Rule-8.9_a
static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

static volatile uint8_t bMCBootCompleted = ((uint8_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0.000\
                                      * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS         (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0\
                                      * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);
MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  */
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  if (MC_NULL == pMCIList)
  {
    /* Nothing to do */
  }
  else
  {

    bMCBootCompleted = (uint8_t )0;

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R3_2_Init(&PWM_Handle_M1);
    ASPEP_start(&aspepOverUartA);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /**************************************/
    /*    Start timers synchronously      */
    /**************************************/
    startTimers();

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    STO_PLL_Init (&STO_PLL_M1);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &STO_PLL_M1._Super);

    /****************************************************/
    /*   Virtual speed sensor component initialization  */
    /****************************************************/
    VSS_Init(&VirtualSpeedSensorM1);

    /**************************************/
    /*   Rev-up component initialization  */
    /**************************************/
    RUC_Init(&RevUpControlM1, pSTC[M1], &VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    //PID_SetIntegralTerm(&PIDIqHandle_M1,3198);
    //PID_SetIntegralTerm(&PIDIdHandle_M1,3198);


    /********************************************************/
    /*   Bus voltage sensor component initialization        */
    /********************************************************/
    (void)RCM_RegisterRegConv(&VbusRegConv_M1);
    RVBS_Init(&BusVoltageSensor_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
    (void)RCM_RegisterRegConv(&TempRegConv_M1);
    NTC_Init(&TempSensor_M1);

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
    MCI_Init(&Mci[M1], pSTC[M1], &FOCVars[M1],pwmcHandle[M1] );
    Mci[M1].pScale = &scaleParams_M1;

    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */
    pMCIList[M1] = &Mci[M1];

    /* Applicative hook in MCBoot() */
    MC_APP_BootHook();

    /* USER CODE BEGIN MCboot 2 */
    PID_SetKI(&PIDSpeedHandle_M1,100);
    /* USER CODE END MCboot 2 */

    bMCBootCompleted = 1U;
  }
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors.
 * - Safety Task.
 * - Power Factor Correction Task (if enabled).
 * - User Interface task.
 */
__weak void MC_RunMotorControlTasks(void)
{
  if (0U == bMCBootCompleted)
  {
    /* Nothing to do */
  }
  else
  {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();

    /* Safety task is run after Medium Frequency task so that
     * it can overcome actions they initiated if needed */
    TSK_SafetyTask();

  }
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
    R3_2_SwitchOffPWM(pwmcHandle[motor]);

  FOC_Clear(motor);
  PQD_Clear(pMPM[motor]);
  /* Disable DPWM mode */
  PWMC_DPWM_ModeDisable(pwmcHandle[motor]);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance.
 *
 * It is to be clocked at the Systick frequency.
 */
__weak void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (((uint8_t)1) == bMCBootCompleted)
  {
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();

      /* Applicative hook at end of Medium Frequency for Motor 1 */
      MC_APP_PostMediumFrequencyHook_M1();

      MCP_Over_UartA.rxBuffer = MCP_Over_UartA.pTransportLayer->fRXPacketProcess(MCP_Over_UartA.pTransportLayer,
                                                                                &MCP_Over_UartA.rxLength);
      if ( 0U == MCP_Over_UartA.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_UartA.pTransportLayer->fGetBuffer(MCP_Over_UartA.pTransportLayer,
                                                     (void **) &MCP_Over_UartA.txBuffer, //cstat !MISRAC2012-Rule-11.3
                                                     MCTL_SYNC))
        {
          /* No buffer available to build the answer ... should not occur */
        }
        else
        {
          MCP_ReceivedPacket(&MCP_Over_UartA);
          MCP_Over_UartA.pTransportLayer->fSendPacket(MCP_Over_UartA.pTransportLayer, MCP_Over_UartA.txBuffer,
                                                      MCP_Over_UartA.txLength, MCTL_SYNC);
          /* No buffer available to build the answer ... should not occur */
        }
      }

      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = (uint16_t)MF_TASK_OCCURENCE_TICKS;
    }
    if(hBootCapDelayCounterM1 > 0U)
    {
      hBootCapDelayCounterM1--;
    }
    else
    {
      /* Nothing to do */
    }
    if(hStopPermanencyCounterM1 > 0U)
    {
      hStopPermanencyCounterM1--;
    }
    else
    {
      /* Nothing to do */
    }
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
  bool IsSpeedReliable = STO_PLL_CalcAvrgMecSpeedUnit(&STO_PLL_M1, &wAux);
  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              /* Calibration already done. Enables only TIM channels */
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);
              R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                PQD_Clear(pMPM[M1]);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
                R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              R3_2_SwitchOffPWM(pwmcHandle[M1]);
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
              STO_PLL_Clear(&STO_PLL_M1);
              /* Enable DPWM mode before Start */
              PWMC_DPWM_ModeEnable( pwmcHandle[M1]);
              FOC_Clear( M1 );

                Mci[M1].State = START;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            qd_t IqdRef;
            bool ObserverConverged;

            /* Execute the Rev Up procedure */
            if(! RUC_Exec(&RevUpControlM1))
            {
            /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
            else
            {
              /* Execute the torque open loop current start-up ramp:
               * Compute the Iq reference current as configured in the Rev Up sequence */
              IqdRef.q = STC_CalcTorqueReference(pSTC[M1]);
              IqdRef.d = FOCVars[M1].UserIdref;
              /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
              FOCVars[M1].Iqdref = IqdRef;
           }

            (void)VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);

            /* Check that startup stage where the observer has to be used has been reached */
            if (true == RUC_FirstAccelerationStageReached(&RevUpControlM1))
            {
              ObserverConverged = STO_PLL_IsObserverConverged(&STO_PLL_M1, &hForcedMecSpeedUnit);
              STO_SetDirection(&STO_PLL_M1, (int8_t)MCI_GetImposedMotorDirection(&Mci[M1]));

              (void)VSS_SetStartTransition(&VirtualSpeedSensorM1, ObserverConverged);
            }
            else
            {
              ObserverConverged = false;
            }
            if (ObserverConverged)
            {
              qd_t StatorCurrent = MCM_Park(FOCVars[M1].Ialphabeta, SPD_GetElAngle(&STO_PLL_M1._Super));

              /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC */
              REMNG_Init(pREMNG[M1]);
              (void)REMNG_ExecRamp(pREMNG[M1], FOCVars[M1].Iqdref.q, 0);
              (void)REMNG_ExecRamp(pREMNG[M1], StatorCurrent.q, TRANSITION_DURATION);
              Mci[M1].State = SWITCH_OVER;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            bool LoopClosed;
            int16_t hForcedMecSpeedUnit;

            if (! RUC_Exec(&RevUpControlM1))
            {
              /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
            else
            {
              /* Compute the virtual speed and positions of the rotor.
                 The function returns true if the virtual speed is in the reliability range */
              LoopClosed = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
              /* Check if the transition ramp has completed. */
              bool tempBool;
              tempBool = VSS_TransitionEnded(&VirtualSpeedSensorM1);
              LoopClosed = LoopClosed || tempBool;

              /* If any of the above conditions is true, the loop is considered closed.
                 The state machine transitions to the RUN state */
              if (true ==  LoopClosed)
              {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
                PID_SetIntegralTerm(&PIDSpeedHandle_M1, 2);
#else
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,2);
#endif
                /* USER CODE BEGIN MediumFrequencyTask M1 1 */

                /* USER CODE END MediumFrequencyTask M1 1 */
                STC_SetSpeedSensor(pSTC[M1], &STO_PLL_M1._Super); /* Observer has converged */
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef(M1);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M1].State = RUN;
              }
            }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

            MCI_ExecBufferedCommands(&Mci[M1]);

              FOC_CalcCurrRef(M1);

              if(!IsSpeedReliable)
              {
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
              }
              else
              {
                /* Nothing to do */
              }
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */

  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)3198));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)3198));

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;

  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1.
  * @param  hTickCount number of ticks to be counted.
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed.
  * @param  none
  * @retval bool true if time has elapsed, false otherwise.
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hBootCapDelayCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1.
  * @param  hTickCount number of ticks to be counted.
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed.
  * @param  none
  * @retval bool true if time is elapsed, false otherwise.
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hStopPermanencyCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t TSK_HighFrequencyTask(void)
{

  uint16_t hFOCreturn;
  uint8_t bMotorNbr = 0;
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

  Observer_Inputs_t STO_Inputs; /* Only if sensorless main */

  STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* Only if sensorless */
  if (SWITCH_OVER == Mci[M1].State)
  {
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
    }
    else
    {
      /* Nothing to do */
    }
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  if(hFOCreturn == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }
  else
  {
    bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);
    STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless */
    STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless */
    (void)( void )STO_PLL_CalcElAngle(&STO_PLL_M1, &STO_Inputs);
    STO_PLL_CalcAvrgElSpeedDpp(&STO_PLL_M1); /* Only in case of Sensor-less */
    if (false == IsAccelerationStageReached)
    {
      STO_ResetPLL(&STO_PLL_M1);
    }
    else
    {
      /* Nothing to do */
    }
    /* Only for sensor-less */
    if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State))
    {
      int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
    }
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

  GLOBAL_TIMESTAMP++;
  if (0U == MCPA_UART_A.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_A);
  }

  return (bMotorNbr);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*PARK_ANGLE_COMPENSATION_FACTOR;
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  RCM_ReadOngoingConv();
  RCM_ExecNextConv();
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
  Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

  return (hCodeError);
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (1U == bMCBootCompleted)
  {
    TSK_SafetyTask_PWMOFF(M1);
    /* User conversion execution */
    RCM_ExecUserConv();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  Safety task implementation if  MC.M1_ON_OVER_VOLTAGE == TURN_OFF_PWM.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink.
  */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  const uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
  /* Check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 = RCM_ExecRegularConv(&TempRegConv_M1);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  CodeReturn |= PWMC_IsFaultOccurred(pwmcHandle[bMotor]);     /* check for fault. It return MC_OVER_CURR or MC_NO_FAULTS
                                                    (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 =  RCM_ExecRegularConv(&VbusRegConv_M1);
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Process faults */

  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    if (MCPA_UART_A.Mark != 0U)
    {
      MCPA_flushDataLog (&MCPA_UART_A);
    }
    else
    {
      /* Nothing to do */
    }
    FOC_Clear(bMotor);
    PQD_Clear(pMPM[bMotor]); //cstat !MISRAC2012-Rule-11.3
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
  }
  else
  {
    /* No errors */
  }
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink.
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
__weak MCI_Handle_t *GetMCI(uint8_t bMotor)
{
  MCI_Handle_t *retVal = MC_NULL; //cstat !MISRAC2012-Rule-8.13
  if (bMotor < (uint8_t)NBR_OF_MOTORS)
  {
    retVal = &Mci[bMotor];
  }
  else
  {
    /* Nothing to do */
  }
  return (retVal);
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
  R3_2_SwitchOffPWM(pwmcHandle[M1]);
  MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);

  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}

__weak void UI_HandleStartStopButton_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  if (IDLE == MC_GetSTMStateMotor1())
  {
    /* Ramp parameters should be tuned for the actual motor */
    (void)MC_StartMotor1();
  }
  else
  {
    (void)MC_StopMotor1();
  }
/* USER CODE END START_STOP_BTN */
}

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration.
  */
__weak void mc_lock_pins (void)
{
LL_GPIO_LockPin(M1_OPAMP3_INT_GAIN_GPIO_Port, M1_OPAMP3_INT_GAIN_Pin);
LL_GPIO_LockPin(M1_OPAMP3_OUT_GPIO_Port, M1_OPAMP3_OUT_Pin);
LL_GPIO_LockPin(M1_TEMPERATURE_GPIO_Port, M1_TEMPERATURE_Pin);
LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
LL_GPIO_LockPin(M1_CURR_SHUNT_W_GPIO_Port, M1_CURR_SHUNT_W_Pin);
LL_GPIO_LockPin(M1_CURR_SHUNT_V_GPIO_Port, M1_CURR_SHUNT_V_Pin);
LL_GPIO_LockPin(M1_CURR_SHUNT_U_GPIO_Port, M1_CURR_SHUNT_U_Pin);
LL_GPIO_LockPin(M1_OPAMP1_INT_GAIN_GPIO_Port, M1_OPAMP1_INT_GAIN_Pin);
LL_GPIO_LockPin(M1_OPAMP1_OUT_GPIO_Port, M1_OPAMP1_OUT_Pin);
LL_GPIO_LockPin(M1_OPAMP2_OUT_GPIO_Port, M1_OPAMP2_OUT_Pin);
LL_GPIO_LockPin(M1_OPAMP2_INT_GAIN_GPIO_Port, M1_OPAMP2_INT_GAIN_Pin);
LL_GPIO_LockPin(M1_PWM_INPUT_GPIO_Port, M1_PWM_INPUT_Pin);
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
}
/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
