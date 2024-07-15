/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/11
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *
 * PD SRC Sample code
 *
 * This sample code may have compatibility issues and is for learning purposes only.
 * If you want to develop a PD project, please contact FAE.
 *
 * Be sure to remove the pull-down resistors on both CC wires when using this Sample code!
 *
 * The inability to control the VBUS voltage on the board may lead to some compatibility problems,
 * mainly manifested in the inability of some devices to complete the PD communication process.
 *
 *If you are using the low-power function, turn on LowpowerON,
 *sleep after disconnecting the sink, and wake up after connecting.
 */

#include "PD_Process.h"
#include "debug.h"

void TIM0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
UINT8 Tim_Ms_Cnt = 0x00;

/*********************************************************************
 * @fn      TIM1_Init
 *
 * @brief   Initialize TIM1
 *
 * @return  none
 */
void TIM0_Init()
{
    TMR0_DeInit();
    TMR0_TimerInit(SystemCoreClock/1000);
    TMR0_ITCfg(RB_TMR_IF_CYC_END, ENABLE);
    NVIC_EnableIRQ(TIM0_IRQn);
    TMR0_Enable();
}


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    uint32_t ChipID;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    __attribute__((unused)) FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("PD SRC TEST\r\n");

    PD_Init();
    TIM0_Init();
    while (1)
    {
        /* Get the calculated timing interval value */
        TMR0_ITCfg(RB_TMR_IF_CYC_END, DISABLE);
        Tmr_Ms_Dlt = Tim_Ms_Cnt - Tmr_Ms_Cnt_Last;
        Tmr_Ms_Cnt_Last = Tim_Ms_Cnt;
        TMR0_ITCfg(RB_TMR_IF_CYC_END, ENABLE);
        PD_Ctl.Det_Timer += Tmr_Ms_Dlt;
        if (PD_Ctl.Det_Timer > 4)
        {
            PD_Ctl.Det_Timer = 0;
            PD_Det_Proc();
        }
        PD_Main_Proc();
    }
}

/*********************************************************************
 * @fn      TIM1_UP_IRQHandler
 *
 * @brief   This function handles TIM1 interrupt.
 *
 * @return  none
 */

void TIM0_IRQHandler(void)
{
    if (TMR0_GetITFlag(RB_TMR_IF_CYC_END) != RESET)
    {
        Tim_Ms_Cnt++;
        TMR0_ClearITFlag(RB_TMR_IF_CYC_END);
    }
}
