/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *
 * PD SNK Sample code
 *
 * This sample code may have compatibility issues and is for learning purposes only.
 * If you want to develop a PD project, please contact FAE.

 * Make sure that the board is not powered on before use.
 * Be sure to pay attention to the voltage when changing the request
 * to prevent burning the board.
 *
 * There is no integrated 5.1K pull-down inside the chip,
 * CC_PD is only for status differentiation,
 * bit write 1 means SNK mode, write 0 means SCR mode
 *
 * Modify "PDO_Request( PDO_INDEX_1 )" in pd process.c, line 742, to modify the request voltage.
 *
 *If you select a gear higher than 12V,it is recommended to replace the devices with a higher power.
 *
 * According to the usage scenario of PD SNK, whether
 * it is removed or not should be determined by detecting
 * the Vbus voltage, this code only shows the detection
 * and the subsequent communication flow.
 */

#include "PD_Process.h"
#include "debug.h"

void TIM0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

UINT8 Tim_Ms_Cnt = 0x00;

/*********************************************************************
 * @fn      TIM0_Init
 *
 * @brief   Initialize TIM0
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
    printf("PD SNK TEST\r\n");
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
 * @fn      TIM0_IRQHandler
 *
 * @brief   This function handles TIM0 interrupt.
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
