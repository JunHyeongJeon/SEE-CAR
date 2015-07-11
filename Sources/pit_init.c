/*
 *######################################################################
 *                                RAppIDJDP
 *           Rapid Application Initialization and Documentation Tool
 *                         Freescale Semiconductor Inc.
 *
 *######################################################################
 *
 * Project Name           : FSK_LED_Example
 *
 * Project File           : FSK_LED_Example.rsp
 *
 * Revision Number        : 1.0
 *
 * Tool Version           : 1.2.1.5
 *
 * file                   : pit_init.c
 *
 * Target Compiler        : Codewarrior
 *
 * Target Part            : MPC5606B
 *
 * Part Errata Fixes      : none
 *
 * Project Last Save Date : 28-Oct-2013 16:04:30
 *
 * Created on Date        : 28-Oct-2013 16:04:31
 *
 * Brief Description      : PIT Initialization file
 *
 ******************************************************************************** 
 *
 * Detail Description     : File contains Periodic Interrupt Timer (PIT) parameter
 *                         initialization function. Along with PIT module, the
 *                         function configures parameters such as individual
 *                         channel status, interrupt status & timer load value.
 *
 ******************************************************************************** 
 *
 *######################################################################
*/

 
 
/********************  Dependent Include files here **********************/

#include "pit_init.h"
#include "common.h"



/********************* Initialization Function(s) *********************** */


void pit_init_fnc(void)
{

    PIT.PITMCR.B.MDIS   = 1;    
        /*Disable PIT for initialization         */

/* ----------------------------------------------------------- */
/*                     Configure Load Value Registers                */
/* ----------------------------------------------------------- */

    PIT.CH[0].LDVAL.R  = PIT_CHANNEL_0_TIMER;    
        /*value loaded in the Timer0: 64000    */
    
    PIT.CH[1].LDVAL.R  = PIT_CHANNEL_1_TIMER;    
        /*value loaded in the Timer1: 0    */
    
    PIT.CH[2].LDVAL.R  = PIT_CHANNEL_2_TIMER;    
        /*value loaded in the Timer2: 0    */
    
    PIT.CH[3].LDVAL.R  = PIT_CHANNEL_3_TIMER;    
        /*value loaded in the Timer3: 0    */
    
    PIT.CH[4].LDVAL.R  = PIT_CHANNEL_4_TIMER;    
        /*value loaded in the Timer4: 0    */
    
    PIT.CH[5].LDVAL.R  = PIT_CHANNEL_5_TIMER;    
        /*value loaded in the Timer5: 0    */
    
    PIT.CH[6].LDVAL.R  = PIT_CHANNEL_6_TIMER;    
        /*value loaded in the Timer6: 0    */
    
    PIT.CH[7].LDVAL.R  = PIT_CHANNEL_7_TIMER;    
        /*value loaded in the Timer7: 0    */
    


/* ----------------------------------------------------------- */
/*                     Enable Interrupts                  */
/* ----------------------------------------------------------- */

    PIT.CH[0].TCTRL.B.TIE  = PIT_ENABLED(PIT_CHANNEL_0_TIMER);    
        /*Timer 0 Interrupt : Enabled    */
    
    PIT.CH[1].TCTRL.B.TIE  = PIT_ENABLED(PIT_CHANNEL_1_TIMER);    
        /*Timer 1 Interrupt : Disabled    */
    
    PIT.CH[2].TCTRL.B.TIE  = PIT_ENABLED(PIT_CHANNEL_2_TIMER);    
        /*Timer 2 Interrupt : Disabled    */
    
    PIT.CH[3].TCTRL.B.TIE  = PIT_ENABLED(PIT_CHANNEL_3_TIMER);    
        /*Timer 3 Interrupt : Disabled    */
    
    PIT.CH[4].TCTRL.B.TIE  = PIT_ENABLED(PIT_CHANNEL_4_TIMER);    
        /*Timer 4 Interrupt : Disabled    */
    
    PIT.CH[5].TCTRL.B.TIE  = PIT_ENABLED(PIT_CHANNEL_5_TIMER);    
        /*Timer 5 Interrupt : Disabled    */
    
    PIT.CH[6].TCTRL.B.TIE  = PIT_ENABLED(PIT_CHANNEL_6_TIMER);    
        /*Timer 6 Interrupt : Disabled    */
    
    PIT.CH[7].TCTRL.B.TIE  = PIT_ENABLED(PIT_CHANNEL_7_TIMER);    
        /*Timer 7 Interrupt : Disabled    */
    


/* ----------------------------------------------------------- */
/*                   Start Timers                 */
/* ----------------------------------------------------------- */

    
    PIT.CH[0].TCTRL.B.TEN = 0x00; //PIT_ENABLED(PIT_CHANNEL_0_TIMER);
        /*Start Timer 0 is : Enabled    */
    
    PIT.CH[1].TCTRL.B.TEN = 0x00; //PIT_ENABLED(PIT_CHANNEL_1_TIMER);    
        /*Start Timer 1 is : Enabled    */
    
    PIT.CH[2].TCTRL.B.TEN = 0x00; //PIT_ENABLED(PIT_CHANNEL_2_TIMER);    
        /*Start Timer 2 is : Disabled    */
    
    PIT.CH[3].TCTRL.B.TEN = 0x00; //PIT_ENABLED(PIT_CHANNEL_3_TIMER);    
        /*Start Timer 3 is : Disabled    */
    
    PIT.CH[4].TCTRL.B.TEN = 0x00; //PIT_ENABLED(PIT_CHANNEL_4_TIMER);    
        /*Start Timer 4 is : Disabled    */
    
    PIT.CH[5].TCTRL.B.TEN = 0x00; //PIT_ENABLED(PIT_CHANNEL_5_TIMER);    
        /*Start Timer 5 is : Disabled    */
    
    PIT.CH[6].TCTRL.B.TEN = 0x00; //PIT_ENABLED(PIT_CHANNEL_6_TIMER);    
        /*Start Timer 6 is : Disabled    */
    
    PIT.CH[7].TCTRL.B.TEN = 0x00; //PIT_ENABLED(PIT_CHANNEL_7_TIMER);    
        /*Start Timer 7 is : Disabled    */
    

    PIT.PITMCR.B.MDIS = 0;    
        /*PIT Module : Enabled        */

}

 
/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

