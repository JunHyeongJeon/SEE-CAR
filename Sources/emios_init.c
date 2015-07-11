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
 * file                   : emios_init.c
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
 * Brief Description      : This file initializes eMIOS registers
 *
 ******************************************************************************** 
 *
 * Detail Description     : This File contains function definitions that initialize eMIOS
 *                         peripherals. The initialization function configures the Module
 *                         Configuration Register and Unified Channel Registers for all
 *                         eMIOS modules present in the selected part. The module
 *                         provides upto 56 Unified Channels to generate or measure
 *                         time events.
 *
 ******************************************************************************** 
 *
 *######################################################################
*/

 
 
/********************  Dependent Include files here **********************/

#include "emios_init.h"
#include "common.h"


/**********************  Initialization Function(s) *************************/


void emios_init_fnc (void) 
{
    emios_0_init_fnc();
    emios_1_init_fnc();
}
void emios_0_init_fnc (void) 
{
/* ----------------------------------------------------------- */
/*	EMIOS Module Configuration Register (EMIOS_MCR)            */
/* ----------------------------------------------------------- */
   EMIOS_0.MCR.R = 0x14003F00;
            /* Module : Enable    */
            /* Global Freeze: Disabled                  */
            /* Global Time Base Enable: Yes             */
            /* Global Prescaler Enable: Yes             */
            /* Global Prescaler Value:   64              */

/* ----------------------------------------------------------- */
/* EMIOS Output Update Disable Register (EMIOS_OUDR)           */
/* ----------------------------------------------------------- */
  EMIOS_0.OUDR.R = 0x00000000;
            /* Channel n output is Enabled  */
            /*   n = 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 */

/* ----------------------------------------------------------- */
/* EMIOS Channel Disable Register  (EMIOS_UCDIS)           */
/* ----------------------------------------------------------- */
  EMIOS_0.UCDIS.R = 0x00000000;
            /* Channel n is Enabled  */
            /*   n = 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31, */

/* ----------------------------------------------------------- */
/*	Unified Channel 00 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[0].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR0 Data Register - eMIOS_0_CADR0 : Value = 0*/

  EMIOS_0.CH[0].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR0 Data Register - eMIOS_0_CBDR0 : Value = 0*/

  EMIOS_0.CH[0].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 01 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[1].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR1 Data Register - eMIOS_0_CADR1 : Value = 0*/

  EMIOS_0.CH[1].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR1 Data Register - eMIOS_0_CBDR1 : Value = 0*/

  EMIOS_0.CH[1].CCR.R = 0x00000000;

  
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 02 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[2].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR2 Data Register - eMIOS_0_CADR2 : Value = 0*/

  EMIOS_0.CH[2].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR2 Data Register - eMIOS_0_CBDR2 : Value = 0*/

  EMIOS_0.CH[2].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 03 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[3].CADR.R = 0x00002710;
            /*  [16-31]  Channel eMIOS_0_CADR3 Data Register - eMIOS_0_CADR3 : Value = 0*/

  EMIOS_0.CH[3].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR3 Data Register - eMIOS_0_CBDR3 : Value = 0*/

  EMIOS_0.CH[3].CCR.R = 0x00000211;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 04 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[4].CADR.R = 0x00002710;
            /*  [16-31]  Channel eMIOS_0_CADR4 Data Register - eMIOS_0_CADR4 : Value = 0*/

  EMIOS_0.CH[4].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR4 Data Register - eMIOS_0_CBDR4 : Value = 0*/

  EMIOS_0.CH[4].CCR.R = 0x00000211;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 05 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[5].CADR.R = 510;
  //350~650 for servo motor
            /*  [16-31]  Channel eMIOS_0_CADR5 Data Register - eMIOS_0_CADR5 : Value = 0*/

  EMIOS_0.CH[5].CBDR.R = 0x00002710;
            /*  [16-31]  Channel eMIOS_0_CBDR5 Data Register - eMIOS_0_CBDR5 : Value = 0*/

  EMIOS_0.CH[5].CCR.R = 0x02000658;
  EMIOS_0.CH[5].CCR.B.UCPRE = 0b01;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 06 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[6].CADR.R = 0x0000000A;
            /*  [16-31]  Channel eMIOS_0_CADR6 Data Register - eMIOS_0_CADR6 : Value = 0*/

  EMIOS_0.CH[6].CBDR.R = 0x000003E8;
            /*  [16-31]  Channel eMIOS_0_CBDR6 Data Register - eMIOS_0_CBDR6 : Value = 0*/

  EMIOS_0.CH[6].CCR.R = 0x02000658;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 07 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[7].CADR.R = 0x0000000A;
            /*  [16-31]  Channel eMIOS_0_CADR7 Data Register - eMIOS_0_CADR7 : Value = 0*/

  EMIOS_0.CH[7].CBDR.R = 0x000003E8;
            /*  [16-31]  Channel eMIOS_0_CBDR7 Data Register - eMIOS_0_CBDR7 : Value = 0*/

  EMIOS_0.CH[7].CCR.R = 0x02000658;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 08 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[8].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR8 Data Register - eMIOS_0_CADR8 : Value = 0*/

  EMIOS_0.CH[8].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR8 Data Register - eMIOS_0_CBDR8 : Value = 0*/

  EMIOS_0.CH[8].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 09 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[9].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR9 Data Register - eMIOS_0_CADR9 : Value = 0*/

  EMIOS_0.CH[9].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR9 Data Register - eMIOS_0_CBDR9 : Value = 0*/

  EMIOS_0.CH[9].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 10 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[10].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR10 Data Register - eMIOS_0_CADR10 : Value = 0*/

  EMIOS_0.CH[10].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR10 Data Register - eMIOS_0_CBDR10 : Value = 0*/

  EMIOS_0.CH[10].CCR.R = 0x00000000;
  
  /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 11 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[11].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR11 Data Register - eMIOS_0_CADR11 : Value = 0*/

  EMIOS_0.CH[11].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR11 Data Register - eMIOS_0_CBDR11 : Value = 0*/

  EMIOS_0.CH[11].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 12 Registers                               */
/* ----------------------------------------------------------- */
  
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 13 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[13].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR13 Data Register - eMIOS_0_CADR13 : Value = 0*/

  EMIOS_0.CH[13].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR13 Data Register - eMIOS_0_CBDR13 : Value = 0*/

  EMIOS_0.CH[13].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 14 Registers                               */
/* ----------------------------------------------------------- */

  EMIOS_0.CH[14].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR14 Data Register - eMIOS_0_CADR14 : Value = 0*/

  EMIOS_0.CH[14].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR14 Data Register - eMIOS_0_CBDR14 : Value = 0*/

  EMIOS_0.CH[14].CCR.R = 0x00000000;
//            /* Channel Control Register                 */
//            /* Freeze Enable : Normal operation         */
//            /* Prescaler Divide Ratio : 1             */
//            /* Prescaler Enable : Disable               */
//            /* Flag generation Request: Assigned to Interrupt */
//            /* Input Filter: Bypassed                   */
//            /* Filter clock select : Prescaler clock    */
//            /* Flag Enable : IRQ/DMA not generated      */
//            /* Force Match A : Disabled                 */
//            /* Force Match B : Disabled                 */
//            /* Bus Select : Counter bus[A]                */
//            /* Edge Selection Bit :FLAG                 */
//            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 15 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[15].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR15 Data Register - eMIOS_0_CADR15 : Value = 0*/

  EMIOS_0.CH[15].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR15 Data Register - eMIOS_0_CBDR15 : Value = 0*/

  EMIOS_0.CH[15].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 16 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[16].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR16 Data Register - eMIOS_0_CADR16 : Value = 0*/

  EMIOS_0.CH[16].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR16 Data Register - eMIOS_0_CBDR16 : Value = 0*/

  EMIOS_0.CH[16].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 17 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[17].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR17 Data Register - eMIOS_0_CADR17 : Value = 0*/

  EMIOS_0.CH[17].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR17 Data Register - eMIOS_0_CBDR17 : Value = 0*/

  EMIOS_0.CH[17].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 18 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[18].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR18 Data Register - eMIOS_0_CADR18 : Value = 0*/

  EMIOS_0.CH[18].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR18 Data Register - eMIOS_0_CBDR18 : Value = 0*/

  EMIOS_0.CH[18].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 19 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[19].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR19 Data Register - eMIOS_0_CADR19 : Value = 0*/

  EMIOS_0.CH[19].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR19 Data Register - eMIOS_0_CBDR19 : Value = 0*/

  EMIOS_0.CH[19].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 20 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[20].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR20 Data Register - eMIOS_0_CADR20 : Value = 0*/

  EMIOS_0.CH[20].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR20 Data Register - eMIOS_0_CBDR20 : Value = 0*/

  EMIOS_0.CH[20].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 21 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[21].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR21 Data Register - eMIOS_0_CADR21 : Value = 0*/

  EMIOS_0.CH[21].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR21 Data Register - eMIOS_0_CBDR21 : Value = 0*/

  EMIOS_0.CH[21].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 22 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[22].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR22 Data Register - eMIOS_0_CADR22 : Value = 0*/

  EMIOS_0.CH[22].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR22 Data Register - eMIOS_0_CBDR22 : Value = 0*/

  EMIOS_0.CH[22].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 23 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[23].CADR.R = 0x00007D00;
            /*  [16-31]  Channel eMIOS_0_CADR23 Data Register - eMIOS_0_CADR23 : Value = 32000*/

  EMIOS_0.CH[23].CBDR.R = 0x0000FA00;
            /*  [16-31]  Channel eMIOS_0_CBDR23 Data Register - eMIOS_0_CBDR23 : Value = 64000*/

  EMIOS_0.CH[23].CCR.R = 0x020006D8;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Enable                */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Not Applicable                */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Internal counter                */
            /* Edge Selection Bit :Not Applicable                 */
            /* Edge Polarity :Set on Match A                   */
            /* Mode : Output Pulse Width and Frequency M*/
            /* odulation Buffered                       */

/* ----------------------------------------------------------- */
/*	Unified Channel 24 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[24].CADR.R = EMIOS_0_CLOCK_MAX;
            /*  [16-31]  Channel eMIOS_0_CADR24 Data Register - eMIOS_0_CADR24 : Value = 0*/

  EMIOS_0.CH[24].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR24 Data Register - eMIOS_0_CBDR24 : Value = 0*/

 /* Use one of the following two lines for mode (Note: some MPC555x devices lack MCB) */   
   EMIOS_0.CH[24].CCR.R = 0x0;
//   EMIOS.CH[24].CCR.B.MODE = 0x50; /* MPC551x, MPC563x: Mod Ctr Bufd (MCB) int clk */   
   EMIOS_0.CH[24].CCR.B.MODE = 0x10; /* MPC555x: Modulus Counter (MC), internal clock */   
   EMIOS_0.CH[24].CCR.B.FCK = 0x1;
   EMIOS_0.CH[24].CCR.B.BSL = 0b11;   /* Use internal counter */
   EMIOS_0.CH[24].CCR.B.UCPEN = 0x1;  /* Enable prescaler; uses default divide by 1 */   
   EMIOS_0.CH[24].CCR.B.EDPOL = 0x1;
   EMIOS_0.CH[24].CCR.B.EDSEL = 0x0;
      
  
  // EMIOS_0.CH[24].CCR.R = 0b10100000000000000110000001000010;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 25 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[25].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR25 Data Register - eMIOS_0_CADR25 : Value = 0*/

  EMIOS_0.CH[25].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR25 Data Register - eMIOS_0_CBDR25 : Value = 0*/

  EMIOS_0.CH[25].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 26 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[26].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR26 Data Register - eMIOS_0_CADR26 : Value = 0*/

  EMIOS_0.CH[26].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR26 Data Register - eMIOS_0_CBDR26 : Value = 0*/

  EMIOS_0.CH[26].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 27 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[27].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR27 Data Register - eMIOS_0_CADR27 : Value = 0*/

  EMIOS_0.CH[27].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR27 Data Register - eMIOS_0_CBDR27 : Value = 0*/

  EMIOS_0.CH[27].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 28 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[28].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR28 Data Register - eMIOS_0_CADR28 : Value = 0*/

  EMIOS_0.CH[28].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR28 Data Register - eMIOS_0_CBDR28 : Value = 0*/

  EMIOS_0.CH[28].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 29 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[29].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR29 Data Register - eMIOS_0_CADR29 : Value = 0*/

  EMIOS_0.CH[29].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR29 Data Register - eMIOS_0_CBDR29 : Value = 0*/

  EMIOS_0.CH[29].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 30 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[30].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR30 Data Register - eMIOS_0_CADR30 : Value = 0*/

  EMIOS_0.CH[30].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR30 Data Register - eMIOS_0_CBDR30 : Value = 0*/

  EMIOS_0.CH[30].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 31 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_0.CH[31].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CADR31 Data Register - eMIOS_0_CADR31 : Value = 0*/

  EMIOS_0.CH[31].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_0_CBDR31 Data Register - eMIOS_0_CBDR31 : Value = 0*/

  EMIOS_0.CH[31].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

  
  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CADR.R = 0x0000;
				/*  [16-31]  Channel eMIOS_0_CADR12 Data Register - eMIOS_0_CADR12 : Value = 0*/
	
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CBDR.R = 0x1000;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.R = 0;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.B.BSL = 0b11;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.B.MODE = 0b000100;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.B.EDPOL = 1;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.B.FEN = 1;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.B.FCK = 0;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.B.ODIS = 1;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.B.UCPEN = 1;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CCR.B.UCPRE = 0b11;
	  
	  EMIOS_0.CH[EMIOS_0_SONA_SENSOR].CSR.B.UCIN = 1;
  
  
/* ----------------------------------------------------------- */
/* EMIOS Channel Disable Register  (EMIOS_UCDIS)           */
/* ----------------------------------------------------------- */
  EMIOS_0.UCDIS.R = 0x00000000;
            /* Channel n is Enabled  */
            /*   n = 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 */
}


void emios_1_init_fnc (void) 
{
/* ----------------------------------------------------------- */
/*	EMIOS Module Configuration Register (EMIOS_MCR)            */
/* ----------------------------------------------------------- */
   EMIOS_1.MCR.R = 0x00000000;
            /* Module : Enable    */
            /* Global Freeze: Disabled                  */
            /* Global Time Base Enable: No              */
            /* Global Prescaler Enable: No              */
            /* Global Prescaler Value:   1              */

/* ----------------------------------------------------------- */
/* EMIOS Output Update Disable Register (EMIOS_OUDR)           */
/* ----------------------------------------------------------- */
  EMIOS_1.OUDR.R = 0x00000000;
            /* Channel n output is Enabled  */
            /*   n = 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 */

/* ----------------------------------------------------------- */
/* EMIOS Channel Disable Register  (EMIOS_UCDIS)           */
/* ----------------------------------------------------------- */
  EMIOS_1.UCDIS.R = 0x00000000;
            /* Channel n is Enabled  */
            /*   n = 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31, */

/* ----------------------------------------------------------- */
/*	Unified Channel 00 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[0].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR0 Data Register - eMIOS_1_CADR0 : Value = 0*/

  EMIOS_1.CH[0].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR0 Data Register - eMIOS_1_CBDR0 : Value = 0*/

  EMIOS_1.CH[0].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 01 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[1].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR1 Data Register - eMIOS_1_CADR1 : Value = 0*/

  EMIOS_1.CH[1].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR1 Data Register - eMIOS_1_CBDR1 : Value = 0*/

  EMIOS_1.CH[1].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 02 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[2].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR2 Data Register - eMIOS_1_CADR2 : Value = 0*/

  EMIOS_1.CH[2].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR2 Data Register - eMIOS_1_CBDR2 : Value = 0*/

  EMIOS_1.CH[2].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 03 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[3].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR3 Data Register - eMIOS_1_CADR3 : Value = 0*/

  EMIOS_1.CH[3].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR3 Data Register - eMIOS_1_CBDR3 : Value = 0*/

  EMIOS_1.CH[3].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 04 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[4].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR4 Data Register - eMIOS_1_CADR4 : Value = 0*/

  EMIOS_1.CH[4].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR4 Data Register - eMIOS_1_CBDR4 : Value = 0*/

  EMIOS_1.CH[4].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 05 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[5].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR5 Data Register - eMIOS_1_CADR5 : Value = 0*/

  EMIOS_1.CH[5].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR5 Data Register - eMIOS_1_CBDR5 : Value = 0*/

  EMIOS_1.CH[5].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 06 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[6].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR6 Data Register - eMIOS_1_CADR6 : Value = 0*/

  EMIOS_1.CH[6].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR6 Data Register - eMIOS_1_CBDR6 : Value = 0*/

  EMIOS_1.CH[6].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 07 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[7].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR7 Data Register - eMIOS_1_CADR7 : Value = 0*/

  EMIOS_1.CH[7].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR7 Data Register - eMIOS_1_CBDR7 : Value = 0*/

  EMIOS_1.CH[7].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 08 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[8].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR8 Data Register - eMIOS_1_CADR8 : Value = 0*/

  EMIOS_1.CH[8].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR8 Data Register - eMIOS_1_CBDR8 : Value = 0*/

  EMIOS_1.CH[8].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 09 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[9].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR9 Data Register - eMIOS_1_CADR9 : Value = 0*/

  EMIOS_1.CH[9].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR9 Data Register - eMIOS_1_CBDR9 : Value = 0*/

  EMIOS_1.CH[9].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 10 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[10].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR10 Data Register - eMIOS_1_CADR10 : Value = 0*/

  EMIOS_1.CH[10].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR10 Data Register - eMIOS_1_CBDR10 : Value = 0*/

  EMIOS_1.CH[10].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 11 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[11].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR11 Data Register - eMIOS_1_CADR11 : Value = 0*/

  EMIOS_1.CH[11].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR11 Data Register - eMIOS_1_CBDR11 : Value = 0*/

  EMIOS_1.CH[11].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 12 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[12].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR12 Data Register - eMIOS_1_CADR12 : Value = 0*/

  EMIOS_1.CH[12].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR12 Data Register - eMIOS_1_CBDR12 : Value = 0*/

  EMIOS_1.CH[12].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 13 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[13].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR13 Data Register - eMIOS_1_CADR13 : Value = 0*/

  EMIOS_1.CH[13].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR13 Data Register - eMIOS_1_CBDR13 : Value = 0*/

  EMIOS_1.CH[13].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 14 Registers                               */
/* ----------------------------------------------------------- */
    
  EMIOS_1.CH[14].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR14 Data Register - eMIOS_1_CADR14 : Value = 0*/

  EMIOS_1.CH[14].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR14 Data Register - eMIOS_1_CBDR14 : Value = 0*/

  EMIOS_1.CH[14].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 15 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[15].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR15 Data Register - eMIOS_1_CADR15 : Value = 0*/

  EMIOS_1.CH[15].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR15 Data Register - eMIOS_1_CBDR15 : Value = 0*/

  EMIOS_1.CH[15].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 16 Registers                               */
/* ----------------------------------------------------------- */
  	  EMIOS_1.CH[16].CADR.R = EMIOS_1_RIGHT_ENCODER_MAX;
                /*  [16-31]  Channel eMIOS_0_CADR24 Data Register - eMIOS_0_CADR24 : Value = 0*/

      EMIOS_1.CH[16].CBDR.R = 0x00000000;
                /*  [16-31]  Channel eMIOS_0_CBDR24 Data Register - eMIOS_0_CBDR24 : Value = 0*/

     /* Use one of the following two lines for mode (Note: some MPC555x devices lack MCB) */   
       EMIOS_1.CH[16].CCR.R = 0x0;
    //   EMIOS.CH[24].CCR.B.MODE = 0x50; /* MPC551x, MPC563x: Mod Ctr Bufd (MCB) int clk */   
       EMIOS_1.CH[16].CCR.B.MODE = 0x11; /* MPC555x: Modulus Counter (MC), internal clock */   
       EMIOS_1.CH[16].CCR.B.FCK = 0x1;
       EMIOS_1.CH[16].CCR.B.BSL = 0b11;   /* Use internal counter */   
     //  EMIOS_0.CH[24].CCR.B.FREN = 1;    /* Freeze channel counting when in debug mode */   
       EMIOS_1.CH[16].CCR.B.UCPEN = 0x0;  /* Dis prescaler; uses default divide by 1 */   
       EMIOS_1.CH[16].CCR.B.EDPOL = 0x1;
       EMIOS_1.CH[16].CCR.B.EDSEL = 0x0;
          
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 17 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[17].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR17 Data Register - eMIOS_1_CADR17 : Value = 0*/

  EMIOS_1.CH[17].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR17 Data Register - eMIOS_1_CBDR17 : Value = 0*/

  EMIOS_1.CH[17].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 18 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[18].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR18 Data Register - eMIOS_1_CADR18 : Value = 0*/

  EMIOS_1.CH[18].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR18 Data Register - eMIOS_1_CBDR18 : Value = 0*/

  EMIOS_1.CH[18].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 19 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[19].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR19 Data Register - eMIOS_1_CADR19 : Value = 0*/

  EMIOS_1.CH[19].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR19 Data Register - eMIOS_1_CBDR19 : Value = 0*/

  EMIOS_1.CH[19].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 20 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[20].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR20 Data Register - eMIOS_1_CADR20 : Value = 0*/

  EMIOS_1.CH[20].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR20 Data Register - eMIOS_1_CBDR20 : Value = 0*/

  EMIOS_1.CH[20].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 21 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[21].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR21 Data Register - eMIOS_1_CADR21 : Value = 0*/

  EMIOS_1.CH[21].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR21 Data Register - eMIOS_1_CBDR21 : Value = 0*/

  EMIOS_1.CH[21].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 22 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[22].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR22 Data Register - eMIOS_1_CADR22 : Value = 0*/

  EMIOS_1.CH[22].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR22 Data Register - eMIOS_1_CBDR22 : Value = 0*/

  EMIOS_1.CH[22].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 23 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[23].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR23 Data Register - eMIOS_1_CADR23 : Value = 0*/

  EMIOS_1.CH[23].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR23 Data Register - eMIOS_1_CBDR23 : Value = 0*/

  EMIOS_1.CH[23].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 24 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[24].CADR.R = EMIOS_1_LEFT_ENCODER_MAX;
              /*  [16-31]  Channel eMIOS_0_CADR24 Data Register - eMIOS_0_CADR24 : Value = 0*/

    EMIOS_1.CH[24].CBDR.R = 0x00000000;
              /*  [16-31]  Channel eMIOS_0_CBDR24 Data Register - eMIOS_0_CBDR24 : Value = 0*/

   /* Use one of the following two lines for mode (Note: some MPC555x devices lack MCB) */   
     EMIOS_1.CH[24].CCR.R = 0x0;
  //   EMIOS.CH[24].CCR.B.MODE = 0x50; /* MPC551x, MPC563x: Mod Ctr Bufd (MCB) int clk */   
     EMIOS_1.CH[24].CCR.B.MODE = 0x11; /* MPC555x: Modulus Counter (MC), internal clock */   
     EMIOS_1.CH[24].CCR.B.FCK = 0x1;
     EMIOS_1.CH[24].CCR.B.BSL = 0b11;   /* Use internal counter */   
   //  EMIOS_0.CH[24].CCR.B.FREN = 1;    /* Freeze channel counting when in debug mode */   
     EMIOS_1.CH[24].CCR.B.UCPEN = 0x0;  /* Dis prescaler; uses default divide by 1 */   
     EMIOS_1.CH[24].CCR.B.EDPOL = 0x1;
     EMIOS_1.CH[24].CCR.B.EDSEL = 0x0;
        
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 25 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[25].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR25 Data Register - eMIOS_1_CADR25 : Value = 0*/

  EMIOS_1.CH[25].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR25 Data Register - eMIOS_1_CBDR25 : Value = 0*/

  EMIOS_1.CH[25].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 26 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[26].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR26 Data Register - eMIOS_1_CADR26 : Value = 0*/

  EMIOS_1.CH[26].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR26 Data Register - eMIOS_1_CBDR26 : Value = 0*/

  EMIOS_1.CH[26].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 27 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[27].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR27 Data Register - eMIOS_1_CADR27 : Value = 0*/

  EMIOS_1.CH[27].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR27 Data Register - eMIOS_1_CBDR27 : Value = 0*/

  EMIOS_1.CH[27].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 28 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[28].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR28 Data Register - eMIOS_1_CADR28 : Value = 0*/

  EMIOS_1.CH[28].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR28 Data Register - eMIOS_1_CBDR28 : Value = 0*/

  EMIOS_1.CH[28].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 29 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[29].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR29 Data Register - eMIOS_1_CADR29 : Value = 0*/

  EMIOS_1.CH[29].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR29 Data Register - eMIOS_1_CBDR29 : Value = 0*/

  EMIOS_1.CH[29].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 30 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[30].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR30 Data Register - eMIOS_1_CADR30 : Value = 0*/

  EMIOS_1.CH[30].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR30 Data Register - eMIOS_1_CBDR30 : Value = 0*/

  EMIOS_1.CH[30].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/*	Unified Channel 31 Registers                               */
/* ----------------------------------------------------------- */
  EMIOS_1.CH[31].CADR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CADR31 Data Register - eMIOS_1_CADR31 : Value = 0*/

  EMIOS_1.CH[31].CBDR.R = 0x00000000;
            /*  [16-31]  Channel eMIOS_1_CBDR31 Data Register - eMIOS_1_CBDR31 : Value = 0*/

  EMIOS_1.CH[31].CCR.R = 0x00000000;
            /* Channel Control Register                 */
            /* Freeze Enable : Normal operation         */
            /* Prescaler Divide Ratio : 1             */
            /* Prescaler Enable : Disable               */
            /* Flag generation Request: Assigned to Interrupt */
            /* Input Filter: Bypassed                   */
            /* Filter clock select : Prescaler clock    */
            /* Flag Enable : IRQ/DMA not generated      */
            /* Force Match A : Disabled                 */
            /* Force Match B : Disabled                 */
            /* Bus Select : Counter bus[A]                */
            /* Edge Selection Bit :FLAG                 */
            /* Edge Polarity :Falling                   */
            /* Mode : Default Mode                      */

/* ----------------------------------------------------------- */
/* EMIOS Channel Disable Register  (EMIOS_UCDIS)           */
/* ----------------------------------------------------------- */
  EMIOS_1.UCDIS.R = 0x00000000;
            /* Channel n is Enabled  */
            /*   n = 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 */
}



 
/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

