/* ============================================================================ */
/* Copyright (c) 2012, Texas Instruments Incorporated                           */
/*  All rights reserved.                                                        */
/*                                                                              */
/*  Redistribution and use in source and binary forms, with or without          */
/*  modification, are permitted provided that the following conditions          */
/*  are met:                                                                    */
/*                                                                              */
/*  *  Redistributions of source code must retain the above copyright           */
/*     notice, this list of conditions and the following disclaimer.            */
/*                                                                              */
/*  *  Redistributions in binary form must reproduce the above copyright        */
/*     notice, this list of conditions and the following disclaimer in the      */
/*     documentation and/or other materials provided with the distribution.     */
/*                                                                              */
/*  *  Neither the name of Texas Instruments Incorporated nor the names of      */
/*     its contributors may be used to endorse or promote products derived      */
/*     from this software without specific prior written permission.            */
/*                                                                              */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" */
/*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,       */
/*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR      */
/*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR            */
/*  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       */
/*  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         */
/*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; */
/*  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    */
/*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     */
/*  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,              */
/*  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                          */
/* ============================================================================ */

/******************************************************************************/
/* Legacy Header File                                                         */
/* Not recommended for use in new projects.                                   */
/* Please use the msp430.h file or the device specific header file            */
/******************************************************************************/

/********************************************************************
*
* Standard register and bit definitions for the Texas Instruments
* MSP430 microcontroller.
*
* This file supports assembler and C development for
* MSP430xG46x devices.
*
* Texas Instruments, Version 1.7
*
* Rev. 1.0, Setup
* Rev. 1.1, Fixed Names for USCI an settings for DMA
* Rev. 1.2, Added VLD bits in SVS module
* Rev. 1.3, Removed definitions for BTRESET
*           Fixed swapped definition of RTCxxx
* Rev. 1.4, Removed unused ME1 definition
* Rev. 1.5, added definitions for Interrupt Vectors xxIV
* Rev. 1.6, changed 'void __data20 * volatile' definition
* Rev. 1.6, added LFXT1DIG
*
*
********************************************************************/

#ifndef __msp430xG46x
#define __msp430xG46x

#define __MSP430_HEADER_VERSION__ 1064

#define __MSP430_TI_HEADERS__

#ifdef __cplusplus
extern "C" {
#endif

#include <iomacros.h>

/************************************************************
* STANDARD BITS
************************************************************/

#define BIT0                (0x0001)
#define BIT1                (0x0002)
#define BIT2                (0x0004)
#define BIT3                (0x0008)
#define BIT4                (0x0010)
#define BIT5                (0x0020)
#define BIT6                (0x0040)
#define BIT7                (0x0080)
#define BIT8                (0x0100)
#define BIT9                (0x0200)
#define BITA                (0x0400)
#define BITB                (0x0800)
#define BITC                (0x1000)
#define BITD                (0x2000)
#define BITE                (0x4000)
#define BITF                (0x8000)

/************************************************************
* STATUS REGISTER BITS
************************************************************/

#define C                   (0x0001)
#define Z                   (0x0002)
#define N                   (0x0004)
#define V                   (0x0100)
#define GIE                 (0x0008)
#define CPUOFF              (0x0010)
#define OSCOFF              (0x0020)
#define SCG0                (0x0040)
#define SCG1                (0x0080)

/* Low Power Modes coded with Bits 4-7 in SR */

#ifndef __STDC__ /* Begin #defines for assembler */
#define LPM0                (CPUOFF)
#define LPM1                (SCG0+CPUOFF)
#define LPM2                (SCG1+CPUOFF)
#define LPM3                (SCG1+SCG0+CPUOFF)
#define LPM4                (SCG1+SCG0+OSCOFF+CPUOFF)
/* End #defines for assembler */

#else /* Begin #defines for C */
#define LPM0_bits           (CPUOFF)
#define LPM1_bits           (SCG0+CPUOFF)
#define LPM2_bits           (SCG1+CPUOFF)
#define LPM3_bits           (SCG1+SCG0+CPUOFF)
#define LPM4_bits           (SCG1+SCG0+OSCOFF+CPUOFF)

#include "in430.h"

#define LPM0      _BIS_SR(LPM0_bits)     /* Enter Low Power Mode 0 */
#define LPM0_EXIT _BIC_SR_IRQ(LPM0_bits) /* Exit Low Power Mode 0 */
#define LPM1      _BIS_SR(LPM1_bits)     /* Enter Low Power Mode 1 */
#define LPM1_EXIT _BIC_SR_IRQ(LPM1_bits) /* Exit Low Power Mode 1 */
#define LPM2      _BIS_SR(LPM2_bits)     /* Enter Low Power Mode 2 */
#define LPM2_EXIT _BIC_SR_IRQ(LPM2_bits) /* Exit Low Power Mode 2 */
#define LPM3      _BIS_SR(LPM3_bits)     /* Enter Low Power Mode 3 */
#define LPM3_EXIT _BIC_SR_IRQ(LPM3_bits) /* Exit Low Power Mode 3 */
#define LPM4      _BIS_SR(LPM4_bits)     /* Enter Low Power Mode 4 */
#define LPM4_EXIT _BIC_SR_IRQ(LPM4_bits) /* Exit Low Power Mode 4 */
#endif /* End #defines for C */

/************************************************************
* CPU
************************************************************/
#define __MSP430_HAS_MSP430X_CPU__    /* Definition to show that it has MSP430X CPU */

/************************************************************
* PERIPHERAL FILE MAP
************************************************************/

/************************************************************
* SPECIAL FUNCTION REGISTER ADDRESSES + CONTROL BITS
************************************************************/

#define IE1_                  0x0000    /* Interrupt Enable 1 */
sfrb(IE1, IE1_);
#define WDTIE               (0x01)
#define OFIE                (0x02)
#define NMIIE               (0x10)
#define ACCVIE              (0x20)

#define IFG1_                 0x0002    /* Interrupt Flag 1 */
sfrb(IFG1, IFG1_);
#define WDTIFG              (0x01)
#define OFIFG               (0x02)
#define NMIIFG              (0x10)

#define IE2_                  0x0001    /* Interrupt Enable 2 */
sfrb(IE2, IE2_);
#define U1IE                IE2       /* UART1 Interrupt Enable Register */
#define UC0IE               IE2
#define UCA0RXIE            (0x01)
#define UCA0TXIE            (0x02)
#define UCB0RXIE            (0x04)
#define UCB0TXIE            (0x08)
#define URXIE1              (0x10)
#define UTXIE1              (0x20)
#define BTIE                (0x80)

#define IFG2_                 0x0003    /* Interrupt Flag 2 */
sfrb(IFG2, IFG2_);
#define U1IFG               IFG2     /* UART1 Interrupt Flag Register */
#define UC0IFG              IFG2
#define UCA0RXIFG           (0x01)
#define UCA0TXIFG           (0x02)
#define UCB0RXIFG           (0x04)
#define UCB0TXIFG           (0x08)
#define URXIFG1             (0x10)
#define UTXIFG1             (0x20)
#define BTIFG               (0x80)

#define ME2_                  0x0005    /* Module Enable 2 */
sfrb(ME2, ME2_);
#define U1ME                ME2       /* UART1 Module Enable Register */
#define URXE1               (0x10)
#define UTXE1               (0x20)
#define USPIE1              (0x10)

/************************************************************
* ADC12
************************************************************/
#define __MSP430_HAS_ADC12__          /* Definition to show that Module is available */

#define ADC12CTL0_            0x01A0    /* ADC12 Control 0 */
sfrw(ADC12CTL0, ADC12CTL0_);
#define ADC12CTL1_            0x01A2    /* ADC12 Control 1 */
sfrw(ADC12CTL1, ADC12CTL1_);
#define ADC12IFG_             0x01A4    /* ADC12 Interrupt Flag */
sfrw(ADC12IFG, ADC12IFG_);
#define ADC12IE_              0x01A6    /* ADC12 Interrupt Enable */
sfrw(ADC12IE, ADC12IE_);
#define ADC12IV_              0x01A8    /* ADC12 Interrupt Vector Word */
sfrw(ADC12IV, ADC12IV_);

#define ADC12MEM_           (0x0140)  /* ADC12 Conversion Memory */
#ifndef __STDC__
#define ADC12MEM            (ADC12MEM_) /* ADC12 Conversion Memory (for assembler) */
#else
#define ADC12MEM            ((volatile int*) ADC12MEM_) /* ADC12 Conversion Memory (for C) */
#endif
#define ADC12MEM0_            0x0140    /* ADC12 Conversion Memory 0 */
sfrw(ADC12MEM0, ADC12MEM0_);
#define ADC12MEM1_            0x0142    /* ADC12 Conversion Memory 1 */
sfrw(ADC12MEM1, ADC12MEM1_);
#define ADC12MEM2_            0x0144    /* ADC12 Conversion Memory 2 */
sfrw(ADC12MEM2, ADC12MEM2_);
#define ADC12MEM3_            0x0146    /* ADC12 Conversion Memory 3 */
sfrw(ADC12MEM3, ADC12MEM3_);
#define ADC12MEM4_            0x0148    /* ADC12 Conversion Memory 4 */
sfrw(ADC12MEM4, ADC12MEM4_);
#define ADC12MEM5_            0x014A    /* ADC12 Conversion Memory 5 */
sfrw(ADC12MEM5, ADC12MEM5_);
#define ADC12MEM6_            0x014C    /* ADC12 Conversion Memory 6 */
sfrw(ADC12MEM6, ADC12MEM6_);
#define ADC12MEM7_            0x014E    /* ADC12 Conversion Memory 7 */
sfrw(ADC12MEM7, ADC12MEM7_);
#define ADC12MEM8_            0x0150    /* ADC12 Conversion Memory 8 */
sfrw(ADC12MEM8, ADC12MEM8_);
#define ADC12MEM9_            0x0152    /* ADC12 Conversion Memory 9 */
sfrw(ADC12MEM9, ADC12MEM9_);
#define ADC12MEM10_           0x0154    /* ADC12 Conversion Memory 10 */
sfrw(ADC12MEM10, ADC12MEM10_);
#define ADC12MEM11_           0x0156    /* ADC12 Conversion Memory 11 */
sfrw(ADC12MEM11, ADC12MEM11_);
#define ADC12MEM12_           0x0158    /* ADC12 Conversion Memory 12 */
sfrw(ADC12MEM12, ADC12MEM12_);
#define ADC12MEM13_           0x015A    /* ADC12 Conversion Memory 13 */
sfrw(ADC12MEM13, ADC12MEM13_);
#define ADC12MEM14_           0x015C    /* ADC12 Conversion Memory 14 */
sfrw(ADC12MEM14, ADC12MEM14_);
#define ADC12MEM15_           0x015E    /* ADC12 Conversion Memory 15 */
sfrw(ADC12MEM15, ADC12MEM15_);

#define ADC12MCTL_          (0x0080)  /* ADC12 Memory Control */
#ifndef __STDC__
#define ADC12MCTL           (ADC12MCTL_) /* ADC12 Memory Control (for assembler) */
#else
#define ADC12MCTL           ((volatile char*) ADC12MCTL_) /* ADC12 Memory Control (for C) */
#endif
#define ADC12MCTL0_           0x0080    /* ADC12 Memory Control 0 */
sfrb(ADC12MCTL0, ADC12MCTL0_);
#define ADC12MCTL1_           0x0081    /* ADC12 Memory Control 1 */
sfrb(ADC12MCTL1, ADC12MCTL1_);
#define ADC12MCTL2_           0x0082    /* ADC12 Memory Control 2 */
sfrb(ADC12MCTL2, ADC12MCTL2_);
#define ADC12MCTL3_           0x0083    /* ADC12 Memory Control 3 */
sfrb(ADC12MCTL3, ADC12MCTL3_);
#define ADC12MCTL4_           0x0084    /* ADC12 Memory Control 4 */
sfrb(ADC12MCTL4, ADC12MCTL4_);
#define ADC12MCTL5_           0x0085    /* ADC12 Memory Control 5 */
sfrb(ADC12MCTL5, ADC12MCTL5_);
#define ADC12MCTL6_           0x0086    /* ADC12 Memory Control 6 */
sfrb(ADC12MCTL6, ADC12MCTL6_);
#define ADC12MCTL7_           0x0087    /* ADC12 Memory Control 7 */
sfrb(ADC12MCTL7, ADC12MCTL7_);
#define ADC12MCTL8_           0x0088    /* ADC12 Memory Control 8 */
sfrb(ADC12MCTL8, ADC12MCTL8_);
#define ADC12MCTL9_           0x0089    /* ADC12 Memory Control 9 */
sfrb(ADC12MCTL9, ADC12MCTL9_);
#define ADC12MCTL10_          0x008A    /* ADC12 Memory Control 10 */
sfrb(ADC12MCTL10, ADC12MCTL10_);
#define ADC12MCTL11_          0x008B    /* ADC12 Memory Control 11 */
sfrb(ADC12MCTL11, ADC12MCTL11_);
#define ADC12MCTL12_          0x008C    /* ADC12 Memory Control 12 */
sfrb(ADC12MCTL12, ADC12MCTL12_);
#define ADC12MCTL13_          0x008D    /* ADC12 Memory Control 13 */
sfrb(ADC12MCTL13, ADC12MCTL13_);
#define ADC12MCTL14_          0x008E    /* ADC12 Memory Control 14 */
sfrb(ADC12MCTL14, ADC12MCTL14_);
#define ADC12MCTL15_          0x008F    /* ADC12 Memory Control 15 */
sfrb(ADC12MCTL15, ADC12MCTL15_);

/* ADC12CTL0 */
#define ADC12SC             (0x001)   /* ADC12 Start Conversion */
#define ENC                 (0x002)   /* ADC12 Enable Conversion */
#define ADC12TOVIE          (0x004)   /* ADC12 Timer Overflow interrupt enable */
#define ADC12OVIE           (0x008)   /* ADC12 Overflow interrupt enable */
#define ADC12ON             (0x010)   /* ADC12 On/enable */
#define REFON               (0x020)   /* ADC12 Reference on */
#define REF2_5V             (0x040)   /* ADC12 Ref 0:1.5V / 1:2.5V */
#define MSC                 (0x080)   /* ADC12 Multiple SampleConversion */
#define SHT00               (0x0100)  /* ADC12 Sample Hold 0 Select 0 */
#define SHT01               (0x0200)  /* ADC12 Sample Hold 0 Select 1 */
#define SHT02               (0x0400)  /* ADC12 Sample Hold 0 Select 2 */
#define SHT03               (0x0800)  /* ADC12 Sample Hold 0 Select 3 */
#define SHT10               (0x1000)  /* ADC12 Sample Hold 0 Select 0 */
#define SHT11               (0x2000)  /* ADC12 Sample Hold 1 Select 1 */
#define SHT12               (0x4000)  /* ADC12 Sample Hold 2 Select 2 */
#define SHT13               (0x8000)  /* ADC12 Sample Hold 3 Select 3 */
#define MSH                 (0x080)

#define SHT0_0               (0x0000) /* ADC12 Sample Hold 0 Select Bit: 0 */
#define SHT0_1               (0x0100) /* ADC12 Sample Hold 0 Select Bit: 1 */
#define SHT0_2               (0x0200) /* ADC12 Sample Hold 0 Select Bit: 2 */
#define SHT0_3               (0x0300) /* ADC12 Sample Hold 0 Select Bit: 3 */
#define SHT0_4               (0x0400) /* ADC12 Sample Hold 0 Select Bit: 4 */
#define SHT0_5               (0x0500) /* ADC12 Sample Hold 0 Select Bit: 5 */
#define SHT0_6               (0x0600) /* ADC12 Sample Hold 0 Select Bit: 6 */
#define SHT0_7               (0x0700) /* ADC12 Sample Hold 0 Select Bit: 7 */
#define SHT0_8               (0x0800) /* ADC12 Sample Hold 0 Select Bit: 8 */
#define SHT0_9               (0x0900) /* ADC12 Sample Hold 0 Select Bit: 9 */
#define SHT0_10             (0x0A00) /* ADC12 Sample Hold 0 Select Bit: 10 */
#define SHT0_11             (0x0B00) /* ADC12 Sample Hold 0 Select Bit: 11 */
#define SHT0_12             (0x0C00) /* ADC12 Sample Hold 0 Select Bit: 12 */
#define SHT0_13             (0x0D00) /* ADC12 Sample Hold 0 Select Bit: 13 */
#define SHT0_14             (0x0E00) /* ADC12 Sample Hold 0 Select Bit: 14 */
#define SHT0_15             (0x0F00) /* ADC12 Sample Hold 0 Select Bit: 15 */

#define SHT1_0               (0x0000) /* ADC12 Sample Hold 1 Select Bit: 0 */
#define SHT1_1               (0x1000) /* ADC12 Sample Hold 1 Select Bit: 1 */
#define SHT1_2               (0x2000) /* ADC12 Sample Hold 1 Select Bit: 2 */
#define SHT1_3               (0x3000) /* ADC12 Sample Hold 1 Select Bit: 3 */
#define SHT1_4               (0x4000) /* ADC12 Sample Hold 1 Select Bit: 4 */
#define SHT1_5               (0x5000) /* ADC12 Sample Hold 1 Select Bit: 5 */
#define SHT1_6               (0x6000) /* ADC12 Sample Hold 1 Select Bit: 6 */
#define SHT1_7               (0x7000) /* ADC12 Sample Hold 1 Select Bit: 7 */
#define SHT1_8               (0x8000) /* ADC12 Sample Hold 1 Select Bit: 8 */
#define SHT1_9               (0x9000) /* ADC12 Sample Hold 1 Select Bit: 9 */
#define SHT1_10             (0xA000) /* ADC12 Sample Hold 1 Select Bit: 10 */
#define SHT1_11             (0xB000) /* ADC12 Sample Hold 1 Select Bit: 11 */
#define SHT1_12             (0xC000) /* ADC12 Sample Hold 1 Select Bit: 12 */
#define SHT1_13             (0xD000) /* ADC12 Sample Hold 1 Select Bit: 13 */
#define SHT1_14             (0xE000) /* ADC12 Sample Hold 1 Select Bit: 14 */
#define SHT1_15             (0xF000) /* ADC12 Sample Hold 1 Select Bit: 15 */

/* ADC12CTL1 */
#define ADC12BUSY           (0x0001)    /* ADC12 Busy */
#define CONSEQ0             (0x0002)    /* ADC12 Conversion Sequence Select 0 */
#define CONSEQ1             (0x0004)    /* ADC12 Conversion Sequence Select 1 */
#define ADC12SSEL0          (0x0008)    /* ADC12 Clock Source Select 0 */
#define ADC12SSEL1          (0x0010)    /* ADC12 Clock Source Select 1 */
#define ADC12DIV0           (0x0020)    /* ADC12 Clock Divider Select 0 */
#define ADC12DIV1           (0x0040)    /* ADC12 Clock Divider Select 1 */
#define ADC12DIV2           (0x0080)    /* ADC12 Clock Divider Select 2 */
#define ISSH                (0x0100)    /* ADC12 Invert Sample Hold Signal */
#define SHP                 (0x0200)    /* ADC12 Sample/Hold Pulse Mode */
#define SHS0                (0x0400)    /* ADC12 Sample/Hold Source 0 */
#define SHS1                (0x0800)    /* ADC12 Sample/Hold Source 1 */
#define CSTARTADD0          (0x1000)    /* ADC12 Conversion Start Address 0 */
#define CSTARTADD1          (0x2000)    /* ADC12 Conversion Start Address 1 */
#define CSTARTADD2          (0x4000)    /* ADC12 Conversion Start Address 2 */
#define CSTARTADD3          (0x8000)    /* ADC12 Conversion Start Address 3 */

#define CONSEQ_0             (0x0000)      /* ADC12 Conversion Sequence Select: 0 */
#define CONSEQ_1             (0x0002)      /* ADC12 Conversion Sequence Select: 1 */
#define CONSEQ_2             (0x0004)      /* ADC12 Conversion Sequence Select: 2 */
#define CONSEQ_3             (0x0006)      /* ADC12 Conversion Sequence Select: 3 */
#define ADC12SSEL_0          (0x0000)      /* ADC12 Clock Source Select: 0 */
#define ADC12SSEL_1          (0x0008)      /* ADC12 Clock Source Select: 1 */
#define ADC12SSEL_2          (0x0010)      /* ADC12 Clock Source Select: 2 */
#define ADC12SSEL_3          (0x0018)      /* ADC12 Clock Source Select: 3 */
#define ADC12DIV_0           (0x0000)   /* ADC12 Clock Divider Select: 0 */
#define ADC12DIV_1           (0x0020)   /* ADC12 Clock Divider Select: 1 */
#define ADC12DIV_2           (0x0040)   /* ADC12 Clock Divider Select: 2 */
#define ADC12DIV_3           (0x0060)   /* ADC12 Clock Divider Select: 3 */
#define ADC12DIV_4           (0x0080)   /* ADC12 Clock Divider Select: 4 */
#define ADC12DIV_5           (0x00A0)   /* ADC12 Clock Divider Select: 5 */
#define ADC12DIV_6           (0x00C0)   /* ADC12 Clock Divider Select: 6 */
#define ADC12DIV_7           (0x00E0)   /* ADC12 Clock Divider Select: 7 */
#define SHS_0                (0x0000)  /* ADC12 Sample/Hold Source: 0 */
#define SHS_1                (0x0400)  /* ADC12 Sample/Hold Source: 1 */
#define SHS_2                (0x0800)  /* ADC12 Sample/Hold Source: 2 */
#define SHS_3                (0x0C00)  /* ADC12 Sample/Hold Source: 3 */
#define CSTARTADD_0          (0x0000) /* ADC12 Conversion Start Address: 0 */
#define CSTARTADD_1          (0x1000) /* ADC12 Conversion Start Address: 1 */
#define CSTARTADD_2          (0x2000) /* ADC12 Conversion Start Address: 2 */
#define CSTARTADD_3          (0x3000) /* ADC12 Conversion Start Address: 3 */
#define CSTARTADD_4          (0x4000) /* ADC12 Conversion Start Address: 4 */
#define CSTARTADD_5          (0x5000) /* ADC12 Conversion Start Address: 5 */
#define CSTARTADD_6          (0x6000) /* ADC12 Conversion Start Address: 6 */
#define CSTARTADD_7          (0x7000) /* ADC12 Conversion Start Address: 7 */
#define CSTARTADD_8          (0x8000) /* ADC12 Conversion Start Address: 8 */
#define CSTARTADD_9          (0x9000) /* ADC12 Conversion Start Address: 9 */
#define CSTARTADD_10        (0xA000) /* ADC12 Conversion Start Address: 10 */
#define CSTARTADD_11        (0xB000) /* ADC12 Conversion Start Address: 11 */
#define CSTARTADD_12        (0xC000) /* ADC12 Conversion Start Address: 12 */
#define CSTARTADD_13        (0xD000) /* ADC12 Conversion Start Address: 13 */
#define CSTARTADD_14        (0xE000) /* ADC12 Conversion Start Address: 14 */
#define CSTARTADD_15        (0xF000) /* ADC12 Conversion Start Address: 15 */

/* ADC12MCTLx */
#define INCH0               (0x0001)    /* ADC12 Input Channel Select Bit 0 */
#define INCH1               (0x0002)    /* ADC12 Input Channel Select Bit 1 */
#define INCH2               (0x0004)    /* ADC12 Input Channel Select Bit 2 */
#define INCH3               (0x0008)    /* ADC12 Input Channel Select Bit 3 */
#define SREF0               (0x0010)    /* ADC12 Select Reference Bit 0 */
#define SREF1               (0x0020)    /* ADC12 Select Reference Bit 1 */
#define SREF2               (0x0040)    /* ADC12 Select Reference Bit 2 */
#define EOS                 (0x0080)    /* ADC12 End of Sequence */

#define INCH_0               (0)        /* ADC12 Input Channel 0 */
#define INCH_1               (1)        /* ADC12 Input Channel 1 */
#define INCH_2               (2)        /* ADC12 Input Channel 2 */
#define INCH_3               (3)        /* ADC12 Input Channel 3 */
#define INCH_4               (4)        /* ADC12 Input Channel 4 */
#define INCH_5               (5)        /* ADC12 Input Channel 5 */
#define INCH_6               (6)        /* ADC12 Input Channel 6 */
#define INCH_7               (7)        /* ADC12 Input Channel 7 */
#define INCH_8               (8)        /* ADC12 Input Channel 8 */
#define INCH_9               (9)        /* ADC12 Input Channel 9 */
#define INCH_10             (10)        /* ADC12 Input Channel 10 */
#define INCH_11             (11)        /* ADC12 Input Channel 11 */
#define INCH_12             (12)        /* ADC12 Input Channel 12 */
#define INCH_13             (13)        /* ADC12 Input Channel 13 */
#define INCH_14             (14)        /* ADC12 Input Channel 14 */
#define INCH_15             (15)        /* ADC12 Input Channel 15 */

#define SREF_0               (0x0000)   /* ADC12 Select Reference 0 */
#define SREF_1               (0x0010)   /* ADC12 Select Reference 1 */
#define SREF_2               (0x0020)   /* ADC12 Select Reference 2 */
#define SREF_3               (0x0030)   /* ADC12 Select Reference 3 */
#define SREF_4               (0x0040)   /* ADC12 Select Reference 4 */
#define SREF_5               (0x0050)   /* ADC12 Select Reference 5 */
#define SREF_6               (0x0060)   /* ADC12 Select Reference 6 */
#define SREF_7               (0x0070)   /* ADC12 Select Reference 7 */

/* ADC12IV Definitions */
#define ADC12IV_NONE        (0x0000)    /* No Interrupt pending */
#define ADC12IV_ADC12OVIFG  (0x0002)    /* ADC12OVIFG */
#define ADC12IV_ADC12TOVIFG (0x0004)    /* ADC12TOVIFG */
#define ADC12IV_ADC12IFG0   (0x0006)    /* ADC12IFG0 */
#define ADC12IV_ADC12IFG1   (0x0008)    /* ADC12IFG1 */
#define ADC12IV_ADC12IFG2   (0x000A)    /* ADC12IFG2 */
#define ADC12IV_ADC12IFG3   (0x000C)    /* ADC12IFG3 */
#define ADC12IV_ADC12IFG4   (0x000E)    /* ADC12IFG4 */
#define ADC12IV_ADC12IFG5   (0x0010)    /* ADC12IFG5 */
#define ADC12IV_ADC12IFG6   (0x0012)    /* ADC12IFG6 */
#define ADC12IV_ADC12IFG7   (0x0014)    /* ADC12IFG7 */
#define ADC12IV_ADC12IFG8   (0x0016)    /* ADC12IFG8 */
#define ADC12IV_ADC12IFG9   (0x0018)    /* ADC12IFG9 */
#define ADC12IV_ADC12IFG10  (0x001A)    /* ADC12IFG10 */
#define ADC12IV_ADC12IFG11  (0x001C)    /* ADC12IFG11 */
#define ADC12IV_ADC12IFG12  (0x001E)    /* ADC12IFG12 */
#define ADC12IV_ADC12IFG13  (0x0020)    /* ADC12IFG13 */
#define ADC12IV_ADC12IFG14  (0x0022)    /* ADC12IFG14 */
#define ADC12IV_ADC12IFG15  (0x0024)    /* ADC12IFG15 */

/************************************************************
* BASIC TIMER with Real Time Clock
************************************************************/
#define __MSP430_HAS_BT_RTC__          /* Definition to show that Module is available */

#define BTCTL_                0x0040    /* Basic Timer Control */
sfrb(BTCTL, BTCTL_);
#define RTCCTL_               0x0041    /* Real Time Clock Control */
sfrb(RTCCTL, RTCCTL_);
#define RTCNT1_               0x0042    /* Real Time Counter 1 */
sfrb(RTCNT1, RTCNT1_);
#define RTCNT2_               0x0043    /* Real Time Counter 2 */
sfrb(RTCNT2, RTCNT2_);
#define RTCNT3_               0x0044    /* Real Time Counter 3 */
sfrb(RTCNT3, RTCNT3_);
#define RTCNT4_               0x0045    /* Real Time Counter 4 */
sfrb(RTCNT4, RTCNT4_);
#define BTCNT1_               0x0046    /* Basic Timer Count 1 */
sfrb(BTCNT1, BTCNT1_);
#define BTCNT2_               0x0047    /* Basic Timer Count 2 */
sfrb(BTCNT2, BTCNT2_);
#define RTCDAY_               0x004C    /* Real Time Clock Day */
sfrb(RTCDAY, RTCDAY_);
#define RTCMON_               0x004D    /* Real Time Clock Month */
sfrb(RTCMON, RTCMON_);
#define RTCYEARL_             0x004E    /* Real Time Clock Year (Low Byte) */
sfrb(RTCYEARL, RTCYEARL_);
#define RTCYEARH_             0x004F    /* Real Time Clock Year (High Byte) */
sfrb(RTCYEARH, RTCYEARH_);
#define RTCSEC              RTCNT1
#define RTCMIN              RTCNT2
#define RTCHOUR             RTCNT3
#define RTCDOW              RTCNT4

#define RTCTL_                0x0040    /* Basic/Real Timer Control */
sfrw(RTCTL, RTCTL_);
#define RTCTIM0_              0x0042    /* Real Time Clock Time 0 */
sfrw(RTCTIM0, RTCTIM0_);
#define RTCTIM1_              0x0044    /* Real Time Clock Time 1 */
sfrw(RTCTIM1, RTCTIM1_);
#define BTCNT12_              0x0046    /* Basic Timer Count 1/2 */
sfrw(BTCNT12, BTCNT12_);
#define RTCDATE_              0x004C    /* Real Time Clock Date */
sfrw(RTCDATE, RTCDATE_);
#define RTCYEAR_              0x004E    /* Real Time Clock Year */
sfrw(RTCYEAR, RTCYEAR_);
#define RTCNT12             RTCTIM0
#define RTCNT34             RTCTIM1

#define BTSSEL              (0x80)                     /* fBT = fMCLK (main clock) */
#define BTHOLD              (0x40)                     /* BT1 is held if this bit is set */
#define BTDIV               (0x20)                     /* fCLK2 = ACLK:256 */
//#define res               (0x10)
//#define res               (0x08)
#define BTIP2               (0x04)
#define BTIP1               (0x02)
#define BTIP0               (0x01)

#define RTCBCD              (0x80)                     /* RTC BCD Select */
#define RTCHOLD             (0x40)                     /* RTC Hold */
#define RTCMODE1            (0x20)                     /* RTC Mode 1 */
#define RTCMODE0            (0x10)                     /* RTC Mode 0 */
#define RTCTEV1             (0x08)                     /* RTC Time Event 1 */
#define RTCTEV0             (0x04)                     /* RTC Time Event 0 */
#define RTCIE               (0x02)                     /* RTC Interrupt Enable */
#define RTCFG               (0x01)                     /* RTC Event Flag */

#define RTCTEV_0            (0x00)                     /* RTC Time Event: 0 */
#define RTCTEV_1            (0x04)                     /* RTC Time Event: 1 */
#define RTCTEV_2            (0x08)                     /* RTC Time Event: 2 */
#define RTCTEV_3            (0x0C)                     /* RTC Time Event: 3 */
#define RTCMODE_0           (0x00)                     /* RTC Mode: 0 */
#define RTCMODE_1           (0x10)                     /* RTC Mode: 1 */
#define RTCMODE_2           (0x20)                     /* RTC Mode: 2 */
#define RTCMODE_3           (0x30)                     /* RTC Mode: 3 */


/* Frequency of the BTCNT2 coded with Bit 5 and 7 in BTCTL */
#define BT_fCLK2_ACLK               (0x00)
#define BT_fCLK2_ACLK_DIV256        (BTDIV)
#define BT_fCLK2_MCLK               (BTSSEL)

/* Interrupt interval time fINT coded with Bits 0-2 in BTCTL */
#define BT_fCLK2_DIV2       (0x00)                    /* fINT = fCLK2:2 (default) */
#define BT_fCLK2_DIV4       (BTIP0)                   /* fINT = fCLK2:4 */
#define BT_fCLK2_DIV8       (BTIP1)                   /* fINT = fCLK2:8 */
#define BT_fCLK2_DIV16      (BTIP1+BTIP0)             /* fINT = fCLK2:16 */
#define BT_fCLK2_DIV32      (BTIP2)                   /* fINT = fCLK2:32 */
#define BT_fCLK2_DIV64      (BTIP2+BTIP0)             /* fINT = fCLK2:64 */
#define BT_fCLK2_DIV128     (BTIP2+BTIP1)             /* fINT = fCLK2:128 */
#define BT_fCLK2_DIV256     (BTIP2+BTIP1+BTIP0)       /* fINT = fCLK2:256 */

/* with assumed vlues of fACLK=32KHz, fMCLK=1MHz */
/* fBT=fACLK is thought for longer interval times */
#define BT_ADLY_0_064       (0x00)                    /* 0.064ms interval (default) */
#define BT_ADLY_0_125       (BTIP0)                   /* 0.125ms    " */
#define BT_ADLY_0_25        (BTIP1)                   /* 0.25ms     " */
#define BT_ADLY_0_5         (BTIP1+BTIP0)             /* 0.5ms      " */
#define BT_ADLY_1           (BTIP2)                   /* 1ms        " */
#define BT_ADLY_2           (BTIP2+BTIP0)             /* 2ms        " */
#define BT_ADLY_4           (BTIP2+BTIP1)             /* 4ms        " */
#define BT_ADLY_8           (BTIP2+BTIP1+BTIP0)       /* 8ms        " */
#define BT_ADLY_16          (BTDIV)                   /* 16ms       " */
#define BT_ADLY_32          (BTDIV+BTIP0)             /* 32ms       " */
#define BT_ADLY_64          (BTDIV+BTIP1)             /* 64ms       " */
#define BT_ADLY_125         (BTDIV+BTIP1+BTIP0)       /* 125ms      " */
#define BT_ADLY_250         (BTDIV+BTIP2)             /* 250ms      " */
#define BT_ADLY_500         (BTDIV+BTIP2+BTIP0)       /* 500ms      " */
#define BT_ADLY_1000        (BTDIV+BTIP2+BTIP1)       /* 1000ms     " */
#define BT_ADLY_2000        (BTDIV+BTIP2+BTIP1+BTIP0) /* 2000ms     " */
/* fCLK2=fMCLK (1MHz) is thought for short interval times */
/* the timing for short intervals is more precise than ACLK */
/* NOTE */
/* Be sure that the SCFQCTL-Register is set to 01Fh so that fMCLK=1MHz */
/* Too low interval time results in interrupts too frequent for the processor to handle! */
#define BT_MDLY_0_002       (BTSSEL)                  /* 0.002ms interval       *** interval times */
#define BT_MDLY_0_004       (BTSSEL+BTIP0)            /* 0.004ms    "           *** too short for */
#define BT_MDLY_0_008       (BTSSEL+BTIP1)            /* 0.008ms    "           *** interrupt */
#define BT_MDLY_0_016       (BTSSEL+BTIP1+BTIP0)      /* 0.016ms    "           *** handling */
#define BT_MDLY_0_032       (BTSSEL+BTIP2)            /* 0.032ms    " */
#define BT_MDLY_0_064       (BTSSEL+BTIP2+BTIP0)      /* 0.064ms    " */
#define BT_MDLY_0_125       (BTSSEL+BTIP2+BTIP1)      /* 0.125ms    " */
#define BT_MDLY_0_25        (BTSSEL+BTIP2+BTIP1+BTIP0)/* 0.25ms     " */

/* Hold coded with Bits 6-7 in BT(1)CTL */
/* this is for BT */
#define BTHOLD_CNT1         (BTHOLD)            /* BTCNT1 is held while BTHOLD is set */
#define BTHOLD_CNT1_2       (BTHOLD+BTDIV)      /* BT1CNT1 .AND. BT1CNT2 are held while ~ is set */

/* INTERRUPT CONTROL BITS */
/* #define BTIE                0x80 */
/* #define BTIFG               0x80 */

/************************************************************
* Comparator A
************************************************************/
#define __MSP430_HAS_COMPA__          /* Definition to show that Module is available */

#define CACTL1_               0x0059    /* Comparator A Control 1 */
sfrb(CACTL1, CACTL1_);
#define CACTL2_               0x005A    /* Comparator A Control 2 */
sfrb(CACTL2, CACTL2_);
#define CAPD_                 0x005B    /* Comparator A Port Disable */
sfrb(CAPD, CAPD_);

#define CAIFG               (0x01)    /* Comp. A Interrupt Flag */
#define CAIE                (0x02)    /* Comp. A Interrupt Enable */
#define CAIES               (0x04)    /* Comp. A Int. Edge Select: 0:rising / 1:falling */
#define CAON                (0x08)    /* Comp. A enable */
#define CAREF0              (0x10)    /* Comp. A Internal Reference Select 0 */
#define CAREF1              (0x20)    /* Comp. A Internal Reference Select 1 */
#define CARSEL              (0x40)    /* Comp. A Internal Reference Enable */
#define CAEX                (0x80)    /* Comp. A Exchange Inputs */

#define CAREF_0             (0x00)    /* Comp. A Int. Ref. Select 0 : Off */
#define CAREF_1             (0x10)    /* Comp. A Int. Ref. Select 1 : 0.25*Vcc */
#define CAREF_2             (0x20)    /* Comp. A Int. Ref. Select 2 : 0.5*Vcc */
#define CAREF_3             (0x30)    /* Comp. A Int. Ref. Select 3 : Vt*/

#define CAOUT               (0x01)    /* Comp. A Output */
#define CAF                 (0x02)    /* Comp. A Enable Output Filter */
#define P2CA0               (0x04)    /* Comp. A Connect External Signal to CA0 : 1 */
#define P2CA1               (0x08)    /* Comp. A Connect External Signal to CA1 : 1 */
#define CACTL24             (0x10)
#define CACTL25             (0x20)
#define CACTL26             (0x40)
#define CACTL27             (0x80)

#define CAPD0               (0x01)    /* Comp. A Disable Input Buffer of Port Register .0 */
#define CAPD1               (0x02)    /* Comp. A Disable Input Buffer of Port Register .1 */
#define CAPD2               (0x04)    /* Comp. A Disable Input Buffer of Port Register .2 */
#define CAPD3               (0x08)    /* Comp. A Disable Input Buffer of Port Register .3 */
#define CAPD4               (0x10)    /* Comp. A Disable Input Buffer of Port Register .4 */
#define CAPD5               (0x20)    /* Comp. A Disable Input Buffer of Port Register .5 */
#define CAPD6               (0x40)    /* Comp. A Disable Input Buffer of Port Register .6 */
#define CAPD7               (0x80)    /* Comp. A Disable Input Buffer of Port Register .7 */

/************************************************************
* DAC12
************************************************************/
#define __MSP430_HAS_DAC12_2__          /* Definition to show that Module is available */

#define DAC12_0CTL_           0x01c0    /* DAC12_0 Control */
sfrw(DAC12_0CTL, DAC12_0CTL_);
#define DAC12_1CTL_           0x01c2    /* DAC12_1 Control */
sfrw(DAC12_1CTL, DAC12_1CTL_);

#define DAC12GRP            (0x0001)    /* DAC12 group */
#define DAC12ENC            (0x0002)    /* DAC12 enable conversion */
#define DAC12IFG            (0x0004)    /* DAC12 interrupt flag */
#define DAC12IE             (0x0008)    /* DAC12 interrupt enable */
#define DAC12DF             (0x0010)    /* DAC12 data format */
#define DAC12AMP0           (0x0020)    /* DAC12 amplifier bit 0 */
#define DAC12AMP1           (0x0040)    /* DAC12 amplifier bit 1 */
#define DAC12AMP2           (0x0080)    /* DAC12 amplifier bit 2 */
#define DAC12IR             (0x0100)    /* DAC12 input reference and output range */
#define DAC12CALON          (0x0200)    /* DAC12 calibration */
#define DAC12LSEL0          (0x0400)    /* DAC12 load select bit 0 */
#define DAC12LSEL1          (0x0800)    /* DAC12 load select bit 1 */
#define DAC12RES            (0x1000)    /* DAC12 resolution */
#define DAC12SREF0          (0x2000)    /* DAC12 reference bit 0 */
#define DAC12SREF1          (0x4000)    /* DAC12 reference bit 1 */
#define DAC12OPS            (0x8000)    /* DAC12 Operation Amp. */

#define DAC12AMP_0          (0x0000)  /* DAC12 amplifier 0: off,    3-state */
#define DAC12AMP_1          (0x0020)  /* DAC12 amplifier 1: off,    off */
#define DAC12AMP_2          (0x0040)  /* DAC12 amplifier 2: low,    low */
#define DAC12AMP_3          (0x0060)  /* DAC12 amplifier 3: low,    medium */
#define DAC12AMP_4          (0x0080)  /* DAC12 amplifier 4: low,    high */
#define DAC12AMP_5          (0x00A0)  /* DAC12 amplifier 5: medium, medium */
#define DAC12AMP_6          (0x00C0)  /* DAC12 amplifier 6: medium, high */
#define DAC12AMP_7          (0x00E0)  /* DAC12 amplifier 7: high,   high */

#define DAC12LSEL_0         (0x0000)  /* DAC12 load select 0: direct */
#define DAC12LSEL_1         (0x0400)  /* DAC12 load select 1: latched with DAT */
#define DAC12LSEL_2         (0x0800)  /* DAC12 load select 2: latched with pos. Timer_A3.OUT1 */
#define DAC12LSEL_3         (0x0C00)  /* DAC12 load select 3: latched with pos. Timer_B7.OUT1 */

#define DAC12SREF_0         (0x0000)  /* DAC12 reference 0: Vref+ */
#define DAC12SREF_1         (0x2000)  /* DAC12 reference 1: Vref+ */
#define DAC12SREF_2         (0x4000)  /* DAC12 reference 2: Veref+ */
#define DAC12SREF_3         (0x6000)  /* DAC12 reference 3: Veref+ */

#define DAC12_0DAT_           0x01c8    /* DAC12_0 Data */
sfrw(DAC12_0DAT, DAC12_0DAT_);
#define DAC12_1DAT_           0x01ca    /* DAC12_1 Data */
sfrw(DAC12_1DAT, DAC12_1DAT_);
/************************************************************
* DMA_X
************************************************************/
#define __MSP430_HAS_DMAX_3__           /* Definition to show that Module is available */

#define DMACTL0_              0x0122    /* DMA Module Control 0 */
sfrw(DMACTL0, DMACTL0_);
#define DMA0TSEL0           (0x0001)    /* DMA channel 0 transfer select bit 0 */
#define DMA0TSEL1           (0x0002)    /* DMA channel 0 transfer select bit 1 */
#define DMA0TSEL2           (0x0004)    /* DMA channel 0 transfer select bit 2 */
#define DMA0TSEL3           (0x0008)    /* DMA channel 0 transfer select bit 3 */
#define DMA1TSEL0           (0x0010)    /* DMA channel 1 transfer select bit 0 */
#define DMA1TSEL1           (0x0020)    /* DMA channel 1 transfer select bit 1 */
#define DMA1TSEL2           (0x0040)    /* DMA channel 1 transfer select bit 2 */
#define DMA1TSEL3           (0x0080)    /* DMA channel 1 transfer select bit 3 */
#define DMA2TSEL0           (0x0100)    /* DMA channel 2 transfer select bit 0 */
#define DMA2TSEL1           (0x0200)    /* DMA channel 2 transfer select bit 1 */
#define DMA2TSEL2           (0x0400)    /* DMA channel 2 transfer select bit 2 */
#define DMA2TSEL3           (0x0800)    /* DMA channel 2 transfer select bit 3 */

#define DMA0TSEL_0          (0x0000)  /* DMA channel 0 transfer select 0:  DMA_REQ (sw)*/
#define DMA0TSEL_1          (0x0001)  /* DMA channel 0 transfer select 1:  Timer_A (TACCR2.IFG) */
#define DMA0TSEL_2          (0x0002)  /* DMA channel 0 transfer select 2:  Timer_B (TBCCR2.IFG) */
#define DMA0TSEL_3          (0x0003)  /* DMA channel 0 transfer select 3:  USCIA receive */
#define DMA0TSEL_4          (0x0004)  /* DMA channel 0 transfer select 4:  USCIA transmit */
#define DMA0TSEL_5          (0x0005)  /* DMA channel 0 transfer select 5:  DAC12_0CTL.DAC12IFG */
#define DMA0TSEL_6          (0x0006)  /* DMA channel 0 transfer select 6:  ADC12 (ADC12IFG) */
#define DMA0TSEL_7          (0x0007)  /* DMA channel 0 transfer select 7:  Timer_A (TACCR0.IFG) */
#define DMA0TSEL_8          (0x0008)  /* DMA channel 0 transfer select 8:  Timer_B (TBCCR0.IFG) */
#define DMA0TSEL_9          (0x0009)  /* DMA channel 0 transfer select 9:  UART1 receive */
#define DMA0TSEL_10         (0x000A) /* DMA channel 0 transfer select 10: UART1 transmit */
#define DMA0TSEL_11         (0x000B) /* DMA channel 0 transfer select 11: Multiplier ready */
#define DMA0TSEL_12         (0x000C) /* DMA channel 0 transfer select 12: USCIB receive */
#define DMA0TSEL_13         (0x000D) /* DMA channel 0 transfer select 13: USCIB transmit */
#define DMA0TSEL_14         (0x000E) /* DMA channel 0 transfer select 14: previous DMA channel DMA2IFG */
#define DMA0TSEL_15         (0x000F) /* DMA channel 0 transfer select 15: ext. Trigger (DMAE0) */

#define DMA1TSEL_0          (0x0000)  /* DMA channel 1 transfer select 0:  DMA_REQ */
#define DMA1TSEL_1          (0x0010)  /* DMA channel 1 transfer select 1:  Timer_A CCRIFG.2 */
#define DMA1TSEL_2          (0x0020)  /* DMA channel 1 transfer select 2:  Timer_B CCRIFG.2 */
#define DMA1TSEL_3          (0x0030)  /* DMA channel 1 transfer select 3:  USCIA receive */
#define DMA1TSEL_4          (0x0040)  /* DMA channel 1 transfer select 4:  USCIA transmit */
#define DMA1TSEL_5          (0x0050)  /* DMA channel 1 transfer select 5:  DAC12.0IFG */
#define DMA1TSEL_6          (0x0060)  /* DMA channel 1 transfer select 6:  ADC12 (ADC12IFG) */
#define DMA1TSEL_7          (0x0070)  /* DMA channel 1 transfer select 7:  Timer_A (TACCR0.IFG) */
#define DMA1TSEL_8          (0x0080)  /* DMA channel 1 transfer select 8:  Timer_B (TBCCR0.IFG) */
#define DMA1TSEL_9          (0x0090)  /* DMA channel 1 transfer select 9:  UART1 receive */
#define DMA1TSEL_10         (0x00A0) /* DMA channel 1 transfer select 10: UART1 transmit */
#define DMA1TSEL_11         (0x00B0) /* DMA channel 1 transfer select 11: Multiplier ready */
#define DMA1TSEL_12         (0x00C0) /* DMA channel 1 transfer select 12: USCIB receive */
#define DMA1TSEL_13         (0x00D0) /* DMA channel 1 transfer select 13: USCIB transmit */
#define DMA1TSEL_14         (0x00E0) /* DMA channel 1 transfer select 14: previous DMA channel DMA0IFG */
#define DMA1TSEL_15         (0x00F0) /* DMA channel 1 transfer select 15: ext. Trigger (DMAE0) */

#define DMA2TSEL_0          (0x0000)  /* DMA channel 2 transfer select 0:  DMA_REQ */
#define DMA2TSEL_1          (0x0100)  /* DMA channel 2 transfer select 1:  Timer_A CCRIFG.2 */
#define DMA2TSEL_2          (0x0200)  /* DMA channel 2 transfer select 2:  Timer_B CCRIFG.2 */
#define DMA2TSEL_3          (0x0300)  /* DMA channel 2 transfer select 3:  USCIA receive */
#define DMA2TSEL_4          (0x0400)  /* DMA channel 2 transfer select 4:  USCIA transmit */
#define DMA2TSEL_5          (0x0500)  /* DMA channel 2 transfer select 5:  DAC12.0IFG */
#define DMA2TSEL_6          (0x0600)  /* DMA channel 2 transfer select 6:  ADC12 (ADC12IFG) */
#define DMA2TSEL_7          (0x0700)  /* DMA channel 2 transfer select 7:  Timer_A (TACCR0.IFG) */
#define DMA2TSEL_8          (0x0800)  /* DMA channel 2 transfer select 8:  Timer_B (TBCCR0.IFG) */
#define DMA2TSEL_9          (0x0900)  /* DMA channel 2 transfer select 9:  UART1 receive */
#define DMA2TSEL_10         (0x0A00) /* DMA channel 2 transfer select 10: UART1 transmit */
#define DMA2TSEL_11         (0x0B00) /* DMA channel 2 transfer select 11: Multiplier ready */
#define DMA2TSEL_12         (0x0C00) /* DMA channel 2 transfer select 12: USCIB receive */
#define DMA2TSEL_13         (0x0D00) /* DMA channel 2 transfer select 13: USCIB transmit */
#define DMA2TSEL_14         (0x0E00) /* DMA channel 2 transfer select 14: previous DMA channel DMA1IFG */
#define DMA2TSEL_15         (0x0F00) /* DMA channel 2 transfer select 15: ext. Trigger (DMAE0) */

#define DMACTL1_              0x0124    /* DMA Module Control 1 */
sfrw(DMACTL1, DMACTL1_);
#define ENNMI               (0x0001)    /* Enable NMI interruption of DMA */
#define ROUNDROBIN          (0x0002)    /* Round-Robin DMA channel priorities */
#define DMAONFETCH          (0x0004)    /* DMA transfer on instruction fetch */

#define DMAIV_                0x0126    /* DMA Interrupt Vector Word */
sfrw(DMAIV, DMAIV_);
#define DMA0CTL_              0x01d0    /* DMA Channel 0 Control */
sfrw(DMA0CTL, DMA0CTL_);
#define DMA1CTL_              0x01dc    /* DMA Channel 1 Control */
sfrw(DMA1CTL, DMA1CTL_);
#define DMA2CTL_              0x01e8    /* DMA Channel 2 Control */
sfrw(DMA2CTL, DMA2CTL_);

#define DMAREQ              (0x0001)    /* Initiate DMA transfer with DMATSEL */
#define DMAABORT            (0x0002)    /* DMA transfer aborted by NMI */
#define DMAIE               (0x0004)    /* DMA interrupt enable */
#define DMAIFG              (0x0008)    /* DMA interrupt flag */
#define DMAEN               (0x0010)    /* DMA enable */
#define DMALEVEL            (0x0020)    /* DMA level sensitive trigger select */
#define DMASRCBYTE          (0x0040)    /* DMA source byte */
#define DMADSTBYTE          (0x0080)    /* DMA destination byte */
#define DMASRCINCR0         (0x0100)    /* DMA source increment bit 0 */
#define DMASRCINCR1         (0x0200)    /* DMA source increment bit 1 */
#define DMADSTINCR0         (0x0400)    /* DMA destination increment bit 0 */
#define DMADSTINCR1         (0x0800)    /* DMA destination increment bit 1 */
#define DMADT0              (0x1000)    /* DMA transfer mode bit 0 */
#define DMADT1              (0x2000)    /* DMA transfer mode bit 1 */
#define DMADT2              (0x4000)    /* DMA transfer mode bit 2 */

#define DMASWDW             (0x0000)  /* DMA transfer: source word to destination word */
#define DMASBDW             (0x0040)  /* DMA transfer: source byte to destination word */
#define DMASWDB             (0x0080)  /* DMA transfer: source word to destination byte */
#define DMASBDB             (0x00C0)  /* DMA transfer: source byte to destination byte */

#define DMASRCINCR_0        (0x0000)  /* DMA source increment 0: source address unchanged */
#define DMASRCINCR_1        (0x0100)  /* DMA source increment 1: source address unchanged */
#define DMASRCINCR_2        (0x0200)  /* DMA source increment 2: source address decremented */
#define DMASRCINCR_3        (0x0300)  /* DMA source increment 3: source address incremented */

#define DMADSTINCR_0        (0x0000)  /* DMA destination increment 0: destination address unchanged */
#define DMADSTINCR_1        (0x0400)  /* DMA destination increment 1: destination address unchanged */
#define DMADSTINCR_2        (0x0800)  /* DMA destination increment 2: destination address decremented */
#define DMADSTINCR_3        (0x0C00)  /* DMA destination increment 3: destination address incremented */

#define DMADT_0             (0x0000)  /* DMA transfer mode 0: single */
#define DMADT_1             (0x1000)  /* DMA transfer mode 1: block */
#define DMADT_2             (0x2000)  /* DMA transfer mode 2: interleaved */
#define DMADT_3             (0x3000)  /* DMA transfer mode 3: interleaved */
#define DMADT_4             (0x4000)  /* DMA transfer mode 4: single, repeat */
#define DMADT_5             (0x5000)  /* DMA transfer mode 5: block, repeat */
#define DMADT_6             (0x6000)  /* DMA transfer mode 6: interleaved, repeat */
#define DMADT_7             (0x7000)  /* DMA transfer mode 7: interleaved, repeat */

#define DMA0SA_               0x01d2    /* DMA Channel 0 Source Address */
sfra(DMA0SA, DMA0SA_);
#define DMA0SAL_              0x01d2    /* DMA Channel 0 Source Address */
sfrw(DMA0SAL, DMA0SAL_);
#define DMA0DA_               0x01d6    /* DMA Channel 0 Destination Address */
sfra(DMA0DA, DMA0DA_);
#define DMA0DAL_              0x01d6    /* DMA Channel 0 Destination Address */
sfrw(DMA0DAL, DMA0DAL_);
#define DMA0SZ_               0x01da    /* DMA Channel 0 Transfer Size */
sfrw(DMA0SZ, DMA0SZ_);
#define DMA1SA_               0x01de    /* DMA Channel 1 Source Address */
sfra(DMA1SA, DMA1SA_);
#define DMA1SAL_              0x01de    /* DMA Channel 1 Source Address */
sfrw(DMA1SAL, DMA1SAL_);
#define DMA1DA_               0x01e2    /* DMA Channel 1 Destination Address */
sfra(DMA1DA, DMA1DA_);
#define DMA1DAL_              0x01e2    /* DMA Channel 1 Destination Address */
sfrw(DMA1DAL, DMA1DAL_);
#define DMA1SZ_               0x01e6    /* DMA Channel 1 Transfer Size */
sfrw(DMA1SZ, DMA1SZ_);
#define DMA2SA_               0x01ea    /* DMA Channel 2 Source Address */
sfra(DMA2SA, DMA2SA_);
#define DMA2SAL_              0x01ea    /* DMA Channel 2 Source Address */
sfrw(DMA2SAL, DMA2SAL_);
#define DMA2DA_               0x01ee    /* DMA Channel 2 Destination Address */
sfra(DMA2DA, DMA2DA_);
#define DMA2DAL_              0x01ee    /* DMA Channel 2 Destination Address */
sfrw(DMA2DAL, DMA2DAL_);
#define DMA2SZ_               0x01f2    /* DMA Channel 2 Transfer Size */
sfrw(DMA2SZ, DMA2SZ_);

/* DMAIV Definitions */
#define DMAIV_NONE           (0x0000)    /* No Interrupt pending */
#define DMAIV_DMA0IFG        (0x0002)    /* DMA0IFG */
#define DMAIV_DMA1IFG        (0x0004)    /* DMA1IFG */
#define DMAIV_DMA2IFG        (0x0006)    /* DMA2IFG */

/*************************************************************
* Flash Memory
*************************************************************/
#define __MSP430_HAS_FLASH__          /* Definition to show that Module is available */
#define __MSP430_HAS_2FLASH_IP__      /* Definition to show that Module is available */

#define FCTL1_                0x0128    /* FLASH Control 1 */
sfrw(FCTL1, FCTL1_);
#define FCTL2_                0x012A    /* FLASH Control 2 */
sfrw(FCTL2, FCTL2_);
#define FCTL3_                0x012C    /* FLASH Control 3 */
sfrw(FCTL3, FCTL3_);

#define FRKEY               (0x9600)  /* Flash key returned by read */
#define FWKEY               (0xA500)  /* Flash key for write */
#define FXKEY               (0x3300)  /* for use with XOR instruction */

#define ERASE               (0x0002)  /* Enable bit for Flash segment erase */
#define MERAS               (0x0004)  /* Enable bit for Flash mass erase */
#define GMERAS              (0x0008)  /* Enable bit for Flash global mass erase */
#define CPUEX               (0x0010)  /* Enable bit for CPU Execution during Flash write/erase */
#define WRT                 (0x0040)  /* Enable bit for Flash write */
#define BLKWRT              (0x0080)  /* Enable bit for Flash segment write */
#define SEGWRT              (0x0080)  /* old definition */ /* Enable bit for Flash segment write */

#define FN0                 (0x0001)  /* Divide Flash clock by 1 to 64 using FN0 to FN5 according to: */
#define FN1                 (0x0002)  /*  32*FN5 + 16*FN4 + 8*FN3 + 4*FN2 + 2*FN1 + FN0 + 1 */
#ifndef FN2
#define FN2                 (0x0004)
#endif
#ifndef FN3
#define FN3                 (0x0008)
#endif
#ifndef FN4
#define FN4                 (0x0010)
#endif
#define FN5                 (0x0020)
#define FSSEL0              (0x0040)  /* Flash clock select 0 */        /* to distinguish from USART SSELx */
#define FSSEL1              (0x0080)  /* Flash clock select 1 */

#define FSSEL_0             (0x0000)  /* Flash clock select: 0 - ACLK */
#define FSSEL_1             (0x0040)  /* Flash clock select: 1 - MCLK */
#define FSSEL_2             (0x0080)  /* Flash clock select: 2 - SMCLK */
#define FSSEL_3             (0x00C0)  /* Flash clock select: 3 - SMCLK */

#define BUSY                (0x0001)  /* Flash busy: 1 */
#define KEYV                (0x0002)  /* Flash Key violation flag */
#define ACCVIFG             (0x0004)  /* Flash Access violation flag */
#define WAIT                (0x0008)  /* Wait flag for segment write */
#define LOCK                (0x0010)  /* Lock bit: 1 - Flash is locked (read only) */
#define EMEX                (0x0020)  /* Flash Emergency Exit */

/************************************************************
* SYSTEM CLOCK, FLL+
************************************************************/
#define __MSP430_HAS_FLLPLUS__        /* Definition to show that Module is available */

#define SCFI0_                0x0050    /* System Clock Frequency Integrator 0 */
sfrb(SCFI0, SCFI0_);
#define FN_2                (0x04)    /* fDCOCLK =   1.4-12MHz*/
#define FN_3                (0x08)    /* fDCOCLK =   2.2-17Mhz*/
#define FN_4                (0x10)    /* fDCOCLK =   3.2-25Mhz*/
#define FN_8                (0x20)    /* fDCOCLK =     5-40Mhz*/
#define FLLD0               (0x40)    /* Loop Divider Bit : 0 */
#define FLLD1               (0x80)    /* Loop Divider Bit : 1 */

#define FLLD_1              (0x00)    /* Multiply Selected Loop Freq. By 1 */
#define FLLD_2              (0x40)    /* Multiply Selected Loop Freq. By 2 */
#define FLLD_4              (0x80)    /* Multiply Selected Loop Freq. By 4 */
#define FLLD_8              (0xC0)    /* Multiply Selected Loop Freq. By 8 */

#define SCFI1_                0x0051    /* System Clock Frequency Integrator 1 */
sfrb(SCFI1, SCFI1_);
#define SCFQCTL_              0x0052    /* System Clock Frequency Control */
sfrb(SCFQCTL, SCFQCTL_);
/* System clock frequency values fMCLK coded with Bits 0-6 in SCFQCTL */
/* #define SCFQ_32K            0x00                        fMCLK=1*fACLK       only a range from */
#define SCFQ_64K            (0x01)                     /* fMCLK=2*fACLK          1+1 to 127+1 is possible */
#define SCFQ_128K           (0x03)                     /* fMCLK=4*fACLK */
#define SCFQ_256K           (0x07)                     /* fMCLK=8*fACLK */
#define SCFQ_512K           (0x0F)                     /* fMCLK=16*fACLK */
#define SCFQ_1M             (0x1F)                     /* fMCLK=32*fACLK */
#define SCFQ_2M             (0x3F)                     /* fMCLK=64*fACLK */
#define SCFQ_4M             (0x7F)                     /* fMCLK=128*fACLK */
#define SCFQ_M              (0x80)                     /* Modulation Disable */

#define FLL_CTL0_             0x0053    /* FLL+ Control 0 */
sfrb(FLL_CTL0, FLL_CTL0_);
#define DCOF                (0x01)                     /* DCO Fault Flag */
#define LFOF                (0x02)                     /* Low Frequency Oscillator Fault Flag */
#define XT1OF               (0x04)                     /* High Frequency Oscillator 1 Fault Flag */
#define XT2OF               (0x08)                     /* High Frequency Oscillator 2 Fault Flag */
#define OSCCAP0             (0x10)                     /* XIN/XOUT Cap 0 */
#define OSCCAP1             (0x20)                     /* XIN/XOUT Cap 1 */
#define XTS_FLL             (0x40)                     /* 1: Selects high-freq. oscillator */
#define DCOPLUS             (0x80)                     /* DCO+ Enable */

#define XCAP0PF             (0x00)                     /* XIN Cap = XOUT Cap = 0pf */
#define XCAP10PF            (0x10)                     /* XIN Cap = XOUT Cap = 10pf */
#define XCAP14PF            (0x20)                     /* XIN Cap = XOUT Cap = 14pf */
#define XCAP18PF            (0x30)                     /* XIN Cap = XOUT Cap = 18pf */
#define OSCCAP_0            (0x00)                     /* XIN Cap = XOUT Cap = 0pf */
#define OSCCAP_1            (0x10)                     /* XIN Cap = XOUT Cap = 10pf */
#define OSCCAP_2            (0x20)                     /* XIN Cap = XOUT Cap = 14pf */
#define OSCCAP_3            (0x30)                     /* XIN Cap = XOUT Cap = 18pf */

#define FLL_CTL1_             0x0054    /* FLL+ Control 1 */
sfrb(FLL_CTL1, FLL_CTL1_);
#define FLL_DIV0            (0x01)                     /* FLL+ Divide Px.x/ACLK 0 */
#define FLL_DIV1            (0x02)                     /* FLL+ Divide Px.x/ACLK 1 */
#define SELS                (0x04)                     /* Peripheral Module Clock Source (0: DCO, 1: XT2) */
#define SELM0               (0x08)                     /* MCLK Source Select 0 */
#define SELM1               (0x10)                     /* MCLK Source Select 1 */
#define XT2OFF              (0x20)                     /* High Frequency Oscillator 2 (XT2) disable */
#define SMCLKOFF            (0x40)                     /* Peripheral Module Clock (SMCLK) disable */
#define LFXT1DIG            (0x80)                     /* Enable Digital input for LF clock */

#define FLL_DIV_1           (0x00)                     /* FLL+ Divide Px.x/ACLK By 1 */
#define FLL_DIV_2           (0x01)                     /* FLL+ Divide Px.x/ACLK By 2 */
#define FLL_DIV_4           (0x02)                     /* FLL+ Divide Px.x/ACLK By 4 */
#define FLL_DIV_8           (0x03)                     /* FLL+ Divide Px.x/ACLK By 8 */

#define SELM_DCO            (0x00)                     /* Select DCO for CPU MCLK */
#define SELM_XT2            (0x10)                     /* Select XT2 for CPU MCLK */
#define SELM_A              (0x18)                     /* Select A (from LFXT1) for CPU MCLK */

/* INTERRUPT CONTROL BITS */
/* These two bits are defined in the Special Function Registers */
/* #define OFIFG               0x02 */
/* #define OFIE                0x02 */

/************************************************************
* LCD_A
************************************************************/
#define __MSP430_HAS_LCD_A__          /* Definition to show that Module is available */

#define LCDACTL_              0x0090    /* LCD_A Control Register */
sfrb(LCDACTL, LCDACTL_);
#define LCDON               (0x01)
#define LCDSON              (0x04)
#define LCDMX0              (0x08)
#define LCDMX1              (0x10)
#define LCDFREQ0            (0x20)
#define LCDFREQ1            (0x40)
#define LCDFREQ2            (0x80)
/* Display modes coded with Bits 2-4 */
#define LCDSTATIC           (LCDSON)
#define LCD2MUX             (LCDMX0+LCDSON)
#define LCD3MUX             (LCDMX1+LCDSON)
#define LCD4MUX             (LCDMX1+LCDMX0+LCDSON)
/* Frequency select code with Bits 5-7 */
#define LCDFREQ_32          (0x00)    /* LCD Freq: ACLK divided by 32 */
#define LCDFREQ_64          (0x20)    /* LCD Freq: ACLK divided by 64 */
#define LCDFREQ_96          (0x40)    /* LCD Freq: ACLK divided by 96 */
#define LCDFREQ_128         (0x60)    /* LCD Freq: ACLK divided by 128 */
#define LCDFREQ_192         (0x80)    /* LCD Freq: ACLK divided by 192 */
#define LCDFREQ_256         (0xA0)    /* LCD Freq: ACLK divided by 256 */
#define LCDFREQ_384         (0xC0)    /* LCD Freq: ACLK divided by 384 */
#define LCDFREQ_512         (0xE0)    /* LCD Freq: ACLK divided by 512 */

#define LCDAPCTL0_            0x00AC    /* LCD_A Port Control Register 0 */
sfrb(LCDAPCTL0, LCDAPCTL0_);
#define LCDS0               (0x01)    /* LCD Segment  0 to  3 Enable. */
#define LCDS4               (0x02)    /* LCD Segment  4 to  7 Enable. */
#define LCDS8               (0x04)    /* LCD Segment  8 to 11 Enable. */
#define LCDS12              (0x08)    /* LCD Segment 12 to 15 Enable. */
#define LCDS16              (0x10)    /* LCD Segment 16 to 19 Enable. */
#define LCDS20              (0x20)    /* LCD Segment 20 to 23 Enable. */
#define LCDS24              (0x40)    /* LCD Segment 24 to 27 Enable. */
#define LCDS28              (0x80)    /* LCD Segment 28 to 31 Enable. */

#define LCDAPCTL1_            0x00AD    /* LCD_A Port Control Register 1 */
sfrb(LCDAPCTL1, LCDAPCTL1_);
#define LCDS32              (0x01)    /* LCD Segment 32 to 35 Enable. */
#define LCDS36              (0x02)    /* LCD Segment 36 to 39 Enable. */

#define LCDAVCTL0_            0x00AE    /* LCD_A Voltage Control Register 0 */
sfrb(LCDAVCTL0, LCDAVCTL0_);
#define LCD2B               (0x01)    /* Selects 1/2 bias. */
#define VLCDREF0            (0x02)    /* Selects reference voltage for regulated charge pump: 0 */
#define VLCDREF1            (0x04)    /* Selects reference voltage for regulated charge pump: 1 */
#define LCDCPEN             (0x08)    /* LCD Voltage Charge Pump Enable. */
#define VLCDEXT             (0x10)    /* Select external source for VLCD. */
#define LCDREXT             (0x20)    /* Selects external connections for LCD mid voltages. */
#define LCDR03EXT           (0x40)    /* Selects external connection for lowest LCD voltage. */

/* Reference voltage source select for the regulated charge pump */
#define VLCDREF_0           (0<<1)    /* Internal */
#define VLCDREF_1           (1<<1)    /* External */
#define VLCDREF_2           (2<<1)    /* Reserved */
#define VLCDREF_3           (3<<1)    /* Reserved */

#define LCDAVCTL1_            0x00AF    /* LCD_A Voltage Control Register 1 */
sfrb(LCDAVCTL1, LCDAVCTL1_);
#define VLCD0               (0x02)    /* VLCD select: 0 */
#define VLCD1               (0x04)    /* VLCD select: 1 */
#define VLCD2               (0x08)    /* VLCD select: 2 */
#define VLCD3               (0x10)    /* VLCD select: 3 */

/* Charge pump voltage selections */
#define VLCD_0              (0<<1)    /* Charge pump disabled */
#define VLCD_1              (1<<1)    /* VLCD = 2.60V */
#define VLCD_2              (2<<1)    /* VLCD = 2.66V */
#define VLCD_3              (3<<1)    /* VLCD = 2.72V */
#define VLCD_4              (4<<1)    /* VLCD = 2.78V */
#define VLCD_5              (5<<1)    /* VLCD = 2.84V */
#define VLCD_6              (6<<1)    /* VLCD = 2.90V */
#define VLCD_7              (7<<1)    /* VLCD = 2.96V */
#define VLCD_8              (8<<1)    /* VLCD = 3.02V */
#define VLCD_9              (9<<1)    /* VLCD = 3.08V */
#define VLCD_10             (10<<1)   /* VLCD = 3.14V */
#define VLCD_11             (11<<1)   /* VLCD = 3.20V */
#define VLCD_12             (12<<1)   /* VLCD = 3.26V */
#define VLCD_13             (12<<1)   /* VLCD = 3.32V */
#define VLCD_14             (13<<1)   /* VLCD = 3.38V */
#define VLCD_15             (15<<1)   /* VLCD = 3.44V */

#define VLCD_DISABLED       (0<<1)    /* Charge pump disabled */
#define VLCD_2_60           (1<<1)    /* VLCD = 2.60V */
#define VLCD_2_66           (2<<1)    /* VLCD = 2.66V */
#define VLCD_2_72           (3<<1)    /* VLCD = 2.72V */
#define VLCD_2_78           (4<<1)    /* VLCD = 2.78V */
#define VLCD_2_84           (5<<1)    /* VLCD = 2.84V */
#define VLCD_2_90           (6<<1)    /* VLCD = 2.90V */
#define VLCD_2_96           (7<<1)    /* VLCD = 2.96V */
#define VLCD_3_02           (8<<1)    /* VLCD = 3.02V */
#define VLCD_3_08           (9<<1)    /* VLCD = 3.08V */
#define VLCD_3_14           (10<<1)   /* VLCD = 3.14V */
#define VLCD_3_20           (11<<1)   /* VLCD = 3.20V */
#define VLCD_3_26           (12<<1)   /* VLCD = 3.26V */
#define VLCD_3_32           (12<<1)   /* VLCD = 3.32V */
#define VLCD_3_38           (13<<1)   /* VLCD = 3.38V */
#define VLCD_3_44           (15<<1)   /* VLCD = 3.44V */

#define LCDMEM_             (0x0091)  /* LCD Memory */
#ifndef __STDC__
#define LCDMEM              (LCDMEM_) /* LCD Memory (for assembler) */
#else
#define LCDMEM              ((volatile char*) LCDMEM_) /* LCD Memory (for C) */
#endif
#define LCDM1_                0x0091    /* LCD Memory 1 */
sfrb(LCDM1, LCDM1_);
#define LCDM2_                0x0092    /* LCD Memory 2 */
sfrb(LCDM2, LCDM2_);
#define LCDM3_                0x0093    /* LCD Memory 3 */
sfrb(LCDM3, LCDM3_);
#define LCDM4_                0x0094    /* LCD Memory 4 */
sfrb(LCDM4, LCDM4_);
#define LCDM5_                0x0095    /* LCD Memory 5 */
sfrb(LCDM5, LCDM5_);
#define LCDM6_                0x0096    /* LCD Memory 6 */
sfrb(LCDM6, LCDM6_);
#define LCDM7_                0x0097    /* LCD Memory 7 */
sfrb(LCDM7, LCDM7_);
#define LCDM8_                0x0098    /* LCD Memory 8 */
sfrb(LCDM8, LCDM8_);
#define LCDM9_                0x0099    /* LCD Memory 9 */
sfrb(LCDM9, LCDM9_);
#define LCDM10_               0x009A    /* LCD Memory 10 */
sfrb(LCDM10, LCDM10_);
#define LCDM11_               0x009B    /* LCD Memory 11 */
sfrb(LCDM11, LCDM11_);
#define LCDM12_               0x009C    /* LCD Memory 12 */
sfrb(LCDM12, LCDM12_);
#define LCDM13_               0x009D    /* LCD Memory 13 */
sfrb(LCDM13, LCDM13_);
#define LCDM14_               0x009E    /* LCD Memory 14 */
sfrb(LCDM14, LCDM14_);
#define LCDM15_               0x009F    /* LCD Memory 15 */
sfrb(LCDM15, LCDM15_);
#define LCDM16_               0x00A0    /* LCD Memory 16 */
sfrb(LCDM16, LCDM16_);
#define LCDM17_               0x00A1    /* LCD Memory 17 */
sfrb(LCDM17, LCDM17_);
#define LCDM18_               0x00A2    /* LCD Memory 18 */
sfrb(LCDM18, LCDM18_);
#define LCDM19_               0x00A3    /* LCD Memory 19 */
sfrb(LCDM19, LCDM19_);
#define LCDM20_               0x00A4    /* LCD Memory 20 */
sfrb(LCDM20, LCDM20_);

#define LCDMA               (LCDM10)  /* LCD Memory A */
#define LCDMB               (LCDM11)  /* LCD Memory B */
#define LCDMC               (LCDM12)  /* LCD Memory C */
#define LCDMD               (LCDM13)  /* LCD Memory D */
#define LCDME               (LCDM14)  /* LCD Memory E */
#define LCDMF               (LCDM15)  /* LCD Memory F */

/************************************************************
* HARDWARE MULTIPLIER
************************************************************/
#define __MSP430_HAS_MPY__            /* Definition to show that Module is available */

#define MPY_                  0x0130    /* Multiply Unsigned/Operand 1 */
sfrw(MPY, MPY_);
#define MPYS_                 0x0132    /* Multiply Signed/Operand 1 */
sfrw(MPYS, MPYS_);
#define MAC_                  0x0134    /* Multiply Unsigned and Accumulate/Operand 1 */
sfrw(MAC, MAC_);
#define MACS_                 0x0136    /* Multiply Signed and Accumulate/Operand 1 */
sfrw(MACS, MACS_);
#define OP2_                  0x0138    /* Operand 2 */
sfrw(OP2, OP2_);
#define RESLO_                0x013A    /* Result Low Word */
sfrw(RESLO, RESLO_);
#define RESHI_                0x013C    /* Result High Word */
sfrw(RESHI, RESHI_);
#define SUMEXT_               0x013E    /* Sum Extend */
const_sfrw(SUMEXT, SUMEXT_);

/************************************************************
* Operational Amplifier
************************************************************/
#define __MSP430_HAS_OA_3__           /* Definition to show that Module is available */

#define OA0CTL0_              0x00C0    /* OA0 Control register 0 */
sfrb(OA0CTL0, OA0CTL0_);
#define OA0CTL1_              0x00C1    /* OA0 Control register 1 */
sfrb(OA0CTL1, OA0CTL1_);
#define OA1CTL0_              0x00C2    /* OA1 Control register 0 */
sfrb(OA1CTL0, OA1CTL0_);
#define OA1CTL1_              0x00C3    /* OA1 Control register 1 */
sfrb(OA1CTL1, OA1CTL1_);
#define OA2CTL0_              0x00C4    /* OA2 Control register 0 */
sfrb(OA2CTL0, OA2CTL0_);
#define OA2CTL1_              0x00C5    /* OA2 Control register 1 */
sfrb(OA2CTL1, OA2CTL1_);

#define OAADC0              (0x01)    /* OAx output to ADC12 input channel select 0 */
#define OAADC1              (0x02)    /* OAx output to ADC12 input channel select 1 */
#define OAPM0               (0x04)    /* OAx Power mode select 0 */
#define OAPM1               (0x08)    /* OAx Power mode select 1 */
#define OAP0                (0x10)    /* OAx Non-inverting input select 0 */
#define OAP1                (0x20)    /* OAx Non-inverting input select 1 */
#define OAN0                (0x40)    /* OAx Inverting input select 0 */
#define OAN1                (0x80)    /* OAx Inverting input select 1 */

#define OAPM_0              (0x00)    /* OAx Power mode select: off */
#define OAPM_1              (0x04)    /* OAx Power mode select: slow */
#define OAPM_2              (0x08)    /* OAx Power mode select: meduim */
#define OAPM_3              (0x0C)    /* OAx Power mode select: fast */
#define OAP_0               (0x00)    /* OAx Non-inverting input select 00 */
#define OAP_1               (0x10)    /* OAx Non-inverting input select 01 */
#define OAP_2               (0x20)    /* OAx Non-inverting input select 10 */
#define OAP_3               (0x30)    /* OAx Non-inverting input select 11 */
#define OAN_0               (0x00)    /* OAx Inverting input select 00 */
#define OAN_1               (0x40)    /* OAx Inverting input select 01 */
#define OAN_2               (0x80)    /* OAx Inverting input select 10 */
#define OAN_3               (0xC0)    /* OAx Inverting input select 11 */

#define OARRIP              (0x01)    /* OAx Rail-to-Rail Input off */
//#define Reserved          (0x02)    /* */
#define OAFC0               (0x04)    /* OAx Function control 0 */
#define OAFC1               (0x08)    /* OAx Function control 1 */
#define OAFC2               (0x10)    /* OAx Function control 2 */
#define OAFBR0              (0x20)    /* OAx Feedback resistor select 0 */
#define OAFBR1              (0x40)    /* OAx Feedback resistor select 1 */
#define OAFBR2              (0x80)    /* OAx Feedback resistor select 2 */

#define OAFC_0              (0x00)    /* OAx Function: Gen. Purpose */
#define OAFC_1              (0x04)    /* OAx Function: Unity gain buffer */
#define OAFC_2              (0x08)    /* OAx Function: Reserved */
#define OAFC_3              (0x0C)    /* OAx Function: Comparator */
#define OAFC_4              (0x10)    /* OAx Function: Non-Inverting PGA */
#define OAFC_5              (0x14)    /* OAx Function: Reserved */
#define OAFC_6              (0x18)    /* OAx Function: Inverting PGA */
#define OAFC_7              (0x1C)    /* OAx Function: Differential PGA */
#define OAFBR_0             (0x00)    /* OAx Feedback resistor: Tap 0 */
#define OAFBR_1             (0x20)    /* OAx Feedback resistor: Tap 1 */
#define OAFBR_2             (0x40)    /* OAx Feedback resistor: Tap 2 */
#define OAFBR_3             (0x60)    /* OAx Feedback resistor: Tap 3 */
#define OAFBR_4             (0x80)    /* OAx Feedback resistor: Tap 4 */
#define OAFBR_5             (0xA0)    /* OAx Feedback resistor: Tap 5 */
#define OAFBR_6             (0xC0)    /* OAx Feedback resistor: Tap 6 */
#define OAFBR_7             (0xE0)    /* OAx Feedback resistor: Tap 7 */

/************************************************************
* DIGITAL I/O Port1/2
************************************************************/
#define __MSP430_HAS_PORT1__          /* Definition to show that Module is available */
#define __MSP430_HAS_PORT2__          /* Definition to show that Module is available */

#define P1IN_                 0x0020    /* Port 1 Input */
const_sfrb(P1IN, P1IN_);
#define P1OUT_                0x0021    /* Port 1 Output */
sfrb(P1OUT, P1OUT_);
#define P1DIR_                0x0022    /* Port 1 Direction */
sfrb(P1DIR, P1DIR_);
#define P1IFG_                0x0023    /* Port 1 Interrupt Flag */
sfrb(P1IFG, P1IFG_);
#define P1IES_                0x0024    /* Port 1 Interrupt Edge Select */
sfrb(P1IES, P1IES_);
#define P1IE_                 0x0025    /* Port 1 Interrupt Enable */
sfrb(P1IE, P1IE_);
#define P1SEL_                0x0026    /* Port 1 Selection */
sfrb(P1SEL, P1SEL_);

#define P2IN_                 0x0028    /* Port 2 Input */
const_sfrb(P2IN, P2IN_);
#define P2OUT_                0x0029    /* Port 2 Output */
sfrb(P2OUT, P2OUT_);
#define P2DIR_                0x002A    /* Port 2 Direction */
sfrb(P2DIR, P2DIR_);
#define P2IFG_                0x002B    /* Port 2 Interrupt Flag */
sfrb(P2IFG, P2IFG_);
#define P2IES_                0x002C    /* Port 2 Interrupt Edge Select */
sfrb(P2IES, P2IES_);
#define P2IE_                 0x002D    /* Port 2 Interrupt Enable */
sfrb(P2IE, P2IE_);
#define P2SEL_                0x002E    /* Port 2 Selection */
sfrb(P2SEL, P2SEL_);

/************************************************************
* DIGITAL I/O Port3/4
************************************************************/
#define __MSP430_HAS_PORT3__          /* Definition to show that Module is available */
#define __MSP430_HAS_PORT4__          /* Definition to show that Module is available */

#define P3IN_                 0x0018    /* Port 3 Input */
const_sfrb(P3IN, P3IN_);
#define P3OUT_                0x0019    /* Port 3 Output */
sfrb(P3OUT, P3OUT_);
#define P3DIR_                0x001A    /* Port 3 Direction */
sfrb(P3DIR, P3DIR_);
#define P3SEL_                0x001B    /* Port 3 Selection */
sfrb(P3SEL, P3SEL_);

#define P4IN_                 0x001C    /* Port 4 Input */
const_sfrb(P4IN, P4IN_);
#define P4OUT_                0x001D    /* Port 4 Output */
sfrb(P4OUT, P4OUT_);
#define P4DIR_                0x001E    /* Port 4 Direction */
sfrb(P4DIR, P4DIR_);
#define P4SEL_                0x001F    /* Port 4 Selection */
sfrb(P4SEL, P4SEL_);

/************************************************************
* DIGITAL I/O Port5/6
************************************************************/
#define __MSP430_HAS_PORT5__          /* Definition to show that Module is available */
#define __MSP430_HAS_PORT6__          /* Definition to show that Module is available */

#define P5IN_                 0x0030    /* Port 5 Input */
const_sfrb(P5IN, P5IN_);
#define P5OUT_                0x0031    /* Port 5 Output */
sfrb(P5OUT, P5OUT_);
#define P5DIR_                0x0032    /* Port 5 Direction */
sfrb(P5DIR, P5DIR_);
#define P5SEL_                0x0033    /* Port 5 Selection */
sfrb(P5SEL, P5SEL_);

#define P6IN_                 0x0034    /* Port 6 Input */
const_sfrb(P6IN, P6IN_);
#define P6OUT_                0x0035    /* Port 6 Output */
sfrb(P6OUT, P6OUT_);
#define P6DIR_                0x0036    /* Port 6 Direction */
sfrb(P6DIR, P6DIR_);
#define P6SEL_                0x0037    /* Port 6 Selection */
sfrb(P6SEL, P6SEL_);

/************************************************************
* DIGITAL I/O Port7/8
************************************************************/
#define __MSP430_HAS_PORT7__          /* Definition to show that Module is available */
#define __MSP430_HAS_PORT8__          /* Definition to show that Module is available */
#define __MSP430_HAS_PORTA__          /* Definition to show that Module is available */

#define P7IN_                 0x0038    /* Port 7 Input */
const_sfrb(P7IN, P7IN_);
#define P7OUT_                0x003A    /* Port 7 Output */
sfrb(P7OUT, P7OUT_);
#define P7DIR_                0x003C    /* Port 7 Direction */
sfrb(P7DIR, P7DIR_);
#define P7SEL_                0x003E    /* Port 7 Selection */
sfrb(P7SEL, P7SEL_);

#define P8IN_                 0x0039    /* Port 8 Input */
const_sfrb(P8IN, P8IN_);
#define P8OUT_                0x003B    /* Port 8 Output */
sfrb(P8OUT, P8OUT_);
#define P8DIR_                0x003D    /* Port 8 Direction */
sfrb(P8DIR, P8DIR_);
#define P8SEL_                0x003F    /* Port 8 Selection */
sfrb(P8SEL, P8SEL_);

#define PAIN_                 0x0038    /* Port A Input */
const_sfrw(PAIN, PAIN_);
#define PAOUT_                0x003A    /* Port A Output */
sfrw(PAOUT, PAOUT_);
#define PADIR_                0x003C    /* Port A Direction */
sfrw(PADIR, PADIR_);
#define PASEL_                0x003E    /* Port A Selection */
sfrw(PASEL, PASEL_);


/************************************************************
* DIGITAL I/O Port9/10
************************************************************/
#define __MSP430_HAS_PORT9__          /* Definition to show that Module is available */
#define __MSP430_HAS_PORT10__         /* Definition to show that Module is available */
#define __MSP430_HAS_PORTB__          /* Definition to show that Module is available */

#define P9IN_                 0x0008    /* Port 9 Input */
const_sfrb(P9IN, P9IN_);
#define P9OUT_                0x000A    /* Port 9 Output */
sfrb(P9OUT, P9OUT_);
#define P9DIR_                0x000C    /* Port 9 Direction */
sfrb(P9DIR, P9DIR_);
#define P9SEL_                0x000E    /* Port 9 Selection */
sfrb(P9SEL, P9SEL_);

#define P10IN_                0x0009    /* Port 10 Input */
const_sfrb(P10IN, P10IN_);
#define P10OUT_               0x000B    /* Port 10 Output */
sfrb(P10OUT, P10OUT_);
#define P10DIR_               0x000D    /* Port 10 Direction */
sfrb(P10DIR, P10DIR_);
#define P10SEL_               0x000F    /* Port 10 Selection */
sfrb(P10SEL, P10SEL_);

#define PBIN_                 0x0008    /* Port B Input */
const_sfrw(PBIN, PBIN_);
#define PBOUT_                0x000A    /* Port B Output */
sfrw(PBOUT, PBOUT_);
#define PBDIR_                0x000C    /* Port B Direction */
sfrw(PBDIR, PBDIR_);
#define PBSEL_                0x000E    /* Port B Selection */
sfrw(PBSEL, PBSEL_);


/************************************************************
* Brown-Out, Supply Voltage Supervision (SVS)
************************************************************/
#define __MSP430_HAS_SVS__            /* Definition to show that Module is available */

#define SVSCTL_               0x0056    /* SVS Control */
sfrb(SVSCTL, SVSCTL_);
#define SVSFG               (0x01)    /* SVS Flag */
#define SVSOP               (0x02)    /* SVS output (read only) */
#define SVSON               (0x04)    /* Switches the SVS on/off */
#define PORON               (0x08)    /* Enable POR Generation if Low Voltage */
#define VLD0                (0x10)
#define VLD1                (0x20)
#define VLD2                (0x40)
#define VLD3                (0x80)

#define VLDON               (0x10)
#define VLDOFF              (0x00)
#define VLD_1_8V            (0x10)

/************************************************************
* Timer A3
************************************************************/
#define __MSP430_HAS_TA3__            /* Definition to show that Module is available */

#define TAIV_                 0x012E    /* Timer A Interrupt Vector Word */
const_sfrw(TAIV, TAIV_);
#define TACTL_                0x0160    /* Timer A Control */
sfrw(TACTL, TACTL_);
#define TACCTL0_              0x0162    /* Timer A Capture/Compare Control 0 */
sfrw(TACCTL0, TACCTL0_);
#define TACCTL1_              0x0164    /* Timer A Capture/Compare Control 1 */
sfrw(TACCTL1, TACCTL1_);
#define TACCTL2_              0x0166    /* Timer A Capture/Compare Control 2 */
sfrw(TACCTL2, TACCTL2_);
#define TAR_                  0x0170    /* Timer A Counter Register */
sfrw(TAR, TAR_);
#define TACCR0_               0x0172    /* Timer A Capture/Compare 0 */
sfrw(TACCR0, TACCR0_);
#define TACCR1_               0x0174    /* Timer A Capture/Compare 1 */
sfrw(TACCR1, TACCR1_);
#define TACCR2_               0x0176    /* Timer A Capture/Compare 2 */
sfrw(TACCR2, TACCR2_);

/* Alternate register names */
#define CCTL0               TACCTL0   /* Timer A Capture/Compare Control 0 */
#define CCTL1               TACCTL1   /* Timer A Capture/Compare Control 1 */
#define CCTL2               TACCTL2   /* Timer A Capture/Compare Control 2 */
#define CCR0                TACCR0    /* Timer A Capture/Compare 0 */
#define CCR1                TACCR1    /* Timer A Capture/Compare 1 */
#define CCR2                TACCR2    /* Timer A Capture/Compare 2 */
#define CCTL0_              TACCTL0_  /* Timer A Capture/Compare Control 0 */
#define CCTL1_              TACCTL1_  /* Timer A Capture/Compare Control 1 */
#define CCTL2_              TACCTL2_  /* Timer A Capture/Compare Control 2 */
#define CCR0_               TACCR0_   /* Timer A Capture/Compare 0 */
#define CCR1_               TACCR1_   /* Timer A Capture/Compare 1 */
#define CCR2_               TACCR2_   /* Timer A Capture/Compare 2 */
/* Alternate register names - 5xx style */
#define TA0IV               TAIV      /* Timer A Interrupt Vector Word */
#define TA0CTL              TACTL     /* Timer A Control */
#define TA0CCTL0            TACCTL0   /* Timer A Capture/Compare Control 0 */
#define TA0CCTL1            TACCTL1   /* Timer A Capture/Compare Control 1 */
#define TA0CCTL2            TACCTL2   /* Timer A Capture/Compare Control 2 */
#define TA0R                TAR       /* Timer A Counter Register */
#define TA0CCR0             TACCR0    /* Timer A Capture/Compare 0 */
#define TA0CCR1             TACCR1    /* Timer A Capture/Compare 1 */
#define TA0CCR2             TACCR2    /* Timer A Capture/Compare 2 */
#define TA0IV_              TAIV_     /* Timer A Interrupt Vector Word */
#define TA0CTL_             TACTL_    /* Timer A Control */
#define TA0CCTL0_           TACCTL0_  /* Timer A Capture/Compare Control 0 */
#define TA0CCTL1_           TACCTL1_  /* Timer A Capture/Compare Control 1 */
#define TA0CCTL2_           TACCTL2_  /* Timer A Capture/Compare Control 2 */
#define TA0R_               TAR_      /* Timer A Counter Register */
#define TA0CCR0_            TACCR0_   /* Timer A Capture/Compare 0 */
#define TA0CCR1_            TACCR1_   /* Timer A Capture/Compare 1 */
#define TA0CCR2_            TACCR2_   /* Timer A Capture/Compare 2 */
#define TIMER0_A1_VECTOR    TIMERA1_VECTOR /* Int. Vector: Timer A CC1-2, TA */
#define TIMER0_A0_VECTOR    TIMERA0_VECTOR /* Int. Vector: Timer A CC0 */

#define TASSEL1             (0x0200)  /* Timer A clock source select 1 */
#define TASSEL0             (0x0100)  /* Timer A clock source select 0 */
#define ID1                 (0x0080)  /* Timer A clock input divider 1 */
#define ID0                 (0x0040)  /* Timer A clock input divider 0 */
#define MC1                 (0x0020)  /* Timer A mode control 1 */
#define MC0                 (0x0010)  /* Timer A mode control 0 */
#define TACLR               (0x0004)  /* Timer A counter clear */
#define TAIE                (0x0002)  /* Timer A counter interrupt enable */
#define TAIFG               (0x0001)  /* Timer A counter interrupt flag */

#define MC_0                (0x0000)  /* Timer A mode control: 0 - Stop */
#define MC_1                (0x0010)  /* Timer A mode control: 1 - Up to CCR0 */
#define MC_2                (0x0020)  /* Timer A mode control: 2 - Continous up */
#define MC_3                (0x0030)  /* Timer A mode control: 3 - Up/Down */
#define ID_0                (0x0000)  /* Timer A input divider: 0 - /1 */
#define ID_1                (0x0040)  /* Timer A input divider: 1 - /2 */
#define ID_2                (0x0080)  /* Timer A input divider: 2 - /4 */
#define ID_3                (0x00C0)  /* Timer A input divider: 3 - /8 */
#define TASSEL_0            (0x0000) /* Timer A clock source select: 0 - TACLK */
#define TASSEL_1            (0x0100) /* Timer A clock source select: 1 - ACLK  */
#define TASSEL_2            (0x0200) /* Timer A clock source select: 2 - SMCLK */
#define TASSEL_3            (0x0300) /* Timer A clock source select: 3 - INCLK */

#define CM1                 (0x8000)  /* Capture mode 1 */
#define CM0                 (0x4000)  /* Capture mode 0 */
#define CCIS1               (0x2000)  /* Capture input select 1 */
#define CCIS0               (0x1000)  /* Capture input select 0 */
#define SCS                 (0x0800)  /* Capture sychronize */
#define SCCI                (0x0400)  /* Latched capture signal (read) */
#define CAP                 (0x0100)  /* Capture mode: 1 /Compare mode : 0 */
#define OUTMOD2             (0x0080)  /* Output mode 2 */
#define OUTMOD1             (0x0040)  /* Output mode 1 */
#define OUTMOD0             (0x0020)  /* Output mode 0 */
#define CCIE                (0x0010)  /* Capture/compare interrupt enable */
#define CCI                 (0x0008)  /* Capture input signal (read) */
#define OUT                 (0x0004)  /* PWM Output signal if output mode 0 */
#define COV                 (0x0002)  /* Capture/compare overflow flag */
#define CCIFG               (0x0001)  /* Capture/compare interrupt flag */

#define OUTMOD_0            (0x0000)  /* PWM output mode: 0 - output only */
#define OUTMOD_1            (0x0020)  /* PWM output mode: 1 - set */
#define OUTMOD_2            (0x0040)  /* PWM output mode: 2 - PWM toggle/reset */
#define OUTMOD_3            (0x0060)  /* PWM output mode: 3 - PWM set/reset */
#define OUTMOD_4            (0x0080)  /* PWM output mode: 4 - toggle */
#define OUTMOD_5            (0x00A0)  /* PWM output mode: 5 - Reset */
#define OUTMOD_6            (0x00C0)  /* PWM output mode: 6 - PWM toggle/set */
#define OUTMOD_7            (0x00E0)  /* PWM output mode: 7 - PWM reset/set */
#define CCIS_0              (0x0000) /* Capture input select: 0 - CCIxA */
#define CCIS_1              (0x1000) /* Capture input select: 1 - CCIxB */
#define CCIS_2              (0x2000) /* Capture input select: 2 - GND */
#define CCIS_3              (0x3000) /* Capture input select: 3 - Vcc */
#define CM_0                (0x0000) /* Capture mode: 0 - disabled */
#define CM_1                (0x4000) /* Capture mode: 1 - pos. edge */
#define CM_2                (0x8000) /* Capture mode: 1 - neg. edge */
#define CM_3                (0xC000) /* Capture mode: 1 - both edges */

/* TA3IV Definitions */
#define TAIV_NONE           (0x0000)    /* No Interrupt pending */
#define TAIV_TACCR1         (0x0002)    /* TACCR1_CCIFG */
#define TAIV_TACCR2         (0x0004)    /* TACCR2_CCIFG */
#define TAIV_6              (0x0006)    /* Reserved */
#define TAIV_8              (0x0008)    /* Reserved */
#define TAIV_TAIFG          (0x000A)    /* TAIFG */

/************************************************************
* Timer B7
************************************************************/
#define __MSP430_HAS_TB7__            /* Definition to show that Module is available */

#define TBIV_                 0x011E    /* Timer B Interrupt Vector Word */
const_sfrw(TBIV, TBIV_);
#define TBCTL_                0x0180    /* Timer B Control */
sfrw(TBCTL, TBCTL_);
#define TBCCTL0_              0x0182    /* Timer B Capture/Compare Control 0 */
sfrw(TBCCTL0, TBCCTL0_);
#define TBCCTL1_              0x0184    /* Timer B Capture/Compare Control 1 */
sfrw(TBCCTL1, TBCCTL1_);
#define TBCCTL2_              0x0186    /* Timer B Capture/Compare Control 2 */
sfrw(TBCCTL2, TBCCTL2_);
#define TBCCTL3_              0x0188    /* Timer B Capture/Compare Control 3 */
sfrw(TBCCTL3, TBCCTL3_);
#define TBCCTL4_              0x018A    /* Timer B Capture/Compare Control 4 */
sfrw(TBCCTL4, TBCCTL4_);
#define TBCCTL5_              0x018C    /* Timer B Capture/Compare Control 5 */
sfrw(TBCCTL5, TBCCTL5_);
#define TBCCTL6_              0x018E    /* Timer B Capture/Compare Control 6 */
sfrw(TBCCTL6, TBCCTL6_);
#define TBR_                  0x0190    /* Timer B Counter Register */
sfrw(TBR, TBR_);
#define TBCCR0_               0x0192    /* Timer B Capture/Compare 0 */
sfrw(TBCCR0, TBCCR0_);
#define TBCCR1_               0x0194    /* Timer B Capture/Compare 1 */
sfrw(TBCCR1, TBCCR1_);
#define TBCCR2_               0x0196    /* Timer B Capture/Compare 2 */
sfrw(TBCCR2, TBCCR2_);
#define TBCCR3_               0x0198    /* Timer B Capture/Compare 3 */
sfrw(TBCCR3, TBCCR3_);
#define TBCCR4_               0x019A    /* Timer B Capture/Compare 4 */
sfrw(TBCCR4, TBCCR4_);
#define TBCCR5_               0x019C    /* Timer B Capture/Compare 5 */
sfrw(TBCCR5, TBCCR5_);
#define TBCCR6_               0x019E    /* Timer B Capture/Compare 6 */
sfrw(TBCCR6, TBCCR6_);

/* Alternate register names - 5xx style */
#define TB0IV               TBIV      /* Timer B Interrupt Vector Word */
#define TB0CTL              TBCTL     /* Timer B Control */
#define TB0CCTL0            TBCCTL0   /* Timer B Capture/Compare Control 0 */
#define TB0CCTL1            TBCCTL1   /* Timer B Capture/Compare Control 1 */
#define TB0CCTL2            TBCCTL2   /* Timer B Capture/Compare Control 2 */
#define TB0CCTL3            TBCCTL3   /* Timer B Capture/Compare Control 3 */
#define TB0CCTL4            TBCCTL4   /* Timer B Capture/Compare Control 4 */
#define TB0CCTL5            TBCCTL5   /* Timer B Capture/Compare Control 5 */
#define TB0CCTL6            TBCCTL6   /* Timer B Capture/Compare Control 6 */
#define TB0R                TBR       /* Timer B Counter Register */
#define TB0CCR0             TBCCR0    /* Timer B Capture/Compare 0 */
#define TB0CCR1             TBCCR1    /* Timer B Capture/Compare 1 */
#define TB0CCR2             TBCCR2    /* Timer B Capture/Compare 2 */
#define TB0CCR3             TBCCR3    /* Timer B Capture/Compare 3 */
#define TB0CCR4             TBCCR4    /* Timer B Capture/Compare 4 */
#define TB0CCR5             TBCCR5    /* Timer B Capture/Compare 5 */
#define TB0CCR6             TBCCR6    /* Timer B Capture/Compare 6 */
#define TB0IV_              TBIV_     /* Timer B Interrupt Vector Word */
#define TB0CTL_             TBCTL_    /* Timer B Control */
#define TB0CCTL0_           TBCCTL0_  /* Timer B Capture/Compare Control 0 */
#define TB0CCTL1_           TBCCTL1_  /* Timer B Capture/Compare Control 1 */
#define TB0CCTL2_           TBCCTL2_  /* Timer B Capture/Compare Control 2 */
#define TB0CCTL3_           TBCCTL3_  /* Timer B Capture/Compare Control 3 */
#define TB0CCTL4_           TBCCTL4_  /* Timer B Capture/Compare Control 4 */
#define TB0CCTL5_           TBCCTL5_  /* Timer B Capture/Compare Control 5 */
#define TB0CCTL6_           TBCCTL6_  /* Timer B Capture/Compare Control 6 */
#define TB0R_               TBR_      /* Timer B Counter Register */
#define TB0CCR0_            TBCCR0_   /* Timer B Capture/Compare 0 */
#define TB0CCR1_            TBCCR1_   /* Timer B Capture/Compare 1 */
#define TB0CCR2_            TBCCR2_   /* Timer B Capture/Compare 2 */
#define TB0CCR3_            TBCCR3_   /* Timer B Capture/Compare 3 */
#define TB0CCR4_            TBCCR4_   /* Timer B Capture/Compare 4 */
#define TB0CCR5_            TBCCR5_   /* Timer B Capture/Compare 5 */
#define TB0CCR6_            TBCCR6_   /* Timer B Capture/Compare 6 */

#define TBCLGRP1            (0x4000)  /* Timer B Compare latch load group 1 */
#define TBCLGRP0            (0x2000)  /* Timer B Compare latch load group 0 */
#define CNTL1               (0x1000)  /* Counter lenght 1 */
#define CNTL0               (0x0800)  /* Counter lenght 0 */
#define TBSSEL1             (0x0200)  /* Clock source 1 */
#define TBSSEL0             (0x0100)  /* Clock source 0 */
#define TBCLR               (0x0004)  /* Timer B counter clear */
#define TBIE                (0x0002)  /* Timer B interrupt enable */
#define TBIFG               (0x0001)  /* Timer B interrupt flag */

#define SHR1                (0x4000)  /* Timer B Compare latch load group 1 */
#define SHR0                (0x2000)  /* Timer B Compare latch load group 0 */

#define TBSSEL_0            (0x0000)  /* Clock Source: TBCLK */
#define TBSSEL_1            (0x0100)  /* Clock Source: ACLK  */
#define TBSSEL_2            (0x0200)  /* Clock Source: SMCLK */
#define TBSSEL_3            (0x0300)  /* Clock Source: INCLK */
#define CNTL_0              (0x0000)  /* Counter lenght: 16 bit */
#define CNTL_1              (0x0800)  /* Counter lenght: 12 bit */
#define CNTL_2              (0x1000)  /* Counter lenght: 10 bit */
#define CNTL_3              (0x1800)  /* Counter lenght:  8 bit */
#define SHR_0               (0x0000)  /* Timer B Group: 0 - individually */
#define SHR_1               (0x2000)  /* Timer B Group: 1 - 3 groups (1-2, 3-4, 5-6) */
#define SHR_2               (0x4000)  /* Timer B Group: 2 - 2 groups (1-3, 4-6)*/
#define SHR_3               (0x6000)  /* Timer B Group: 3 - 1 group (all) */
#define TBCLGRP_0           (0x0000)  /* Timer B Group: 0 - individually */
#define TBCLGRP_1           (0x2000)  /* Timer B Group: 1 - 3 groups (1-2, 3-4, 5-6) */
#define TBCLGRP_2           (0x4000)  /* Timer B Group: 2 - 2 groups (1-3, 4-6)*/
#define TBCLGRP_3           (0x6000)  /* Timer B Group: 3 - 1 group (all) */

/* Additional Timer B Control Register bits are defined in Timer A */
#define CLLD1               (0x0400)  /* Compare latch load source 1 */
#define CLLD0               (0x0200)  /* Compare latch load source 0 */

#define SLSHR1              (0x0400)  /* Compare latch load source 1 */
#define SLSHR0              (0x0200)  /* Compare latch load source 0 */

#define SLSHR_0             (0x0000)  /* Compare latch load sourec : 0 - immediate */
#define SLSHR_1             (0x0200)  /* Compare latch load sourec : 1 - TBR counts to 0 */
#define SLSHR_2             (0x0400)  /* Compare latch load sourec : 2 - up/down */
#define SLSHR_3             (0x0600)  /* Compare latch load sourec : 3 - TBR counts to TBCTL0 */

#define CLLD_0              (0x0000)  /* Compare latch load sourec : 0 - immediate */
#define CLLD_1              (0x0200)  /* Compare latch load sourec : 1 - TBR counts to 0 */
#define CLLD_2              (0x0400)  /* Compare latch load sourec : 2 - up/down */
#define CLLD_3              (0x0600)  /* Compare latch load sourec : 3 - TBR counts to TBCTL0 */

/* TB7IV Definitions */
#define TBIV_NONE           (0x0000)    /* No Interrupt pending */
#define TBIV_TBCCR1         (0x0002)    /* TBCCR1_CCIFG */
#define TBIV_TBCCR2         (0x0004)    /* TBCCR2_CCIFG */
#define TBIV_TBCCR3         (0x0006)    /* TBCCR3_CCIFG */
#define TBIV_TBCCR4         (0x0008)    /* TBCCR4_CCIFG */
#define TBIV_TBCCR5         (0x000A)    /* TBCCR3_CCIFG */
#define TBIV_TBCCR6         (0x000C)    /* TBCCR4_CCIFG */
#define TBIV_TBIFG          (0x000E)    /* TBIFG */

/************************************************************
* USCI
************************************************************/
#define __MSP430_HAS_USCI__           /* Definition to show that Module is available */

#define UCA0CTL0_             0x0060    /* USCI A0 Control Register 0 */
sfrb(UCA0CTL0, UCA0CTL0_);
#define UCA0CTL1_             0x0061    /* USCI A0 Control Register 1 */
sfrb(UCA0CTL1, UCA0CTL1_);
#define UCA0BR0_              0x0062    /* USCI A0 Baud Rate 0 */
sfrb(UCA0BR0, UCA0BR0_);
#define UCA0BR1_              0x0063    /* USCI A0 Baud Rate 1 */
sfrb(UCA0BR1, UCA0BR1_);
#define UCA0MCTL_             0x0064    /* USCI A0 Modulation Control */
sfrb(UCA0MCTL, UCA0MCTL_);
#define UCA0STAT_             0x0065    /* USCI A0 Status Register */
sfrb(UCA0STAT, UCA0STAT_);
#define UCA0RXBUF_            0x0066    /* USCI A0 Receive Buffer */
const_sfrb(UCA0RXBUF, UCA0RXBUF_);
#define UCA0TXBUF_            0x0067    /* USCI A0 Transmit Buffer */
sfrb(UCA0TXBUF, UCA0TXBUF_);
#define UCA0ABCTL_            0x005D    /* USCI A0 LIN Control */
sfrb(UCA0ABCTL, UCA0ABCTL_);
#define UCA0IRTCTL_           0x005E    /* USCI A0 IrDA Transmit Control */
sfrb(UCA0IRTCTL, UCA0IRTCTL_);
#define UCA0IRRCTL_           0x005F    /* USCI A0 IrDA Receive Control */
sfrb(UCA0IRRCTL, UCA0IRRCTL_);



#define UCB0CTL0_             0x0068    /* USCI B0 Control Register 0 */
sfrb(UCB0CTL0, UCB0CTL0_);
#define UCB0CTL1_             0x0069    /* USCI B0 Control Register 1 */
sfrb(UCB0CTL1, UCB0CTL1_);
#define UCB0BR0_              0x006A    /* USCI B0 Baud Rate 0 */
sfrb(UCB0BR0, UCB0BR0_);
#define UCB0BR1_              0x006B    /* USCI B0 Baud Rate 1 */
sfrb(UCB0BR1, UCB0BR1_);
#define UCB0I2CIE_            0x006C    /* USCI B0 I2C Interrupt Enable Register */
sfrb(UCB0I2CIE, UCB0I2CIE_);
#define UCB0STAT_             0x006D    /* USCI B0 Status Register */
sfrb(UCB0STAT, UCB0STAT_);
#define UCB0RXBUF_            0x006E    /* USCI B0 Receive Buffer */
const_sfrb(UCB0RXBUF, UCB0RXBUF_);
#define UCB0TXBUF_            0x006F    /* USCI B0 Transmit Buffer */
sfrb(UCB0TXBUF, UCB0TXBUF_);
#define UCB0I2COA_            0x0118    /* USCI B0 I2C Own Address */
sfrw(UCB0I2COA, UCB0I2COA_);
#define UCB0I2CSA_            0x011A    /* USCI B0 I2C Slave Address */
sfrw(UCB0I2CSA, UCB0I2CSA_);

// UART-Mode Bits
#define UCPEN               (0x80)    /* Async. Mode: Parity enable */
#define UCPAR               (0x40)    /* Async. Mode: Parity     0:odd / 1:even */
#define UCMSB               (0x20)    /* Async. Mode: MSB first  0:LSB / 1:MSB */
#define UC7BIT              (0x10)    /* Async. Mode: Data Bits  0:8-bits / 1:7-bits */
#define UCSPB               (0x08)    /* Async. Mode: Stop Bits  0:one / 1: two */
#define UCMODE1             (0x04)    /* Async. Mode: USCI Mode 1 */
#define UCMODE0             (0x02)    /* Async. Mode: USCI Mode 0 */
#define UCSYNC              (0x01)    /* Sync-Mode  0:UART-Mode / 1:SPI-Mode */

// SPI-Mode Bits
#define UCCKPH              (0x80)    /* Sync. Mode: Clock Phase */
#define UCCKPL              (0x40)    /* Sync. Mode: Clock Polarity */
#define UCMST               (0x08)    /* Sync. Mode: Master Select */

// I2C-Mode Bits
#define UCA10               (0x80)    /* 10-bit Address Mode */
#define UCSLA10             (0x40)    /* 10-bit Slave Address Mode */
#define UCMM                (0x20)    /* Multi-Master Environment */
//#define res               (0x10)    /* reserved */
#define UCMODE_0            (0x00)    /* Sync. Mode: USCI Mode: 0 */
#define UCMODE_1            (0x02)    /* Sync. Mode: USCI Mode: 1 */
#define UCMODE_2            (0x04)    /* Sync. Mode: USCI Mode: 2 */
#define UCMODE_3            (0x06)    /* Sync. Mode: USCI Mode: 3 */

// UART-Mode Bits
#define UCSSEL1             (0x80)    /* USCI 0 Clock Source Select 1 */
#define UCSSEL0             (0x40)    /* USCI 0 Clock Source Select 0 */
#define UCRXEIE             (0x20)    /* RX Error interrupt enable */
#define UCBRKIE             (0x10)    /* Break interrupt enable */
#define UCDORM              (0x08)    /* Dormant (Sleep) Mode */
#define UCTXADDR            (0x04)    /* Send next Data as Address */
#define UCTXBRK             (0x02)    /* Send next Data as Break */
#define UCSWRST             (0x01)    /* USCI Software Reset */

// SPI-Mode Bits
//#define res               (0x20)    /* reserved */
//#define res               (0x10)    /* reserved */
//#define res               (0x08)    /* reserved */
//#define res               (0x04)    /* reserved */
//#define res               (0x02)    /* reserved */

// I2C-Mode Bits
//#define res               (0x20)    /* reserved */
#define UCTR                (0x10)    /* Transmit/Receive Select/Flag */
#define UCTXNACK            (0x08)    /* Transmit NACK */
#define UCTXSTP             (0x04)    /* Transmit STOP */
#define UCTXSTT             (0x02)    /* Transmit START */
#define UCSSEL_0            (0x00)    /* USCI 0 Clock Source: 0 */
#define UCSSEL_1            (0x40)    /* USCI 0 Clock Source: 1 */
#define UCSSEL_2            (0x80)    /* USCI 0 Clock Source: 2 */
#define UCSSEL_3            (0xC0)    /* USCI 0 Clock Source: 3 */

#define UCBRF3              (0x80)    /* USCI First Stage Modulation Select 3 */
#define UCBRF2              (0x40)    /* USCI First Stage Modulation Select 2 */
#define UCBRF1              (0x20)    /* USCI First Stage Modulation Select 1 */
#define UCBRF0              (0x10)    /* USCI First Stage Modulation Select 0 */
#define UCBRS2              (0x08)    /* USCI Second Stage Modulation Select 2 */
#define UCBRS1              (0x04)    /* USCI Second Stage Modulation Select 1 */
#define UCBRS0              (0x02)    /* USCI Second Stage Modulation Select 0 */
#define UCOS16              (0x01)    /* USCI 16-times Oversampling enable */

#define UCBRF_0             (0x00)    /* USCI First Stage Modulation: 0 */
#define UCBRF_1             (0x10)    /* USCI First Stage Modulation: 1 */
#define UCBRF_2             (0x20)    /* USCI First Stage Modulation: 2 */
#define UCBRF_3             (0x30)    /* USCI First Stage Modulation: 3 */
#define UCBRF_4             (0x40)    /* USCI First Stage Modulation: 4 */
#define UCBRF_5             (0x50)    /* USCI First Stage Modulation: 5 */
#define UCBRF_6             (0x60)    /* USCI First Stage Modulation: 6 */
#define UCBRF_7             (0x70)    /* USCI First Stage Modulation: 7 */
#define UCBRF_8             (0x80)    /* USCI First Stage Modulation: 8 */
#define UCBRF_9             (0x90)    /* USCI First Stage Modulation: 9 */
#define UCBRF_10            (0xA0)    /* USCI First Stage Modulation: A */
#define UCBRF_11            (0xB0)    /* USCI First Stage Modulation: B */
#define UCBRF_12            (0xC0)    /* USCI First Stage Modulation: C */
#define UCBRF_13            (0xD0)    /* USCI First Stage Modulation: D */
#define UCBRF_14            (0xE0)    /* USCI First Stage Modulation: E */
#define UCBRF_15            (0xF0)    /* USCI First Stage Modulation: F */

#define UCBRS_0             (0x00)    /* USCI Second Stage Modulation: 0 */
#define UCBRS_1             (0x02)    /* USCI Second Stage Modulation: 1 */
#define UCBRS_2             (0x04)    /* USCI Second Stage Modulation: 2 */
#define UCBRS_3             (0x06)    /* USCI Second Stage Modulation: 3 */
#define UCBRS_4             (0x08)    /* USCI Second Stage Modulation: 4 */
#define UCBRS_5             (0x0A)    /* USCI Second Stage Modulation: 5 */
#define UCBRS_6             (0x0C)    /* USCI Second Stage Modulation: 6 */
#define UCBRS_7             (0x0E)    /* USCI Second Stage Modulation: 7 */

#define UCLISTEN            (0x80)    /* USCI Listen mode */
#define UCFE                (0x40)    /* USCI Frame Error Flag */
#define UCOE                (0x20)    /* USCI Overrun Error Flag */
#define UCPE                (0x10)    /* USCI Parity Error Flag */
#define UCBRK               (0x08)    /* USCI Break received */
#define UCRXERR             (0x04)    /* USCI RX Error Flag */
#define UCADDR              (0x02)    /* USCI Address received Flag */
#define UCBUSY              (0x01)    /* USCI Busy Flag */
#define UCIDLE              (0x02)    /* USCI Idle line detected Flag */

//#define res               (0x80)    /* reserved */
//#define res               (0x40)    /* reserved */
//#define res               (0x20)    /* reserved */
//#define res               (0x10)    /* reserved */
#define UCNACKIE            (0x08)    /* NACK Condition interrupt enable */
#define UCSTPIE             (0x04)    /* STOP Condition interrupt enable */
#define UCSTTIE             (0x02)    /* START Condition interrupt enable */
#define UCALIE              (0x01)    /* Arbitration Lost interrupt enable */


#define UCSCLLOW            (0x40)    /* SCL low */
#define UCGC                (0x20)    /* General Call address received Flag */
#define UCBBUSY             (0x10)    /* Bus Busy Flag */
#define UCNACKIFG           (0x08)    /* NAK Condition interrupt Flag */
#define UCSTPIFG            (0x04)    /* STOP Condition interrupt Flag */
#define UCSTTIFG            (0x02)    /* START Condition interrupt Flag */
#define UCALIFG             (0x01)    /* Arbitration Lost interrupt Flag */

#define UCIRTXPL5           (0x80)    /* IRDA Transmit Pulse Length 5 */
#define UCIRTXPL4           (0x40)    /* IRDA Transmit Pulse Length 4 */
#define UCIRTXPL3           (0x20)    /* IRDA Transmit Pulse Length 3 */
#define UCIRTXPL2           (0x10)    /* IRDA Transmit Pulse Length 2 */
#define UCIRTXPL1           (0x08)    /* IRDA Transmit Pulse Length 1 */
#define UCIRTXPL0           (0x04)    /* IRDA Transmit Pulse Length 0 */
#define UCIRTXCLK           (0x02)    /* IRDA Transmit Pulse Clock Select */
#define UCIREN              (0x01)    /* IRDA Encoder/Decoder enable */

#define UCIRRXFL5           (0x80)    /* IRDA Receive Filter Length 5 */
#define UCIRRXFL4           (0x40)    /* IRDA Receive Filter Length 4 */
#define UCIRRXFL3           (0x20)    /* IRDA Receive Filter Length 3 */
#define UCIRRXFL2           (0x10)    /* IRDA Receive Filter Length 2 */
#define UCIRRXFL1           (0x08)    /* IRDA Receive Filter Length 1 */
#define UCIRRXFL0           (0x04)    /* IRDA Receive Filter Length 0 */
#define UCIRRXPL            (0x02)    /* IRDA Receive Input Polarity */
#define UCIRRXFE            (0x01)    /* IRDA Receive Filter enable */

//#define res               (0x80)    /* reserved */
//#define res               (0x40)    /* reserved */
#define UCDELIM1            (0x20)    /* Break Sync Delimiter 1 */
#define UCDELIM0            (0x10)    /* Break Sync Delimiter 0 */
#define UCSTOE              (0x08)    /* Sync-Field Timeout error */
#define UCBTOE              (0x04)    /* Break Timeout error */
//#define res               (0x02)    /* reserved */
#define UCABDEN             (0x01)    /* Auto Baud Rate detect enable */

#define UCGCEN              (0x8000)  /* I2C General Call enable */
#define UCOA9               (0x0200)  /* I2C Own Address 9 */
#define UCOA8               (0x0100)  /* I2C Own Address 8 */
#define UCOA7               (0x0080)  /* I2C Own Address 7 */
#define UCOA6               (0x0040)  /* I2C Own Address 6 */
#define UCOA5               (0x0020)  /* I2C Own Address 5 */
#define UCOA4               (0x0010)  /* I2C Own Address 4 */
#define UCOA3               (0x0008)  /* I2C Own Address 3 */
#define UCOA2               (0x0004)  /* I2C Own Address 2 */
#define UCOA1               (0x0002)  /* I2C Own Address 1 */
#define UCOA0               (0x0001)  /* I2C Own Address 0 */

#define UCSA9               (0x0200)  /* I2C Slave Address 9 */
#define UCSA8               (0x0100)  /* I2C Slave Address 8 */
#define UCSA7               (0x0080)  /* I2C Slave Address 7 */
#define UCSA6               (0x0040)  /* I2C Slave Address 6 */
#define UCSA5               (0x0020)  /* I2C Slave Address 5 */
#define UCSA4               (0x0010)  /* I2C Slave Address 4 */
#define UCSA3               (0x0008)  /* I2C Slave Address 3 */
#define UCSA2               (0x0004)  /* I2C Slave Address 2 */
#define UCSA1               (0x0002)  /* I2C Slave Address 1 */
#define UCSA0               (0x0001)  /* I2C Slave Address 0 */

/************************************************************
* USART
************************************************************/

/* UxCTL */
#define PENA                (0x80)        /* Parity enable */
#define PEV                 (0x40)        /* Parity 0:odd / 1:even */
#define SPB                 (0x20)        /* Stop Bits 0:one / 1: two */
#define CHAR                (0x10)        /* Data 0:7-bits / 1:8-bits */
#define LISTEN              (0x08)        /* Listen mode */
#define SYNC                (0x04)        /* UART / SPI mode */
#define MM                  (0x02)        /* Master Mode off/on */
#define SWRST               (0x01)        /* USART Software Reset */

/* UxTCTL */
#define CKPH                (0x80)        /* SPI: Clock Phase */
#define CKPL                (0x40)        /* Clock Polarity */
#define SSEL1               (0x20)        /* Clock Source Select 1 */
#define SSEL0               (0x10)        /* Clock Source Select 0 */
#define URXSE               (0x08)        /* Receive Start edge select */
#define TXWAKE              (0x04)        /* TX Wake up mode */
#define STC                 (0x02)        /* SPI: STC enable 0:on / 1:off */
#define TXEPT               (0x01)        /* TX Buffer empty */

/* UxRCTL */
#define FE                  (0x80)        /* Frame Error */
#define PE                  (0x40)        /* Parity Error */
#define OE                  (0x20)        /* Overrun Error */
#define BRK                 (0x10)        /* Break detected */
#define URXEIE              (0x08)        /* RX Error interrupt enable */
#define URXWIE              (0x04)        /* RX Wake up interrupt enable */
#define RXWAKE              (0x02)        /* RX Wake up detect */
#define RXERR               (0x01)        /* RX Error Error */

/************************************************************
* USART 1
************************************************************/
#define __MSP430_HAS_UART1__          /* Definition to show that Module is available */

#define U1CTL_                0x0078    /* USART 1 Control */
sfrb(U1CTL, U1CTL_);
#define U1TCTL_               0x0079    /* USART 1 Transmit Control */
sfrb(U1TCTL, U1TCTL_);
#define U1RCTL_               0x007A    /* USART 1 Receive Control */
sfrb(U1RCTL, U1RCTL_);
#define U1MCTL_               0x007B    /* USART 1 Modulation Control */
sfrb(U1MCTL, U1MCTL_);
#define U1BR0_                0x007C    /* USART 1 Baud Rate 0 */
sfrb(U1BR0, U1BR0_);
#define U1BR1_                0x007D    /* USART 1 Baud Rate 1 */
sfrb(U1BR1, U1BR1_);
#define U1RXBUF_              0x007E    /* USART 1 Receive Buffer */
const_sfrb(U1RXBUF, U1RXBUF_);
#define U1TXBUF_              0x007F    /* USART 1 Transmit Buffer */
sfrb(U1TXBUF, U1TXBUF_);

/* Alternate register names */

#define UCTL1               U1CTL     /* USART 1 Control */
#define UTCTL1              U1TCTL    /* USART 1 Transmit Control */
#define URCTL1              U1RCTL    /* USART 1 Receive Control */
#define UMCTL1              U1MCTL    /* USART 1 Modulation Control */
#define UBR01               U1BR0     /* USART 1 Baud Rate 0 */
#define UBR11               U1BR1     /* USART 1 Baud Rate 1 */
#define RXBUF1              U1RXBUF   /* USART 1 Receive Buffer */
#define TXBUF1              U1TXBUF   /* USART 1 Transmit Buffer */
#define UCTL1_              U1CTL_    /* USART 1 Control */
#define UTCTL1_             U1TCTL_   /* USART 1 Transmit Control */
#define URCTL1_             U1RCTL_   /* USART 1 Receive Control */
#define UMCTL1_             U1MCTL_   /* USART 1 Modulation Control */
#define UBR01_              U1BR0_    /* USART 1 Baud Rate 0 */
#define UBR11_              U1BR1_    /* USART 1 Baud Rate 1 */
#define RXBUF1_             U1RXBUF_  /* USART 1 Receive Buffer */
#define TXBUF1_             U1TXBUF_  /* USART 1 Transmit Buffer */
#define UCTL_1              U1CTL     /* USART 1 Control */
#define UTCTL_1             U1TCTL    /* USART 1 Transmit Control */
#define URCTL_1             U1RCTL    /* USART 1 Receive Control */
#define UMCTL_1             U1MCTL    /* USART 1 Modulation Control */
#define UBR0_1              U1BR0     /* USART 1 Baud Rate 0 */
#define UBR1_1              U1BR1     /* USART 1 Baud Rate 1 */
#define RXBUF_1             U1RXBUF   /* USART 1 Receive Buffer */
#define TXBUF_1             U1TXBUF   /* USART 1 Transmit Buffer */
#define UCTL_1_             U1CTL_    /* USART 1 Control */
#define UTCTL_1_            U1TCTL_   /* USART 1 Transmit Control */
#define URCTL_1_            U1RCTL_   /* USART 1 Receive Control */
#define UMCTL_1_            U1MCTL_   /* USART 1 Modulation Control */
#define UBR0_1_             U1BR0_    /* USART 1 Baud Rate 0 */
#define UBR1_1_             U1BR1_    /* USART 1 Baud Rate 1 */
#define RXBUF_1_            U1RXBUF_  /* USART 1 Receive Buffer */
#define TXBUF_1_            U1TXBUF_  /* USART 1 Transmit Buffer */

/************************************************************
* WATCHDOG TIMER
************************************************************/
#define __MSP430_HAS_WDT__            /* Definition to show that Module is available */

#define WDTCTL_               0x0120    /* Watchdog Timer Control */
sfrw(WDTCTL, WDTCTL_);
/* The bit names have been prefixed with "WDT" */
#define WDTIS0              (0x0001)
#define WDTIS1              (0x0002)
#define WDTSSEL             (0x0004)
#define WDTCNTCL            (0x0008)
#define WDTTMSEL            (0x0010)
#define WDTNMI              (0x0020)
#define WDTNMIES            (0x0040)
#define WDTHOLD             (0x0080)

#define WDTPW               (0x5A00)

/* WDT-interval times [1ms] coded with Bits 0-2 */
/* WDT is clocked by fSMCLK (assumed 1MHz) */
#define WDT_MDLY_32         (WDTPW+WDTTMSEL+WDTCNTCL)                         /* 32ms interval (default) */
#define WDT_MDLY_8          (WDTPW+WDTTMSEL+WDTCNTCL+WDTIS0)                  /* 8ms     " */
#define WDT_MDLY_0_5        (WDTPW+WDTTMSEL+WDTCNTCL+WDTIS1)                  /* 0.5ms   " */
#define WDT_MDLY_0_064      (WDTPW+WDTTMSEL+WDTCNTCL+WDTIS1+WDTIS0)           /* 0.064ms " */
/* WDT is clocked by fACLK (assumed 32KHz) */
#define WDT_ADLY_1000       (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL)                 /* 1000ms  " */
#define WDT_ADLY_250        (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS0)          /* 250ms   " */
#define WDT_ADLY_16         (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1)          /* 16ms    " */
#define WDT_ADLY_1_9        (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0)   /* 1.9ms   " */
/* Watchdog mode -> reset after expired time */
/* WDT is clocked by fSMCLK (assumed 1MHz) */
#define WDT_MRST_32         (WDTPW+WDTCNTCL)                                  /* 32ms interval (default) */
#define WDT_MRST_8          (WDTPW+WDTCNTCL+WDTIS0)                           /* 8ms     " */
#define WDT_MRST_0_5        (WDTPW+WDTCNTCL+WDTIS1)                           /* 0.5ms   " */
#define WDT_MRST_0_064      (WDTPW+WDTCNTCL+WDTIS1+WDTIS0)                    /* 0.064ms " */
/* WDT is clocked by fACLK (assumed 32KHz) */
#define WDT_ARST_1000       (WDTPW+WDTCNTCL+WDTSSEL)                          /* 1000ms  " */
#define WDT_ARST_250        (WDTPW+WDTCNTCL+WDTSSEL+WDTIS0)                   /* 250ms   " */
#define WDT_ARST_16         (WDTPW+WDTCNTCL+WDTSSEL+WDTIS1)                   /* 16ms    " */
#define WDT_ARST_1_9        (WDTPW+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0)            /* 1.9ms   " */

/* INTERRUPT CONTROL */
/* These two bits are defined in the Special Function Registers */
/* #define WDTIE               0x01 */
/* #define WDTIFG              0x01 */

/************************************************************
* Interrupt Vectors (offset from 0xFFC0)
************************************************************/


#define DAC12_VECTOR        (0x001C) /* 0xFFDC DAC 12 */
#define DMA_VECTOR          (0x001E) /* 0xFFDE DMA */
#define BASICTIMER_VECTOR   (0x0020) /* 0xFFE0 Basic Timer / RTC */
#define PORT2_VECTOR        (0x0022) /* 0xFFE2 Port 2 */
#define USART1TX_VECTOR     (0x0024) /* 0xFFE4 USART 1 Transmit */
#define USART1RX_VECTOR     (0x0026) /* 0xFFE6 USART 1 Receive */
#define PORT1_VECTOR        (0x0028) /* 0xFFE8 Port 1 */
#define TIMERA1_VECTOR      (0x002A) /* 0xFFEA Timer A CC1-2, TA */
#define TIMERA0_VECTOR      (0x002C) /* 0xFFEC Timer A CC0 */
#define ADC12_VECTOR        (0x002E) /* 0xFFEE ADC */
#define USCIAB0TX_VECTOR    (0x0030) /* 0xFFF0 USCI A0/B0 Transmit */
#define USCIAB0RX_VECTOR    (0x0032) /* 0xFFF2 USCI A0/B0 Receive */
#define WDT_VECTOR          (0x0034) /* 0xFFF4 Watchdog Timer */
#define COMPARATORA_VECTOR  (0x0036) /* 0xFFF6 Comparator A */
#define TIMERB1_VECTOR      (0x0038) /* 0xFFF8 Timer B CC1-2, TB */
#define TIMERB0_VECTOR      (0x003A) /* 0xFFFA Timer B CC0 */
#define NMI_VECTOR          (0x003C) /* 0xFFFC Non-maskable */
#define RESET_VECTOR        (0x003E) /* 0xFFFE Reset [Highest Priority] */

/************************************************************
* End of Modules
************************************************************/
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* #ifndef __msp430xG46x */

