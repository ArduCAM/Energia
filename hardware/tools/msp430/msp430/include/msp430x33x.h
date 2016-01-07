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
* MSP430x33x devices.
*
* Texas Instruments, Version 2.3
*
* Rev. 1.2, Added definition of USPIE.
*
* Rev. 1.3, Removed leading 0 to aviod interpretation as octal
*            values under C
*           Changed definition of LPM4 bits (device effect not changed)
*           Corrected LPMx_EXIT to reference new intrinsic _BIC_SR_IRQ
*           The file contents were reordered
*           Changed TAIV to be read-only
* Rev. 1.4, Enclose all #define statements with parentheses
* Rev. 1.5, Added sfrb for TCDAT and TCPLD
* Rev. 1.6, Defined vectors for USART (in addition to UART)
* Rev. 1.7, Removed incorrect label 'BTRESET'
* Rev. 2.1, Alignment of defintions in Users Guide and of version numbers
* Rev. 2.2, Removed unused def of TASSEL2
* Rev. 2.3, Removed definitions for BTRESET
*
********************************************************************/

#ifndef __msp430x33x
#define __msp430x33x

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
* PERIPHERAL FILE MAP
************************************************************/

/************************************************************
* SPECIAL FUNCTION REGISTER ADDRESSES + CONTROL BITS
************************************************************/

#define IE1_                  0x0000    /* Interrupt Enable 1 */
sfrb(IE1, IE1_);
#define WDTIE               (0x01)
#define OFIE                (0x02)
#define P0IE_0              (0x04)
#define P0IE_1              (0x08)

#define IFG1_                 0x0002    /* Interrupt Flag 1 */
sfrb(IFG1, IFG1_);
#define WDTIFG              (0x01)
#define OFIFG               (0x02)
#define P0IFG_0             (0x04)
#define P0IFG_1             (0x08)
#define NMIIFG              (0x10)

#define IE2_                  0x0001    /* Interrupt Enable 2 */
sfrb(IE2, IE2_);
#define URXIE               (0x01)
#define UTXIE               (0x02)
#define TPIE                (0x08)
#define BTIE                (0x80)

#define IFG2_                 0x0003    /* Interrupt Flag 2 */
sfrb(IFG2, IFG2_);
#define URXIFG              (0x01)
#define UTXIFG              (0x02)
#define BTIFG               (0x80)

#define ME2_                  0x0005    /* Module Enable 2 */
sfrb(ME2, ME2_);
#define URXE                (0x01)
#define USPIE               (0x01)
#define UTXE                (0x02)

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
* DIGITAL I/O PORT0
************************************************************/
#define __MSP430_HAS_PORT0__          /* Definition to show that Module is available */

#define P0IN_                 0x0010    /* Port 0 Input */
const_sfrb(P0IN, P0IN_);
#define P0IN_0              (0x01)
#define P0IN_1              (0x02)
#define P0IN_2              (0x04)
#define P0IN_3              (0x08)
#define P0IN_4              (0x10)
#define P0IN_5              (0x20)
#define P0IN_6              (0x40)
#define P0IN_7              (0x80)

#define P0OUT_                0x0011    /* Port 0 Output */
sfrb(P0OUT, P0OUT_);
#define P0OUT_0             (0x01)
#define P0OUT_1             (0x02)
#define P0OUT_2             (0x04)
#define P0OUT_3             (0x08)
#define P0OUT_4             (0x10)
#define P0OUT_5             (0x20)
#define P0OUT_6             (0x40)
#define P0OUT_7             (0x80)

#define P0DIR_                0x0012    /* Port 0 Direction */
sfrb(P0DIR, P0DIR_);
#define P0DIR_0             (0x01)
#define P0DIR_1             (0x02)
#define P0DIR_2             (0x04)
#define P0DIR_3             (0x08)
#define P0DIR_4             (0x10)
#define P0DIR_5             (0x20)
#define P0DIR_6             (0x40)
#define P0DIR_7             (0x80)

#define P0IFG_                0x0013    /* Port 0 Interrupt Flag */
sfrb(P0IFG, P0IFG_);
/* These two bits are defined in Interrupt Flag 1 */
/* #define P0IFG_0             0x01 */
/* #define P0IFG_1             0x02 */
#define P0IFG_2             (0x04)
#define P0IFG_3             (0x08)
#define P0IFG_4             (0x10)
#define P0IFG_5             (0x20)
#define P0IFG_6             (0x40)
#define P0IFG_7             (0x80)

#define P0IES_                0x0014    /* Port 0 Interrupt Edge Select */
sfrb(P0IES, P0IES_);
#define P0IES_0             (0x01)
#define P0IES_1             (0x02)
#define P0IES_2             (0x04)
#define P0IES_3             (0x08)
#define P0IES_4             (0x10)
#define P0IES_5             (0x20)
#define P0IES_6             (0x40)
#define P0IES_7             (0x80)

#define P0IE_                 0x0015    /* Port 0 Interrupt Enable */
sfrb(P0IE, P0IE_);
/* These two bits are defined in Interrupt Enable 1 */
/* #define P0IE_0              0x01 */
/* #define P0IE_1              0x02 */
#define P0IE_2              (0x04)
#define P0IE_3              (0x08)
#define P0IE_4              (0x10)
#define P0IE_5              (0x20)
#define P0IE_6              (0x40)
#define P0IE_7              (0x80)

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
* BASIC TIMER
************************************************************/
#define __MSP430_HAS_BT__             /* Definition to show that Module is available */

#define BTCTL_                0x0040    /* Basic Timer Control */
sfrb(BTCTL, BTCTL_);
/* The bit names have been prefixed with "BT" */
#define BTIP0               (0x01)
#define BTIP1               (0x02)
#define BTIP2               (0x04)
#define BTFRFQ0             (0x08)
#define BTFRFQ1             (0x10)
#define BTDIV               (0x20)                     /* fCLK2 = ACLK:256 */
#define BTHOLD              (0x40)                     /* BT1 is held if this bit is set */
#define BTSSEL              (0x80)                     /* fBT = fMCLK (main clock) */

#define BTCNT1_               0x0046    /* Basic Timer Count 1 */
sfrb(BTCNT1, BTCNT1_);
#define BTCNT2_               0x0047    /* Basic Timer Count 2 */
sfrb(BTCNT2, BTCNT2_);

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
/* Frequency of LCD coded with Bits 3-4 */
#define BT_fLCD_DIV32       (0x00)                    /* fLCD = fACLK:32 (default) */
#define BT_fLCD_DIV64       (BTFRFQ0)                 /* fLCD = fACLK:64 */
#define BT_fLCD_DIV128      (BTFRFQ1)                 /* fLCD = fACLK:128 */
#define BT_fLCD_DIV256      (BTFRFQ1+BTFRFQ0)         /* fLCD = fACLK:256 */
/* LCD frequency values with fBT=fACLK */
#define BT_fLCD_1K          (0x00)                    /* fACLK:32 (default) */
#define BT_fLCD_512         (BTFRFQ0)                 /* fACLK:64 */
#define BT_fLCD_256         (BTFRFQ1)                 /* fACLK:128 */
#define BT_fLCD_128         (BTFRFQ1+BTFRFQ0)         /* fACLK:256 */
/* LCD frequency values with fBT=fMCLK */
#define BT_fLCD_31K         (BTSSEL)                  /* fMCLK:32 */
#define BT_fLCD_15_5K       (BTSSEL+BTFRFQ0)          /* fMCLK:64 */
#define BT_fLCD_7_8K        (BTSSEL+BTFRFQ1+BTFRFQ0)  /* fMCLK:256 */
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

/* Reset/Hold coded with Bits 6-7 in BT(1)CTL */
/* this is for BT */
//#define BTRESET_CNT1        (BTRESET)           /* BTCNT1 is reset while BTRESET is set */
//#define BTRESET_CNT1_2      (BTRESET+BTDIV)     /* BTCNT1 .AND. BTCNT2 are reset while ~ is set */
/* this is for BT1 */
#define BTHOLD_CNT1         (BTHOLD)            /* BTCNT1 is held while BTHOLD is set */
#define BTHOLD_CNT1_2       (BTHOLD+BTDIV)      /* BT1CNT1 .AND. BT1CNT2 are held while ~ is set */

/* INTERRUPT CONTROL BITS */
/* #define BTIE                0x80 */
/* #define BTIFG               0x80 */

/************************************************************
* SYSTEM CLOCK GENERATOR
************************************************************/
#define __MSP430_HAS_FLL__            /* Definition to show that Module is available */

#define SCFI0_                0x0050    /* System Clock Frequency Integrator 0 */
sfrb(SCFI0, SCFI0_);
#define FN_2                 (0x04)
#define FN_3                 (0x08)
#define FN_4                 (0x10)

#define SCFI1_                0x0051    /* System Clock Frequency Integrator 1 */
sfrb(SCFI1, SCFI1_);
#define SCFQCTL_              0x0052    /* System Clock Frequency Control */
sfrb(SCFQCTL, SCFQCTL_);
/* System clock frequency values fMCLK coded with Bits 0-6 in SCFQCTL */
/* #define SCFQ_32K            0x00                        fMCLK=1*fACLK          only a range from */
/* #define SCFQ_64K            0x01                        fMCLK=2*fACLK          3+1 to 127+1 is possible */
#define SCFQ_128K           (0x03)                     /* fMCLK=4*fACLK */
#define SCFQ_256K           (0x07)                     /* fMCLK=8*fACLK */
#define SCFQ_512K           (0x0F)                     /* fMCLK=16*fACLK */
#define SCFQ_1M             (0x1F)                     /* fMCLK=32*fACLK */
#define SCFQ_2M             (0x3F)                     /* fMCLK=64*fACLK */
#define SCFQ_4M             (0x7F)                     /* fMCLK=128*fACLK        not possible for ICE */

#define CBCTL_                0x0053    /* Crystal Buffer Control *** WRITE-ONLY *** */
sfrb(CBCTL, CBCTL_);
#define CBE                 (0x01)
#define CBSEL0              (0x02)
#define CBSEL1              (0x04)
/* Source select of frequency at output pin XBUF coded with Bits 1-2 in CBCTL */
#define CBSEL_ACLK          (0x00)                    /* source is ACLK         (default after POR) */
#define CBSEL_ACLK_DIV2     (CBSEL0)                  /* source is ACLK/2 */
#define CBSEL_ACLK_DIV4     (CBSEL1)                  /* source is ACLK/4 */
#define CBSEL_MCLK          (CBSEL1+CBSEL0)           /* source is MCLK */

/* INTERRUPT CONTROL BITS */
/* These two bits are defined in the Special Function Registers */
/* #define OFIFG               0x02 */
/* #define OFIE                0x02 */

/************************************************************
* LCD REGISTER
************************************************************/
#define __MSP430_HAS_LCD__            /* Definition to show that Module is available */

#define LCDCTL_               0x0030    /* LCD Control */
sfrb(LCDCTL, LCDCTL_);
/* the names of the mode bits are different from the spec */
#define LCDON               (0x01)
#define LCDLOWR             (0x02)
#define LCDSON              (0x04)
#define LCDMX0              (0x08)
#define LCDMX1              (0x10)
#define LCDP0               (0x20)
#define LCDP1               (0x40)
#define LCDP2               (0x80)
/* Display modes coded with Bits 2-4 */
#define LCDSTATIC           (LCDSON)
#define LCD2MUX             (LCDMX0+LCDSON)
#define LCD3MUX             (LCDMX1+LCDSON)
#define LCD4MUX             (LCDMX1+LCDMX0+LCDSON)
/* Group select code with Bits 5-7                     Seg.lines   Dig.output */
#define LCDSG0              (0x00)                    /* S0  - S1    O2  - O29 (default) */
#define LCDSG0_1            (LCDP0)                   /* S0  - S5    O6  - O29 */
#define LCDSG0_2            (LCDP1)                   /* S0  - S9    O10 - O29 */
#define LCDSG0_3            (LCDP1+LCDP0)             /* S0  - S13   O14 - O29 */
#define LCDSG0_4            (LCDP2)                   /* S0  - S17   O18 - O29 */
#define LCDSG0_5            (LCDP2+LCDP0)             /* S0  - S21   O22 - O29 */
#define LCDSG0_6            (LCDP2+LCDP1)             /* S0  - S25   O26 - O29 */
#define LCDSG0_7            (LCDP2+LCDP1+LCDP0)       /* S0  - S29   --------- */
/* NOTE: YOU CAN ONLY USE THE 'S' OR 'G' DECLARATIONS FOR A COMMAND */
/* MOV  #LCDSG0_3+LCDOG2_7,&LCDCTL ACTUALY MEANS MOV  #LCDP1,&LCDCTL! */
#define LCDOG1_7            (0x00)                    /* S0  - S1    O2  - O29 (default) */
#define LCDOG2_7            (LCDP0)                   /* S0  - S5    O6  - O29 */
#define LCDOG3_7            (LCDP1)                   /* S0  - S9    O10 - O29 */
#define LCDOG4_7            (LCDP1+LCDP0)             /* S0  - S13   O14 - O29 */
#define LCDOG5_7            (LCDP2)                   /* S0  - S17   O18 - O29 */
#define LCDOG6_7            (LCDP2+LCDP0)             /* S0  - S21   O22 - O29 */
#define LCDOG7              (LCDP2+LCDP1)             /* S0  - S25   O26 - O29 */
#define LCDOGOFF            (LCDP2+LCDP1+LCDP0)       /* S0  - S29   --------- */

#define LCDMEM_             (0x0031)  /* LCD Memory */
#ifndef __STDC__
#define LCDMEM              (LCDMEM_) /* LCD Memory (for assembler) */
#else
#define LCDMEM              ((volatile char*) LCDMEM_) /* LCD Memory (for C) */
#endif
#define LCDM1_                0x0031    /* LCD Memory 1 */
sfrb(LCDM1, LCDM1_);
#define LCDM2_                0x0032    /* LCD Memory 2 */
sfrb(LCDM2, LCDM2_);
#define LCDM3_                0x0033    /* LCD Memory 3 */
sfrb(LCDM3, LCDM3_);
#define LCDM4_                0x0034    /* LCD Memory 4 */
sfrb(LCDM4, LCDM4_);
#define LCDM5_                0x0035    /* LCD Memory 5 */
sfrb(LCDM5, LCDM5_);
#define LCDM6_                0x0036    /* LCD Memory 6 */
sfrb(LCDM6, LCDM6_);
#define LCDM7_                0x0037    /* LCD Memory 7 */
sfrb(LCDM7, LCDM7_);
#define LCDM8_                0x0038    /* LCD Memory 8 */
sfrb(LCDM8, LCDM8_);
#define LCDM9_                0x0039    /* LCD Memory 9 */
sfrb(LCDM9, LCDM9_);
#define LCDM10_               0x003A    /* LCD Memory 10 */
sfrb(LCDM10, LCDM10_);
#define LCDM11_               0x003B    /* LCD Memory 11 */
sfrb(LCDM11, LCDM11_);
#define LCDM12_               0x003C    /* LCD Memory 12 */
sfrb(LCDM12, LCDM12_);
#define LCDM13_               0x003D    /* LCD Memory 13 */
sfrb(LCDM13, LCDM13_);
#define LCDM14_               0x003E    /* LCD Memory 14 */
sfrb(LCDM14, LCDM14_);
#define LCDM15_               0x003F    /* LCD Memory 15 */
sfrb(LCDM15, LCDM15_);

#define LCDMA               (LCDM10)  /* LCD Memory A */
#define LCDMB               (LCDM11)  /* LCD Memory B */
#define LCDMC               (LCDM12)  /* LCD Memory C */
#define LCDMD               (LCDM13)  /* LCD Memory D */
#define LCDME               (LCDM14)  /* LCD Memory E */
#define LCDMF               (LCDM15)  /* LCD Memory F */

/************************************************************
* USART
************************************************************/
#define __MSP430_HAS_UART0__          /* Definition to show that Module is available */

#define UCTL_                 0x0070    /* USART Control */
sfrb(UCTL, UCTL_);
#define UTCTL_                0x0071    /* USART Transmit Control */
sfrb(UTCTL, UTCTL_);
#define URCTL_                0x0072    /* USART Receive Control */
sfrb(URCTL, URCTL_);
#define UMCTL_                0x0073    /* USART Modulation Control */
sfrb(UMCTL, UMCTL_);
#define UBR0_                 0x0074    /* USART Baud Rate 0 */
sfrb(UBR0, UBR0_);
#define UBR1_                 0x0075    /* USART Buad Rate 1 */
sfrb(UBR1, UBR1_);
#define RXBUF_                0x0076    /* USART Receive Buffer */
const_sfrb(RXBUF, RXBUF_);
#define TXBUF_                0x0077    /* USART Transmit Buffer */
sfrb(TXBUF, TXBUF_);

#define PENA                (0x80)        /* UCTL */
#define PEV                 (0x40)
#define SPB                 (0x20)        /* to distinguish from stackpointer SP */
#define CHAR                (0x10)
#define LISTEN              (0x08)
#define SYNC                (0x04)
#define MM                  (0x02)
#define SWRST               (0x01)

#define CKPH                (0x80)        /* UTCTL */
#define CKPL                (0x40)
#define SSEL1               (0x20)
#define SSEL0               (0x10)
#define URXSE               (0x08)
#define TXWAKE              (0x04)
#define STC                 (0x02)
#define TXEPT               (0x01)

#define FE                  (0x80)        /* URCTL */
#define PE                  (0x40)
#define OE                  (0x20)
#define BRK                 (0x10)
#define URXEIE              (0x08)
#define URXWIE              (0x04)
#define RXWAKE              (0x02)
#define RXERR               (0x01)

/************************************************************
* Timer A5
************************************************************/
#define __MSP430_HAS_TA5__            /* Definition to show that Module is available */

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
#define TACCTL3_              0x0168    /* Timer A Capture/Compare Control 3 */
sfrw(TACCTL3, TACCTL3_);
#define TACCTL4_              0x016A    /* Timer A Capture/Compare Control 4 */
sfrw(TACCTL4, TACCTL4_);
#define TAR_                  0x0170    /* Timer A Counter Register */
sfrw(TAR, TAR_);
#define TACCR0_               0x0172    /* Timer A Capture/Compare 0 */
sfrw(TACCR0, TACCR0_);
#define TACCR1_               0x0174    /* Timer A Capture/Compare 1 */
sfrw(TACCR1, TACCR1_);
#define TACCR2_               0x0176    /* Timer A Capture/Compare 2 */
sfrw(TACCR2, TACCR2_);
#define TACCR3_               0x0178    /* Timer A Capture/Compare 3 */
sfrw(TACCR3, TACCR3_);
#define TACCR4_               0x017A    /* Timer A Capture/Compare 4 */
sfrw(TACCR4, TACCR4_);

/* Alternate register names */
#define CCTL0               TACCTL0   /* Timer A Capture/Compare Control 0 */
#define CCTL1               TACCTL1   /* Timer A Capture/Compare Control 1 */
#define CCTL2               TACCTL2   /* Timer A Capture/Compare Control 2 */
#define CCTL3               TACCTL3   /* Timer A Capture/Compare Control 3 */
#define CCTL4               TACCTL4   /* Timer A Capture/Compare Control 4 */
#define CCR0                TACCR0    /* Timer A Capture/Compare 0 */
#define CCR1                TACCR1    /* Timer A Capture/Compare 1 */
#define CCR2                TACCR2    /* Timer A Capture/Compare 2 */
#define CCR3                TACCR3    /* Timer A Capture/Compare 3 */
#define CCR4                TACCR4    /* Timer A Capture/Compare 4 */
#define CCTL0_              TACCTL0_  /* Timer A Capture/Compare Control 0 */
#define CCTL1_              TACCTL1_  /* Timer A Capture/Compare Control 1 */
#define CCTL2_              TACCTL2_  /* Timer A Capture/Compare Control 2 */
#define CCTL3_              TACCTL3_  /* Timer A Capture/Compare Control 3 */
#define CCTL4_              TACCTL4_  /* Timer A Capture/Compare Control 4 */
#define CCR0_               TACCR0_   /* Timer A Capture/Compare 0 */
#define CCR1_               TACCR1_   /* Timer A Capture/Compare 1 */
#define CCR2_               TACCR2_   /* Timer A Capture/Compare 2 */
#define CCR3_               TACCR3_   /* Timer A Capture/Compare 3 */
#define CCR4_               TACCR4_   /* Timer A Capture/Compare 4 */
/* Alternate register names - 5xx style */
#define TA0IV               TAIV      /* Timer A Interrupt Vector Word */
#define TA0CTL              TACTL     /* Timer A Control */
#define TA0CCTL0            TACCTL0   /* Timer A Capture/Compare Control 0 */
#define TA0CCTL1            TACCTL1   /* Timer A Capture/Compare Control 1 */
#define TA0CCTL2            TACCTL2   /* Timer A Capture/Compare Control 2 */
#define TA0CCTL3            TACCTL3   /* Timer A Capture/Compare Control 3 */
#define TA0CCTL4            TACCTL4   /* Timer A Capture/Compare Control 4 */
#define TA0R                TAR       /* Timer A Counter Register */
#define TA0CCR0             TACCR0    /* Timer A Capture/Compare 0 */
#define TA0CCR1             TACCR1    /* Timer A Capture/Compare 1 */
#define TA0CCR2             TACCR2    /* Timer A Capture/Compare 2 */
#define TA0CCR3             TACCR3    /* Timer A Capture/Compare 3 */
#define TA0CCR4             TACCR4    /* Timer A Capture/Compare 4 */
#define TA0IV_              TAIV_     /* Timer A Interrupt Vector Word */
#define TA0CTL_             TACTL_    /* Timer A Control */
#define TA0CCTL0_           TACCTL0_  /* Timer A Capture/Compare Control 0 */
#define TA0CCTL1_           TACCTL1_  /* Timer A Capture/Compare Control 1 */
#define TA0CCTL2_           TACCTL2_  /* Timer A Capture/Compare Control 2 */
#define TA0CCTL3_           TACCTL3_  /* Timer A Capture/Compare Control 3 */
#define TA0CCTL4_           TACCTL4_  /* Timer A Capture/Compare Control 4 */
#define TA0R_               TAR_      /* Timer A Counter Register */
#define TA0CCR0_            TACCR0_   /* Timer A Capture/Compare 0 */
#define TA0CCR1_            TACCR1_   /* Timer A Capture/Compare 1 */
#define TA0CCR2_            TACCR2_   /* Timer A Capture/Compare 2 */
#define TA0CCR3_            TACCR3_   /* Timer A Capture/Compare 3 */
#define TA0CCR4_            TACCR4_   /* Timer A Capture/Compare 4 */
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

/* TA5IV Definitions */
#define TAIV_NONE           (0x0000)    /* No Interrupt pending */
#define TAIV_TACCR1         (0x0002)    /* TACCR1_CCIFG */
#define TAIV_TACCR2         (0x0004)    /* TACCR2_CCIFG */
#define TAIV_TACCR3         (0x0006)    /* TACCR3_CCIFG */
#define TAIV_TACCR4         (0x0008)    /* TACCR4_CCIFG */
#define TAIV_TAIFG          (0x000A)    /* TAIFG */

/************************************************************
* 8BIT TIMER/COUNTER
************************************************************/
#define __MSP430_HAS_8BTC__           /* Definition to show that Module is available */

#define TCCTL_                0x0042    /* Timer/Counter Control */
sfrb(TCCTL, TCCTL_);
/* The bit names have been prefixed with "TC" */
#define TCRXD               (0x01)
#define TCTXD               (0x02)
#define TCRXACT             (0x04)
#define TCENCNT             (0x08)
#define TCTXE               (0x10)
#define TCISCTL             (0x20)
#define TCSSEL0             (0x40)
#define TCSSEL1             (0x80)
/* Source select of clock input coded with Bits 6-7 */
#define TCSSEL_P01          (0x00)                    /* source is signal at pin P0.1 (default) */
#define TCSSEL_ACLK         (TCSSEL0)                 /* source is ACLK */
#define TCSSEL_MCLK         (TCSSEL1)                 /* source is MCLK */
#define TCSSEL_P01_MCLK     (TCSSEL1+TCSSEL0)         /* source is signal pin P0.1 .AND. MCLK */

#define TCPLD_                0x0043    /* Timer/Counter Preload */
sfrb(TCPLD, TCPLD_);
#define TCDAT_                0x0044    /* Timer/Counter Data */
sfrb(TCDAT, TCDAT_);

/************************************************************
* TIMER/PORT
************************************************************/
#define __MSP430_HAS_TP__             /* Definition to show that Module is available */

#define TPCTL_                0x004B    /* Timer/Port Control */
sfrb(TPCTL, TPCTL_);
#define EN1FG               (0x01)
#define RC1FG               (0x02)
#define RC2FG               (0x04)
#define EN1                 (0x08)
#define ENA                 (0x10)
#define ENB                 (0x20)
#define TPSSEL0             (0x40)
#define TPSSEL1             (0x80)
/* The EN1 signal of TPCNT1 is coded with with Bits 3-5 in TPCTL */
#define TPCNT1_EN_OFF        (0x00)                  /* TPCNT1 is disabled */
#define TPCNT1_EN_ON         (ENA)                   /*   "    is enabled */
#define TPCNT1_EN_nTPIN5     (ENB)                   /*   "    is enabled with ~TPIN.5 */
#define TPCNT1_EN_TPIN5      (TPSSEL0+ENB)           /*   "    is enabled with TPIN.5 */
#define TPCNT1_EN_nCIN       (ENB+ENA)               /*   "    is enabled with ~CIN */
#define TPCNT1_EN_CIN        (TPSSEL0+ENB+ENA)       /*   "    is enabled with CIN */

/* Source select of clock input coded with Bits 6-7 in TPCTL */
#define TPSSEL_CLK1_CIN     (0x00)                   /* CLK1 source is signal at CIN   (default) */
#define TPSSEL_CLK1_ACLK    (TPSSEL0)                /* CLK1 source is ACLK */
#define TPSSEL_CLK1_MCLK    (TPSSEL1)                /* CLK1 source is MCLK */

/* DATA REGISTER ADDRESSES */
#define TPCNT1_               0x004C    /* Timer/Port Counter 1 */
sfrb(TPCNT1, TPCNT1_);
#define TPCNT2_               0x004D    /* Timer/Port Counter 2 */
sfrb(TPCNT2, TPCNT2_);

#define TPD_                  0x004E    /* Timer/Port Data */
sfrb(TPD, TPD_);
#define TPD_0               (0x01)
#define TPD_1               (0x02)
#define TPD_2               (0x04)
#define TPD_3               (0x08)
#define TPD_4               (0x10)
#define TPD_5               (0x20)
#define CPON                (0x40)
#define B16                 (0x80)

#define TPE_                  0x004F    /* Timer/Port Enable */
sfrb(TPE, TPE_);
#define TPE_0               (0x01)
#define TPE_1               (0x02)
#define TPE_2               (0x04)
#define TPE_3               (0x08)
#define TPE_4               (0x10)
#define TPE_5               (0x20)
#define TPSSEL2             (0x40)
#define TPSSEL3             (0x80)
/* Source select of clock input coded with Bits 6-7 in TPE
   NOTE: If the control bit B16 in TPD is set, TPSSEL2/3
         are 'don't care' and the clock source of counter
         TPCNT2 is the same as of the counter TPCNT1. */
#define TPSSEL_CLK2_TPIN5   (0x00)           /* CLK2 source is signal TPIN.5 (default) */
#define TPSSEL_CLK2_ACLK    (TPSSEL2)        /* CLK2 source is ACLK */
#define TPSSEL_CLK2_MCLK    (TPSSEL3)        /* CLK2 source is MCLK */
#define TPSSEL_CLK2_OFF     (TPSSEL3+TPSSEL2)/* CLK2 source is disabled  */

/************************************************************
* EPROM CONTROL
************************************************************/
#define __MSP430_HAS_EPROM__          /* Definition to show that Module is available */

#define EPCTL_                0x0054    /* EPROM Control */
sfrb(EPCTL, EPCTL_);
#define EPEXE               (0x01)
#define EPVPPS              (0x02)

/************************************************************
* Interrupt Vectors (offset from 0xFFE0)
************************************************************/

#define PORT0_VECTOR        (0x0000)  /* 0xFFE0 Port 0 Bits 2-7 [Lowest Priority] */
#define BASICTIMER_VECTOR   (0x0002)  /* 0xFFE2 Basic Timer */
#define PORT1_VECTOR        (0x0004)  /* 0xFFE4 Port 1 */
#define PORT2_VECTOR        (0x0006)  /* 0xFFE6 Port 2 */
#define TIMERPORT_VECTOR    (0x0008)  /* 0xFFE8 Timer/Port */
#define USARTTX_VECTOR      (0x000C)  /* 0xFFEC USART Transmit */
#define USARTRX_VECTOR      (0x000E)  /* 0xFFEE USART Receive */
#define TIMERA1_VECTOR      (0x0010)  /* 0xFFF0 Timer A CC1-4, TA */
#define TIMERA0_VECTOR      (0x0012)  /* 0xFFF2 Timer A CC0 */
#define WDT_VECTOR          (0x0014) /* 0xFFF4 Watchdog Timer */
#define IO1_VECTOR          (0x0018) /* 0xFFF8 Dedicated IO (P0.1) */
#define IO0_VECTOR          (0x001A) /* 0xFFFA Dedicated IO (P0.0) */
#define NMI_VECTOR          (0x001C) /* 0xFFFC Non-maskable */
#define RESET_VECTOR        (0x001E) /* 0xFFFE Reset [Highest Priority] */

#define UARTTX_VECTOR       USARTTX_VECTOR
#define UARTRX_VECTOR       USARTRX_VECTOR

/************************************************************
* End of Modules
************************************************************/
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* #ifndef __msp430x33x */

