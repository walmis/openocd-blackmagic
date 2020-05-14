/***************************************************************************
 *   Copyright 2016,2017 Sony Video & Sound Products Inc.                  *
 *   Masatoshi Tateishi - Masatoshi.Tateishi@jp.sony.com                   *
 *   Masayuki Ishikawa - Masayuki.Ishikawa@jp.sony.com                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_RTOS_NUTTX_HEADER_H
#define OPENOCD_RTOS_NUTTX_HEADER_H

/*  gdb script to update the header file
  according to kernel version and build option
  before executing function awareness
  kernel symbol must be loaded : symbol nuttx

define awareness
 set logging off
 set logging file nuttx_header.h
 set logging on

 printf "#define PID  %p\n",&((struct tcb_s *)(0))->pid
 printf "#define XCPREG  %p\n",&((struct tcb_s *)(0))->xcp.regs
 printf "#define STATE %p\n",&((struct tcb_s *)(0))->task_state
 printf "#define NAME %p\n",&((struct tcb_s *)(0))->name
 printf "#define NAME_SIZE %d\n",sizeof(((struct tcb_s *)(0))->name)
 end


 OR ~/.gdbinit


define hookpost-file

 if &g_readytorun != 0
  eval "monitor nuttx.pid_offset %d", &((struct tcb_s *)(0))->pid
  eval "monitor nuttx.xcp__REG_offset %d", &((struct tcb_s *)(0))->xcp.regs
  eval "monitor nuttx.state_offset %d", &((struct tcb_s *)(0))->task_state
  eval "monitor nuttx.name_offset %d", &((struct tcb_s *)(0))->name
  eval "monitor nuttx.name_size %d", sizeof(((struct tcb_s *)(0))->name)
 end

end

*/

/* default offset */
#define PID  0xc
#define XCPREG  0x70
#define STATE 0x19
#define NAME 0xb8
#define NAME_SIZE 32

//#define CONFIG_BUILD_PROTECTED
#define CONFIG_LAZY_FPU
#define CONFIG_ARCH_FPU

/* defconfig of nuttx */
/* #define CONFIG_DISABLE_SIGNALS */
//#define CONFIG_DISABLE_MQUEUE
/* #define CONFIG_PAGING */

#ifdef CONFIG_LAZY_FPU
#define __REG_R13             (0)  /* R13 = SP at time of interrupt */
#ifdef CONFIG_ARMV7M_USEBASEPRI
#  define __REG_BASEPRI       (1)  /* BASEPRI */
#else
#  define __REG_PRIMASK       (1)  /* PRIMASK */
#endif
#define __REG_R4              (2)  /* R4 */
#define __REG_R5              (3)  /* R5 */
#define __REG_R6              (4)  /* R6 */
#define __REG_R7              (5)  /* R7 */
#define __REG_R8              (6)  /* R8 */
#define __REG_R9              (7)  /* R9 */
#define __REG_R10             (8)  /* R10 */
#define __REG_R11             (9)  /* R11 */

#ifdef CONFIG_BUILD_PROTECTED
#  define __REG_EXC_RETURN    (10) /* EXC_RETURN */
#  define SW_INT_REGS       (11)
#else
#  define SW_INT_REGS       (10)
#endif

/* If the MCU supports a floating point unit, then it will be necessary
 * to save the state of the FPU status register and data registers on
 * each context switch.  These registers are not saved during interrupt
 * level processing, however. So, as a consequence, floating point
 * operations may NOT be performed in interrupt handlers.
 *
 * The FPU provides an extension register file containing 32 single-
 * precision registers. These can be viewed as:
 *
 * - Sixteen 64-bit doubleword registers, D0-D15
 * - Thirty-two 32-bit single-word registers, S0-S31
 *   S<2n> maps to the least significant half of D<n>
 *   S<2n+1> maps to the most significant half of D<n>.
 */

#ifdef CONFIG_ARCH_FPU
#  define __REG_D0            (SW_INT_REGS+0)  /* D0 */
#  define __REG_S0            (SW_INT_REGS+0)  /* S0 */
#  define __REG_S1            (SW_INT_REGS+1)  /* S1 */
#  define __REG_D1            (SW_INT_REGS+2)  /* D1 */
#  define __REG_S2            (SW_INT_REGS+2)  /* S2 */
#  define __REG_S3            (SW_INT_REGS+3)  /* S3 */
#  define __REG_D2            (SW_INT_REGS+4)  /* D2 */
#  define __REG_S4            (SW_INT_REGS+4)  /* S4 */
#  define __REG_S5            (SW_INT_REGS+5)  /* S5 */
#  define __REG_D3            (SW_INT_REGS+6)  /* D3 */
#  define __REG_S6            (SW_INT_REGS+6)  /* S6 */
#  define __REG_S7            (SW_INT_REGS+7)  /* S7 */
#  define __REG_D4            (SW_INT_REGS+8)  /* D4 */
#  define __REG_S8            (SW_INT_REGS+8)  /* S8 */
#  define __REG_S9            (SW_INT_REGS+9)  /* S9 */
#  define __REG_D5            (SW_INT_REGS+10) /* D5 */
#  define __REG_S10           (SW_INT_REGS+10) /* S10 */
#  define __REG_S11           (SW_INT_REGS+11) /* S11 */
#  define __REG_D6            (SW_INT_REGS+12) /* D6 */
#  define __REG_S12           (SW_INT_REGS+12) /* S12 */
#  define __REG_S13           (SW_INT_REGS+13) /* S13 */
#  define __REG_D7            (SW_INT_REGS+14) /* D7 */
#  define __REG_S14           (SW_INT_REGS+14) /* S14 */
#  define __REG_S15           (SW_INT_REGS+15) /* S15 */
#  define __REG_D8            (SW_INT_REGS+16) /* D8 */
#  define __REG_S16           (SW_INT_REGS+16) /* S16 */
#  define __REG_S17           (SW_INT_REGS+17) /* S17 */
#  define __REG_D9            (SW_INT_REGS+18) /* D9 */
#  define __REG_S18           (SW_INT_REGS+18) /* S18 */
#  define __REG_S19           (SW_INT_REGS+19) /* S19 */
#  define __REG_D10           (SW_INT_REGS+20) /* D10 */
#  define __REG_S20           (SW_INT_REGS+20) /* S20 */
#  define __REG_S21           (SW_INT_REGS+21) /* S21 */
#  define __REG_D11           (SW_INT_REGS+22) /* D11 */
#  define __REG_S22           (SW_INT_REGS+22) /* S22 */
#  define __REG_S23           (SW_INT_REGS+23) /* S23 */
#  define __REG_D12           (SW_INT_REGS+24) /* D12 */
#  define __REG_S24           (SW_INT_REGS+24) /* S24 */
#  define __REG_S25           (SW_INT_REGS+25) /* S25 */
#  define __REG_D13           (SW_INT_REGS+26) /* D13 */
#  define __REG_S26           (SW_INT_REGS+26) /* S26 */
#  define __REG_S27           (SW_INT_REGS+27) /* S27 */
#  define __REG_D14           (SW_INT_REGS+28) /* D14 */
#  define __REG_S28           (SW_INT_REGS+28) /* S28 */
#  define __REG_S29           (SW_INT_REGS+29) /* S29 */
#  define __REG_D15           (SW_INT_REGS+30) /* D15 */
#  define __REG_S30           (SW_INT_REGS+30) /* S30 */
#  define __REG_S31           (SW_INT_REGS+31) /* S31 */
#  define __REG_FPSCR         (SW_INT_REGS+32) /* Floating point status and control */
#  define SW_FPU_REGS       (33)
#else
#  define SW_FPU_REGS       (0)
#endif

/* The total number of registers saved by software */

#define SW_XCPT_REGS        (SW_INT_REGS + SW_FPU_REGS)
#define SW_XCPT_SIZE        (4 * SW_XCPT_REGS)

/* On entry into an IRQ, the hardware automatically saves the following
 * registers on the stack in this (address) order:
 */

#define __REG_R0              (SW_XCPT_REGS+0) /* R0 */
#define __REG_R1              (SW_XCPT_REGS+1) /* R1 */
#define __REG_R2              (SW_XCPT_REGS+2) /* R2 */
#define __REG_R3              (SW_XCPT_REGS+3) /* R3 */
#define __REG_R12             (SW_XCPT_REGS+4) /* R12 */
#define __REG_R14             (SW_XCPT_REGS+5) /* R14 = LR */
#define __REG_R15             (SW_XCPT_REGS+6) /* R15 = PC */
#define __REG_XPSR            (SW_XCPT_REGS+7) /* xPSR */

#define HW_XCPT_REGS        (8)
#define HW_XCPT_SIZE        (4 * HW_XCPT_REGS)

#define XCPTCONTEXT_REGS    (HW_XCPT_REGS + SW_XCPT_REGS)
#define XCPTCONTEXT_SIZE    (HW_XCPT_SIZE + SW_XCPT_SIZE)

#else 

#define __REG_R13             (0)  /* R13 = SP at time of interrupt */
#ifdef CONFIG_ARMV7M_USEBASEPRI
#  define __REG_BASEPRI       (1)  /* BASEPRI */
#else
#  define __REG_PRIMASK       (1)  /* PRIMASK */
#endif
#define __REG_R4              (2)  /* R4 */
#define __REG_R5              (3)  /* R5 */
#define __REG_R6              (4)  /* R6 */
#define __REG_R7              (5)  /* R7 */
#define __REG_R8              (6)  /* R8 */
#define __REG_R9              (7)  /* R9 */
#define __REG_R10             (8)  /* R10 */
#define __REG_R11             (9)  /* R11 */
#define __REG_EXC_RETURN      (10) /* EXC_RETURN */
#define SW_INT_REGS         (11)

#ifdef CONFIG_ARCH_FPU

/* If the MCU supports a floating point unit, then it will be necessary
 * to save the state of the non-volatile registers before calling code
 * that may save and overwrite them.
 */

#  define __REG_S16           (SW_INT_REGS+0) /* S16 */
#  define __REG_S17           (SW_INT_REGS+1) /* S17 */
#  define __REG_S18           (SW_INT_REGS+2) /* S18 */
#  define __REG_S19           (SW_INT_REGS+3) /* S19 */
#  define __REG_S20           (SW_INT_REGS+4) /* S20 */
#  define __REG_S21           (SW_INT_REGS+5) /* S21 */
#  define __REG_S22           (SW_INT_REGS+6) /* S22 */
#  define __REG_S23           (SW_INT_REGS+7) /* S23 */
#  define __REG_S24           (SW_INT_REGS+8) /* S24 */
#  define __REG_S25           (SW_INT_REGS+9) /* S25 */
#  define __REG_S26           (SW_INT_REGS+10) /* S26 */
#  define __REG_S27           (SW_INT_REGS+11) /* S27 */
#  define __REG_S28           (SW_INT_REGS+12) /* S28 */
#  define __REG_S29           (SW_INT_REGS+13) /* S29 */
#  define __REG_S30           (SW_INT_REGS+14) /* S30 */
#  define __REG_S31           (SW_INT_REGS+15) /* S31 */
#  define SW_FPU_REGS       (16)
#else
#  define SW_FPU_REGS       (0)
#endif

/* The total number of registers saved by software */

#define SW_XCPT_REGS        (SW_INT_REGS + SW_FPU_REGS)
#define SW_XCPT_SIZE        (4 * SW_XCPT_REGS)

/* On entry into an IRQ, the hardware automatically saves the following
 * registers on the stack in this (address) order:
 */

#define __REG_R0              (SW_XCPT_REGS+0) /* R0 */
#define __REG_R1              (SW_XCPT_REGS+1) /* R1 */
#define __REG_R2              (SW_XCPT_REGS+2) /* R2 */
#define __REG_R3              (SW_XCPT_REGS+3) /* R3 */
#define __REG_R12             (SW_XCPT_REGS+4) /* R12 */
#define __REG_R14             (SW_XCPT_REGS+5) /* R14 = LR */
#define __REG_R15             (SW_XCPT_REGS+6) /* R15 = PC */
#define __REG_XPSR            (SW_XCPT_REGS+7) /* xPSR */
#define HW_INT_REGS         (8)

#ifdef CONFIG_ARCH_FPU

/* If the FPU is enabled, the hardware also saves the volatile FP registers.
 */

#  define __REG_S0            (SW_XCPT_REGS+8)  /* S0 */
#  define __REG_S1            (SW_XCPT_REGS+9)  /* S1 */
#  define __REG_S2            (SW_XCPT_REGS+10) /* S2 */
#  define __REG_S3            (SW_XCPT_REGS+11) /* S3 */
#  define __REG_S4            (SW_XCPT_REGS+12) /* S4 */
#  define __REG_S5            (SW_XCPT_REGS+13) /* S5 */
#  define __REG_S6            (SW_XCPT_REGS+14) /* S6 */
#  define __REG_S7            (SW_XCPT_REGS+15) /* S7 */
#  define __REG_S8            (SW_XCPT_REGS+16) /* S8 */
#  define __REG_S9            (SW_XCPT_REGS+17) /* S9 */
#  define __REG_S10           (SW_XCPT_REGS+18) /* S10 */
#  define __REG_S11           (SW_XCPT_REGS+19) /* S11 */
#  define __REG_S12           (SW_XCPT_REGS+20) /* S12 */
#  define __REG_S13           (SW_XCPT_REGS+21) /* S13 */
#  define __REG_S14           (SW_XCPT_REGS+22) /* S14 */
#  define __REG_S15           (SW_XCPT_REGS+23) /* S15 */
#  define __REG_FPSCR         (SW_XCPT_REGS+24) /* FPSCR */
#  define __REG_FPReserved    (SW_XCPT_REGS+25) /* Reserved */
#  define HW_FPU_REGS       (18)
#else
#  define HW_FPU_REGS       (0)
#endif

#define HW_XCPT_REGS        (HW_INT_REGS + HW_FPU_REGS)
#define HW_XCPT_SIZE        (4 * HW_XCPT_REGS)

#define XCPTCONTEXT_REGS    (HW_XCPT_REGS + SW_XCPT_REGS)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

#endif


#endif /* OPENOCD_RTOS_NUTTX_HEADER_H */
