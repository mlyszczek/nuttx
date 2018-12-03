/******************************************************************************
 *  Filename:       hw_i2c_h
 *  Revised:        2018-05-14 12:24:52 +0200 (Mon, 14 May 2018)
 *  Revision:       51990
 *
 * Copyright (c) 2015 - 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3) Neither the name NuttX nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_I2C_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_I2C_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * I2C component
 *
 ******************************************************************************
 * Slave Own Address
 */

#define I2C_SOAR_OFFSET                                             0x00000000

/* Slave Status */

#define I2C_SSTAT_OFFSET                                            0x00000004

/* Slave Control */

#define I2C_SCTL_OFFSET                                             0x00000004

/* Slave Data */

#define I2C_SDR_OFFSET                                              0x00000008

/* Slave Interrupt Mask */

#define I2C_SIMR_OFFSET                                             0x0000000c

/* Slave Raw Interrupt Status */

#define I2C_SRIS_OFFSET                                             0x00000010

/* Slave Masked Interrupt Status */

#define I2C_SMIS_OFFSET                                             0x00000014

/* Slave Interrupt Clear */

#define I2C_SICR_OFFSET                                             0x00000018

/* Master Salve Address */

#define I2C_MSA_OFFSET                                              0x00000800

/* Master Status */

#define I2C_MSTAT_OFFSET                                            0x00000804

/* Master Control */

#define I2C_MCTRL_OFFSET                                            0x00000804

/* Master Data */

#define I2C_MDR_OFFSET                                              0x00000808

/* I2C Master Timer Period */

#define I2C_MTPR_OFFSET                                             0x0000080c

/* Master Interrupt Mask */

#define I2C_MIMR_OFFSET                                             0x00000810

/* Master Raw Interrupt Status */

#define I2C_MRIS_OFFSET                                             0x00000814

/* Master Masked Interrupt Status */

#define I2C_MMIS_OFFSET                                             0x00000818

/* Master Interrupt Clear */

#define I2C_MICR_OFFSET                                             0x0000081c

/* Master Configuration */

#define I2C_MCR_OFFSET                                              0x00000820

/******************************************************************************
 *
 * Register: I2C_SOAR
 *
 ******************************************************************************
 * Field:   [6:0] OAR
 *
 * I2C slave own address
 * This field specifies bits a6 through a0 of the slave address.
 */

#define I2C_SOAR_OAR_MASK                                           0x0000007f
#define I2C_SOAR_OAR_SHIFT                                                   0

/******************************************************************************
 *
 * Register: I2C_SSTAT
 *
 ******************************************************************************
 * Field:     [2] FBR
 *
 * First byte received
 *
 * 0: The first byte has not been received.
 * 1: The first byte following the slave's own address has been received.
 *
 * This bit is only valid when the RREQ bit is set and is automatically cleared
 * when data has been read from the SDR register.
 * Note: This bit is not used for slave transmit operations.
 */

#define I2C_SSTAT_FBR                                               0x00000004
#define I2C_SSTAT_FBR_MASK                                          0x00000004
#define I2C_SSTAT_FBR_SHIFT                                                  2

/* Field:     [1] TREQ
 *
 * Transmit request
 *
 * 0: No outstanding transmit request.
 * 1: The I2C controller has been addressed as a slave transmitter and is using
 * clock stretching to delay the master until data has been written to the SDR
 * register.
 */

#define I2C_SSTAT_TREQ                                              0x00000002
#define I2C_SSTAT_TREQ_MASK                                         0x00000002
#define I2C_SSTAT_TREQ_SHIFT                                                 1

/* Field:     [0] RREQ
 *
 * Receive request
 *
 * 0: No outstanding receive data
 * 1: The I2C controller has outstanding receive data from the I2C master and
 * is using clock stretching to delay the master until data has been read from
 * the SDR register.
 */

#define I2C_SSTAT_RREQ                                              0x00000001
#define I2C_SSTAT_RREQ_MASK                                         0x00000001
#define I2C_SSTAT_RREQ_SHIFT                                                 0

/******************************************************************************
 *
 * Register: I2C_SCTL
 *
 ******************************************************************************
 * Field:     [0] DA
 *
 * Device active
 *
 * 0: Disables the I2C slave operation
 * 1: Enables the I2C slave operation
 */

#define I2C_SCTL_DA                                                 0x00000001
#define I2C_SCTL_DA_MASK                                            0x00000001
#define I2C_SCTL_DA_SHIFT                                                    0

/******************************************************************************
 *
 * Register: I2C_SDR
 *
 ******************************************************************************
 * Field:   [7:0] DATA
 *
 * Data for transfer
 * This field contains the data for transfer during a slave receive or transmit
 * operation.  When written the register data is used as transmit data.  When
 * read, this register returns the last data received.
 * Data is stored until next update, either by a system write for transmit or
 * by an external master for receive.
 */

#define I2C_SDR_DATA_MASK                                           0x000000ff
#define I2C_SDR_DATA_SHIFT                                                   0

/******************************************************************************
 *
 * Register: I2C_SIMR
 *
 ******************************************************************************
 * Field:     [2] STOPIM
 *
 * Stop condition interrupt mask
 *
 * 0: The SRIS.STOPRIS interrupt is suppressed and not sent to the interrupt
 * controller.
 * 1: The SRIS.STOPRIS interrupt is enabled and sent to the interrupt
 * controller.
 * ENUMs:
 * EN                       Enable Interrupt
 * DIS                      Disable Interrupt
 */

#define I2C_SIMR_STOPIM                                             0x00000004
#define I2C_SIMR_STOPIM_MASK                                        0x00000004
#define I2C_SIMR_STOPIM_SHIFT                                                2
#define I2C_SIMR_STOPIM_EN                                          0x00000004
#define I2C_SIMR_STOPIM_DIS                                         0x00000000

/* Field:     [1] STARTIM
 *
 * Start condition interrupt mask
 *
 * 0: The SRIS.STARTRIS interrupt is suppressed and not sent to the interrupt
 * controller.
 * 1: The SRIS.STARTRIS interrupt is enabled and sent to the interrupt
 * controller.
 * ENUMs:
 * EN                       Enable Interrupt
 * DIS                      Disable Interrupt
 */

#define I2C_SIMR_STARTIM                                            0x00000002
#define I2C_SIMR_STARTIM_MASK                                       0x00000002
#define I2C_SIMR_STARTIM_SHIFT                                               1
#define I2C_SIMR_STARTIM_EN                                         0x00000002
#define I2C_SIMR_STARTIM_DIS                                        0x00000000

/* Field:     [0] DATAIM
 *
 * Data interrupt mask
 *
 * 0: The SRIS.DATARIS interrupt is suppressed and not sent to the interrupt
 * controller.
 * 1: The SRIS.DATARIS interrupt is enabled and sent to the interrupt
 * controller.
 */

#define I2C_SIMR_DATAIM                                             0x00000001
#define I2C_SIMR_DATAIM_MASK                                        0x00000001
#define I2C_SIMR_DATAIM_SHIFT                                                0

/******************************************************************************
 *
 * Register: I2C_SRIS
 *
 ******************************************************************************
 * Field:     [2] STOPRIS
 *
 * Stop condition raw interrupt status
 *
 * 0: No interrupt
 * 1: A Stop condition interrupt is pending.
 *
 * This bit is cleared by writing a 1 to SICR.STOPIC.
 */

#define I2C_SRIS_STOPRIS                                            0x00000004
#define I2C_SRIS_STOPRIS_MASK                                       0x00000004
#define I2C_SRIS_STOPRIS_SHIFT                                               2

/* Field:     [1] STARTRIS
 *
 * Start condition raw interrupt status
 *
 * 0: No interrupt
 * 1: A Start condition interrupt is pending.
 *
 * This bit is cleared by writing a 1 to SICR.STARTIC.
 */

#define I2C_SRIS_STARTRIS                                           0x00000002
#define I2C_SRIS_STARTRIS_MASK                                      0x00000002
#define I2C_SRIS_STARTRIS_SHIFT                                              1

/* Field:     [0] DATARIS
 *
 * Data raw interrupt status
 *
 * 0: No interrupt
 * 1: A data received or data requested interrupt is pending.
 *
 * This bit is cleared by writing a 1 to the SICR.DATAIC.
 */

#define I2C_SRIS_DATARIS                                            0x00000001
#define I2C_SRIS_DATARIS_MASK                                       0x00000001
#define I2C_SRIS_DATARIS_SHIFT                                               0

/******************************************************************************
 *
 * Register: I2C_SMIS
 *
 ******************************************************************************
 * Field:     [2] STOPMIS
 *
 * Stop condition masked interrupt status
 *
 * 0: An interrupt has not occurred or is masked/disabled.
 * 1: An unmasked Stop condition interrupt is pending.
 *
 * This bit is cleared by writing a 1 to the SICR.STOPIC.
 */

#define I2C_SMIS_STOPMIS                                            0x00000004
#define I2C_SMIS_STOPMIS_MASK                                       0x00000004
#define I2C_SMIS_STOPMIS_SHIFT                                               2

/* Field:     [1] STARTMIS
 *
 * Start condition masked interrupt status
 *
 * 0: An interrupt has not occurred or is masked/disabled.
 * 1: An unmasked Start condition interrupt is pending.
 *
 * This bit is cleared by writing a 1 to the SICR.STARTIC.
 */

#define I2C_SMIS_STARTMIS                                           0x00000002
#define I2C_SMIS_STARTMIS_MASK                                      0x00000002
#define I2C_SMIS_STARTMIS_SHIFT                                              1

/* Field:     [0] DATAMIS
 *
 * Data masked interrupt status
 *
 * 0: An interrupt has not occurred or is masked/disabled.
 * 1: An unmasked data received or data requested interrupt is pending.
 *
 * This bit is cleared by writing a 1 to the SICR.DATAIC.
 */

#define I2C_SMIS_DATAMIS                                            0x00000001
#define I2C_SMIS_DATAMIS_MASK                                       0x00000001
#define I2C_SMIS_DATAMIS_SHIFT                                               0

/******************************************************************************
 *
 * Register: I2C_SICR
 *
 ******************************************************************************
 * Field:     [2] STOPIC
 *
 * Stop condition interrupt clear
 *
 * Writing 1 to this bit clears SRIS.STOPRIS and SMIS.STOPMIS.
 */

#define I2C_SICR_STOPIC                                             0x00000004
#define I2C_SICR_STOPIC_MASK                                        0x00000004
#define I2C_SICR_STOPIC_SHIFT                                                2

/* Field:     [1] STARTIC
 *
 * Start condition interrupt clear
 *
 * Writing 1 to this bit clears SRIS.STARTRIS SMIS.STARTMIS.
 */

#define I2C_SICR_STARTIC                                            0x00000002
#define I2C_SICR_STARTIC_MASK                                       0x00000002
#define I2C_SICR_STARTIC_SHIFT                                               1

/* Field:     [0] DATAIC
 *
 * Data interrupt clear
 *
 * Writing 1 to this bit clears SRIS.DATARIS SMIS.DATAMIS.
 */

#define I2C_SICR_DATAIC                                             0x00000001
#define I2C_SICR_DATAIC_MASK                                        0x00000001
#define I2C_SICR_DATAIC_SHIFT                                                0

/******************************************************************************
 *
 * Register: I2C_MSA
 *
 ******************************************************************************
 * Field:   [7:1] SA
 *
 * I2C master slave address
 * Defines which slave is addressed for the transaction in master mode
 */

#define I2C_MSA_SA_MASK                                             0x000000fe
#define I2C_MSA_SA_SHIFT                                                     1

/* Field:     [0] RS
 *
 * Receive or Send
 * This bit-field specifies if the next operation is a receive (high) or a
 * transmit/send (low) from the addressed slave SA.
 * ENUMs:
 * RX                       Receive data from slave
 * TX                       Transmit/send data to slave
 */

#define I2C_MSA_RS                                                  0x00000001
#define I2C_MSA_RS_MASK                                             0x00000001
#define I2C_MSA_RS_SHIFT                                                     0
#define I2C_MSA_RS_RX                                               0x00000001
#define I2C_MSA_RS_TX                                               0x00000000

/******************************************************************************
 *
 * Register: I2C_MSTAT
 *
 ******************************************************************************
 * Field:     [6] BUSBSY
 *
 * Bus busy
 *
 * 0: The I2C bus is idle.
 * 1: The I2C bus is busy.
 *
 * The bit changes based on the MCTRL.START and MCTRL.STOP conditions.
 */

#define I2C_MSTAT_BUSBSY                                            0x00000040
#define I2C_MSTAT_BUSBSY_MASK                                       0x00000040
#define I2C_MSTAT_BUSBSY_SHIFT                                               6

/* Field:     [5] IDLE
 *
 * I2C idle
 *
 * 0: The I2C controller is not idle.
 * 1: The I2C controller is idle.
 */

#define I2C_MSTAT_IDLE                                              0x00000020
#define I2C_MSTAT_IDLE_MASK                                         0x00000020
#define I2C_MSTAT_IDLE_SHIFT                                                 5

/* Field:     [4] ARBLST
 *
 * Arbitration lost
 *
 * 0: The I2C controller won arbitration.
 * 1: The I2C controller lost arbitration.
 */

#define I2C_MSTAT_ARBLST                                            0x00000010
#define I2C_MSTAT_ARBLST_MASK                                       0x00000010
#define I2C_MSTAT_ARBLST_SHIFT                                               4

/* Field:     [3] DATACK_N
 *
 * Data Was Not Acknowledge
 *
 * 0: The transmitted data was acknowledged.
 * 1: The transmitted data was not acknowledged.
 */

#define I2C_MSTAT_DATACK_N                                          0x00000008
#define I2C_MSTAT_DATACK_N_MASK                                     0x00000008
#define I2C_MSTAT_DATACK_N_SHIFT                                             3

/* Field:     [2] ADRACK_N
 *
 * Address Was Not Acknowledge
 *
 * 0: The transmitted address was acknowledged.
 * 1: The transmitted address was not acknowledged.
 */

#define I2C_MSTAT_ADRACK_N                                          0x00000004
#define I2C_MSTAT_ADRACK_N_MASK                                     0x00000004
#define I2C_MSTAT_ADRACK_N_SHIFT                                             2

/* Field:     [1] ERR
 *
 * Error
 *
 * 0: No error was detected on the last operation.
 * 1: An error occurred on the last operation.
 */

#define I2C_MSTAT_ERR                                               0x00000002
#define I2C_MSTAT_ERR_MASK                                          0x00000002
#define I2C_MSTAT_ERR_SHIFT                                                  1

/* Field:     [0] BUSY
 *
 * I2C busy
 *
 * 0: The controller is idle.
 * 1: The controller is busy.
 *
 * When this bit-field is set, the other status bits are not valid.
 *
 * Note: The I2C controller requires four SYSBUS clock cycles to assert the
 * BUSY status after I2C master operation has been initiated through MCTRL
 * register.
 * Hence after programming MCTRL register, application is requested to wait for
 * four SYSBUS clock cycles before issuing a controller status inquiry through
 * MSTAT register.
 * Any prior inquiry would result in wrong status being reported.
 */

#define I2C_MSTAT_BUSY                                              0x00000001
#define I2C_MSTAT_BUSY_MASK                                         0x00000001
#define I2C_MSTAT_BUSY_SHIFT                                                 0

/******************************************************************************
 *
 * Register: I2C_MCTRL
 *
 ******************************************************************************
 * Field:     [3] ACK
 *
 * Data acknowledge enable
 *
 * 0: The received data byte is not acknowledged automatically by the master.
 * 1: The received data byte is acknowledged automatically by the master.
 *
 * This bit-field must be cleared when the I2C bus controller requires no
 * further data to be transmitted from the slave transmitter.
 * ENUMs:
 * EN                       Enable acknowledge
 * DIS                      Disable acknowledge
 */

#define I2C_MCTRL_ACK                                               0x00000008
#define I2C_MCTRL_ACK_MASK                                          0x00000008
#define I2C_MCTRL_ACK_SHIFT                                                  3
#define I2C_MCTRL_ACK_EN                                            0x00000008
#define I2C_MCTRL_ACK_DIS                                           0x00000000

/* Field:     [2] STOP
 *
 * This bit-field determines if the cycle stops at the end of the data cycle or
 * continues on to a repeated START condition.
 *
 * 0: The controller does not generate the Stop condition.
 * 1: The controller generates the Stop condition.
 * ENUMs:
 * EN                       Enable STOP
 * DIS                      Disable STOP
 */

#define I2C_MCTRL_STOP                                              0x00000004
#define I2C_MCTRL_STOP_MASK                                         0x00000004
#define I2C_MCTRL_STOP_SHIFT                                                 2
#define I2C_MCTRL_STOP_EN                                           0x00000004
#define I2C_MCTRL_STOP_DIS                                          0x00000000

/* Field:     [1] START
 *
 * This bit-field generates the Start or Repeated Start condition.
 *
 * 0: The controller does not generate the Start condition.
 * 1: The controller generates the Start condition.
 * ENUMs:
 * EN                       Enable START
 * DIS                      Disable START
 */

#define I2C_MCTRL_START                                             0x00000002
#define I2C_MCTRL_START_MASK                                        0x00000002
#define I2C_MCTRL_START_SHIFT                                                1
#define I2C_MCTRL_START_EN                                          0x00000002
#define I2C_MCTRL_START_DIS                                         0x00000000

/* Field:     [0] RUN
 *
 * I2C master enable
 *
 * 0: The master is disabled.
 * 1: The master is enabled to transmit or receive data.
 * ENUMs:
 * EN                       Enable Master
 * DIS                      Disable Master
 */

#define I2C_MCTRL_RUN                                               0x00000001
#define I2C_MCTRL_RUN_MASK                                          0x00000001
#define I2C_MCTRL_RUN_SHIFT                                                  0
#define I2C_MCTRL_RUN_EN                                            0x00000001
#define I2C_MCTRL_RUN_DIS                                           0x00000000

/******************************************************************************
 *
 * Register: I2C_MDR
 *
 ******************************************************************************
 * Field:   [7:0] DATA
 *
 * When Read: Last RX Data is returned
 * When Written: Data is transferred during TX  transaction
 */

#define I2C_MDR_DATA_MASK                                           0x000000ff
#define I2C_MDR_DATA_SHIFT                                                   0

/******************************************************************************
 *
 * Register: I2C_MTPR
 *
 ******************************************************************************
 * Field:     [7] TPR_7
 *
 * Must be set to 0 to set TPR. If set to 1, a write to TPR will be ignored.
 */

#define I2C_MTPR_TPR_7                                              0x00000080
#define I2C_MTPR_TPR_7_MASK                                         0x00000080
#define I2C_MTPR_TPR_7_SHIFT                                                 7

/* Field:   [6:0] TPR
 *
 * SCL clock period
 * This field specifies the period of the SCL clock.
 * SCL_PRD = 2*(1+TPR)*(SCL_LP + SCL_HP)*CLK_PRD
 * where:
 * SCL_PRD is the SCL line period (I2C clock).
 * TPR is the timer period register value (range of 1 to 127)
 * SCL_LP is the SCL low period (fixed at 6).
 * SCL_HP is the SCL high period (fixed at 4).
 * CLK_PRD is the system clock period in ns.
 */

#define I2C_MTPR_TPR_MASK                                           0x0000007f
#define I2C_MTPR_TPR_SHIFT                                                   0

/******************************************************************************
 *
 * Register: I2C_MIMR
 *
 ******************************************************************************
 * Field:     [0] IM
 *
 * Interrupt mask
 *
 * 0: The MRIS.RIS interrupt is suppressed and not sent to the interrupt
 * controller.
 * 1: The master interrupt is sent to the interrupt controller when the
 * MRIS.RIS is set.
 * ENUMs:
 * EN                       Enable Interrupt
 * DIS                      Disable Interrupt
 */

#define I2C_MIMR_IM                                                 0x00000001
#define I2C_MIMR_IM_MASK                                            0x00000001
#define I2C_MIMR_IM_SHIFT                                                    0
#define I2C_MIMR_IM_EN                                              0x00000001
#define I2C_MIMR_IM_DIS                                             0x00000000

/******************************************************************************
 *
 * Register: I2C_MRIS
 *
 ******************************************************************************
 * Field:     [0] RIS
 *
 * Raw interrupt status
 *
 * 0: No interrupt
 * 1: A master interrupt is pending.
 *
 * This bit is cleared by writing 1 to the MICR.IC bit .
 */

#define I2C_MRIS_RIS                                                0x00000001
#define I2C_MRIS_RIS_MASK                                           0x00000001
#define I2C_MRIS_RIS_SHIFT                                                   0

/******************************************************************************
 *
 * Register: I2C_MMIS
 *
 ******************************************************************************
 * Field:     [0] MIS
 *
 * Masked interrupt status
 *
 * 0: An interrupt has not occurred or is masked.
 * 1: A master interrupt is pending.
 *
 * This bit is cleared by writing 1 to the MICR.IC bit .
 */

#define I2C_MMIS_MIS                                                0x00000001
#define I2C_MMIS_MIS_MASK                                           0x00000001
#define I2C_MMIS_MIS_SHIFT                                                   0

/******************************************************************************
 *
 * Register: I2C_MICR
 *
 ******************************************************************************
 * Field:     [0] IC
 *
 * Interrupt clear
 * Writing 1 to this bit clears MRIS.RIS and  MMIS.MIS .
 *
 * Reading this register returns no meaningful data.
 */

#define I2C_MICR_IC                                                 0x00000001
#define I2C_MICR_IC_MASK                                            0x00000001
#define I2C_MICR_IC_SHIFT                                                    0

/******************************************************************************
 *
 * Register: I2C_MCR
 *
 ******************************************************************************
 * Field:     [5] SFE
 *
 * I2C slave function enable
 * ENUMs:
 * EN                       Slave mode is enabled.
 * DIS                      Slave mode is disabled.
 */

#define I2C_MCR_SFE                                                 0x00000020
#define I2C_MCR_SFE_MASK                                            0x00000020
#define I2C_MCR_SFE_SHIFT                                                    5
#define I2C_MCR_SFE_EN                                              0x00000020
#define I2C_MCR_SFE_DIS                                             0x00000000

/* Field:     [4] MFE
 *
 * I2C master function enable
 * ENUMs:
 * EN                       Master mode is enabled.
 * DIS                      Master mode is disabled.
 */

#define I2C_MCR_MFE                                                 0x00000010
#define I2C_MCR_MFE_MASK                                            0x00000010
#define I2C_MCR_MFE_SHIFT                                                    4
#define I2C_MCR_MFE_EN                                              0x00000010
#define I2C_MCR_MFE_DIS                                             0x00000000

/* Field:     [0] LPBK
 *
 * I2C loopback
 *
 * 0: Normal operation
 * 1: Loopback operation (test mode)
 * ENUMs:
 * EN                       Enable Test Mode
 * DIS                      Disable Test Mode
 */

#define I2C_MCR_LPBK                                                0x00000001
#define I2C_MCR_LPBK_MASK                                           0x00000001
#define I2C_MCR_LPBK_SHIFT                                                   0
#define I2C_MCR_LPBK_EN                                             0x00000001
#define I2C_MCR_LPBK_DIS                                            0x00000000

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_I2C_H */
