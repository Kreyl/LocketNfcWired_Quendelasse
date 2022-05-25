/*
 * pn_defins.h
 *
 * Contains constants and definitions for PN.
 *
 *  Created on: 25.11.2011
 *      Author: Kreyl
 */

#ifndef PN_DEFINS_H_
#define PN_DEFINS_H_

// =================================== Constants ==============================
#define WITHOUT_DATA                    0
// Preamble command bytes
#define PN_PRE_STATUS_READ              0b00000010
#define PN_PRE_DATA_WRITE               0b00000001
#define PN_PRE_DATA_READ                0b00000011

// Frame identifier
// Pn CMDs
#define PN_FRAME_PREAMBLE               0x00
#define PN_FRAME_TFI_TRANSMIT           0xD4
#define PN_FRAME_TFI_RECIEVE            0xD5
#define PN_FRAME_POSTAMBLE              0x00

// Command codes
// Miscellaneous
#define PN_CMD_DIAGNOSE                 0x00
#define PN_CMD_GET_FIRMWARE_VERSION     0x02
#define PN_CMD_GET_GENERAL_STATUS       0x04
#define PN_CMD_READ_REGISTER            0x06
#define PN_CMD_WRITE_REGISTER           0x08
#define PN_CMD_READ_GPIO                0x0C
#define PN_CMD_WRITE_GPIO               0x0E
#define PN_CMD_SET_SERIAL_BAUD_RATE     0x10
#define PN_CMD_SET_PARAMETERS           0x12
#define PN_CMD_SAM_CONFIGURATION        0x14
#define PN_CMD_POWER_DOWN               0x16
// RF communication
#define PN_CMD_RF_CONFIGURATION         0x32
#define PN_CMD_RF_REGULATION_TEST       0x58
// Initiatior
#define PN_CMD_IN_JUMP_FOR_DEP          0x56
#define PN_CMD_IN_JUMP_FOR_PSL          0x46
#define PN_CMD_IN_LIST_PASSIVE_TARGET   0x4A
#define PN_CMD_IN_ATR                   0x50
#define PN_CMD_IN_PSL                   0x4E
#define PN_CMD_IN_DATA_EXCHANGE         0x40
#define PN_CMD_IN_COMMUNICATE_THRU      0x42
#define PN_CMD_IN_DESELECT              0x44
#define PN_CMD_IN_RELEASE               0x52
#define PN_CMD_IN_SELECT                0x54
#define PN_CMD_IN_AUTO_POLL             0x60
// Target
#define PN_CMD_TG_INIT_AS_TARGET        0x8C
#define PN_CMD_TG_SET_GENERAL_BYTES     0x92
#define PN_CMD_TG_GET_DATA              0x86
#define PN_CMD_TG_SET_DATA              0x8E
#define PN_CMD_TG_SET_META_DATA         0x94
#define PN_CMD_TG_GET_INITIATOR_COMMAND 0x88
#define PN_CMD_TG_RESPONSE_TO_INITIATOR 0x90
#define PN_CMD_TG_GET_TARGET_STATUS     0x8A

// Replies
#define PN_RPL_DIAGNOSE                 0x01
#define PN_RPL_GET_FIRMWARE_VERSION     0x03
#define PN_RPL_GET_GENERAL_STATUS       0x05
#define PN_RPL_POWER_DOWN               0x17
#define PN_RPL_TG_INIT_AS_TARGET        0x8D
#define PN_RPL_TG_GET_DATA              0x87
#define PN_RPL_TG_SET_DATA              0x8F
#define PN_RPL_TG_GET_INITIATOR_COMMAND 0x89
#define PN_RPL_TG_RESPONSE_TO_INITIATOR 0x91
#define PN_RPL_TG_GET_TARGET_STATUS     0x8B

#define PN_RPL_READ_REGISTER            0x07
#define PN_RPL_WRITE_REGISTER           0x09
#define PN_RPL_IN_LIST_PASSIVE_TARGET   0x4B
#define PN_RPL_IN_DESELECT              0x45
#define PN_RPL_IN_SELECT                0x55

// Parameters bits for SetParameters function (p. 83)
#define PN_fNADUsed                     0b00000001
#define PN_fDIDUsed                     0b00000010
#define PN_fAutomaticATR_RES            0b00000100
#define PN_fIRQUsed                     0b00001000
#define PN_fAutomaticRATS               0b00010000
#define PN_fISO14443_4_PICC             0b00100000
#define PN_fRemovePrePostAmble          0b01000000

// Mifare commands
#define MIFARE_CMD_READ                 0x30
#define MIFARE_CMD_WRITE                0xA2



#endif /* PN_DEFINS_H_ */
