/*
 * pn.h
 *
 *  Created on: 09  эт. 2015 у.
 *      Author: Kreyl
 */

#pragma once

#include "ch.h"
#include "kl_lib.h"
#include "pn_defins.h"
#include "shell.h"

// Buf sizes
#define PN_MAX_DATA_SZ      265  // Max Data Lenghth=265 including TFI um. page 29

// Timings
#define PN_ACK_TIMEOUT      27  // ms
#define PN_DATA_TIMEOUT     180 // ms
#define PN_POLL_INTERVAL    504 // ms

// Struct of single ID. ID is 8-byte wide.
struct MifareID_t {
    union {
        uint32_t ID32[2];
        uint8_t ID8[8];
    } __attribute__ ((__packed__));
    void ConstructOfBuf(uint8_t *p) {
        chSysLock();
        uint32_t *p32 = (uint32_t*)p;
        ID32[0] = *p32++;
        ID32[1] = *p32;
        //memcpy(ID8, p, 8);
        chSysUnlock();
    }
    void Print() { Printf("%04X %04X\r", ID32[0], ID32[1]); }
    bool operator == (const MifareID_t &AID) { return (ID32[0] == AID.ID32[0]) and (ID32[1] == AID.ID32[1]); }
    bool operator != (const MifareID_t &AID) { return (ID32[0] != AID.ID32[0]) or  (ID32[1] != AID.ID32[1]); }
    MifareID_t& operator = (const MifareID_t &AID) { ID32[0] = AID.ID32[0]; ID32[1] = AID.ID32[1]; return *this; }
} __attribute__ ((__packed__));

#if 1 // ======================= Auxilary structures ===========================
struct PnPrologue_t {
    uint8_t Preamble;       // Always 0x00
    uint8_t SoP0;           // Start of Packet Code 0
    uint8_t SoP1;           // Start of Packet Code 1
    uint8_t Len;            // Length
    uint8_t LCS;            // Length Checksum
    bool IsStartOk()  { return (Preamble == 0 and SoP0 == 0 and SoP1 == 0xFF); }
    bool IsExtended() { return (Len == 0xFF and LCS == 0xFF); }
    bool IsLcsOk()    { return ((uint8_t)(Len + LCS) == 0); }
} __attribute__ ((__packed__));
#define PROLOGUE_SZ  sizeof(PnPrologue_t)

struct PnPrologueExt_t {
    uint8_t Preamble;       // Always 0x00
    uint8_t SoP0;           // Start of Packet Code 0
    uint8_t SoP1;           // Start of Packet Code 1
    uint8_t NPLC;           // Normal Packet Length - 0xFF always for extended frame
    uint8_t NPL;            // Normal Packet Length Checksum - 0xFF always for extended frame
    uint8_t LengthHi;       // MSByte of Length
    uint8_t LengthLo;       // LSByte of Length
    uint8_t LCS;            // Length Checksum
    void CalcLCS() { LCS = -(LengthHi + LengthLo); }
    bool IsLcsOk()    { return ((uint8_t)(LengthHi + LengthLo + LCS) == 0); }
} __attribute__ ((__packed__));
#define PROLOGUE_EXT_SZ  sizeof(PnPrologueExt_t)

struct PnEpilogue_t {
    uint8_t DCS;
    uint8_t Postamble;
} __attribute__ ((__packed__));
#define EPILOGUE_SZ  sizeof(PnEpilogue_t)

struct PnAckNack_t {
    uint8_t Preamble;
    uint8_t SoP0;           // Start of Packet Code 0
    uint8_t SoP1;           // Start of Packet Code 1
    uint8_t Code0;
    uint8_t Code1;
    uint8_t Postamble;
    bool operator == (const PnAckNack_t &APkt) { return (Preamble == APkt.Preamble) and (SoP0 == APkt.SoP0) and (SoP1 == APkt.SoP1) and (Code0 == APkt.Code0) and (Code1 == APkt.Code1) and (Postamble == APkt.Postamble); }
} __attribute__ ((__packed__));
#define PN_ACK_NACK_SZ sizeof(PnAckNack_t)

const PnAckNack_t PnPktAck = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

// Auxilary Data indxs, sizes etc
#define PN_PROLOGUE_INDX    1
#define PN_DATA_EXT_INDX    (1 + PROLOGUE_EXT_SZ)   // index in buffer
#define PN_DATA_NORMAL_INDX (1 + PROLOGUE_SZ)       // index in buffer
#define PN_TX_SZ(ALength)   (1 + PROLOGUE_EXT_SZ + ALength + EPILOGUE_SZ)

struct PnReply_t {
    uint8_t TFI;
    struct {
        uint8_t RplCode;
        union {
            uint8_t NbTg;
            uint8_t Err;
            uint8_t Slot;
        };
        uint8_t Buf[PN_MAX_DATA_SZ-3]; // exclude TFI, RplCode and ErrCode
    } __attribute__ ((__packed__));
} __attribute__ ((__packed__));
#endif

#if 1 // =========================== PN class ==================================
enum PnState_t {psOff, psSetup, psConfigured};

class PN532_t : public IrqHandler_t {
private:
    PnState_t State;
    PinIrq_t IIrqPin;
    // Frame
    uint8_t IBuf[1 + PROLOGUE_EXT_SZ + PN_MAX_DATA_SZ + EPILOGUE_SZ]; // Seq type + prologue +...
    PnPrologue_t    *Prologue    = (PnPrologue_t*)   &IBuf[PN_PROLOGUE_INDX];  // Exclude Seq type
    PnPrologueExt_t *PrologueExt = (PnPrologueExt_t*)&IBuf[PN_PROLOGUE_INDX];  // Exclude Seq type
    PnReply_t *PReply;
    uint32_t RxDataSz;
    void WriteEpilogue(uint16_t ALength) { // [TFI + PD0 + PD1 + Е + PDn + DCS] = 0x00
        uint8_t *p = &IBuf[PN_DATA_EXT_INDX]; // Beginning of TFI+Data
        uint8_t Dcs = 0;
        for(uint16_t i=0; i<ALength; i++) Dcs += *p++;
        Dcs = - Dcs;
        *p++ = Dcs; // DCS
        *p = 0x00;  // Postamble
    }
    bool CardOk = false;
    // Gpio
    inline void IRstLo()  { PinSetLo(PN_RST_PIN); }
    inline void IRstHi()  { PinSetHi(PN_RST_PIN); }
    inline void INssLo()  {
        PinSetLo(PN_NSS_PIN);
        chThdSleepMicroseconds(50); // PN will not work without this delay
    }
    inline void INssHi()  { PinSetHi(PN_NSS_PIN); }
    // Inner use
    void IReset();
    // ==== Data Exchange ====
    uint8_t Cmd(uint8_t CmdID, uint32_t ADataLength, ...);
    uint8_t ReceiveAck();
    uint8_t ReceiveData();
    void ITxRx(uint8_t *PRx, uint32_t ALength);
    uint8_t WaitReplyReady(uint32_t Timeout_ms);
    // ==== Hi lvl ====
    bool CardAppeared();
    bool CardIsStillNear();
    void FieldOn()  { Cmd(PN_CMD_RF_CONFIGURATION, 2, 0x01, 0x01); }
    void FieldOff() { Cmd(PN_CMD_RF_CONFIGURATION, 2, 0x01, 0x00); }
    uint8_t MifareRead(uint32_t AAddr);
public:
    MifareID_t CardID;
    void Init();
    // Inner use
    void ITask();
    Spi_t ISpi;
    void IIrqHandler();    // P70_IRQ Handler
    PN532_t() : State(psOff), IIrqPin(PN_IRQ_PIN, this), PReply(nullptr),
            RxDataSz(0), ISpi(PN_SPI) {}
};
#endif

extern PN532_t Pn;
