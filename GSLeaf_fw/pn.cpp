/*
 * pn.cpp
 *
 *  Created on: 09 џэт. 2015 у.
 *      Author: Kreyl
 */

#include "pn.h"
#include "ch.h"
#include "MsgQ.h"

//#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO1   GPIOC
#define DBG_PIN1    5
#define DBG1_SET()  PinSet(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinClear(DBG_GPIO1, DBG_PIN1)
#endif

//#define PRINT_IO
//#define PRINT_DATA
//#define PRINT_TAGS

// SPI clock is up to 5MHz (um p.45)
#define PN_MAX_BAUDRATE_HZ  5000000

#if 1 // ================================ DMA ==================================
#define PN_DMA_TX_MODE \
    STM32_DMA_CR_CHSEL(PN_DMA_CHNL) |                    \
    DMA_PRIORITY_MEDIUM |                                \
    STM32_DMA_CR_MSIZE_BYTE | /* Size byte */            \
    STM32_DMA_CR_PSIZE_BYTE |                            \
    STM32_DMA_CR_MINC |       /* Mem pointer increase */ \
    STM32_DMA_CR_DIR_M2P |    /* Mem to peripheral */    \
    STM32_DMA_CR_TCIE

#define PN_DMA_RX_MODE  \
    STM32_DMA_CR_CHSEL(PN_DMA_CHNL) |                  \
    DMA_PRIORITY_MEDIUM |                              \
    STM32_DMA_CR_MSIZE_BYTE | /* Memory Size - Byte */ \
    STM32_DMA_CR_PSIZE_BYTE | /* Periph Size - Byte */ \
    STM32_DMA_CR_MINC |    /* Mem pointer increase */  \
    STM32_DMA_CR_DIR_P2M | /* Periph to Mem */         \
    STM32_DMA_CR_TCIE      /* Enable TxCompleted Int  */
#endif

PN532_t Pn;
static thread_reference_t ThdRef;

extern "C" {
    void PnDmaTxCompIrq(void *p, uint32_t flags);
    void PnDmaRxCompIrq(void *p, uint32_t flags);
}

// Thread
static THD_WORKING_AREA(waPnThread, 256);
__noreturn
static void PnThread(void *arg) {
    chRegSetThreadName("PnTask");
    Pn.ITask();
}

void PN532_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull);
    DBG1_CLR();
#endif
    // ==== GPIO ====
    PinSetupOut(PN_RST_PIN, omPushPull);
    PinSetupOut(PN_NSS_PIN, omPushPull);
    PinSetupAlterFunc(PN_SCK_PIN);
    PinSetupAlterFunc(PN_MISO_PIN);
    PinSetupAlterFunc(PN_MOSI_PIN);
    IRstLo();
    INssHi();

    // ==== SPI ====    LSB first, master, ClkLowIdle, FirstEdge, Baudrate=f/2
    uint32_t div;
#if defined STM32L1XX || defined STM32F4XX || defined STM32L4XX
    if(PN_SPI == SPI1) div = Clk.APB2FreqHz / PN_MAX_BAUDRATE_HZ;
    else div = Clk.APB1FreqHz / PN_MAX_BAUDRATE_HZ;
#elif defined STM32F030 || defined STM32F0
    div = Clk.APBFreqHz / PN_MAX_BAUDRATE_HZ;
#endif
    SpiClkDivider_t ClkDiv = sclkDiv2;
    if     (div > 128) ClkDiv = sclkDiv256;
    else if(div > 64) ClkDiv = sclkDiv128;
    else if(div > 32) ClkDiv = sclkDiv64;
    else if(div > 16) ClkDiv = sclkDiv32;
    else if(div > 8)  ClkDiv = sclkDiv16;
    else if(div > 4)  ClkDiv = sclkDiv8;
    else if(div > 2)  ClkDiv = sclkDiv4;

    ISpi.Setup(boLSB, cpolIdleLow, cphaFirstEdge, ClkDiv);
    ISpi.EnableRxDma();
    ISpi.EnableTxDma();
    ISpi.Enable();
    // ==== DMA ====
    // Tx
    dmaStreamAllocate(PN_DMA_TX, IRQ_PRIO_MEDIUM, PnDmaTxCompIrq, nullptr);
    dmaStreamSetPeripheral(PN_DMA_TX, &PN_SPI->DR);
    dmaStreamSetMode      (PN_DMA_TX, PN_DMA_TX_MODE);
    // Rx
    dmaStreamAllocate(PN_DMA_RX, IRQ_PRIO_MEDIUM, PnDmaRxCompIrq, nullptr);
    dmaStreamSetPeripheral(PN_DMA_RX, &PN_SPI->DR);
    dmaStreamSetMode      (PN_DMA_RX, PN_DMA_RX_MODE);
    // ==== IRQ ====
    IIrqPin.Init(ttFalling);
    // ==== Variables ====
    State = psSetup;
    chThdCreateStatic(waPnThread, sizeof(waPnThread), NORMALPRIO, (tfunc_t)PnThread, NULL);
}

void PN532_t::IReset() {
    IRstLo();
    chThdSleepMilliseconds(9);
    IRstHi();
    chThdSleepMilliseconds(9);
}

__noreturn
void PN532_t::ITask() {
    while(true) {
        switch (State) {
            case psConfigured:
                chThdSleepMilliseconds(PN_POLL_INTERVAL);
                if(!CardOk) {
                    if(CardAppeared()) {
                        if(MifareRead(0) == retvOk) {
                            CardOk = true;
//                            Printf("Card Appeared\r");
                            CardID.ConstructOfBuf(PReply->Buf);
                            EvtMsg_t Msg(evtIdCardAppeared, &CardID);
                            EvtQMain.SendWaitingAbility(Msg, MS2ST(900));
                        }
                    } // if appeared
                }
                else {
                    if(!CardIsStillNear()) {
//                        Printf("Card Lost\r");
                        CardOk = false;
                        EvtMsg_t Msg(evtIdCardDisappeared);
                        EvtQMain.SendWaitingAbility(Msg, MS2ST(900));
                    }
                } // if Card is ok
                break;

            case psSetup:
                IReset();
                Cmd(PN_CMD_GET_FIRMWARE_VERSION, WITHOUT_DATA);  // First Cmd will be discarded
                Cmd(PN_CMD_GET_FIRMWARE_VERSION, WITHOUT_DATA);
//                Printf("DS: %u\r", RxDataSz);
                if(PReply->RplCode == 0x03 and PReply->Slot == 0x32) {
                    Printf("Pn reply ok\r");
                    Cmd(PN_CMD_SAM_CONFIGURATION, 1, 0x01);          // Disable SAM to calm PN: Normal mode, the SAM is not used
                    Cmd(PN_CMD_RF_CONFIGURATION, 4, 0x05, 0x02, 0x01, 0x05);
                    Cmd(PN_CMD_RF_CONFIGURATION, 4, 0x02, 0x00, 0x0B, 0x10);
                    State = psConfigured;
                }
                else {
                    Printf("Pn reply error\r");
                    State = psOff;
                }
                break;

            case psOff:
                chSysLock();
                chThdSuspendS(&ThdRef);
                chSysUnlock();
                break;
        } // switch
    } // while true
}

bool PN532_t::CardAppeared() {
    FieldOn();
    if(Cmd(PN_CMD_IN_LIST_PASSIVE_TARGET, 2, 0x01, 0x00) == retvOk) {
        if(PReply->RplCode != PN_RPL_IN_LIST_PASSIVE_TARGET or PReply->NbTg == 0) { // Incorrect reply or Nothing found
            FieldOff();
            return false;
        }
        // ==== Tag is found ====
#ifdef PRINT_TAGS
        Uart.Printf("\rTag1: %A", PReply->Buf, (RxDataSz-3), ' '); // without TFI, Rpl code and NbTg
#endif
        return true;
    }
    else return false;
}

bool PN532_t::CardIsStillNear() {
    // Try to read data from Mifare to determine if it still near
    //if(MifareRead(nullptr, 0) == OK) return true;
    if(Cmd(PN_CMD_IN_DESELECT, 1, 0x01) == retvOk) {
        if(Cmd(PN_CMD_IN_SELECT, 1, 0x01) == retvOk) {
            if(PReply->Err == 0) return true;
        }
    }
    FieldOff();
    return false;
}

// Read 16 bytes starting from address AAddr into ABuf.
uint8_t PN532_t::MifareRead(uint32_t AAddr) {
    if(Cmd(PN_CMD_IN_DATA_EXCHANGE, 3, 0x01, MIFARE_CMD_READ, AAddr) == retvOk) {
        //klPrintf("PN reply: %H\r", Buf, Length);
        if ((PReply->RplCode == PN_CMD_IN_DATA_EXCHANGE+1) and (PReply->Err == 0x00)) { // Correct reply & errorcode == 0
            return retvOk;
        }
    }
    return retvFail;
}

#if 1 // ========================== Data exchange ==============================
uint8_t PN532_t::Cmd(uint8_t CmdID, uint32_t ADataLength, ...) {
    uint8_t Rslt = retvOk;
    uint32_t FLength;
    // ==== Prepare message to send, always in extended format ====
    IBuf[0] = PN_PRE_DATA_WRITE;
    // Prologue
    PrologueExt->Preamble = 0x00;     // Always
    PrologueExt->SoP0     = 0x00;     // Always
    PrologueExt->SoP1     = 0xFF;     // Always
    PrologueExt->NPLC     = 0xFF;     // Always, extended format
    PrologueExt->NPL      = 0xFF;     // Always, extended format
    // Length = 1 (TFI) + 1 (CMD == PD0) + ADataLength
    FLength = 1 + 1 + ADataLength;
    PrologueExt->LengthHi = (uint8_t)((FLength >> 8) & 0xFF);
    PrologueExt->LengthLo = (uint8_t)( FLength       & 0xFF);
    PrologueExt->CalcLCS();           // LCS + LENGTH == 0
    // Data
    uint8_t *pd = &IBuf[PN_DATA_EXT_INDX];  // Beginning of TFI+Data
    *pd++ = PN_FRAME_TFI_TRANSMIT;
    *pd++ = CmdID;
    if(ADataLength != 0) {              // If data present copy it to ComboData Buffer
        va_list Arg;                    // Init Agr
        va_start(Arg, ADataLength);     // Set pointer to last argument
        for(uint32_t i=0; i<ADataLength; i++) *pd++ = (uint8_t)va_arg(Arg, int);
        va_end(Arg);
    }
    // Epilogue
    WriteEpilogue(FLength);
    // ==== Transmit frame ====
#ifdef PRINT_DATA
    Printf(">> %A\r", &IBuf[PN_DATA_EXT_INDX], 2+ADataLength, ' ');
#endif
    INssLo();
    ITxRx(nullptr, PN_TX_SZ(FLength));    // Transmit message
    INssHi();
    // ======= Receive =======
    Rslt = ReceiveAck();
    if(Rslt != retvOk) return Rslt;
    else return ReceiveData();
}

uint8_t PN532_t::ReceiveAck() {
    uint8_t Rslt = retvOk;
    if((Rslt = WaitReplyReady(PN_ACK_TIMEOUT)) != retvOk) return Rslt;
//    Printf("PN Rpl rdy\r");
    IBuf[0] = PN_PRE_DATA_READ;     // First byte is sequence "read"
    INssLo();
    ITxRx(IBuf, PN_ACK_NACK_SZ+1); // Transmit read request and receive ACK/NACK
    INssHi();
    PnAckNack_t *PAckNack = (PnAckNack_t*)&IBuf[1]; // First byte is reply to sequence "read"
    if(*PAckNack == PnPktAck) return retvOk;
    else {
        Printf("PN Nack\r");
        return retvFail;
    }
}

uint8_t PN532_t::ReceiveData() {
    uint8_t Rslt;
    uint8_t* PRxData;
    PReply = nullptr;
    if((Rslt = WaitReplyReady(PN_DATA_TIMEOUT)) != retvOk) return Rslt;
    IBuf[0] = PN_PRE_DATA_READ; // First byte is sequence "read"
    INssLo();
    // Receive reply's prologue
    ITxRx(IBuf, PROLOGUE_SZ+1); // Transmit read request and receive prologue
    // Check if wrong beginning
    if(!Prologue->IsStartOk()) {
        Printf("Bad start\r");
        INssHi();
        return retvFail;
    }
    // Check if extended frame
    if(Prologue->IsExtended()) {
        ITxRx(&PrologueExt->LengthHi, (PROLOGUE_EXT_SZ - PROLOGUE_SZ)); // Receive remainder of prologueExt
        // Check length crc
        if(!PrologueExt->IsLcsOk()) {
            Printf("Bad Ext LCS\r");
            INssHi();
            return retvFail;
        }
        PRxData = &IBuf[PN_DATA_EXT_INDX];
        RxDataSz = Convert::BuildUint16(PrologueExt->LengthLo, PrologueExt->LengthHi);
    }
    // Normal frame
    else {
        // Check length crc
        if(!Prologue->IsLcsOk()) {
            Printf("Bad LCS\r");
            INssHi();
            return retvFail;
        }
        PRxData = &IBuf[PN_DATA_NORMAL_INDX];
        RxDataSz = Prologue->Len;
    }
    ITxRx(PRxData, (RxDataSz + EPILOGUE_SZ));  // Receive data and epilogue
    INssHi();
    // Check DCS
    uint8_t DCS = 0;
    for(uint32_t i=0; i < RxDataSz+1; i++) DCS += PRxData[i]; // TFI + D0 + D1 + ... + DCS
    if(DCS != 0) {
        Printf("Bad DCS\r");
        return retvFail;
    }
    // All ok
    PReply = (PnReply_t*)PRxData;
#ifdef PRINT_DATA
    Printf("<< %A\r", PRxData, RxDataSz, ' ');
#endif
    return retvOk;
}

// Transmit IBuf always
void PN532_t::ITxRx(uint8_t *PRx, uint32_t ALength) {
#ifdef PRINT_IO
    Printf(">> %A\r", IBuf, ALength, ' ');
#endif
    chSysLock();
    // RX
    if(PRx != nullptr) { dmaStreamSetMemory0(PN_DMA_RX, PRx); }
    else { dmaStreamSetMemory0(PN_DMA_RX, IBuf); }
    dmaStreamSetTransactionSize(PN_DMA_RX, ALength);
    dmaStreamSetMode(PN_DMA_RX, PN_DMA_RX_MODE);
    dmaStreamEnable(PN_DMA_RX);
    // TX
    dmaStreamSetMemory0(PN_DMA_TX, IBuf);
    dmaStreamSetTransactionSize(PN_DMA_TX, ALength);
    dmaStreamSetMode(PN_DMA_TX, PN_DMA_TX_MODE);
    dmaStreamEnable(PN_DMA_TX);
    chThdSuspendS(&ThdRef); // Wait IRQ
    chSysUnlock();
#ifdef PRINT_IO
    if(PRx != nullptr) Printf("<< %A\r", PRx, ALength, ' ');
#endif
}

uint8_t PN532_t::WaitReplyReady(uint32_t Timeout_ms) {
    // Enable IRQ and wait
    chSysLock();
    IIrqPin.CleanIrqFlag();
    IIrqPin.EnableIrq(IRQ_PRIO_MEDIUM);
    msg_t Rslt = chThdSuspendTimeoutS(&ThdRef, MS2ST(Timeout_ms));    // Wait IRQ
    IIrqPin.DisableIrq();   // Disable anyway, in case of timeout
    chSysUnlock();
    if(Rslt == MSG_TIMEOUT) {
//        Printf("PN Timeout\r");
        return retvTimeout;
    }
    else return retvOk;
}
#endif

#if 1 // ========================= IRQs ========================================
void PN532_t::IIrqHandler() { // Interrupt caused by Low level on IRQ_Pin
    IIrqPin.DisableIrq();     // Disable IRQ
    chThdResumeI(&ThdRef, MSG_OK);
}

extern "C" {
// DMA transmission complete
void PnDmaTxCompIrq(void *p, uint32_t flags) {
    dmaStreamDisable(PN_DMA_TX);    // Disable DMA
}
// DMA reception complete
void PnDmaRxCompIrq(void *p, uint32_t flags) {
    dmaStreamDisable(PN_DMA_RX); // Disable DMA
    chSysLockFromISR();
    chThdResumeI(&ThdRef, MSG_OK);
    chSysUnlockFromISR();
}
} // extern C
#endif
