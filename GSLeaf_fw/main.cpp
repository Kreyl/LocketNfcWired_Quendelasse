/*
 * main.cpp
 *
 *  Created on: 20 февр. 2014 г.
 *      Author: g.kruglov
 */

#include "hal.h"
#include "MsgQ.h"
#include "kl_i2c.h"
#include "Sequences.h"
#include "shell.h"
#include "led.h"
#include "CS42L52.h"
#include "kl_sd.h"
#include "AuPlayer.h"
#include "acc_mma8452.h"
#include "kl_fs_utils.h"
#include "SimpleSensors.h"
#include "pn.h"

// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
extern CmdUart_t Uart;
void OnCmd(Shell_t *PShell);
void ITask();

#define PAUSE_BEFORE_REPEAT_S       7

LedRGB_t Led { LED_RED_CH, LED_GREEN_CH, LED_BLUE_CH };
CS42L52_t Audio;
AuPlayer_t Player;

// ==== Dir works ====
#define TABLE_FILENAME      "table.ini"
#define DIRNAME_SURROUND    "Surround"
#define DIRNAME_MAX_LEN     18

static char DirName[DIRNAME_MAX_LEN] = DIRNAME_SURROUND;
static char CurrentDir[DIRNAME_MAX_LEN] = DIRNAME_SURROUND;
static char NextDir[DIRNAME_MAX_LEN] = DIRNAME_SURROUND;

void SwitchToDir() {
    if((strcmp(CurrentDir, DIRNAME_SURROUND) == 0) or  // if playing surround
       (strcmp(CurrentDir, DirName) != 0)) {           // or new name received
        strcpy(NextDir, DirName);
        if(Player.IsPlayingNow) {
            // Fadeout surround and play rcvd id
            Player.FadeOut();
        }
        else {
            strcpy(CurrentDir, NextDir);
            Player.PlayRandomFileFromDir(CurrentDir);
        }
    }
}

// ==== Table "ID-Dirname" ====
#define DIRTABLE_MAX_CNT    99
class IDTable_t {
private:
    uint32_t Cnt;
    MifareID_t IId[DIRTABLE_MAX_CNT];
    char IDirName[DIRTABLE_MAX_CNT][DIRNAME_MAX_LEN];
public:
    uint8_t GetDirnameByID(MifareID_t *pID, char *PDirName) {
        for(uint32_t i=0; i<Cnt; i++) {
            if(IId[i] == *pID) {
                strcpy(PDirName, IDirName[i]);
                Printf("Dir: %S\r", PDirName);
                return retvOk;
            }
        }
        Printf("ID not found\r");
        return retvNotFound;
    }

    void Load() {
        Cnt = 0;
        if(csvOpenFile("table.csv") == retvOk) {
            while(true) {
                if(csvReadNextLine() == retvOk) {
                    if(csvGetNextCell<uint32_t>(&IId[Cnt].ID32[0]) != retvOk) break;
                    if(csvGetNextCell<uint32_t>(&IId[Cnt].ID32[1]) != retvOk) break;
                    csvGetNextCellString(IDirName[Cnt]);
                    Printf(" %X %X %S\r", IId[Cnt].ID32[0], IId[Cnt].ID32[1], IDirName[Cnt]);
                    Cnt++;
                }
                else break;
            } // while true
            csvCloseFile();
        }
    }
} IDTable;

TmrKL_t tmrPauseAfter {evtIdPauseEnds, tktOneShot};

int main(void) {
    // ==== Setup clock frequency ====
    Clk.SetHiPerfMode();
    Clk.Select48MhzSrc(src48PllQ);
    Clk.UpdateFreqValues();

    // Init OS
    halInit();
    chSysInit();
    // ==== Init hardware ====
    EvtQMain.Init();
    Uart.Init(115200);
    Printf("\r%S %S\r\n", APP_NAME, BUILD_TIME);
    Clk.PrintFreqs();

    Led.Init();
    Led.StartOrRestart(lsqStart);
    Pn.Init();

    // Power on Acc to eliminate phantom powering of it through i2c pull-ups
    PinSetupOut(ACC_PWR_PIN, omPushPull);
    PinSetHi(ACC_PWR_PIN);
    chThdSleepMilliseconds(18);

    // Audio
    Audio.Init();   // i2c initialized inside, as pull-ups powered by VAA's LDO
    Audio.SetSpeakerVolume(-96);    // To remove speaker pop at power on
    Audio.DisableSpeakers();
    Audio.SetHeadphoneVolume(-18);
    Audio.EnableHeadphones();

//    Acc.Init();

    SD.Init();
    IDTable.Load();
    Player.Init();

    SimpleSensors::Init();

    // Start playing surround music
    Player.PlayRandomFileFromDir(DIRNAME_SURROUND);

    // Main cycle
    ITask();
}

__noreturn
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdShellCmd:
                OnCmd((Shell_t*)Msg.Ptr);
                ((Shell_t*)Msg.Ptr)->SignalCmdProcessed();
                break;

            case evtIdCardAppeared: {
                MifareID_t *PId = (MifareID_t*)Msg.Ptr;
                Printf("Card: 0x%X 0x%X\r", PId->ID32[0], PId->ID32[1], 8, ' ');
                // Blink LED
                Led.StartOrContinue(lsqCardFound);
                // Get dirname according to card ID
                if(IDTable.GetDirnameByID(PId, DirName) == retvOk) SwitchToDir();
            } break;

            case evtIdCardDisappeared:
                Printf("Card lost\r");
                break;

//            case evtIdAcc:
//                if(State == stIdle) {
//                    Printf("AccWhenIdle\r");
//                    Led.StartOrRestart(lsqAccIdle);
//                    State = stPlaying;
//                    Audio.Resume();
//                    Player.PlayRandomFileFromDir("Sounds");
////                    Player.Play("Alive.wav");
//                }
//                else if(State == stWaiting) {
//                    Printf("AccWhenW\r");
//                    Led.StartOrRestart(lsqAccWaiting);
//                    tmrPauseAfter.StartOrRestart();
//                }
//                break;

            case evtIdPlayEnd: {
                Printf("PlayEnd\r");
                strcpy(CurrentDir, NextDir);
                strcpy(NextDir, DIRNAME_SURROUND);
                Printf("PlayDir %S\r", CurrentDir);
                Player.PlayRandomFileFromDir(CurrentDir);
            } break;

            case evtIdButtons:
                Printf("Btn %u\r", Msg.BtnEvtInfo.BtnID);
                if(Msg.BtnEvtInfo.BtnID == 1) Audio.VolumeUp();
                else if(Msg.BtnEvtInfo.BtnID == 2) Audio.VolumeDown();
                break;

            default: break;
        } // switch
    } // while true
}

#if 1 // ======================= Command processing ============================
void OnCmd(Shell_t *PShell) {
	Cmd_t *PCmd = &PShell->Cmd;
//    __unused int32_t dw32 = 0;  // May be unused in some configurations
//    Uart.Printf("\r%S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ack(retvOk);
    else if(PCmd->NameIs("Version")) PShell->Printf("%S %S\r", APP_NAME, BUILD_TIME);

    else if(PCmd->NameIs("V")) {
        int8_t v;
        if(PCmd->GetNext<int8_t>(&v) != retvOk) { PShell->Ack(retvCmdError); return; }
        Audio.SetMasterVolume(v);
    }
    else if(PCmd->NameIs("SV")) {
        int8_t v;
        if(PCmd->GetNext<int8_t>(&v) != retvOk) { PShell->Ack(retvCmdError); return; }
        Audio.SetSpeakerVolume(v);
    }

    else if(PCmd->NameIs("A")) Player.Play("Alive.wav");
    else if(PCmd->NameIs("48")) {
        Audio.Resume();
        Player.Play("Mocart48.wav");
    }
    else if(PCmd->NameIs("FO")) Player.FadeOut();


    else PShell->Ack(retvCmdUnknown);
}
#endif
