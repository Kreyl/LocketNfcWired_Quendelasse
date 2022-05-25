/*
 * EvtMsgIDs.h
 *
 *  Created on: 21 апр. 2017 г.
 *      Author: Kreyl
 */

#pragma once

enum EvtMsgId_t {
    evtIdNone = 0, // Always

    // Pretending to eternity
    evtIdShellCmd = 1,
    evtIdEverySecond = 2,

    evtIdButtons = 15,
    evtIdAcc = 16,
    evtIdPlayEnd = 17,
    evtIdPauseEnds = 18,
    evtIdOnRx = 19,

    evtIdCardAppeared = 30,
    evtIdCardDisappeared = 31,
};
