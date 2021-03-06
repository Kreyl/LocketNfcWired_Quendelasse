/*
 * kl_fs_common.h
 *
 *  Created on: 30 ???. 2016 ?.
 *      Author: Kreyl
 */

#pragma once

#include "ff.h"
#include "kl_lib.h"
#include "shell.h"

// Variables
extern FILINFO FileInfo;
extern DIR Dir;

uint8_t TryOpenFileRead(const char *Filename, FIL *PFile);
uint8_t CheckFileNotEmpty(FIL *PFile);
uint8_t TryRead(FIL *PFile, void *Ptr, uint32_t Sz);

//template <typename T>
//uint8_t TryRead(FIL *PFile, T *Ptr);

template <typename T>
uint8_t TryRead(FIL *PFile, T *Ptr) {
    uint32_t ReadSz=0;
    uint8_t r = f_read(PFile, Ptr, sizeof(T), &ReadSz);
    return (r == FR_OK and ReadSz == sizeof(T))? retvOk : retvFail;
}

//bool CurrentDirIsRoot();
//void ResetCurrDir

uint8_t ReadLine(FIL *PFile, char* S, uint32_t MaxLen);

uint8_t CountFilesInDir(const char* DirName, const char* Extension, uint32_t *PCnt);

#define SD_STRING_SZ    256 // for operations with strings
// =========================== ini file operations =============================
/*
 * ini file has the following structure:
 *
 * # This is Comment: comment uses either '#' or ';' symbol
 * ; This is Comment too
 *
 * [Section]    ; This is name of section
 * Count=6      ; This is key with value of int32
 * Volume=-1    ; int32
 * SoundFileName=phrase01.wav   ; string
 *
 * [Section2]
 * Key1=1
 * ...
 */

uint8_t iniReadString(const char *AFileName, const char *ASection, const char *AKey, char **PPOutput);

//template <typename T>
//uint8_t iniRead(const char *AFileName, const char *ASection, const char *AKey, T *POutput);


template <typename T>
static uint8_t iniRead(const char *AFileName, const char *ASection, const char *AKey, T *POutput) {
    char *S = nullptr;
    if(iniReadString(AFileName, ASection, AKey, &S) == retvOk) {
        int32_t tmp = strtol(S, NULL, 10);
        *POutput = (T)tmp;
        return retvOk;
    }
    else return retvFail;
}

// =========================== csv file operations =============================
/*
 * csv file has the following structure:
 *
 * # this is comment
 * 14, 0x38, "DirName1"
 * 15, 0, "DirName2"
 * ...
 *
 * Usage:
 */

uint8_t csvOpenFile(const char *AFileName);
void csvCloseFile();
uint8_t csvReadNextLine();
void csvGetNextCellString(char* POutput);
uint8_t csvGetNextToken(char** POutput);

template <typename T>
static uint8_t csvGetNextCell(T *POutput) {
    char *Token;
    if(csvGetNextToken(&Token) == retvOk) {
        char *p;
        *POutput = (T)strtoul(Token, &p, 0);
        if(*p == '\0') return retvOk;
        else return retvNotANumber;
    }
    else return retvEmpty;
}
