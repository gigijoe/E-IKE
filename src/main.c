/*
 *  OpenBTD
 *
 *  Copyright (c) 2017, Steve Chang
 *  stevegigijoe@yahoo.com.tw
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#undef IGNITION_STATUS
//#define DEBUG

#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "bool.h"
#include "delay.h"
#include "usart.h"
#if 0
#include "adc.h"
#include "pwm.h"
#endif

#define MAX_ITEMS 11
static uint8_t currentItem = 0;

/*
*
*/

#if 0
static uint32_t tim4Tick_1ms = 0;
static uint32_t tim4Tick_10ms = 0;
static uint32_t tim4Tick_50ms = 0;
static uint32_t tim4Tick_100ms = 0;
static uint32_t tim4Tick_200ms = 0;
#endif
static uint32_t tim4Tick_1000ms = 0;

static uint32_t tim4Tick = 0;
static uint32_t e_ike_tick = 0;
static uint32_t e_ike_interval = 2000; /* Default 2 secs */

/*
*
*/

static char *hextoa(uint8_t hex)
{
  static char a[3];
  uint8_t v = (hex & 0xf0) >> 4;
  switch(v) {
    case 10: a[0] = 'a';
      break;
    case 11: a[0] = 'b';
      break; 
    case 12: a[0] = 'c';
      break; 
    case 13: a[0] = 'd';
      break; 
    case 14: a[0] = 'e';
      break; 
    case 15: a[0] = 'f';
      break; 
    default: a[0] = (v + 0x30);
      break;
  }
  v = hex & 0xf;
  switch(v) {
    case 10: a[1] = 'a';
      break;
    case 11: a[1] = 'b';
      break; 
    case 12: a[1] = 'c';
      break; 
    case 13: a[1] = 'd';
      break; 
    case 14: a[1] = 'e';
      break; 
    case 15: a[1] = 'f';
      break; 
    default: a[1] = (v + 0x30);
      break;
  }
  a[2] = '\0';
  return a;
}

static char *hextodec(uint8_t hex)
{
  static char a[4];
  memset(a, 0x20, 4);
  if(hex >= 100) {
    a[0] = hex / 100 + 0x30;
    a[1] = (hex % 100) / 10 + 0x30;
    a[2] = (hex % 10) + 0x30;
    a[3] = '\0';
  } else if(hex >= 10) {
    a[0] = hex / 10 + 0x30;
    a[1] = hex % 10 + 0x30;
  } else
    a[0] = hex + 0x30;
  a[3] = '\0';
  return a;
}

static uint8_t atohex(char *s)
{
  uint8_t value = 0;
  if(!s)
    return 0;

  if(*s >= '0' && *s <= '9')
    value = (*s - '0') << 4;
  else if(*s >= 'A' && *s <= 'F')
    value = ((*s - 'A') + 10) << 4;
  else if(*s >= 'a' && *s <= 'f')
    value = ((*s - 'a') + 10) << 4;

  s++;

  if(*s >= '0' && *s <= '9')
    value |= (*s - '0');
  else if(*s >= 'A' && *s <= 'F')
    value |= ((*s - 'A') + 10);
  else if(*s >= 'a' && *s <= 'f')
    value |= ((*s - 'a') + 10);

  return value;
}

static uint8_t atodec(char *s)
{
  uint8_t value = 0;
  if(!s)
    return 0;

  if(*s >= '0' && *s <= '9')
    value = (*s - '0') * 10;
  
  s++;

  if(*s >= '0' && *s <= '9')
    value += (*s - '0');
  else
    value /= 10;

  return value;
}

static uint8_t XOR_Checksum(uint8_t *buf, uint16_t len)
{
  uint8_t checksum = 0;
  uint16_t i;
  for(i=0;i<len;i++)
    checksum = checksum ^ buf[i];
  return checksum;
}

/*
*
*/

static char *ibus_device_alias(uint8_t id) 
{
  switch(id) {
    case 0x00: return "GM";
    case 0x08: return "SHD";
    case 0x18: return "CDC";
    case 0x24: return "HKM";
    case 0x28: return "FUH";
    case 0x30: return "CCM";
    case 0x3b: return "NAV";
    case 0x3f: return "DIA";
    case 0x40: return "FBZV";
    case 0x43: return "GTF";
    case 0x44: return "EWS";
    case 0x46: return "CID";
    case 0x50: return "MFL";
    case 0x51: return "MM0";
    case 0x5b: return "IHK";
    case 0x60: return "PDC";
    case 0x68: return "RAD";
    case 0x6A: return "DSP";
    case 0x70: return "RDC";
    case 0x72: return "SM0";
    case 0x73: return "SDRS";
    case 0x76: return "CDCD";
    case 0x7f: return "NAVE";
    case 0x80: return "IKE";
    case 0x9b: return "MM1";
    case 0x9c: return "MM2";
    case 0xa0: return "FMID";
    case 0xa4: return "ABM";
    case 0xac: return "EHC";
    case 0xb0: return "SES";
    case 0xbb: return "NAVJ";
    case 0xbf: return "GLO";
    case 0xc0: return "MID";
    case 0xc8: return "TEL";
    case 0xd0: return "LCM";
    case 0xe0: return "IRIS";
    case 0xe7: return "ANZV";
    case 0xed: return "TV";
    case 0xf0: return "BMBT";
    case 0xff: return "LOC";
    default: return "Unknown";
  }
}

int IBus_Write2(uint8_t src, uint8_t dest, uint8_t *data, uint8_t dataLen) 
{
  //uint8_t code[MAX_TX_LEN];
  uint8_t *code = Usart3_TxBuf();
  code[0] = src; /* src */
  /* code[1] : The length of the packet whithout Source ID and length it-self. */
  code[2] = dest; /* dest */
  int i, len = 1; /* dest length is 1 */
  for(i=0;i<dataLen;i++) {
    code[i+3] = data[i]; /* data */
    len++;
  }
  len++; /* check sum length is 1 */
  code[1] = len; 
  code[i+3] = XOR_Checksum((uint8_t *)&code[0], len + 1);
#if 0
  Usart2_Puts("\r\n");
  Usart2_Printf("Source : %s\r\n", ibus_device_name(code[0]));
  Usart2_Printf("Length : 0x%x\r\n", code[1]);
  Usart2_Printf("Destination : %s\r\n", ibus_device_name(code[2]));
  Usart2_Printf("CRC : %s\r\n", hextoa(code[i+3])); /* All exclude CRC itself */
#endif
  Usart3_Write(&code[0], len + 2);

  return 0;
}

#define MAX_IKE_SCREEN_LENGTH 20

void IBus_RedrawIkeScreen(const char *text)
{
  uint8_t d[] = { 0x23, 0x62, 0x30, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x04 };

  if(text == 0 || strlen(text) <= 0) {
    IBus_Write2(0x68, 0x80, d, 4+MAX_IKE_SCREEN_LENGTH+1); /* Clear IKE screen */
    return;
  }

  uint8_t len = strlen(text);

  if(len > MAX_IKE_SCREEN_LENGTH)
    len = MAX_IKE_SCREEN_LENGTH;

  uint8_t *p = &d[4];
  uint8_t frontPorch = (MAX_IKE_SCREEN_LENGTH - len) >> 1; /* To centralize display text */
  memcpy(p + frontPorch, text, len);

  IBus_Write2(0x68, 0x80, d, 4+MAX_IKE_SCREEN_LENGTH+1); /* Display "12345678901234567890" on IKE - Text Screen (20) */
}

void IBus_SetTime(uint8_t hour, uint8_t minute)
{ 
  uint8_t d[] = { 0x40, 0x01, 0x00, 0x00 }; /* 0x40 : OBC set time, 0x01 : time, Others : 0:00 */ 
  d[2] = hour;
  d[3] = minute;
  IBus_Write2(0x3b, 0x80, d, 4); 
}

void IBus_SetDate(uint8_t day, uint8_t month, uint8_t year)
{ 
  uint8_t d[] = { 0x40, 0x02, 0x11, 0x04, 0x12 }; /* 0x40 : OBC set time, 0x02 : date, Others : 17/4/2018 */ 
  d[2] = day;
  d[3] = month;
  d[4] = year;
  IBus_Write2(0x3b, 0x80, d, 5); 
}

/*
*
*/

int KBus_Write(uint8_t *c, uint8_t len)
{
  uint8_t i;
  for(i=0;i<len;i++) {
    Usart1_Write(&c[i], 1);
    delay_ms(5);
  }
}

int GS8602_Send_0b03()
{
  uint8_t code[] = { 0x32, 0x05, 0x0b, 0x03, 0x3f };
  KBus_Write(&code[0], 5);
  return 0;
}

int ME72_Send_224000()
{
  uint8_t code[] = { 0xb8, 0x12, 0xf1, 0x03, 0x22, 0x40, 0x00, 0x3a };
  KBus_Write(&code[0], 8);
  return 0;
}

int ME72_Send_224004()
{
  uint8_t code[] = { 0xb8, 0x12, 0xf1, 0x03, 0x22, 0x40, 0x04, 0x3e };
  KBus_Write(&code[0], 8);
  return 0;
}

int IKE_Send_80048480()
{
  uint8_t code[] = { 0x80, 0x04, 0x00, 0x84 };
  KBus_Write(&code[0], 4);
  return 0;  
}

int IKE_CheckTroubleCode()
{
  uint8_t code[] = { 0x80, 0x05, 0x04, 0x00, 0x81 };
  //uint8_t code[] = { 0x80, 0x05, 0x04, 0x01, 0x80 };
  KBus_Write(&code[0], 5);
  return 0;  
}

int IKE_CheckMileage()
{
  uint8_t code[] = { 0x80, 0x05, 0x02, 0x01, 0x86 };
  //uint8_t code[] = { 0x80, 0x05, 0x04, 0x01, 0x80 };
  KBus_Write(&code[0], 5);
  return 0;  
}

int IKE_CheckVIN()
{
  uint8_t code[] = { 0x80, 0x05, 0x02, 0x02, 0x85 };
  //uint8_t code[] = { 0x80, 0x05, 0x04, 0x01, 0x80 };
  KBus_Write(&code[0], 5);
  return 0;  
}

int EWS_Init()
{
  uint8_t code[] = { 0x44, 0x04, 0x00, 0x40 };
  KBus_Write(&code[0], 4);
  return 0;  
}

/*
*
*/

uint8_t ikeDisplay[MAX_IKE_SCREEN_LENGTH];

void DBus_DecodeIke(const uint8_t *p)
{
  if(p[1] == 0x12) {
#if 0
    Usart2_Printf("Part number %x%x%x%x\r\n", p[3] & 0x0f, p[4], p[5], p[6]);
    snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "PN %x%x%x%x\r\n", p[3] & 0x0f, p[4], p[5], p[6]);
    IBus_RedrawIkeScreen(ikeDisplay);
#endif
  }
}

const char transTemp[] = "Trans Temp";

void DBus_DecodeEgs(const uint8_t *p)
{
  if(p[1] == 0x1c) {
    switch(currentItem) {
      case 3: snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %d%cC", transTemp, p[9] - 56, 0xa8);
        break;
    }
    IBus_RedrawIkeScreen(ikeDisplay);
#ifdef DEBUG    
    Usart2_Printf(ikeDisplay);
#endif
  }
}

#define endian_swap16(x) \
  ((((x)>>8) & 0x00ff) | (((x)<<8) & 0xff00))

const char coolantTemp[] = "Coolant Temp";
const char coolOutTemp[] = "Cool Out Temp";
const char intakeTemp[] = "Intake Temp";
const char airMass[] = "Air Mass";
const char batVoltage[] = "Battery";
const char engLoad[] = "Engine Load";
const char engRpm[] = "Engine RPM";
const char adaptAdd[] = "Add";
const char adaptMulti[] = "Multi";

void DBus_DecodeDme(const uint8_t *p)
{
  if(p[1] == 0xf1 && p[2] == 0x12) {
    uint16_t *t = 0;
    int16_t *a = 0;
    int16_t *b = 0;

    switch(currentItem) {
      case 1: snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %d%cC", coolantTemp, (int)p[22] * 3 / 4 - 48, 0xa8);
        break;
      case 2: snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %d%cC", coolOutTemp, (int)p[32] * 3 / 4 - 48, 0xa8);
        break;
      case 3: break;
      case 4: snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %d%cC", intakeTemp, (int)p[21] * 3 / 4 - 48, 0xa8);
        break;
      case 5: t = (uint16_t *)&p[25];
        snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %.2fkg/h", airMass, endian_swap16(*t) * 0.1);
        break;
      case 6: snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %.2fV", batVoltage, p[29] * 0.095);
        break;
      case 7: t = (uint16_t *)&p[27]; 
        snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %.2f%%", engLoad, endian_swap16(*t) * 0.0015259);
        break; 
      case 8: t = (uint16_t *)&p[14];
        snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %d", engRpm, endian_swap16(*t) >> 2);
        break;
      case 9: a = (int16_t *)&p[7];
        b = (int16_t *)&p[9];
        snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %.2f%% %.2f%%", adaptAdd, endian_swap16(*a) * 0.046875, endian_swap16(*b) * 0.046875);
        break;
      case 10: a = (int16_t *)&p[11];
        b = (int16_t *)&p[13];
        snprintf(ikeDisplay, MAX_IKE_SCREEN_LENGTH, "%s %.2f%% %.2f%%", adaptMulti, endian_swap16(*a) * 0.0000305, endian_swap16(*b) * 0.0000305);
        break;
      default:
        return;
    }
    IBus_RedrawIkeScreen(ikeDisplay);
#ifdef DEBUG    
    Usart2_Printf(ikeDisplay);
#endif
  }
}

/*
*
*/

#ifdef IGNITION_STATUS

#define IGN_OFF 0x00
#define IGN_ACC 0x01
#define IGN_POS2 0x02

uint8_t ignStatus = IGN_OFF;

#endif

void DBus_Request(void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // turn on all led

  switch(currentItem) {
    case 0: 
      break;
    case 3: GS8602_Send_0b03();
      break;
    case 9:
    case 10: ME72_Send_224004();
      break;
    default: ME72_Send_224000();
      break;
  }

  GPIO_SetBits(GPIOB, GPIO_Pin_12);  // turn off all led
}

#ifdef IGNITION_STATUS

void IBus_IgnitionRequest(void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // turn on all led

  uint8_t d[] = { 0x10 };
  IBus_Write2(0xbf, 0x80, d, 1); /* GLO -> IKE : Ignition status request  (Jochen) */

  GPIO_SetBits(GPIOB, GPIO_Pin_12);  // turn off all led
}

#endif

/*
*
*/

static uint16_t speed = 0, rpm = 0;

void IBus_DecodeIke(const uint8_t *p)
{
  if(p[2] == 0xbf) { /* GLO */
    switch(p[3]) { /* Message ID */
      case 0x11: { /* ignition status */
#if 0
/*
Bit 1 = KL_R (Pos1_Acc)
Bit 2 = KL_15 (Pos2_On)
Bit 3 = KL_50 (Pos3_Start)
*/
#endif
#ifdef IGNITION_STATUS
        ignStatus = p[4];
#ifdef DEBUG
        if(ignStatus == IGN_OFF)
          Usart2_Printf("\r\nIgn Off\r\n");
        else if(ignStatus == IGN_ACC)
          Usart2_Printf("\r\nIgn Pos1_Acc\r\n");
        else if(ignStatus == (IGN_ACC | IGN_POS2))
          Usart2_Printf("\r\nIgn Pos1_Acc Pos2_On\r\n");
        else
          Usart2_Printf("\r\nIgn Unknown\r\n");
#endif      
#endif
      } break;
      case 0x18: { /* Speed & RPM */
/*
80 05 BF 18 ss rr cc 

ss = speed / 2 [km/h] (512km/h max)
rr = revs / 100 rpm
*/
        speed = p[4] << 1;
        rpm = p[5] * 100;
      } break;
    }
  }
}

#if 0

static bool bRadioOn = false;
const uint8_t RADIO_POWER_OFF[] = { 0x68, 0x05, 0xe7, 0x23, 0x00, 0x20, 0x89 };

void IBus_DecodeRad(uint8_t *p)
{
  if(memcmp(RADIO_POWER_OFF, p, 6) == 0)
    bRadioOn = false;
  else if(p[2] == 0xe7) { /* ANZV : Front display */
    if((p[3] == 0x21 || p[3] == 0x23) && (p[4] == 0x40 || p[4] == 0xc0 || p[4] == 0x80))
      bRadioOn = true;
  }
#if 1
  Usart2_Printf("\r\nRadio Power %s\r\n", bRadioOn ? "On" : "Off");
#endif
}

#else

static bool bRadioOn = false;

void IBus_DecodeRad(uint8_t *p)
{
  if(p[2] == 0xe7) { /* ANZV */
    if(p[3] == 0x23 && p[4] == 0x00 && p[5] == 0x20 && p[6] == 0x89) /* Title field clear */
      bRadioOn = false;
    else if(p[3] == 0x21 && p[4] == 0x00 && p[5] == 0x00 && p[6] == 0x20 && p[7] == 0x88) /* Bottom field clear */
      bRadioOn = false;
    else if((p[3] == 0x23 || p[3] == 0x21) && (p[4] == 0x40 || p[4] == 0xc0 || p[4] == 0x80))
      bRadioOn = true;
  }
#if 1
  Usart2_Printf("\r\nRadio Power %s\r\n", bRadioOn ? "On" : "Off");
#endif
}

#endif

void CurrentItem_Update()
{
#ifdef IGNITION_STATUS
  if(currentItem >= MAX_ITEMS || (ignStatus != (IGN_ACC | IGN_POS2)))
#else
  if(currentItem >= MAX_ITEMS)
#endif
    currentItem = 0;

  switch(currentItem) {
      case 1: IBus_RedrawIkeScreen(coolantTemp);
        break;
      case 2: IBus_RedrawIkeScreen(coolOutTemp);
        break;
      case 3: IBus_RedrawIkeScreen(transTemp); 
        break;
      case 4: IBus_RedrawIkeScreen(intakeTemp);
        break;
      case 5: IBus_RedrawIkeScreen(airMass);
        break;
      case 6: IBus_RedrawIkeScreen(batVoltage);
        break;
      case 7: IBus_RedrawIkeScreen(engLoad);
        break;
      case 8: IBus_RedrawIkeScreen(engRpm);
        break;
      case 9: IBus_RedrawIkeScreen(adaptAdd);
        break;
      case 10: IBus_RedrawIkeScreen(adaptMulti);
        break;
      default: IBus_RedrawIkeScreen("Enhanced-IKE Off");
        e_ike_tick = tim4Tick + e_ike_interval;
        return;
    }
}

void Btn_Next_Released()
{
  if(bRadioOn == true)
    return;
  currentItem++;
  CurrentItem_Update();
}

void Btn_Prev_Released()
{
  if(bRadioOn == true)
    return;
  if(currentItem > 0)
    currentItem--;
  else if(currentItem == 0)
    currentItem = MAX_ITEMS - 1;

  CurrentItem_Update();
}

void Btn_RT_Telephone()
{
  currentItem++;
  CurrentItem_Update();
}

void IBus_DecodeMfl(const uint8_t *p)
{ 
  const uint8_t BTN_NEXT_PRESSED[] = { 0x3b, 0x01, 0x06 };
  const uint8_t BTN_NEXT_RELEASED[] = { 0x3b, 0x21, 0x26 };

  const uint8_t BTN_PREV_PRESSED[] = { 0x3b, 0x08, 0x0f };
  const uint8_t BTN_PREV_RELEASED[] = { 0x3b, 0x28, 0x2f };

  const uint8_t BTN_VOLUME_UP[] = { 0x32, 0x11, 0x1f };
  const uint8_t BTN_VOLUME_DOWN[] = { 0x32, 0x10, 0x1e };

  const uint8_t BTN_RT_TELEPHONE_ON[] = { 0x3b, 0x40, 0xe7 };
  const uint8_t BTN_RT_TELEPHONE_OFF[] = { 0x3b, 0x00, 0xa7 };

  const uint8_t BTN_TELEPHONE_PRESSED[] = { 0x3b, 0x80, 0x27 };
  const uint8_t BTN_TELEPHONE_KEEP_PRESSED[] = { 0x3b, 0x90, 0x37 };
  const uint8_t BTN_TELEPHONE_RELEASED[] = { 0x3b, 0xa0, 0x07 };

  if(p[0] == 0x50 && p[1] == 0x04) {
    if(p[2] == 0x68) { /* To Radio */
      if(memcmp(BTN_NEXT_PRESSED, &p[3], 3) == 0) {
      } else if(memcmp(BTN_NEXT_RELEASED, &p[3], 3) == 0) {
        Btn_Next_Released();
      } else if(memcmp(BTN_PREV_PRESSED, &p[3], 3) == 0) {
      } else if(memcmp(BTN_PREV_RELEASED, &p[3], 3) == 0) {
        Btn_Prev_Released();
      } else if(memcmp(BTN_VOLUME_UP, &p[3], 3) == 0) {
      } else if(memcmp(BTN_VOLUME_DOWN, &p[3], 3) == 0) {
      }
    } else if(p[2] == 0xc8) { /* To Telephone */
      if(memcmp(BTN_RT_TELEPHONE_ON, &p[3], 3) == 0) {
        Btn_RT_Telephone();
      } else if(memcmp(BTN_RT_TELEPHONE_OFF, &p[3], 3) == 0) {
        Btn_RT_Telephone();
      } else if(memcmp(BTN_TELEPHONE_PRESSED, &p[3], 3) == 0) {
      } else if(memcmp(BTN_TELEPHONE_KEEP_PRESSED, &p[3], 3) == 0) {
      } else if(memcmp(BTN_TELEPHONE_RELEASED, &p[3], 3) == 0) {
      } 
    }
  }
}

const uint8_t BTN_MID_TOKEN[] = { 0x31, 0x80, 0x00 };  

void IBus_DecodeMid(uint8_t *p)
{
  if(p[2] == 0x68) { /* To Radio */
    if(memcmp(&p[3], BTN_MID_TOKEN, 3) == 0) {
      if(p[6] & 0xf0) { /* Button released */
        switch(p[6] & 0x0f) {
          case 0: break;
          case 1: break;
          case 2: break;
          case 3: break;
          case 4: break;
          case 5: break;
          case 6: break;
          case 7: break;
          case 8: break;
          case 9: break;
          case 10: break;
          case 11: break;
          case 12: break;
        }
      } else { /* Button pressed */
//Usart2_Printf("\r\nRadio button %d pressed\r\n", p[6] & 0x0f);
        switch(p[6] & 0x0f) {
          case 0: break;
          case 1: break;
          case 2: break;
          case 3: break;
          case 4: break;
          case 5: break;
          case 6: break;
          case 7: break;
          case 8: break;
          case 9: break;
          case 10: break;
          case 11: break;
          case 12: break;
        }
      }
    }
  }
}

/*
*
*/

#define MAX_CMD_LEN 32

typedef struct {
  char data[MAX_CMD_LEN + 1];
  int len;
} Shell;

void Shell_Input(Shell *s, char c)
{
  if(s->len < (MAX_CMD_LEN - 1)) {
    if(c == 0x8) { /* backspace */
      s->data[--s->len] = '\0';
      return;
    } else if(c < 0x20)
      return;

    s->data[s->len++] = c;
    s->data[s->len] = '\0';
  }
}

char *Shell_InputString(Shell *s)
{
  return s->len > 0 ? s->data : 0;
}

void Shell_InputReset(Shell *s)
{
  memset(s->data, 0, MAX_CMD_LEN + 1);
  s->len = 0;
}

int Shell_Run(Shell *s)
{
  if(s->len == 0)
    return -1;

  int ret = -1;

  char *p = 0;
#define MAX_ARGC 16 
  char *argv[MAX_ARGC];
  uint16_t argc = 0;

  int i;
  for(i=(s->len-1);i>=0;i--) {
    if(s->data[i] <= 0x20 || s->data[i] > 0x7e) {
      s->data[i] = '\0'; /* Strip back */
      s->len--;
    } else
      break;
  }

  for(i=0;i<s->len;i++) {
    if(s->data[i] > 0x20 && s->data[i] <= 0x7e)
      break; 
    s->data[i] = '\0'; /* Strip front */
    s->len--;
  }

  p = &s->data[i];
  i = 0;
  while(i < s->len) {
    if(p[i] == 0x20) {
      p[i++] = '\0';
      continue;
    }
    argv[argc++] = &p[i++];
    while(p[i] > 0x20 && p[i] <= 0x7e)
      i++;
    p[i++] = '\0'; /* end string */
    if(argc >= MAX_ARGC)
      break;
  }

  if(argc >= 2 && strcmp("scan", argv[0]) == 0) {
    if(strcmp("ike", argv[1]) == 0)
      ret = IKE_Send_80048480();
      //ret = IKE_CheckTroubleCode();    
    else if(strcmp("egs", argv[1]) == 0)
      ret = GS8602_Send_0b03();
    else if(strcmp("dme", argv[1]) == 0)
      ret = ME72_Send_224000();
  } else if(argc >= 2 && strcmp("btn", argv[0]) == 0) {
    if(strcmp("rt", argv[1]) == 0) {
      Btn_RT_Telephone();
      ret = 0;
    } else if(strcmp("next", argv[1]) == 0) {
      Btn_Next_Released();
      ret = 0;
    } else if(strcmp("prev", argv[1]) == 0) {
      Btn_Prev_Released();
      ret = 0;
    }
  } else if(strcmp("status", argv[0]) == 0) {
    Usart2_Printf("Speed %d km/h, %d RPM\r\n", speed, rpm);
    Usart2_Printf("\r\nRadio Power %s\r\n", bRadioOn ? "On" : "Off");
    ret = 0;
  } else if(argc >= 3 && strcmp("time", argv[0]) == 0) {
    IBus_SetTime(atodec(argv[1]), atodec(argv[2])); /* hour, minute */
    ret = 0;
  } else if(argc >= 4 && strcmp("date", argv[0]) == 0) { 
    IBus_SetDate(atodec(argv[1]), atodec(argv[2]), atodec(argv[3])); /* day, month, year */
    ret = 0;
  }

  Shell_InputReset(s);

  return ret;
}

static Shell shell;

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Tim4_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* NVIC_PriorityGroup */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  //基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  
  TIM_DeInit(TIM4);

  TIM_TimeBaseStructure.TIM_Period = 1000;//装载值
  //prescaler is 72, that is 72000000/72/1000 = 1000Hz;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;//分频系数
  //set clock division 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
  //count up
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //clear the TIM4 overflow interrupt flag
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  //TIM4 overflow interrupt enable
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //enable TIM4
  TIM_Cmd(TIM4, DISABLE);
}

void Tim4_Enable(void)
{
  TIM_Cmd(TIM4, ENABLE);
}
#if 0

void Tim4_1ms(void)
{
}

void Tim4_10ms(void)
{
}

void Tim4_50ms(void)
{  
}

void Tim4_100ms(void)
{
}

void Tim4_200ms(void)
{
}
#endif
void Tim4_1000ms(void)
{
#if 0  
  static uint8_t c = 0x0;
  char str[5];
  snprintf(str, 5, "%s%c", hextoa(c), c);
  IBus_RedrawIkeScreen(str);
  c++;
#endif
#ifdef IGNITION_STATUS
  static uint8_t counter = 0;
  if(counter++ >= 10) {
    IBus_IgnitionRequest();
    counter = 0;
  }
#endif
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
  
  delay_init(72); /* 72 MHz system clock */

  /* GPIOB Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure PB12 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB, GPIO_Pin_12); /* pull high */
#if 0
  /* GPIOA and GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_1);

  ADC1_Init();

  Pwm_Init();
  Pwm1_Reverse();
  Pwm1_Pulse(0);
#endif

  Usart2_Init(115200);
  Usart2_Puts("\r\nKBus\\> ");

  while(1) {
#if 0
    /* K-Line KWP 2000 fast initialization */
    USART_DeInit(USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_SetBits(GPIOA, GPIO_Pin_9);
    delay_ms(25);
    GPIO_ResetBits(GPIOA, GPIO_Pin_9);
    delay_ms(25);
    GPIO_SetBits(GPIOA, GPIO_Pin_9);
    delay_ms(25);

    Usart1_Init(10400); /* D-Bus operates at a rate of 9.6 Kbps */
 
    uint8_t initialization_code[] = { 0xC1, 0x33, 0xF1, 0x81, 0x66 };
    Usart1_Write(&initialization_code[0], 5);

    delay_ms(1200);

    int len = Usart1_Poll();
    if(len > 0) {
      char *p = Usart1_Gets();
      Usart2_Puts("\r\n");
      int i;
      for(i=0;i<len;i++) {
          Usart2_Write((uint8_t *)hextoa(p[i]), 2);
          Usart2_Write((uint8_t *)" ", 1);
      }
      Usart2_Puts("\r\n");          
      Usart2_Puts("KWP2000 fast init okay\r\n");
      USART_DeInit(USART1);
      break;
    } else
      Usart2_Puts("KWP2000 fast init failure\r\n");

    /* K-Line slow initialization KWP 2000 or ISO 9141-2 */
    USART_DeInit(USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_SetBits(GPIOA, GPIO_Pin_9);

    delay_ms(2600); //2,6s delay before attempting 5 baud init

    /* ISO 9141 with 5-baud initialization. starts with a 200ms low, 400ms high/low/high/low, 227ms high */
    GPIO_ResetBits(GPIOA, GPIO_Pin_9);
    delay_ms(200);
    GPIO_SetBits(GPIOA, GPIO_Pin_9);
    delay_ms(400);
    GPIO_ResetBits(GPIOA, GPIO_Pin_9);
    delay_ms(400);
    GPIO_SetBits(GPIOA, GPIO_Pin_9);
    delay_ms(400);
    GPIO_ResetBits(GPIOA, GPIO_Pin_9);
    delay_ms(400);
    GPIO_SetBits(GPIOA, GPIO_Pin_9);
    delay_ms(227);

    Usart1_Init(10400); /* D-Bus operates at a rate of 9.6 Kbps */
    bool use_iso9141 = false;
    len = Usart1_Poll3(3 /*desired length*/, 500 /*timeout ms*/);
    if(len == 3) {
      char *p = Usart1_Gets();
      if(p[0] == 0x55){ //first response byte must always be 0x55
        if((p[1] == 0x08 && p[2] == 0x08) ||
          (p[1] == 0x94 && p[2] == 0x94)) //if both KB1 and KB2 are 0x08 or 0x94 then use ISO 9141-2
          use_iso9141 = true;

        p[2] = ~p[2];
        Usart1_Write(&p[2], 1);  //reply with inverted KB2
        len = Usart1_Poll3(1 /*desired length*/, 100 /*timeout ms*/);
        if(len == 1) {
          if(use_iso9141)
            Usart2_Puts("ISO9141 slow init okay\r\n");
          else
            Usart2_Puts("KWP slow init okay\r\n");
          USART_DeInit(USART1);
          break;
        }
      }
    }

    Usart2_Puts("Slow init failure\r\n");
    USART_DeInit(USART1);
#endif
    break;
  }

  Usart1_Init(9600); /* D-Bus operates at a rate of 9.6 Kbps */
#ifdef USART3_ENABLE
  Usart3_Init(9600); /* I-Bus operates at a rate of 9.6 Kbps */
#endif

  Tim4_Init();
  Tim4_Enable();

#if 0
  uint32_t tick_1ms = tim4Tick_1ms;
  uint32_t tick_10ms = tim4Tick_10ms;
  uint32_t tick_50ms = tim4Tick_50ms;
  uint32_t tick_100ms = tim4Tick_100ms;  
  uint32_t tick_200ms = tim4Tick_200ms;
#endif
  uint32_t tick_1000ms = tim4Tick_1000ms;

  e_ike_tick = tim4Tick + e_ike_interval; /* Restore */

  /* To achieve GPIO toggling maximum frequency, the following  sequence is mandatory. 
     You can monitor PD0 or PD2 on the scope to measure the output signal. 
     If you need to fine tune this frequency, you can add more GPIO set/reset 
     cycles to minimize more the infinite loop timing.
     This code needs to be compiled with high speed optimization option.  */
  for(;;) {
#if 0    
    if(tick_1ms != tim4Tick_1ms) {
      tick_1ms = tim4Tick_1ms;
      Tim4_1ms();
    }

    if(tick_10ms != tim4Tick_10ms) {
      tick_10ms = tim4Tick_10ms;
      Tim4_10ms();
    }

    if(tick_50ms != tim4Tick_50ms) {
      tick_50ms = tim4Tick_50ms;
      Tim4_50ms();
    }

    if(tick_100ms != tim4Tick_100ms) {
      tick_100ms = tim4Tick_100ms;
      Tim4_100ms();
    }

    if(tick_200ms != tim4Tick_200ms) {
      tick_200ms = tim4Tick_200ms;
      Tim4_200ms();
    }
#endif
    if(tick_1000ms != tim4Tick_1000ms) {
      tick_1000ms = tim4Tick_1000ms;
      Tim4_1000ms();
    }

    if(tim4Tick >= e_ike_tick) {
      DBus_Request();
      e_ike_tick = tim4Tick + e_ike_interval; /* Restore */
    } else if(e_ike_tick > tim4Tick && e_ike_tick - tim4Tick > 0xffffff) { /* timer tick over flow */
      e_ike_tick = tim4Tick + e_ike_interval; /* Restore */      
    }
    
    int len = Usart2_Poll();
    if(len > 0) {
      uint8_t *p = (uint8_t *)Usart2_Gets();
#if 1      
      int i;
      for(i=0;i<len;i++) {
        if(p[i] == 0xd) { /* CR */
          char *sc = Shell_InputString(&shell);
          if(sc)
            if(Shell_Run(&shell) == -1)
              Usart2_Puts("\r\nIllegal command\r\nKBus\\> ");
            else
              Usart2_Puts("\r\nKBus\\> ");
          else
            Usart2_Puts("\r\nKBus\\> ");
        } else if((p[i] >= 0x20 && p[i] <= 0x7e) || p[i] == 0x8) {
          if(p[i] == 0x8) /*backspace */
            Usart2_Puts("\b \b"); /* backspace + space + backspace */
          else 
            Usart2_Write((uint8_t *)&p[i], 1);
          Shell_Input(&shell, p[i]);
        }
      }
#endif      
    }

#ifdef USART3_ENABLE
    len = Usart3_Poll();
/*    
      if(len > 0) 
        Usart2_Write(Usart3_Gets(), len);
*/
    if(len > 0) {
      char *p = Usart3_Gets();
#ifdef DEBUG
      Usart2_Puts("<IBus>\r\n");
      int i;
      for(i=0;i<len;i++) {
        Usart2_Write((uint8_t *)hextoa(p[i]), 2);
        Usart2_Write((uint8_t *)" ", 1);
        if(i == 2 || i == (len - 2))
          Usart2_Write((uint8_t *)"| ", 2);
      }
      Usart2_Puts("\r\n");
      Usart2_Printf(" %s --> %s : ", ibus_device_alias(p[0]), ibus_device_alias(p[2]));
      for(i=0;i<len;i++) {
        if(p[i] < 0x20 || p[i] > 0x7e)
          Usart2_Write((uint8_t *)".", 1);
        else
          Usart2_Write((uint8_t *)&p[i], 1);
      }
      Usart2_Puts("\r\n");
#endif
      switch(p[0]) { /* src */
        case 0x50: /* MFL Multi Functional Steering Wheel Buttons */
          IBus_DecodeMfl(&p[0]);
          break;
        case 0x80: /* IKE Instrument Control Electronics */
          IBus_DecodeIke(&p[0]);
          break;
        case 0x68: /* Radio */
          IBus_DecodeRad(&p[0]);
          break;
        case 0xc0: /* Multi-info display */
          IBus_DecodeMid(&p[0]);
          break;
      }
    }
#endif

#ifdef USART1_ENABLE    
    len = Usart1_Poll();
/*    
      if(len > 0) 
        Usart2_Write(Usart1_Gets(), len);
*/
    if(len > 0) {
      char *p = Usart1_Gets();
#ifdef DEBUG
      Usart2_Puts("<KBus>\r\n");

      int i;
      for(i=0;i<len;i++) {
        Usart2_Write((uint8_t *)hextoa(p[i]), 2);
        Usart2_Write((uint8_t *)" ", 1);
      }
      Usart2_Puts("\r\n");
#endif
      if((p[0] == 0xb8 && p[1] == 0x12 && p[2] == 0xf1) ||
        (p[1] == 0x04 && p[2] == 0x00)) { /* D-Bus busy */
        e_ike_tick = tim4Tick + 30000; /* wait 30 secs */
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // turn on all led
      } else {
        switch(p[0]) {
          case 0x32: /* EGS */
            DBus_DecodeEgs(&p[0]);
            break;
          case 0x80: /* IKE */
            DBus_DecodeIke(&p[0]);
            break;
          case 0xb8: /* DME */
            DBus_DecodeDme(&p[0]);
            break;
        }
      }
    }
#endif        
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
    tim4Tick++;
#if 0    
    tim4Tick_1ms++;
    if(tim4Tick % 10 == 0)
      tim4Tick_10ms++;
    if(tim4Tick % 50 == 0)
      tim4Tick_50ms++;
    if(tim4Tick % 100 == 0)
      tim4Tick_100ms++;
    if(tim4Tick % 200 == 0)
      tim4Tick_200ms++;
#endif            
    if(tim4Tick % 1000 == 0)
      tim4Tick_1000ms++;

#if (defined USART1_IBUS) && (defined USART1_ENABLE)
    usart1_idle_tick++;
#endif
#if (defined USART2_IBUS) && (defined USART2_ENABLE)
    usart2_idle_tick++;
#endif
#if (defined USART3_IBUS) && (defined USART3_ENABLE)
    usart3_idle_tick++;
#endif
    //
    // 清除 TIM4
    TIM_ClearITPendingBit(TIM4, /*TIM_IT_Update*/ TIM_FLAG_Update);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
