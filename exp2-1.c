
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c1294ncpdt.h"
#include "interrupt.h"
#include "systick.h"
#include "uart.h"
#include "string.h"
#include "qei.h"
#include "pwm.h"
#include "eeprom.h"

//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 0x22
#define PCA9557_I2CADDR 0x18

#define PCA9557_INPUT 0x00
#define PCA9557_OUTPUT 0x01
#define PCA9557_POLINVERT 0x02
#define PCA9557_CONFIG 0x03

#define TCA6424_CONFIG_PORT0 0x0c
#define TCA6424_CONFIG_PORT1 0x0d
#define TCA6424_CONFIG_PORT2 0x0e

#define TCA6424_INPUT_PORT0 0x00
#define TCA6424_INPUT_PORT1 0x01
#define TCA6424_INPUT_PORT2 0x02

#define TCA6424_OUTPUT_PORT0 0x04
#define TCA6424_OUTPUT_PORT1 0x05
#define TCA6424_OUTPUT_PORT2 0x06

#define SYSTICK_FREQUENCY 1000

#define MASTER_MODE_CALENDER 0
#define MASTER_MODE_SOLAR_TERMS 1
#define MASTER_MODE_TIME 2
#define MASTER_MODE_COUNTDOWN 3
#define MASTER_MODE_ALARM 4
#define MASTER_MODE_BOOT 5
#define NUMBER_OF_MASTER_MODES 5 //排除启动模式.
#define TIME_MODE_DISPLAY 0
#define TIME_MODE_SET 1
#define CALENDER_MODE_DISPLAY 0
#define CALENDER_MODE_SET 1
#define COUNTDOWN_MODE_CLEAR 0
#define COUNTDOWN_MODE_FORWARD 1
#define COUNTDOWN_MODE_PAUSE 2
#define COUNTDOWN_MODE_TIMEOUT 3
#define COUNTDOWN_MODE_SET 4
#define ALARM_MODE_DISPLAY 0
#define ALARM_MODE_SET 1
#define ALARM_MODE_RINGING 2
#define NUMBER_OF_ALARMS 2

#define SECONDS_IN_TROPICAL_YEAR 31556925

void Delay(uint32_t value);
void UARTStringPut(const char *cMessage);

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);

void S800_GPIO_Init(void);
void S800_I2C0_Init(void);
void S800_Int_Init(void);
void S800_UART_Init(void);
void S800_QEI_Init(void);
void S800_PWM_Init(void);

void flash_seg(uint8_t display_index, uint8_t control_word);
uint32_t ui32SysClock;

struct PeripheralDeviceInput
{
	uint8_t keyPadStateByte;
	uint8_t buttonStateByte;
	char UARTMessage[100];
	char *UARTMessageTail;
	uint16_t UARTMessageReceiveFinishedCountdown;
} peripheralDeviceInput;

struct PeripheralDeviceOutput
{
	uint8_t segmentDisplayControlWord[8];
	uint8_t LEDDisplayByte;
	uint32_t beepFrequency; //Set to 0 to turn off.

} peripheralDeviceOutput;

struct Time
{
	uint16_t year;
	uint16_t month;
	uint16_t day;
	uint16_t hour;
	uint16_t minute;
	uint16_t second;
};
const uint16_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
const uint16_t daysInMonthInLeapYear[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
const uint16_t daysInYear = 365;
const uint16_t daysInLeapYear = 366;
const uint64_t solarTermsTimestampIn2021[] = {160978820500 + 8 * 3600 * 100,
											  161105999000 + 8 * 3600 * 100,
											  161233552700 + 8 * 3600 * 100,
											  161361623600 + 8 * 3600 * 100,
											  161490562000 + 8 * 3600 * 100,
											  161620424700 + 8 * 3600 * 100,
											  161751450600 + 8 * 3600 * 100,
											  161883560200 + 8 * 3600 * 100,
											  162016843000 + 8 * 3600 * 100,
											  162151062600 + 8 * 3600 * 100,
											  162286152500 + 8 * 3600 * 100,
											  162421752900 + 8 * 3600 * 100,
											  162557672800 + 8 * 3600 * 100,
											  162693518400 + 8 * 3600 * 100,
											  162829043700 + 8 * 3600 * 100,
											  162963929600 + 8 * 3600 * 100,
											  163097957500 + 8 * 3600 * 100,
											  163230966300 + 8 * 3600 * 100,
											  163362834100 + 8 * 3600 * 100,
											  163493586900 + 8 * 3600 * 100,
											  163623232600 + 8 * 3600 * 100,
											  163751962300 + 8 * 3600 * 100,
											  163879902300 + 8 * 3600 * 100,
											  164007355800 + 8 * 3600 * 100};
const uint16_t notes[][2] = {
	{-2,
	 2},
	{3,
	 2},
	{-2,
	 6},
	{3,
	 6},
	{-2,
	 10},
	{3,
	 10},
	{-2,
	 14},
	{3,
	 14},
	{-2,
	 16},
	{3,
	 16},
	{-2,
	 19},
	{3,
	 19},
	{-2,
	 23},
	{3,
	 23},
	{-2,
	 28},
	{3,
	 28},
	{-2,
	 36},
	{3,
	 36},
	{-2,
	 41},
	{3,
	 41},
	{-2,
	 45},
	{3,
	 45},
	{-2,
	 48},
	{3,
	 48},
	{-2,
	 51},
	{3,
	 51},
	{-2,
	 54},
	{3,
	 54},
	{-2,
	 58},
	{3,
	 58},
	{-2,
	 63},
	{3,
	 63},
	{-2,
	 71},
	{3,
	 71},
	{-2,
	 76},
	{3,
	 76},
	{-2,
	 80},
	{3,
	 80},
	{-2,
	 83},
	{3,
	 83},
	{-2,
	 85},
	{3,
	 85},
	{-2,
	 89},
	{3,
	 89},
	{-2,
	 93},
	{3,
	 93},
	{-2,
	 97},
	{3,
	 97},
	{-2,
	 106},
	{3,
	 106},
	{-2,
	 110},
	{3,
	 110},
	{-2,
	 115},
	{3,
	 115},
	{-2,
	 118},
	{3,
	 118},
	{-2,
	 120},
	{3,
	 120},
	{-2,
	 123},
	{3,
	 123},
	{-2,
	 128},
	{3,
	 128},
	{52,
	 134},
	{50,
	 134},
	{51,
	 134},
	{48,
	 135},
	{49,
	 135},
	{47,
	 135},
	{46,
	 135},
	{45,
	 135},
	{44,
	 135},
	{43,
	 135},
	{42,
	 135},
	{41,
	 135},
	{40,
	 135},
	{38,
	 136},
	{39,
	 136},
	{36,
	 136},
	{37,
	 136},
	{35,
	 136},
	{34,
	 136},
	{33,
	 136},
	{32,
	 136},
	{31,
	 136},
	{30,
	 136},
	{29,
	 136},
	{28,
	 136},
	{-2,
	 142},
	{3,
	 142},
	{-2,
	 144},
	{3,
	 144},
	{1,
	 145},
	{3,
	 146},
	{-2,
	 151},
	{3,
	 151},
	{-2,
	 153},
	{3,
	 153},
	{1,
	 154},
	{3,
	 155},
	{-2,
	 159},
	{3,
	 159},
	{-2,
	 161},
	{3,
	 161},
	{1,
	 163},
	{3,
	 164},
	{3,
	 167},
	{6,
	 168},
	{6,
	 171},
	{8,
	 172},
	{-2,
	 177},
	{3,
	 177},
	{-2,
	 179},
	{3,
	 179},
	{1,
	 180},
	{3,
	 181},
	{-2,
	 185},
	{3,
	 185},
	{-2,
	 188},
	{3,
	 188},
	{1,
	 189},
	{3,
	 190},
	{-2,
	 194},
	{3,
	 194},
	{-2,
	 196},
	{3,
	 196},
	{1,
	 197},
	{3,
	 198},
	{6,
	 202},
	{8,
	 203},
	{3,
	 206},
	{6,
	 207},
	{-2,
	 211},
	{3,
	 211},
	{-2,
	 214},
	{3,
	 214},
	{1,
	 215},
	{3,
	 216},
	{-2,
	 220},
	{3,
	 220},
	{-2,
	 222},
	{3,
	 222},
	{1,
	 223},
	{3,
	 225},
	{-2,
	 229},
	{3,
	 229},
	{-2,
	 231},
	{3,
	 231},
	{1,
	 232},
	{3,
	 233},
	{3,
	 236},
	{6,
	 238},
	{6,
	 241},
	{8,
	 242},
	{-2,
	 246},
	{3,
	 246},
	{-2,
	 248},
	{3,
	 248},
	{1,
	 250},
	{3,
	 251},
	{-2,
	 255},
	{3,
	 255},
	{-2,
	 257},
	{3,
	 257},
	{1,
	 258},
	{3,
	 259},
	{-2,
	 264},
	{3,
	 264},
	{-2,
	 266},
	{3,
	 266},
	{1,
	 267},
	{3,
	 268},
	{8,
	 269},
	{6,
	 271},
	{8,
	 272},
	{6,
	 273},
	{3,
	 275},
	{6,
	 276},
	{15,
	 278},
	{17,
	 280},
	{18,
	 282},
	{20,
	 284},
	{22,
	 286},
	{27,
	 291},
	{25,
	 293},
	{22,
	 295},
	{15,
	 300},
	{22,
	 304},
	{20,
	 306},
	{18,
	 308},
	{17,
	 310},
	{15,
	 313},
	{17,
	 315},
	{18,
	 317},
	{20,
	 319},
	{22,
	 321},
	{20,
	 326},
	{18,
	 328},
	{17,
	 330},
	{15,
	 332},
	{17,
	 334},
	{18,
	 336},
	{17,
	 339},
	{15,
	 341},
	{14,
	 343},
	{17,
	 345},
	{15,
	 347},
	{17,
	 350},
	{18,
	 352},
	{20,
	 354},
	{22,
	 356},
	{27,
	 360},
	{25,
	 363},
	{22,
	 365},
	{15,
	 369},
	{22,
	 373},
	{20,
	 376},
	{18,
	 378},
	{17,
	 380},
	{15,
	 382},
	{17,
	 384},
	{18,
	 386},
	{20,
	 389},
	{22,
	 391},
	{20,
	 395},
	{18,
	 397},
	{17,
	 400},
	{18,
	 404},
	{20,
	 408},
	{22,
	 413},
	{10,
	 417},
	{15,
	 417},
	{17,
	 419},
	{18,
	 421},
	{20,
	 423},
	{18,
	 426},
	{22,
	 426},
	{27,
	 430},
	{25,
	 432},
	{18,
	 434},
	{22,
	 434},
	{10,
	 439},
	{15,
	 439},
	{22,
	 443},
	{20,
	 445},
	{18,
	 447},
	{17,
	 450},
	{11,
	 452},
	{15,
	 452},
	{17,
	 454},
	{18,
	 456},
	{20,
	 458},
	{18,
	 460},
	{22,
	 460},
	{20,
	 465},
	{18,
	 467},
	{13,
	 469},
	{17,
	 469},
	{15,
	 471},
	{17,
	 473},
	{18,
	 476},
	{13,
	 478},
	{17,
	 478},
	{15,
	 480},
	{14,
	 482},
	{17,
	 484},
	{10,
	 486},
	{15,
	 486},
	{17,
	 489},
	{18,
	 491},
	{20,
	 493},
	{18,
	 495},
	{22,
	 495},
	{27,
	 500},
	{25,
	 502},
	{18,
	 504},
	{22,
	 504},
	{10,
	 508},
	{15,
	 508},
	{22,
	 513},
	{20,
	 515},
	{18,
	 517},
	{17,
	 519},
	{11,
	 521},
	{15,
	 521},
	{17,
	 523},
	{18,
	 526},
	{20,
	 528},
	{18,
	 530},
	{22,
	 530},
	{20,
	 534},
	{18,
	 536},
	{17,
	 539},
	{18,
	 543},
	{20,
	 547},
	{22,
	 552},
	{25,
	 556},
	{27,
	 558},
	{22,
	 560},
	{20,
	 563},
	{18,
	 565},
	{22,
	 565},
	{20,
	 569},
	{22,
	 571},
	{25,
	 573},
	{27,
	 576},
	{22,
	 578},
	{20,
	 580},
	{17,
	 582},
	{22,
	 582},
	{20,
	 586},
	{22,
	 589},
	{20,
	 591},
	{18,
	 593},
	{17,
	 595},
	{13,
	 597},
	{10,
	 600},
	{15,
	 600},
	{13,
	 604},
	{15,
	 606},
	{17,
	 608},
	{18,
	 610},
	{20,
	 613},
	{22,
	 615},
	{10,
	 617},
	{15,
	 617},
	{22,
	 621},
	{25,
	 623},
	{25,
	 626},
	{27,
	 628},
	{22,
	 630},
	{20,
	 632},
	{18,
	 634},
	{22,
	 634},
	{20,
	 639},
	{22,
	 641},
	{25,
	 643},
	{27,
	 645},
	{22,
	 647},
	{20,
	 650},
	{17,
	 652},
	{22,
	 652},
	{20,
	 656},
	{22,
	 658},
	{20,
	 660},
	{18,
	 663},
	{17,
	 665},
	{13,
	 667},
	{10,
	 669},
	{15,
	 669},
	{13,
	 673},
	{15,
	 676},
	{17,
	 678},
	{18,
	 680},
	{20,
	 682},
	{22,
	 684},
	{10,
	 686},
	{15,
	 686},
	{22,
	 691},
	{34,
	 691},
	{25,
	 693},
	{37,
	 693},
	{25,
	 695},
	{37,
	 695},
	{27,
	 697},
	{39,
	 697},
	{22,
	 700},
	{34,
	 700},
	{20,
	 702},
	{32,
	 702},
	{22,
	 704},
	{34,
	 704},
	{20,
	 708},
	{32,
	 708},
	{22,
	 710},
	{34,
	 710},
	{25,
	 713},
	{37,
	 713},
	{27,
	 715},
	{39,
	 715},
	{22,
	 717},
	{34,
	 717},
	{20,
	 719},
	{32,
	 719},
	{22,
	 721},
	{34,
	 721},
	{20,
	 726},
	{32,
	 726},
	{22,
	 728},
	{34,
	 728},
	{20,
	 730},
	{32,
	 730},
	{18,
	 732},
	{30,
	 732},
	{17,
	 734},
	{29,
	 734},
	{13,
	 736},
	{25,
	 736},
	{15,
	 739},
	{27,
	 739},
	{13,
	 743},
	{25,
	 743},
	{15,
	 745},
	{27,
	 745},
	{17,
	 747},
	{29,
	 747},
	{18,
	 750},
	{30,
	 750},
	{20,
	 752},
	{32,
	 752},
	{22,
	 754},
	{34,
	 754},
	{15,
	 756},
	{27,
	 756},
	{22,
	 760},
	{34,
	 760},
	{25,
	 763},
	{37,
	 763},
	{25,
	 765},
	{37,
	 765},
	{27,
	 767},
	{39,
	 767},
	{22,
	 769},
	{34,
	 769},
	{20,
	 771},
	{32,
	 771},
	{22,
	 773},
	{34,
	 773},
	{20,
	 778},
	{32,
	 778},
	{22,
	 780},
	{34,
	 780},
	{25,
	 782},
	{37,
	 782},
	{27,
	 784},
	{39,
	 784},
	{22,
	 786},
	{34,
	 786},
	{20,
	 789},
	{32,
	 789},
	{22,
	 791},
	{34,
	 791},
	{27,
	 795},
	{39,
	 795},
	{29,
	 797},
	{41,
	 797},
	{30,
	 800},
	{42,
	 800},
	{29,
	 802},
	{41,
	 802},
	{27,
	 804},
	{39,
	 804},
	{25,
	 806},
	{37,
	 806},
	{22,
	 808},
	{34,
	 808},
	{20,
	 813},
	{32,
	 813},
	{22,
	 815},
	{34,
	 815},
	{20,
	 817},
	{32,
	 817},
	{18,
	 819},
	{30,
	 819},
	{17,
	 821},
	{29,
	 821},
	{13,
	 823},
	{25,
	 823},
	{15,
	 826},
	{27,
	 826},
	{22,
	 830},
	{25,
	 832},
	{25,
	 834},
	{27,
	 836},
	{22,
	 839},
	{20,
	 841},
	{22,
	 843},
	{20,
	 847},
	{22,
	 850},
	{25,
	 852},
	{27,
	 854},
	{22,
	 856},
	{20,
	 858},
	{22,
	 860},
	{20,
	 865},
	{22,
	 867},
	{20,
	 869},
	{18,
	 871},
	{17,
	 873},
	{13,
	 876},
	{15,
	 878},
	{13,
	 882},
	{15,
	 884},
	{17,
	 886},
	{18,
	 889},
	{20,
	 891},
	{22,
	 893},
	{15,
	 895},
	{22,
	 900},
	{25,
	 902},
	{25,
	 904},
	{27,
	 906},
	{22,
	 908},
	{20,
	 910},
	{22,
	 913},
	{20,
	 917},
	{22,
	 919},
	{25,
	 921},
	{27,
	 923},
	{22,
	 926},
	{20,
	 928},
	{22,
	 930},
	{20,
	 934},
	{22,
	 936},
	{20,
	 939},
	{18,
	 941},
	{17,
	 943},
	{13,
	 945},
	{15,
	 947},
	{13,
	 952},
	{15,
	 954},
	{17,
	 956},
	{18,
	 958},
	{20,
	 960},
	{22,
	 963},
	{15,
	 965},
	{22,
	 969},
	{34,
	 969},
	{25,
	 971},
	{37,
	 971},
	{25,
	 973},
	{37,
	 973},
	{27,
	 976},
	{39,
	 976},
	{22,
	 978},
	{34,
	 978},
	{20,
	 980},
	{32,
	 980},
	{22,
	 982},
	{34,
	 982},
	{20,
	 986},
	{32,
	 986},
	{22,
	 989},
	{34,
	 989},
	{25,
	 991},
	{37,
	 991},
	{27,
	 993},
	{39,
	 993},
	{22,
	 995},
	{34,
	 995},
	{20,
	 997},
	{32,
	 997},
	{22,
	 1000},
	{34,
	 1000},
	{20,
	 1004},
	{32,
	 1004},
	{22,
	 1006},
	{34,
	 1006},
	{20,
	 1008},
	{32,
	 1008},
	{18,
	 1010},
	{30,
	 1010},
	{17,
	 1013},
	{29,
	 1013},
	{13,
	 1015},
	{25,
	 1015},
	{15,
	 1017},
	{27,
	 1017},
	{13,
	 1021},
	{25,
	 1021},
	{15,
	 1023},
	{27,
	 1023},
	{17,
	 1026},
	{29,
	 1026},
	{18,
	 1028},
	{30,
	 1028},
	{20,
	 1030},
	{32,
	 1030},
	{22,
	 1032},
	{34,
	 1032},
	{15,
	 1034},
	{27,
	 1034},
	{22,
	 1039},
	{34,
	 1039},
	{25,
	 1041},
	{37,
	 1041},
	{25,
	 1043},
	{37,
	 1043},
	{27,
	 1045},
	{39,
	 1045},
	{22,
	 1047},
	{34,
	 1047},
	{20,
	 1050},
	{32,
	 1050},
	{22,
	 1052},
	{34,
	 1052},
	{20,
	 1056},
	{32,
	 1056},
	{22,
	 1058},
	{34,
	 1058},
	{25,
	 1060},
	{37,
	 1060},
	{27,
	 1063},
	{39,
	 1063},
	{22,
	 1065},
	{34,
	 1065},
	{20,
	 1067},
	{32,
	 1067},
	{22,
	 1069},
	{34,
	 1069},
	{27,
	 1073},
	{39,
	 1073},
	{29,
	 1076},
	{41,
	 1076},
	{30,
	 1078},
	{42,
	 1078},
	{29,
	 1080},
	{41,
	 1080},
	{27,
	 1082},
	{39,
	 1082},
	{25,
	 1084},
	{37,
	 1084},
	{22,
	 1086},
	{34,
	 1086},
	{20,
	 1091},
	{32,
	 1091},
	{22,
	 1093},
	{34,
	 1093},
	{20,
	 1095},
	{32,
	 1095},
	{18,
	 1097},
	{30,
	 1097},
	{17,
	 1100},
	{29,
	 1100},
	{13,
	 1102},
	{25,
	 1102},
	{15,
	 1104},
	{27,
	 1104},
	{-2,
	 1116},
	{3,
	 1116},
	{-2,
	 1118},
	{3,
	 1118},
	{1,
	 1119},
	{3,
	 1120},
	{-2,
	 1125},
	{3,
	 1125},
	{-2,
	 1127},
	{3,
	 1127},
	{1,
	 1128},
	{3,
	 1129},
	{-2,
	 1133},
	{3,
	 1133},
	{-2,
	 1135},
	{3,
	 1135},
	{1,
	 1136},
	{3,
	 1138},
	{3,
	 1141},
	{6,
	 1142},
	{6,
	 1145},
	{8,
	 1146},
	{-2,
	 1151},
	{3,
	 1151},
	{-2,
	 1153},
	{3,
	 1153},
	{1,
	 1154},
	{3,
	 1155},
	{-2,
	 1159},
	{3,
	 1159},
	{-2,
	 1161},
	{3,
	 1161},
	{1,
	 1163},
	{3,
	 1164},
	{-2,
	 1168},
	{3,
	 1168},
	{-2,
	 1170},
	{3,
	 1170},
	{1,
	 1171},
	{3,
	 1172},
	{6,
	 1176},
	{8,
	 1177},
	{3,
	 1180},
	{6,
	 1181},
	{-2,
	 1185},
	{3,
	 1185},
	{-2,
	 1188},
	{3,
	 1188},
	{1,
	 1189},
	{3,
	 1190},
	{-2,
	 1194},
	{3,
	 1194},
	{-2,
	 1196},
	{3,
	 1196},
	{1,
	 1197},
	{3,
	 1198},
	{-2,
	 1203},
	{3,
	 1203},
	{-2,
	 1205},
	{3,
	 1205},
	{1,
	 1206},
	{3,
	 1207},
	{3,
	 1210},
	{6,
	 1211},
	{6,
	 1215},
	{8,
	 1216},
	{-2,
	 1220},
	{3,
	 1220},
	{-2,
	 1222},
	{3,
	 1222},
	{1,
	 1223},
	{3,
	 1225},
	{-2,
	 1229},
	{3,
	 1229},
	{-2,
	 1231},
	{3,
	 1231},
	{1,
	 1232},
	{3,
	 1233},
	{-2,
	 1238},
	{3,
	 1238},
	{-2,
	 1240},
	{3,
	 1240},
	{1,
	 1241},
	{3,
	 1242},
	{8,
	 1243},
	{6,
	 1244},
	{8,
	 1246},
	{6,
	 1247},
	{3,
	 1249},
	{6,
	 1250},
	{15,
	 1252},
	{17,
	 1254},
	{18,
	 1256},
	{20,
	 1258},
	{22,
	 1260},
	{27,
	 1265},
	{25,
	 1267},
	{22,
	 1269},
	{15,
	 1273},
	{22,
	 1278},
	{20,
	 1280},
	{18,
	 1282},
	{17,
	 1284},
	{15,
	 1286},
	{17,
	 1289},
	{18,
	 1291},
	{20,
	 1293},
	{22,
	 1295},
	{20,
	 1300},
	{18,
	 1302},
	{17,
	 1304},
	{15,
	 1306},
	{17,
	 1308},
	{18,
	 1310},
	{17,
	 1313},
	{15,
	 1315},
	{14,
	 1317},
	{17,
	 1319},
	{15,
	 1321},
	{17,
	 1323},
	{18,
	 1326},
	{20,
	 1328},
	{22,
	 1330},
	{27,
	 1334},
	{25,
	 1336},
	{22,
	 1339},
	{15,
	 1343},
	{22,
	 1347},
	{20,
	 1350},
	{18,
	 1352},
	{17,
	 1354},
	{15,
	 1356},
	{17,
	 1358},
	{18,
	 1360},
	{20,
	 1363},
	{22,
	 1365},
	{20,
	 1369},
	{18,
	 1371},
	{17,
	 1373},
	{18,
	 1378},
	{20,
	 1382},
	{22,
	 1386},
	{10,
	 1391},
	{15,
	 1391},
	{17,
	 1393},
	{18,
	 1395},
	{20,
	 1397},
	{18,
	 1400},
	{22,
	 1400},
	{27,
	 1404},
	{25,
	 1406},
	{18,
	 1408},
	{22,
	 1408},
	{10,
	 1413},
	{15,
	 1413},
	{22,
	 1417},
	{20,
	 1419},
	{18,
	 1421},
	{17,
	 1423},
	{11,
	 1426},
	{15,
	 1426},
	{17,
	 1428},
	{18,
	 1430},
	{20,
	 1432},
	{18,
	 1434},
	{22,
	 1434},
	{20,
	 1439},
	{18,
	 1441},
	{13,
	 1443},
	{17,
	 1443},
	{15,
	 1445},
	{17,
	 1447},
	{18,
	 1450},
	{13,
	 1452},
	{17,
	 1452},
	{15,
	 1454},
	{14,
	 1456},
	{17,
	 1458},
	{10,
	 1460},
	{15,
	 1460},
	{17,
	 1463},
	{18,
	 1465},
	{20,
	 1467},
	{18,
	 1469},
	{22,
	 1469},
	{27,
	 1473},
	{25,
	 1476},
	{18,
	 1478},
	{22,
	 1478},
	{10,
	 1482},
	{15,
	 1482},
	{22,
	 1486},
	{20,
	 1489},
	{18,
	 1491},
	{17,
	 1493},
	{11,
	 1495},
	{15,
	 1495},
	{17,
	 1497},
	{18,
	 1500},
	{20,
	 1502},
	{18,
	 1504},
	{22,
	 1504},
	{20,
	 1508},
	{18,
	 1510},
	{17,
	 1513},
	{18,
	 1517},
	{20,
	 1521},
	{22,
	 1526},
	{25,
	 1530},
	{27,
	 1532},
	{22,
	 1534},
	{20,
	 1536},
	{18,
	 1539},
	{22,
	 1539},
	{20,
	 1543},
	{22,
	 1545},
	{25,
	 1547},
	{27,
	 1550},
	{22,
	 1552},
	{20,
	 1554},
	{17,
	 1556},
	{22,
	 1556},
	{20,
	 1560},
	{22,
	 1563},
	{20,
	 1565},
	{18,
	 1567},
	{17,
	 1569},
	{13,
	 1571},
	{10,
	 1573},
	{15,
	 1573},
	{13,
	 1578},
	{15,
	 1580},
	{17,
	 1582},
	{18,
	 1584},
	{20,
	 1586},
	{22,
	 1589},
	{10,
	 1591},
	{15,
	 1591},
	{22,
	 1595},
	{25,
	 1597},
	{25,
	 1600},
	{27,
	 1602},
	{22,
	 1604},
	{20,
	 1606},
	{18,
	 1608},
	{22,
	 1608},
	{20,
	 1613},
	{22,
	 1615},
	{25,
	 1617},
	{27,
	 1619},
	{22,
	 1621},
	{20,
	 1623},
	{17,
	 1626},
	{22,
	 1626},
	{20,
	 1630},
	{22,
	 1632},
	{20,
	 1634},
	{18,
	 1636},
	{17,
	 1639},
	{13,
	 1641},
	{10,
	 1643},
	{15,
	 1643},
	{13,
	 1647},
	{15,
	 1650},
	{17,
	 1652},
	{18,
	 1654},
	{20,
	 1656},
	{22,
	 1658},
	{10,
	 1660},
	{15,
	 1660},
	{22,
	 1665},
	{34,
	 1665},
	{25,
	 1667},
	{37,
	 1667},
	{25,
	 1669},
	{37,
	 1669},
	{27,
	 1671},
	{39,
	 1671},
	{22,
	 1673},
	{34,
	 1673},
	{20,
	 1676},
	{32,
	 1676},
	{22,
	 1678},
	{34,
	 1678},
	{20,
	 1682},
	{32,
	 1682},
	{22,
	 1684},
	{34,
	 1684},
	{25,
	 1686},
	{37,
	 1686},
	{27,
	 1689},
	{39,
	 1689},
	{22,
	 1691},
	{34,
	 1691},
	{20,
	 1693},
	{32,
	 1693},
	{22,
	 1695},
	{34,
	 1695},
	{20,
	 1700},
	{32,
	 1700},
	{22,
	 1702},
	{34,
	 1702},
	{20,
	 1704},
	{32,
	 1704},
	{18,
	 1706},
	{30,
	 1706},
	{17,
	 1708},
	{29,
	 1708},
	{13,
	 1710},
	{25,
	 1710},
	{15,
	 1713},
	{27,
	 1713},
	{13,
	 1717},
	{25,
	 1717},
	{15,
	 1719},
	{27,
	 1719},
	{17,
	 1721},
	{29,
	 1721},
	{18,
	 1723},
	{30,
	 1723},
	{20,
	 1726},
	{32,
	 1726},
	{22,
	 1728},
	{34,
	 1728},
	{15,
	 1730},
	{27,
	 1730},
	{22,
	 1734},
	{34,
	 1734},
	{25,
	 1736},
	{37,
	 1736},
	{25,
	 1739},
	{37,
	 1739},
	{27,
	 1741},
	{39,
	 1741},
	{22,
	 1743},
	{34,
	 1743},
	{20,
	 1745},
	{32,
	 1745},
	{22,
	 1747},
	{34,
	 1747},
	{20,
	 1752},
	{32,
	 1752},
	{22,
	 1754},
	{34,
	 1754},
	{25,
	 1756},
	{37,
	 1756},
	{27,
	 1758},
	{39,
	 1758},
	{22,
	 1760},
	{34,
	 1760},
	{20,
	 1763},
	{32,
	 1763},
	{22,
	 1765},
	{34,
	 1765},
	{27,
	 1769},
	{39,
	 1769},
	{29,
	 1771},
	{41,
	 1771},
	{30,
	 1773},
	{42,
	 1773},
	{29,
	 1776},
	{41,
	 1776},
	{27,
	 1778},
	{39,
	 1778},
	{25,
	 1780},
	{37,
	 1780},
	{22,
	 1782},
	{34,
	 1782},
	{20,
	 1786},
	{32,
	 1786},
	{22,
	 1789},
	{34,
	 1789},
	{20,
	 1791},
	{32,
	 1791},
	{18,
	 1793},
	{30,
	 1793},
	{17,
	 1795},
	{29,
	 1795},
	{13,
	 1797},
	{25,
	 1797},
	{15,
	 1800},
	{27,
	 1800},
	{23,
	 1804},
	{26,
	 1806},
	{26,
	 1808},
	{28,
	 1810},
	{23,
	 1813},
	{21,
	 1815},
	{23,
	 1817},
	{21,
	 1821},
	{23,
	 1823},
	{26,
	 1826},
	{28,
	 1828},
	{23,
	 1830},
	{21,
	 1832},
	{23,
	 1834},
	{21,
	 1839},
	{23,
	 1841},
	{21,
	 1843},
	{19,
	 1845},
	{18,
	 1847},
	{14,
	 1850},
	{16,
	 1852},
	{14,
	 1856},
	{16,
	 1858},
	{18,
	 1860},
	{19,
	 1863},
	{21,
	 1865},
	{23,
	 1867},
	{16,
	 1869},
	{23,
	 1873},
	{26,
	 1876},
	{26,
	 1878},
	{28,
	 1880},
	{23,
	 1882},
	{21,
	 1884},
	{23,
	 1886},
	{21,
	 1891},
	{23,
	 1893},
	{26,
	 1895},
	{28,
	 1897},
	{23,
	 1900},
	{21,
	 1902},
	{23,
	 1904},
	{21,
	 1908},
	{23,
	 1910},
	{21,
	 1913},
	{19,
	 1915},
	{18,
	 1917},
	{14,
	 1919},
	{16,
	 1921},
	{14,
	 1926},
	{16,
	 1928},
	{18,
	 1930},
	{19,
	 1932},
	{21,
	 1934},
	{23,
	 1936},
	{16,
	 1939},
	{23,
	 1943},
	{35,
	 1943},
	{26,
	 1945},
	{38,
	 1945},
	{26,
	 1947},
	{38,
	 1947},
	{28,
	 1950},
	{40,
	 1950},
	{23,
	 1952},
	{35,
	 1952},
	{21,
	 1954},
	{33,
	 1954},
	{23,
	 1956},
	{35,
	 1956},
	{21,
	 1960},
	{33,
	 1960},
	{23,
	 1963},
	{35,
	 1963},
	{26,
	 1965},
	{38,
	 1965},
	{28,
	 1967},
	{40,
	 1967},
	{23,
	 1969},
	{35,
	 1969},
	{21,
	 1971},
	{33,
	 1971},
	{23,
	 1973},
	{35,
	 1973},
	{21,
	 1978},
	{33,
	 1978},
	{23,
	 1980},
	{35,
	 1980},
	{21,
	 1982},
	{33,
	 1982},
	{19,
	 1984},
	{31,
	 1984},
	{18,
	 1986},
	{30,
	 1986},
	{14,
	 1989},
	{26,
	 1989},
	{16,
	 1991},
	{28,
	 1991},
	{14,
	 1995},
	{26,
	 1995},
	{16,
	 1997},
	{28,
	 1997},
	{18,
	 2000},
	{30,
	 2000},
	{19,
	 2002},
	{31,
	 2002},
	{21,
	 2004},
	{33,
	 2004},
	{23,
	 2006},
	{35,
	 2006},
	{16,
	 2008},
	{28,
	 2008},
	{23,
	 2013},
	{35,
	 2013},
	{26,
	 2015},
	{38,
	 2015},
	{26,
	 2017},
	{38,
	 2017},
	{28,
	 2019},
	{40,
	 2019},
	{23,
	 2021},
	{35,
	 2021},
	{21,
	 2023},
	{33,
	 2023},
	{23,
	 2026},
	{35,
	 2026},
	{21,
	 2030},
	{33,
	 2030},
	{23,
	 2032},
	{35,
	 2032},
	{26,
	 2034},
	{38,
	 2034},
	{28,
	 2036},
	{40,
	 2036},
	{23,
	 2039},
	{35,
	 2039},
	{21,
	 2041},
	{33,
	 2041},
	{23,
	 2043},
	{35,
	 2043},
	{28,
	 2047},
	{40,
	 2047},
	{30,
	 2050},
	{42,
	 2050},
	{31,
	 2052},
	{43,
	 2052},
	{30,
	 2054},
	{42,
	 2054},
	{28,
	 2056},
	{40,
	 2056},
	{26,
	 2058},
	{38,
	 2058},
	{23,
	 2060},
	{35,
	 2060},
	{21,
	 2065},
	{33,
	 2065},
	{23,
	 2067},
	{35,
	 2067},
	{21,
	 2069},
	{33,
	 2069},
	{19,
	 2071},
	{31,
	 2071},
	{18,
	 2073},
	{30,
	 2073},
	{14,
	 2076},
	{26,
	 2076},
	{16,
	 2078},
	{28,
	 2078},
	{9,
	 2086},
	{9,
	 2089},
	{11,
	 2090},
	{9,
	 2092},
	{11,
	 2093},
	{9,
	 2095},
	{9,
	 2097},
	{11,
	 2098},
	{9,
	 2101},
	{11,
	 2102},
	{9,
	 2104},
	{9,
	 2106},
	{11,
	 2107},
	{9,
	 2109},
	{11,
	 2110},
	{9,
	 2113},
	{9,
	 2115},
	{11,
	 2116},
	{9,
	 2118},
	{11,
	 2119},
	{9,
	 2121},
	{9,
	 2123},
	{11,
	 2125},
	{9,
	 2127},
	{11,
	 2128},
	{9,
	 2130},
	{9,
	 2132},
	{11,
	 2133},
	{9,
	 2135},
	{11,
	 2136},
	{9,
	 2139},
	{9,
	 2141},
	{11,
	 2142},
	{9,
	 2144},
	{11,
	 2145},
	{9,
	 2147},
	{9,
	 2150},
	{11,
	 2151},
	{9,
	 2153},
	{11,
	 2154}};
//{{4, 0}, {8, 1}, {12, 2}, {16, 3}, {20, 4}, {24, 5}, {28, 6}, {32, 7}, {36, 8}, {40, 9}, {44, 10}, {48, 11}, {0, 0}};

bool isLeapYear(uint16_t year);
void getTimeFromTimestamp(struct Time *time, uint64_t timestamp, uint32_t timeZone);
void updateSolarTermsTimestamp(uint64_t solarTermsTimestamp[], uint16_t year);

char convertNumberToChar(uint8_t number);
uint8_t getSegmentDisplayControlWord(char character);

void segmentDisplayBlink(char *segmentDisplayCharacter, uint64_t counter, uint8_t blinkDigitByte);
void LEDBlink(uint64_t counter, uint8_t blinkDigitByte);
void beepPlay(uint32_t *beepFrequency, uint64_t counter, uint16_t *beepSequence);
const uint16_t noteFrequency[] = {523,
								  554,
								  587,
								  622,
								  659,
								  698,
								  740,
								  784,
								  831,
								  880,
								  932,
								  988,
								  1047,
								  1109,
								  1175,
								  1245,
								  1319,
								  1397,
								  1480,
								  1568,
								  1661,
								  1760,
								  1865,
								  1976,
								  2093,
								  2217,
								  2349,
								  2489,
								  2637,
								  2794,
								  2960,
								  3136,
								  3322,
								  3520,
								  3729,
								  3951};

int main(void)
{
	uint8_t i = 0;

	uint8_t tempKeyboardStateByte = 0xff;

	//use internal 16M oscillator, HSI
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ | SYSCTL_OSC_INT | SYSCTL_USE_OSC), 16000000);
	peripheralDeviceInput.keyPadStateByte = tempKeyboardStateByte;
	peripheralDeviceInput.UARTMessageReceiveFinishedCountdown = 1926;
	peripheralDeviceInput.UARTMessageTail = peripheralDeviceInput.UARTMessage;
	peripheralDeviceOutput.beepFrequency = 0;

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	S800_Int_Init();
	S800_QEI_Init();
	S800_PWM_Init();

	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
		;

	EEPROMInit();

	while (1)
	{

		I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~peripheralDeviceOutput.LEDDisplayByte);
		for (i = 0; i < 8; i++)
		{
			//在读写操作同时存在的情况下,似乎有时读得数据会变为0x0.取代直接读取按钮状态的代码,下列代码防止错误数据的传入.
			//放在循环内的原因是减少delay的影响,奇怪的是gpio的读取似乎不必这么做.
			tempKeyboardStateByte = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tempKeyboardStateByte != 0x0)
				peripheralDeviceInput.keyPadStateByte = tempKeyboardStateByte;
			flash_seg(i, peripheralDeviceOutput.segmentDisplayControlWord[i]);
		}

		PWMOutputState(PWM0_BASE, (PWM_OUT_7_BIT), peripheralDeviceOutput.beepFrequency != 0);

		if (peripheralDeviceOutput.beepFrequency != 0)
		{
			PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32SysClock / peripheralDeviceOutput.beepFrequency);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ui32SysClock / peripheralDeviceOutput.beepFrequency / 4);
		}

		peripheralDeviceInput.buttonStateByte = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1 | GPIO_PIN_0);
	}
	uint8_t test;
}
void flash_seg(uint8_t display_index, uint8_t control_word)
{
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, control_word);
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)(1 << display_index));
	Delay(2000);
	// I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~((uint8_t)(1 << display_index)));
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)(0));
	Delay(0);
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Enable PortF
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
		;										 //Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); //Enable PortJ
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
		; //Wait for the GPIO moduleJ ready

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);				//Set PF0 as Output pin
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); //Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}
void S800_I2C0_Init(void)
{
	uint8_t result;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);		//初始化i2c模块
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	//使用I2C模块0，引脚配置为I2C0SCL--PB2、I2C0SDA--PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);				//配置PB2为I2C0SCL
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);				//配置PB3为I2C0SDA
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); //I2C将GPIO_PIN_2用作SCL
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);	//I2C将GPIO_PIN_3用作SDA

	I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true); //config I2C0 400k
	I2CMasterEnable(I2C0_BASE);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0, 0x0ff); //config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1, 0x0);   //config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2, 0x0);   //config port 2 as output

	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG, 0x00);	 //config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff); //turn off the LED1-8
}
void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Enable PortA
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
		; //Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX); // Set GPIO A0 and A1 as UART pins.
	GPIOPinConfigure(GPIO_PA1_U0TX);

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
	UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHello, world!\r\n");
}
void S800_Int_Init(void)
{
	// IntPriorityGroupingSet(7);
	// IntPrioritySet(INT_GPIOJ, 0x20);

	SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY);
	SysTickEnable();
	SysTickIntEnable();

	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	IntEnable(INT_UART0);

	IntMasterEnable();
}
void S800_QEI_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	// Enable the QEI0 peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	// Wait for the QEI0 module to be ready.
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0))
	{
	}

	GPIOPinConfigure(GPIO_PL1_PHA0);
	GPIOPinConfigure(GPIO_PL2_PHB0);
	//software patch to force the PL3 to low voltage
	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0);

	GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1);
	GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_2);
	//
	// Configure the quadrature encoder to capture edges on both signals and
	// maintain an absolute position by resetting on index pulses. Using a
	// 1000 line encoder at four edges per line, there are 4000 pulses per
	// revolution; therefore set the maximum position to 3999 as the count
	// is zero based.
	//
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 100);
	// Enable the quadrature encoder.
	QEIEnable(QEI0_BASE);
}
void S800_PWM_Init(void)
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	//
	// Enable the PWM0 peripheral
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	//
	// Wait for the PWM0 module to be ready.
	//
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
		;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); //Enable PortK
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK))
		; //Wait for the GPIO moduleN readK
	GPIOPinConfigure(GPIO_PK5_M0PWM7);
	GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
	//
	// Configure the PWM generator for count down mode with immediate updates
	// to the parameters.
	//
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	//
	// Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
	// microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
	// Use this value to set the period.
	//
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32SysClock / 4000);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ui32SysClock / 12000);
	//
	// Enable the outputs.
	//
	// PWMOutputState(PWM0_BASE, (PWM_OUT_7_BIT), true);
	//
	// Start the timers in generator 0.
	//
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

void Delay(uint32_t value)
{
	uint32_t i = 0;
	for (; i < value; i++)
		;
}

void UARTStringPut(const char *cMessage)
{
	while (*cMessage != '\0')
		UARTCharPut(UART0_BASE, *(cMessage++));
}

void SysTick_Handler(void)
{
	uint8_t i = 0;
	static uint64_t counter;

	struct Time time;
	static uint64_t timestampInUTC = 162309904000; //单位为0.01s
	uint32_t timestampInUTCHighBit = (timestampInUTC >> 32) & 0xffffffff, timestampInUTCLowBit = timestampInUTC & 0xffffffff;

	struct Time countdownTime;
	static uint64_t countdownTimestamp;

	struct Time alarmTime[NUMBER_OF_ALARMS];
	static uint64_t alarmTimestamp[NUMBER_OF_ALARMS];

	static uint16_t solarTermsCorespondingYear;
	static uint64_t solarTermsTimestamp[24];
	static struct Time solarTerms[24];

	static uint16_t bootCountdown;

	static uint16_t noteTime = 0, noteIndex = 0;

	struct Mode
	{
		uint8_t master;
		uint8_t time;
		uint8_t calender;
		uint8_t countdown;
		uint8_t alarm;
	};
	static struct Mode mode = {
		MASTER_MODE_BOOT,
		TIME_MODE_DISPLAY,
		CALENDER_MODE_DISPLAY,
		COUNTDOWN_MODE_PAUSE,
		ALARM_MODE_DISPLAY};

	static uint8_t timeSelectedDigit, calenderSelectedDigit, countdownSelectedDigit, alarmSelectedDigit;
	static uint8_t alarmOrdinalNumber;

	//外设输出
	char segmentDisplayCharacter[8];

	//外设输入
	static uint8_t previousKeypadState[8];
	static uint8_t previousButtonState[2];
	uint8_t currentKeypadState[8] = {0};
	uint8_t currentButtonState[2] = {0};
	uint16_t keypadPressed[8] = {0}, keypadHold[8] = {0};
	uint16_t buttonPressed[2] = {0}, buttonHold[2] = {0};

	uint8_t UARTMessageReceived = false;
	char *msg = peripheralDeviceInput.UARTMessage;

	counter++;

	//UART接受数据完整性验证:
	if (peripheralDeviceInput.UARTMessageReceiveFinishedCountdown < 100)
	{
		peripheralDeviceInput.UARTMessageReceiveFinishedCountdown++;
	}
	else if (peripheralDeviceInput.UARTMessageReceiveFinishedCountdown == 100)
	{
		peripheralDeviceInput.UARTMessageReceiveFinishedCountdown = 1926;
		UARTMessageReceived = true;
		peripheralDeviceInput.UARTMessageTail = peripheralDeviceInput.UARTMessage;
	}

	//Keypad及按钮状态.
	if (counter % (SYSTICK_FREQUENCY * 10 / 1000) == 0) //20ms
	{
		for (i = 0; i < 8; i++)
		{
			currentKeypadState[i] = ((~peripheralDeviceInput.keyPadStateByte) >> i) & 1;
			keypadPressed[i] = currentKeypadState[i] && !previousKeypadState[i];
			keypadHold[i] = (counter % (SYSTICK_FREQUENCY * 500 / 1000) == 0) && (currentKeypadState[i] && previousKeypadState[i]); //500ms.这里的实现可能会导致首次转换状态时间短于设定。
			previousKeypadState[i] = currentKeypadState[i];
		}
		for (i = 0; i < 2; i++)
		{
			currentButtonState[i] = ((~peripheralDeviceInput.buttonStateByte) >> i) & 1;
			buttonPressed[i] = currentButtonState[i] && !previousButtonState[i];
			buttonHold[i] = (counter % (SYSTICK_FREQUENCY * 500 / 1000) == 0) && (currentButtonState[i] && previousButtonState[i]);
			previousButtonState[i] = currentButtonState[i];
		}
	}

	//重启:
	if (buttonHold[0] && buttonHold[1])
	{
		EEPROMProgram(&timestampInUTCLowBit, 0x400, sizeof(timestampInUTCLowBit));
		EEPROMProgram(&timestampInUTCHighBit, 0x400 + sizeof(timestampInUTCLowBit), sizeof(timestampInUTCHighBit));
		SysCtlReset();
	}

	//时间控制.
	if (mode.time == TIME_MODE_DISPLAY && counter % (SYSTICK_FREQUENCY * 10 / 1000) == 0) //0.01s
		timestampInUTC++;
	getTimeFromTimestamp(&time, timestampInUTC, 8);
	//倒计时控制.
	if (mode.countdown == COUNTDOWN_MODE_FORWARD && counter % (SYSTICK_FREQUENCY * 10 / 1000) == 0) //0.01s
	{
		if (countdownTimestamp == 0)
			mode.countdown = COUNTDOWN_MODE_TIMEOUT;
		else
			countdownTimestamp--;
	}
	getTimeFromTimestamp(&countdownTime, countdownTimestamp, 0);

	//闹钟控制.
	for (i = 0; i < NUMBER_OF_ALARMS; i++)
	{
		getTimeFromTimestamp(&alarmTime[i], alarmTimestamp[i], 0);

		if (alarmTime[i].hour == time.hour && alarmTime[i].minute == time.minute && alarmTime[i].second == time.second)
			mode.alarm = ALARM_MODE_RINGING;
	}
	if (mode.alarm == ALARM_MODE_RINGING && keypadPressed[7])
		mode.alarm = ALARM_MODE_DISPLAY;

	//节气计算
	if (solarTermsCorespondingYear != time.year)
	{
		solarTermsCorespondingYear = time.year;
		updateSolarTermsTimestamp(solarTermsTimestamp, solarTermsCorespondingYear);
		for (i = 0; i < 24; i++)
		{
			getTimeFromTimestamp(&solarTerms[i], solarTermsTimestamp[i], 8);
		}
	}

	//主模式切换.
	if (buttonPressed[0])
		mode.master = (mode.master + NUMBER_OF_MASTER_MODES - 1) % NUMBER_OF_MASTER_MODES;
	if (buttonPressed[1])
		mode.master = (mode.master + 1) % NUMBER_OF_MASTER_MODES;

	//
	if (bootCountdown > 0)
	{
		segmentDisplayCharacter[0] = convertNumberToChar(2);
		segmentDisplayCharacter[1] = convertNumberToChar(1);
		segmentDisplayCharacter[2] = convertNumberToChar(9);
		segmentDisplayCharacter[3] = convertNumberToChar(1);
		segmentDisplayCharacter[4] = convertNumberToChar(0);
		segmentDisplayCharacter[5] = convertNumberToChar(2);
		segmentDisplayCharacter[6] = convertNumberToChar(7);
		segmentDisplayCharacter[7] = convertNumberToChar(9);
		segmentDisplayBlink(segmentDisplayCharacter, counter, 0xff);
		if (counter % (SYSTICK_FREQUENCY * 200 / 1000) == 0)
		{
			peripheralDeviceOutput.LEDDisplayByte = ~peripheralDeviceOutput.LEDDisplayByte;
		}
		bootCountdown--;
	}
	else
	{
		peripheralDeviceOutput.LEDDisplayByte = 0;
		for (i = 0; i < 8; i++)
			segmentDisplayCharacter[i] = ' ';
		switch (mode.master)
		{
		case MASTER_MODE_TIME:
		{
			peripheralDeviceOutput.LEDDisplayByte = 1 << MASTER_MODE_TIME;
			if (keypadPressed[5])
			{
				mode.time = TIME_MODE_SET;
				timeSelectedDigit = 5;
			}
			else if (keypadPressed[7])
				mode.time = TIME_MODE_DISPLAY;
			segmentDisplayCharacter[0] = convertNumberToChar(time.hour / 10);
			segmentDisplayCharacter[1] = convertNumberToChar(time.hour % 10);
			segmentDisplayCharacter[2] = convertNumberToChar(time.minute / 10);
			segmentDisplayCharacter[3] = convertNumberToChar(time.minute % 10);
			segmentDisplayCharacter[4] = convertNumberToChar(time.second / 10);
			segmentDisplayCharacter[5] = convertNumberToChar(time.second % 10);
			segmentDisplayCharacter[6] = convertNumberToChar(timestampInUTC % 100 / 10);
			segmentDisplayCharacter[7] = convertNumberToChar(timestampInUTC % 10);

			if (mode.time == TIME_MODE_SET)
			{
				if (keypadPressed[0])
					timeSelectedDigit = (timeSelectedDigit + 5) % 6;
				if (keypadPressed[2])
					timeSelectedDigit = (timeSelectedDigit + 1) % 6;
				if (keypadPressed[6])
					switch (timeSelectedDigit)
					{
					case 0:
						timestampInUTC += 36000 * 100;
						break;
					case 1:
						timestampInUTC += 3600 * 100;
						break;
					case 2:
						timestampInUTC += 600 * 100;
						break;
					case 3:
						timestampInUTC += 60 * 100;
						break;
					case 4:
						timestampInUTC += 10 * 100;
						break;
					case 5:
						timestampInUTC += 1 * 100;
						break;
					default:
						break;
					}
				if (keypadPressed[1])
					switch (timeSelectedDigit)
					{
					case 0:
						timestampInUTC -= 36000 * 100;
						break;
					case 1:
						timestampInUTC -= 3600 * 100;
						break;
					case 2:
						timestampInUTC -= 600 * 100;
						break;
					case 3:
						timestampInUTC -= 60 * 100;
						break;
					case 4:
						timestampInUTC -= 10 * 100;
						break;
					case 5:
						timestampInUTC -= 1 * 100;
						break;
					default:
						break;
					}
				if (counter % (SYSTICK_FREQUENCY * 200 / 1000) < (SYSTICK_FREQUENCY * 200 / 1000) / 2)
					segmentDisplayCharacter[timeSelectedDigit] = ' ';
			}
		}
		break;
		case MASTER_MODE_CALENDER:
		{
			peripheralDeviceOutput.LEDDisplayByte = 1 << MASTER_MODE_CALENDER;
			if (keypadPressed[5])
			{
				mode.calender = CALENDER_MODE_SET;
				calenderSelectedDigit = 7;
			}
			else if (keypadPressed[7])
				mode.calender = CALENDER_MODE_DISPLAY;
			segmentDisplayCharacter[0] = convertNumberToChar(time.year % 10000 / 1000);
			segmentDisplayCharacter[1] = convertNumberToChar(time.year % 1000 / 100);
			segmentDisplayCharacter[2] = convertNumberToChar(time.year % 100 / 10);
			segmentDisplayCharacter[3] = convertNumberToChar(time.year % 10);
			segmentDisplayCharacter[4] = convertNumberToChar((time.month + 1) % 100 / 10);
			segmentDisplayCharacter[5] = convertNumberToChar((time.month + 1) % 10);
			segmentDisplayCharacter[6] = convertNumberToChar((time.day + 1) % 100 / 10);
			segmentDisplayCharacter[7] = convertNumberToChar((time.day + 1) % 10);

			if (mode.calender == CALENDER_MODE_SET)
			{
				if (keypadPressed[0])
					do
					{
						calenderSelectedDigit = (calenderSelectedDigit + 7) % 8;
					} while (!(calenderSelectedDigit == 3 || calenderSelectedDigit == 5 || calenderSelectedDigit == 6 || calenderSelectedDigit == 7));
				if (keypadPressed[2])
					do
					{
						calenderSelectedDigit = (calenderSelectedDigit + 1) % 8;
					} while (!(calenderSelectedDigit == 3 || calenderSelectedDigit == 5 || calenderSelectedDigit == 6 || calenderSelectedDigit == 7));
				if (keypadPressed[6])
					switch (calenderSelectedDigit)
					{
					case 0:
						break;
					case 1:
						break;
					case 2:
						break;
					case 3:
						timestampInUTC += (uint32_t)(((time.month <= 2 && isLeapYear(time.year)) || (time.month > 2 && isLeapYear(time.year + 1))) ? daysInLeapYear : daysInYear) * 86400 * 100;
						break;
					case 4:
						break;
					case 5:
						timestampInUTC += (isLeapYear(time.year) ? daysInMonthInLeapYear[time.month] : daysInMonth[time.month]) * 86400 * 100;
						break;
					case 6:
						timestampInUTC += 10 * 86400 * 100;
						break;
					case 7:
						timestampInUTC += 1 * 86400 * 100;
						break;
					default:
						break;
					}
				if (keypadPressed[1])
					switch (calenderSelectedDigit)
					{
					case 0:
						break;
					case 1:
						break;
					case 2:
						break;
					case 3:
						timestampInUTC -= (uint32_t)(((time.month <= 2 && isLeapYear(time.year)) || (time.month > 2 && isLeapYear(time.year + 1))) ? daysInLeapYear : daysInYear) * 86400 * 100;
						break;
					case 4:
						break;
					case 5:
						timestampInUTC -= (isLeapYear(time.year) ? daysInMonthInLeapYear[(time.month + 11) % 12] : daysInMonth[(time.month + 11) % 12]) * 86400 * 100;
						break;
					case 6:
						timestampInUTC -= 10 * 86400 * 100;
						break;
					case 7:
						timestampInUTC -= 1 * 86400 * 100;
						break;
					default:
						break;
					}
				if (counter % (SYSTICK_FREQUENCY * 200 / 1000) < (SYSTICK_FREQUENCY * 200 / 1000) / 2)
					segmentDisplayCharacter[calenderSelectedDigit] = ' ';
			}
		}
		break;
		case MASTER_MODE_SOLAR_TERMS:
		{
			peripheralDeviceOutput.LEDDisplayByte = 1 << MASTER_MODE_SOLAR_TERMS;
			segmentDisplayCharacter[0] = convertNumberToChar(time.year % 10000 / 1000);
			segmentDisplayCharacter[1] = convertNumberToChar(time.year % 1000 / 100);
			segmentDisplayCharacter[2] = convertNumberToChar(time.year % 100 / 10);
			segmentDisplayCharacter[3] = convertNumberToChar(time.year % 10);
			for (i = 0; i < 24; i++)
			{
				if (solarTerms[i].month == time.month && solarTerms[i].day == time.day)
				{
					segmentDisplayCharacter[4] = 'S';
					segmentDisplayCharacter[5] = 'P';
					segmentDisplayCharacter[6] = convertNumberToChar(((i + 24 - 2) % 24 + 1) / 10);
					segmentDisplayCharacter[7] = convertNumberToChar(((i + 24 - 2) % 24 + 1) % 10);
					break;
				}
			}
		}
		break;
		case MASTER_MODE_COUNTDOWN:
		{
			peripheralDeviceOutput.LEDDisplayByte = 1 << MASTER_MODE_COUNTDOWN;
			if (keypadPressed[5])
				mode.countdown = (mode.countdown == COUNTDOWN_MODE_SET) ? COUNTDOWN_MODE_PAUSE : COUNTDOWN_MODE_SET;
			else if (keypadPressed[7])
				mode.countdown = (mode.countdown == COUNTDOWN_MODE_PAUSE) ? COUNTDOWN_MODE_FORWARD : COUNTDOWN_MODE_PAUSE;

			segmentDisplayCharacter[0] = convertNumberToChar(countdownTime.hour / 10);
			segmentDisplayCharacter[1] = convertNumberToChar(countdownTime.hour % 10);
			segmentDisplayCharacter[2] = convertNumberToChar(countdownTime.minute / 10);
			segmentDisplayCharacter[3] = convertNumberToChar(countdownTime.minute % 10);
			segmentDisplayCharacter[4] = convertNumberToChar(countdownTime.second / 10);
			segmentDisplayCharacter[5] = convertNumberToChar(countdownTime.second % 10);
			segmentDisplayCharacter[6] = convertNumberToChar(countdownTimestamp % 100 / 10);
			segmentDisplayCharacter[7] = convertNumberToChar(countdownTimestamp % 10);

			if (mode.countdown == COUNTDOWN_MODE_TIMEOUT && counter % (SYSTICK_FREQUENCY * 200 / 1000) < (SYSTICK_FREQUENCY * 200 / 1000) / 2)
			{
				for (i = 0; i < 8; i++)
					segmentDisplayCharacter[i] = ' ';
			}

			if (mode.countdown == COUNTDOWN_MODE_SET)
			{
				if (keypadPressed[0])
					countdownSelectedDigit = (countdownSelectedDigit + 5) % 6;
				if (keypadPressed[2])
					countdownSelectedDigit = (countdownSelectedDigit + 1) % 6;
				countdownTimestamp += 86400 * 100;
				if (keypadPressed[6])
					switch (countdownSelectedDigit)
					{
					case 0:
						countdownTimestamp += 36000 * 100;
						break;
					case 1:
						countdownTimestamp += 3600 * 100;
						break;
					case 2:
						countdownTimestamp += 600 * 100;
						break;
					case 3:
						countdownTimestamp += 60 * 100;
						break;
					case 4:
						countdownTimestamp += 10 * 100;
						break;
					case 5:
						countdownTimestamp += 1 * 100;
						break;
					default:
						break;
					}
				if (keypadPressed[1])
					switch (countdownSelectedDigit)
					{
					case 0:
						countdownTimestamp -= 36000 * 100;
						break;
					case 1:
						countdownTimestamp -= 3600 * 100;
						break;
					case 2:
						countdownTimestamp -= 600 * 100;
						break;
					case 3:
						countdownTimestamp -= 60 * 100;
						break;
					case 4:
						countdownTimestamp -= 10 * 100;
						break;
					case 5:
						countdownTimestamp -= 1 * 100;
						break;
					default:
						break;
					}
				countdownTimestamp %= 86400 * 100;
				segmentDisplayBlink(segmentDisplayCharacter, counter, 1 << countdownSelectedDigit);
			}
		}
		break;
		case MASTER_MODE_ALARM:
		{
			peripheralDeviceOutput.LEDDisplayByte = 1 << MASTER_MODE_ALARM;
			if (keypadPressed[4])
				alarmOrdinalNumber = (alarmOrdinalNumber + 1) % NUMBER_OF_ALARMS;
			if (keypadPressed[3])
				alarmOrdinalNumber = (alarmOrdinalNumber + NUMBER_OF_ALARMS - 1) % NUMBER_OF_ALARMS;

			if (keypadPressed[5])
			{
				mode.alarm = ALARM_MODE_SET;
				alarmSelectedDigit = 7;
			}
			else if (keypadPressed[7])
				mode.alarm = ALARM_MODE_DISPLAY;
			segmentDisplayCharacter[0] = convertNumberToChar(alarmOrdinalNumber);
			segmentDisplayCharacter[1] = ' ';
			segmentDisplayCharacter[2] = convertNumberToChar(alarmTime[alarmOrdinalNumber].hour / 10);
			segmentDisplayCharacter[3] = convertNumberToChar(alarmTime[alarmOrdinalNumber].hour % 10);
			segmentDisplayCharacter[4] = convertNumberToChar(alarmTime[alarmOrdinalNumber].minute / 10);
			segmentDisplayCharacter[5] = convertNumberToChar(alarmTime[alarmOrdinalNumber].minute % 10);
			segmentDisplayCharacter[6] = convertNumberToChar(alarmTime[alarmOrdinalNumber].second / 10);
			segmentDisplayCharacter[7] = convertNumberToChar(alarmTime[alarmOrdinalNumber].second % 10);

			if (mode.alarm == ALARM_MODE_SET)
			{
				if (keypadPressed[0])
					alarmSelectedDigit = (alarmSelectedDigit - 2 + 5) % 6 + 2;
				if (keypadPressed[2])
					alarmSelectedDigit = (alarmSelectedDigit - 2 + 1) % 6 + 2;
				alarmTimestamp[alarmOrdinalNumber] += 86400 * 100;
				if (keypadPressed[6])
					switch (alarmSelectedDigit)
					{
					case 2:
						alarmTimestamp[alarmOrdinalNumber] += 36000 * 100;
						break;
					case 3:
						alarmTimestamp[alarmOrdinalNumber] += 3600 * 100;
						break;
					case 4:
						alarmTimestamp[alarmOrdinalNumber] += 600 * 100;
						break;
					case 5:
						alarmTimestamp[alarmOrdinalNumber] += 60 * 100;
						break;
					case 6:
						alarmTimestamp[alarmOrdinalNumber] += 10 * 100;
						break;
					case 7:
						alarmTimestamp[alarmOrdinalNumber] += 1 * 100;
						break;
					default:
						break;
					}
				if (keypadPressed[1])
				{

					switch (alarmSelectedDigit)
					{
					case 2:
						alarmTimestamp[alarmOrdinalNumber] -= 36000 * 100;
						break;
					case 3:
						alarmTimestamp[alarmOrdinalNumber] -= 3600 * 100;
						break;
					case 4:
						alarmTimestamp[alarmOrdinalNumber] -= 600 * 100;
						break;
					case 5:
						alarmTimestamp[alarmOrdinalNumber] -= 60 * 100;
						break;
					case 6:
						alarmTimestamp[alarmOrdinalNumber] -= 10 * 100;
						break;
					case 7:
						alarmTimestamp[alarmOrdinalNumber] -= 1 * 100;
						break;
					default:
						break;
					}
				}
				alarmTimestamp[alarmOrdinalNumber] %= 86400 * 100;
				segmentDisplayBlink(segmentDisplayCharacter, counter, 1 << alarmSelectedDigit);
			}
			if (mode.alarm == ALARM_MODE_RINGING)
			{
				segmentDisplayBlink(segmentDisplayCharacter, counter, 0xff);
			}
		}
		break;
		case MASTER_MODE_BOOT:
		{
			mode.master = MASTER_MODE_TIME;
			EEPROMRead(&timestampInUTCLowBit, 0x400, sizeof(timestampInUTCLowBit));
			EEPROMRead(&timestampInUTCHighBit, 0x400 + sizeof(timestampInUTCLowBit), sizeof(timestampInUTCHighBit));
			timestampInUTC = (uint64_t)timestampInUTCLowBit + (((uint64_t)timestampInUTCHighBit) << 32);
			bootCountdown = SYSTICK_FREQUENCY * 6;
		}
		break;
		default:
			break;
		}
	}
	for (i = 0; i < 8; i++)
	{
		peripheralDeviceOutput.segmentDisplayControlWord[i] = getSegmentDisplayControlWord(segmentDisplayCharacter[i]);
		segmentDisplayCharacter[i] = ' ';
	}

	if (mode.alarm == ALARM_MODE_RINGING || mode.countdown == COUNTDOWN_MODE_TIMEOUT)
		peripheralDeviceOutput.beepFrequency = 2000;
	else
	{

		if (counter % (SYSTICK_FREQUENCY * 100 / 1000) == 0)
		{
			noteTime++;

			if (notes[noteIndex][0] == 0 && notes[noteIndex][1] == 0)
			{
				noteTime = 0;
				noteIndex = 0;
			}
			while (noteTime >= notes[noteIndex + 1][1])
			{
				noteIndex++;
			}
			peripheralDeviceOutput.beepFrequency = noteFrequency[notes[noteIndex][0]];
		}

		peripheralDeviceOutput.beepFrequency = 0;
	}
	peripheralDeviceOutput.beepFrequency = 0;

	if (UARTMessageReceived)
	{
		if (strncasecmp(msg, "set", strlen("set")) == 0)
		{
			if (strncasecmp(msg + strlen("set") + 1, "timestamp", strlen("timestamp")) == 0)
			{
				uint8_t positionInMessage = strlen("set") + 1;
				uint64_t timestampFromMessage = strlen("set") + 1 + strlen("timestamp") + 1;
				while (msg[positionInMessage] != '\0')
				{
					timestampFromMessage = timestampFromMessage * 10 + (uint64_t)(msg[positionInMessage++] - '0');
				}

				char tmp[100];
				sprintf(tmp, "Updating current timestamp to:%llu(times ten milliseconds)", timestampFromMessage);
				UARTStringPut(tmp);

				timestampInUTC = timestampFromMessage;
			}
		}
	}
}
void UART0_Handler(void)
{

	uint32_t ulStatus;
	ulStatus = UARTIntStatus(UART0_BASE, true);
	UARTIntClear(UART0_BASE, ulStatus);

	while (UARTCharsAvail(UART0_BASE))
	{
		*(peripheralDeviceInput.UARTMessageTail++) = UARTCharGetNonBlocking(UART0_BASE);
	}
	*(peripheralDeviceInput.UARTMessageTail) = '\0';
	peripheralDeviceInput.UARTMessageReceiveFinishedCountdown = 0;
}
uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while (I2CMasterBusy(I2C0_BASE))
	{
	};	//如果I2C0模块忙，等待
		//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	//设置主机要放到总线上的从机地址。false表示主机写从机，true表示主机读从机

	I2CMasterDataPut(I2C0_BASE, RegAddr);						  //主机写设备寄存器地址
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); //执行重复写入操作
	while (I2CMasterBusy(I2C0_BASE))
	{
	};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE); //调试用

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); //执行重复写入操作并结束
	while (I2CMasterBusy(I2C0_BASE))
	{
	};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE); //调试用

	return rop; //返回错误类型，无错返回0
}
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value, rop;
	while (I2CMasterBusy(I2C0_BASE))
	{
	};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND); //执行单词写入操作
	while (I2CMasterBusBusy(I2C0_BASE))
		;
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);			//设置从机地址
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); //执行单次读操作
	while (I2CMasterBusBusy(I2C0_BASE))
		;
	value = I2CMasterDataGet(I2C0_BASE); //获取读取的数据
	Delay(1);
	return value;
}

bool isLeapYear(uint16_t year)
{
	return year % 4 == 0 && (!(year % 100 == 0) || year % 400 == 0);
}
char convertNumberToChar(uint8_t number)
{
	if (number >= 0 && number <= 9)
		return number + '0';
	else
		return number + 'a';
}
uint8_t getSegmentDisplayControlWord(char character)
{
	char const characterSet[] = {'0', '1', '2', '3', '4', '5',
								 '6', '7', '8', '9', 'A', 'b', 'B',
								 'C', 'd', 'D', 'E', 'F', 'G',
								 'H', 'J', 'K', 'L', 'M', 'N',
								 'P', 'o', 'O', 'Q', 'R', 'T',
								 'S', 'U', 'V', 'W', 'X', 'Y', 'Z',
								 '.', '-', '_', ' ', '\0'};
	uint8_t const controlWordSet[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D,
									  0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x7c,
									  0x39, 0x5E, 0x5e, 0x79, 0x71, 0x3d,
									  0x76, 0x0d, 0x72, 0x38, 0x55, 0x54,
									  0x73, 0x5c, 0x5c, 0x6f, 0x50, 0x78,
									  0x6d, 0x3e, 0x1c, 0x6a, 0x1d, 0x6e, 0x49,
									  0x80, 0x40, 0x08, 0x00};
	char *characterPosition = strchr(characterSet, character);
	return (characterPosition != NULL ? *(controlWordSet + (characterPosition - characterSet)) : 0x0);
}
void getTimeFromTimestamp(struct Time *time, uint64_t timestamp, uint32_t timeZone)
{
	// if (timestamp >= (uint64_t)142004160000)
	// {
	// 	timestamp = timestamp - (uint64_t)142004160000;
	// 	time->year = 2015;
	// }
	// else
	time->year = 1970;
	time->second = timestamp / 100 % 60;
	time->minute = timestamp / 100 % 3600 / 60;
	time->hour = timestamp / 100 % 86400 / 3600 + timeZone;

	time->day = timestamp / 100 / 86400;
	if (time->hour >= 24)
	{
		time->hour -= 24;
		time->day++;
	}

	time->month = 0;

	while (time->day >= (isLeapYear(time->year) ? daysInLeapYear : daysInYear))
	{
		time->day -= (isLeapYear(time->year) ? daysInLeapYear : daysInYear);
		time->year++;
	}
	while (time->day >= (isLeapYear(time->year) ? daysInMonthInLeapYear[time->month] : daysInMonth[time->month]))
	{
		time->day -= (isLeapYear(time->year) ? daysInMonthInLeapYear[time->month] : daysInMonth[time->month]);
		time->month++;
	}
}
void updateSolarTermsTimestamp(uint64_t solarTermsTimestamp[], uint16_t year)
{
	uint16_t i;
	for (i = 0; i < 24; i++)
	{
		solarTermsTimestamp[i] = solarTermsTimestampIn2021[i];
	}
	if (year < 2021)
	{
		while ((year++) != 2021)
		{
			for (i = 0; i < 24; i++)
			{
				solarTermsTimestamp[i] = solarTermsTimestamp[i] - (uint64_t)SECONDS_IN_TROPICAL_YEAR * 100;
			}
		}
	}
	else if (year > 2021)
	{
		while ((year--) != 2021)
		{
			for (i = 0; i < 24; i++)
			{
				solarTermsTimestamp[i] = solarTermsTimestamp[i] + (uint64_t)SECONDS_IN_TROPICAL_YEAR * 100;
			}
		}
	}
}
void segmentDisplayBlink(char *segmentDisplayCharacter, uint64_t counter, uint8_t blinkDigitByte)
{
	uint8_t i;
	if (counter % (SYSTICK_FREQUENCY * 200 / 1000) < (SYSTICK_FREQUENCY * 200 / 1000) / 2)
	{
		for (i = 0; i < 8; i++)
			if (blinkDigitByte & 1 << i)
				segmentDisplayCharacter[i] = ' ';
	}
}
void beepPlay(uint32_t *beepFrequency, uint64_t counter, uint16_t *beepSequence)
{
}
