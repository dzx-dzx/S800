
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
#define NUMBERS_OF_LIGHT_LEVELS 5 //0 for off and NUMBERS_OF_LIGHT_LEVELS-1 for full.

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
	uint8_t lightLevel;
	uint8_t motorClockwise;
	uint8_t motorCycle;
} peripheralDeviceOutput;

struct Time
{
	uint16_t year;
	uint16_t month;
	uint16_t day;
	uint16_t hour;
	uint16_t minute;
	uint16_t second;
	uint16_t tenMillisecond;
};
const uint16_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
const uint16_t daysInMonthInLeapYear[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
const uint16_t daysInYear = 365;
const uint16_t daysInLeapYear = 366;
const uint64_t solarTermsTimestampIn2021[] = {160978820500 + 8 * 3600 * 100, 161105999000 + 8 * 3600 * 100, 161233552700 + 8 * 3600 * 100, 161361623600 + 8 * 3600 * 100, 161490562000 + 8 * 3600 * 100, 161620424700 + 8 * 3600 * 100, 161751450600 + 8 * 3600 * 100, 161883560200 + 8 * 3600 * 100, 162016843000 + 8 * 3600 * 100, 162151062600 + 8 * 3600 * 100, 162286152500 + 8 * 3600 * 100, 162421752900 + 8 * 3600 * 100, 162557672800 + 8 * 3600 * 100, 162693518400 + 8 * 3600 * 100, 162829043700 + 8 * 3600 * 100, 162963929600 + 8 * 3600 * 100, 163097957500 + 8 * 3600 * 100, 163230966300 + 8 * 3600 * 100, 163362834100 + 8 * 3600 * 100, 163493586900 + 8 * 3600 * 100, 163623232600 + 8 * 3600 * 100, 163751962300 + 8 * 3600 * 100, 163879902300 + 8 * 3600 * 100, 164007355800 + 8 * 3600 * 100};
const uint16_t notes[][2] = //{{50, 189}, {52, 205}, {54, 221}, {57, 236}, {59, 252}, {54, 283}, {57, 299}, {52, 315}, {50, 377}, {54, 377}, {47, 408}, {50, 408}, {49, 424}, {52, 424}, {50, 455}, {54, 455}, {59, 502}, {54, 534}, {57, 550}, {52, 565}, {50, 628}, {54, 628}, {47, 659}, {50, 659}, {47, 675}, {59, 753}, {54, 784}, {57, 800}, {52, 815}, {50, 878}, {54, 878}, {47, 909}, {50, 909}, {49, 925}, {52, 925}, {47, 940}, {50, 971}, {59, 1003}, {54, 1034}, {57, 1050}, {54, 1081}, {59, 1081}, {57, 1128}, {61, 1128}, {50, 1190}, {52, 1206}, {54, 1221}, {57, 1237}, {59, 1253}, {54, 1284}, {57, 1300}, {52, 1315}, {50, 1378}, {54, 1378}, {47, 1409}, {50, 1409}, {49, 1425}, {52, 1425}, {50, 1456}, {54, 1456}, {59, 1503}, {54, 1534}, {57, 1550}, {59, 1565}, {62, 1581}, {59, 1612}, {59, 1628}, {54, 1659}, {54, 1706}, {59, 1721}, {62, 1737}, {59, 1753}, {54, 1784}, {57, 1800}, {52, 1815}, {50, 1878}, {54, 1878}, {47, 1909}, {50, 1909}, {49, 1925}, {52, 1925}, {47, 1940}, {47, 1971}, {59, 2003}, {57, 2050}, {59, 2081}, {54, 2112}, {50, 2128}, {52, 2159}, {54, 2190}, {50, 2206}, {57, 2221}, {62, 2237}, {54, 2253}, {59, 2261}, {59, 2379}, {62, 2379}, {54, 2442}, {59, 2442}, {57, 2505}, {59, 2537}, {50, 2568}, {54, 2568}, {52, 2600}, {50, 2632}, {54, 2632}, {50, 2695}, {54, 2695}, {54, 2726}, {57, 2726}, {55, 2758}, {59, 2766}, {55, 2884}, {59, 2884}, {59, 2947}, {62, 2947}, {62, 3011}, {64, 3042}, {54, 3074}, {59, 3074}, {57, 3105}, {54, 3137}, {59, 3137}, {57, 3200}, {61, 3200}, {57, 3232}, {61, 3232}, {54, 3263}, {59, 3271}, {59, 3389}, {62, 3389}, {54, 3453}, {59, 3453}, {57, 3516}, {59, 3547}, {50, 3579}, {54, 3579}, {52, 3611}, {50, 3642}, {54, 3642}, {50, 3705}, {54, 3705}, {54, 3737}, {57, 3737}, {55, 3768}, {59, 3777}, {55, 3895}, {59, 3895}, {59, 3958}, {62, 3958}, {62, 4021}, {64, 4053}, {54, 4084}, {59, 4084}, {57, 4116}, {59, 4147}, {61, 4179}, {62, 4211}, {64, 4242}, {62, 4274}, {50, 4305}, {50, 4337}, {52, 4368}, {52, 4400}, {54, 4432}, {59, 4463}, {57, 4495}, {57, 4526}, {59, 4558}, {59, 4589}, {62, 4621}, {62, 4653}, {64, 4684}, {64, 4716}, {66, 4747}, {66, 4779}, {64, 4811}, {64, 4842}, {62, 4874}, {62, 4905}, {59, 4937}, {59, 4968}, {57, 5000}, {57, 5032}, {54, 5063}, {54, 5095}, {52, 5126}, {52, 5158}, {50, 5189}, {50, 5221}, {47, 5253}, {62, 5284}, {64, 5316}, {64, 5347}, {62, 5363}, {66, 5363}, {62, 5395}, {66, 5395}, {66, 5411}, {67, 5474}, {69, 5537}, {64, 5568}, {64, 5600}, {62, 5616}, {66, 5616}, {62, 5647}, {66, 5647}, {71, 5758}, {69, 5774}, {69, 5789}, {64, 5821}, {64, 5853}, {62, 5868}, {66, 5868}, {62, 5900}, {66, 5900}, {62, 5916}, {71, 6011}, {69, 6026}, {69, 6042}, {64, 6074}, {64, 6105}, {62, 6121}, {66, 6121}, {62, 6153}, {66, 6153}, {62, 6168}, {61, 6232}, {59, 6295}, {57, 6421}, {64, 6516}, {62, 6547}, {54, 6579}, {59, 6611}, {62, 6642}, {64, 6674}, {50, 6737}, {52, 6752}, {54, 6768}, {57, 6784}, {59, 6799}, {54, 6831}, {57, 6846}, {52, 6862}, {50, 6924}, {54, 6924}, {47, 6956}, {50, 6956}, {49, 6971}, {52, 6971}, {50, 7002}, {54, 7002}, {59, 7049}, {54, 7081}, {57, 7097}, {52, 7112}, {50, 7175}, {54, 7175}, {47, 7206}, {50, 7206}, {47, 7222}, {59, 7300}, {54, 7331}, {57, 7347}, {52, 7362}, {50, 7425}, {54, 7425}, {47, 7456}, {50, 7456}, {49, 7472}, {52, 7472}, {47, 7487}, {50, 7519}, {59, 7550}, {54, 7581}, {57, 7597}, {54, 7628}, {59, 7628}, {57, 7675}, {61, 7675}, {50, 7737}, {52, 7753}, {54, 7769}, {57, 7784}, {59, 7800}, {54, 7831}, {57, 7847}, {52, 7862}, {50, 7925}, {54, 7925}, {47, 7956}, {50, 7956}, {49, 7972}, {52, 7972}, {50, 8003}, {54, 8003}, {59, 8050}, {54, 8081}, {57, 8097}, {59, 8112}, {62, 8128}, {59, 8159}, {59, 8175}, {54, 8206}, {54, 8253}, {59, 8269}, {62, 8284}, {59, 8300}, {54, 8331}, {57, 8347}, {52, 8362}, {50, 8425}, {54, 8425}, {47, 8456}, {50, 8456}, {49, 8472}, {52, 8472}, {47, 8487}, {47, 8519}, {59, 8550}, {57, 8597}, {59, 8628}, {54, 8659}, {50, 8675}, {52, 8706}, {54, 8737}, {50, 8753}, {57, 8769}, {62, 8784}, {54, 8800}, {59, 8808}, {59, 8926}, {62, 8926}, {54, 8989}, {59, 8989}, {57, 9052}, {59, 9084}, {50, 9116}, {54, 9116}, {52, 9147}, {50, 9179}, {54, 9179}, {50, 9242}, {54, 9242}, {54, 9274}, {57, 9274}, {55, 9305}, {59, 9313}, {55, 9431}, {59, 9431}, {59, 9495}, {62, 9495}, {62, 9558}, {64, 9589}, {54, 9621}, {59, 9621}, {57, 9652}, {54, 9684}, {59, 9684}, {57, 9747}, {61, 9747}, {57, 9779}, {61, 9779}, {54, 9810}, {59, 9819}, {59, 9937}, {62, 9937}, {54, 10000}, {59, 10000}, {57, 10063}, {59, 10095}, {50, 10126}, {54, 10126}, {52, 10158}, {50, 10189}, {54, 10189}, {50, 10252}, {54, 10252}, {54, 10284}, {57, 10284}, {55, 10316}, {59, 10324}, {55, 10442}, {59, 10442}, {59, 10505}, {62, 10505}, {62, 10568}, {64, 10600}, {54, 10631}, {59, 10631}, {57, 10663}, {59, 10695}, {61, 10726}, {62, 10758}, {64, 10789}, {62, 10821}, {50, 10852}, {50, 10884}, {52, 10916}, {52, 10947}, {54, 10979}, {59, 11010}, {57, 11042}, {57, 11074}, {59, 11105}, {59, 11137}, {62, 11168}, {62, 11200}, {64, 11231}, {64, 11263}, {66, 11295}, {66, 11326}, {64, 11358}, {64, 11389}, {62, 11421}, {62, 11452}, {59, 11484}, {59, 11516}, {57, 11547}, {57, 11579}, {54, 11610}, {54, 11642}, {52, 11674}, {52, 11705}, {50, 11737}, {50, 11768}, {47, 11800}, {62, 11831}, {64, 11863}, {64, 11895}, {62, 11910}, {66, 11910}, {62, 11942}, {66, 11942}, {66, 11958}, {67, 12021}, {69, 12084}, {64, 12116}, {64, 12147}, {62, 12163}, {66, 12163}, {62, 12195}, {66, 12195}, {71, 12305}, {69, 12321}, {69, 12337}, {64, 12368}, {64, 12400}, {62, 12416}, {66, 12416}, {62, 12447}, {66, 12447}, {62, 12463}, {71, 12558}, {69, 12574}, {69, 12589}, {64, 12621}, {64, 12652}, {62, 12668}, {66, 12668}, {62, 12700}, {66, 12700}, {62, 12716}, {61, 12779}, {59, 12842}, {57, 12968}, {64, 13063}, {59, 13095}, {62, 13095}, {54, 13132}, {59, 13170}, {62, 13207}, {64, 13245}, {62, 13295}, {59, 13345}, {57, 13395}, {50, 13445}, {59, 13458}};
	{{53, 0}, {65, 30}, {65, 50}, {63, 60}, {65, 70}, {53, 80}, {65, 110}, {65, 130}, {63, 140}, {65, 150}, {53, 160}, {65, 190}, {65, 210}, {63, 220}, {65, 230}, {53, 240}, {65, 260}, {68, 270}, {70, 280}, {68, 300}, {70, 310}, {53, 320}, {65, 350}, {65, 370}, {63, 380}, {65, 390}, {53, 400}, {65, 430}, {65, 450}, {63, 460}, {65, 470}, {53, 480}, {65, 510}, {65, 530}, {63, 540}, {65, 550}, {58, 560}, {68, 580}, {70, 590}, {56, 600}, {65, 620}, {68, 630}, {53, 640}, {65, 670}, {65, 690}, {63, 700}, {65, 710}, {53, 720}, {65, 750}, {65, 770}, {63, 780}, {65, 790}, {53, 800}, {65, 830}, {65, 850}, {63, 860}, {65, 870}, {53, 880}, {65, 900}, {68, 910}, {70, 920}, {68, 940}, {70, 950}, {53, 960}, {65, 990}, {65, 1010}, {63, 1020}, {65, 1030}, {53, 1040}, {65, 1070}, {65, 1090}, {63, 1100}, {65, 1110}, {53, 1120}, {65, 1150}, {65, 1170}, {63, 1180}, {65, 1190}, {58, 1200}, {56, 1213}, {58, 1226}, {56, 1240}, {53, 1253}, {56, 1266}, {53, 1280}, {55, 1300}, {56, 1320}, {58, 1340}, {60, 1360}, {65, 1400}, {63, 1420}, {60, 1440}, {53, 1480}, {60, 1520}, {58, 1540}, {56, 1560}, {55, 1580}, {53, 1600}, {55, 1620}, {56, 1640}, {58, 1660}, {60, 1680}, {58, 1720}, {56, 1740}, {55, 1760}, {53, 1780}, {55, 1800}, {56, 1820}, {55, 1840}, {53, 1860}, {52, 1880}, {55, 1900}, {53, 1920}, {55, 1940}, {56, 1960}, {58, 1980}, {60, 2000}, {65, 2040}, {63, 2060}, {60, 2080}, {53, 2120}, {60, 2160}, {58, 2180}, {56, 2200}, {55, 2220}, {53, 2240}, {55, 2260}, {56, 2280}, {58, 2300}, {60, 2320}, {58, 2360}, {56, 2380}, {55, 2400}, {56, 2440}, {58, 2480}, {60, 2520}, {53, 2560}, {55, 2580}, {56, 2600}, {58, 2620}, {60, 2640}, {65, 2680}, {63, 2700}, {60, 2720}, {53, 2760}, {60, 2800}, {58, 2820}, {56, 2840}, {55, 2860}, {53, 2880}, {55, 2900}, {56, 2920}, {58, 2940}, {60, 2960}, {58, 3000}, {56, 3020}, {55, 3040}, {53, 3060}, {55, 3080}, {56, 3100}, {55, 3120}, {53, 3140}, {52, 3160}, {55, 3180}, {53, 3200}, {55, 3220}, {56, 3240}, {58, 3260}, {60, 3280}, {65, 3320}, {63, 3340}, {60, 3360}, {53, 3400}, {60, 3440}, {58, 3460}, {56, 3480}, {55, 3500}, {53, 3520}, {55, 3540}, {56, 3560}, {58, 3580}, {60, 3600}, {58, 3640}, {56, 3660}, {55, 3680}, {56, 3720}, {58, 3760}, {60, 3800}, {63, 3840}, {65, 3860}, {60, 3880}, {58, 3900}, {60, 3920}, {58, 3960}, {60, 3980}, {63, 4000}, {65, 4020}, {60, 4040}, {58, 4060}, {60, 4080}, {58, 4120}, {60, 4140}, {58, 4160}, {56, 4180}, {55, 4200}, {51, 4220}, {53, 4240}, {51, 4280}, {53, 4300}, {55, 4320}, {56, 4340}, {58, 4360}, {60, 4380}, {53, 4400}, {60, 4440}, {63, 4460}, {63, 4480}, {65, 4500}, {60, 4520}, {58, 4540}, {60, 4560}, {58, 4600}, {60, 4620}, {63, 4640}, {65, 4660}, {60, 4680}, {58, 4700}, {60, 4720}, {58, 4760}, {60, 4780}, {58, 4800}, {56, 4820}, {55, 4840}, {51, 4860}, {53, 4880}, {51, 4920}, {53, 4940}, {55, 4960}, {56, 4980}, {58, 5000}, {60, 5020}, {53, 5040}, {60, 5080}, {63, 5100}, {63, 5120}, {65, 5140}, {60, 5160}, {58, 5180}, {60, 5200}, {58, 5240}, {60, 5260}, {63, 5280}, {65, 5300}, {60, 5320}, {58, 5340}, {60, 5360}, {58, 5400}, {60, 5420}, {58, 5440}, {56, 5460}, {55, 5480}, {51, 5500}, {53, 5520}, {51, 5560}, {53, 5580}, {55, 5600}, {56, 5620}, {58, 5640}, {60, 5660}, {53, 5680}, {60, 5720}, {63, 5740}, {63, 5760}, {65, 5780}, {60, 5800}, {58, 5820}, {60, 5840}, {58, 5880}, {60, 5900}, {63, 5920}, {65, 5940}, {60, 5960}, {58, 5980}, {60, 6000}, {65, 6040}, {67, 6060}, {68, 6080}, {67, 6100}, {65, 6120}, {63, 6140}, {60, 6160}, {58, 6200}, {60, 6220}, {58, 6240}, {56, 6260}, {55, 6280}, {51, 6300}, {53, 6320}, {60, 6360}, {63, 6380}, {63, 6400}, {65, 6420}, {60, 6440}, {58, 6460}, {60, 6480}, {58, 6520}, {60, 6540}, {63, 6560}, {65, 6580}, {60, 6600}, {58, 6620}, {60, 6640}, {58, 6680}, {60, 6700}, {58, 6720}, {56, 6740}, {55, 6760}, {51, 6780}, {53, 6800}, {51, 6840}, {53, 6860}, {55, 6880}, {56, 6900}, {58, 6920}, {60, 6940}, {53, 6960}, {60, 7000}, {63, 7020}, {63, 7040}, {65, 7060}, {60, 7080}, {58, 7100}, {60, 7120}, {58, 7160}, {60, 7180}, {63, 7200}, {65, 7220}, {60, 7240}, {58, 7260}, {60, 7280}, {58, 7320}, {60, 7340}, {58, 7360}, {56, 7380}, {55, 7400}, {51, 7420}, {53, 7440}, {51, 7480}, {53, 7500}, {55, 7520}, {56, 7540}, {58, 7560}, {60, 7580}, {53, 7600}, {60, 7640}, {63, 7660}, {63, 7680}, {65, 7700}, {60, 7720}, {58, 7740}, {60, 7760}, {58, 7800}, {60, 7820}, {63, 7840}, {65, 7860}, {60, 7880}, {58, 7900}, {60, 7920}, {58, 7960}, {60, 7980}, {58, 8000}, {56, 8020}, {55, 8040}, {51, 8060}, {53, 8080}, {51, 8120}, {53, 8140}, {55, 8160}, {56, 8180}, {58, 8200}, {60, 8220}, {53, 8240}, {60, 8280}, {63, 8300}, {63, 8320}, {65, 8340}, {60, 8360}, {58, 8380}, {60, 8400}, {58, 8440}, {60, 8460}, {63, 8480}, {65, 8500}, {60, 8520}, {58, 8540}, {60, 8560}, {65, 8600}, {67, 8620}, {68, 8640}, {67, 8660}, {65, 8680}, {63, 8700}, {60, 8720}, {58, 8760}, {60, 8780}, {58, 8800}, {56, 8820}, {55, 8840}, {51, 8860}, {53, 8880}, {53, 8960}, {65, 8990}, {65, 9010}, {63, 9020}, {65, 9030}, {53, 9040}, {65, 9070}, {65, 9090}, {63, 9100}, {65, 9110}, {53, 9120}, {65, 9150}, {65, 9170}, {63, 9180}, {65, 9190}, {53, 9200}, {65, 9220}, {68, 9230}, {70, 9240}, {68, 9260}, {70, 9270}, {53, 9280}, {65, 9310}, {65, 9330}, {63, 9340}, {65, 9350}, {53, 9360}, {65, 9390}, {65, 9410}, {63, 9420}, {65, 9430}, {53, 9440}, {65, 9470}, {65, 9490}, {63, 9500}, {65, 9510}, {58, 9520}, {68, 9540}, {70, 9550}, {56, 9560}, {65, 9580}, {68, 9590}, {53, 9600}, {65, 9630}, {65, 9650}, {63, 9660}, {65, 9670}, {53, 9680}, {65, 9710}, {65, 9730}, {63, 9740}, {65, 9750}, {53, 9760}, {65, 9790}, {65, 9810}, {63, 9820}, {65, 9830}, {53, 9840}, {65, 9860}, {68, 9870}, {70, 9880}, {68, 9900}, {70, 9910}, {53, 9920}, {65, 9950}, {65, 9970}, {63, 9980}, {65, 9990}, {53, 10000}, {65, 10030}, {65, 10050}, {63, 10060}, {65, 10070}, {53, 10080}, {65, 10110}, {65, 10130}, {63, 10140}, {65, 10150}, {58, 10160}, {56, 10173}, {58, 10186}, {56, 10200}, {53, 10213}, {56, 10226}, {53, 10240}, {55, 10260}, {56, 10280}, {58, 10300}, {60, 10320}, {65, 10360}, {63, 10380}, {60, 10400}, {53, 10440}, {60, 10480}, {58, 10500}, {56, 10520}, {55, 10540}, {53, 10560}, {55, 10580}, {56, 10600}, {58, 10620}, {60, 10640}, {58, 10680}, {56, 10700}, {55, 10720}, {53, 10740}, {55, 10760}, {56, 10780}, {55, 10800}, {53, 10820}, {52, 10840}, {55, 10860}, {53, 10880}, {55, 10900}, {56, 10920}, {58, 10940}, {60, 10960}, {65, 11000}, {63, 11020}, {60, 11040}, {53, 11080}, {60, 11120}, {58, 11140}, {56, 11160}, {55, 11180}, {53, 11200}, {55, 11220}, {56, 11240}, {58, 11260}, {60, 11280}, {58, 11320}, {56, 11340}, {55, 11360}, {56, 11400}, {58, 11440}, {60, 11480}, {53, 11520}, {55, 11540}, {56, 11560}, {58, 11580}, {60, 11600}, {65, 11640}, {63, 11660}, {60, 11680}, {53, 11720}, {60, 11760}, {58, 11780}, {56, 11800}, {55, 11820}, {53, 11840}, {55, 11860}, {56, 11880}, {58, 11900}, {60, 11920}, {58, 11960}, {56, 11980}, {55, 12000}, {53, 12020}, {55, 12040}, {56, 12060}, {55, 12080}, {53, 12100}, {52, 12120}, {55, 12140}, {53, 12160}, {55, 12180}, {56, 12200}, {58, 12220}, {60, 12240}, {65, 12280}, {63, 12300}, {60, 12320}, {53, 12360}, {60, 12400}, {58, 12420}, {56, 12440}, {55, 12460}, {53, 12480}, {55, 12500}, {56, 12520}, {58, 12540}, {60, 12560}, {58, 12600}, {56, 12620}, {55, 12640}, {56, 12680}, {58, 12720}, {60, 12760}, {63, 12800}, {65, 12820}, {60, 12840}, {58, 12860}, {60, 12880}, {58, 12920}, {60, 12940}, {63, 12960}, {65, 12980}, {60, 13000}, {58, 13020}, {60, 13040}, {58, 13080}, {60, 13100}, {58, 13120}, {56, 13140}, {55, 13160}, {51, 13180}, {53, 13200}, {51, 13240}, {53, 13260}, {55, 13280}, {56, 13300}, {58, 13320}, {60, 13340}, {53, 13360}, {60, 13400}, {63, 13420}, {63, 13440}, {65, 13460}, {60, 13480}, {58, 13500}, {60, 13520}, {58, 13560}, {60, 13580}, {63, 13600}, {65, 13620}, {60, 13640}, {58, 13660}, {60, 13680}, {58, 13720}, {60, 13740}, {58, 13760}, {56, 13780}, {55, 13800}, {51, 13820}, {53, 13840}, {51, 13880}, {53, 13900}, {55, 13920}, {56, 13940}, {58, 13960}, {60, 13980}, {53, 14000}, {60, 14040}, {63, 14060}, {63, 14080}, {65, 14100}, {60, 14120}, {58, 14140}, {60, 14160}, {58, 14200}, {60, 14220}, {63, 14240}, {65, 14260}, {60, 14280}, {58, 14300}, {60, 14320}, {58, 14360}, {60, 14380}, {58, 14400}, {56, 14420}, {55, 14440}, {51, 14460}, {53, 14480}, {51, 14520}, {53, 14540}, {55, 14560}, {56, 14580}, {58, 14600}, {60, 14620}, {53, 14640}, {60, 14680}, {63, 14700}, {63, 14720}, {65, 14740}, {60, 14760}, {58, 14780}, {60, 14800}, {58, 14840}, {60, 14860}, {63, 14880}, {65, 14900}, {60, 14920}, {58, 14940}, {60, 14960}, {65, 15000}, {67, 15020}, {68, 15040}, {67, 15060}, {65, 15080}, {63, 15100}, {60, 15120}, {58, 15160}, {60, 15180}, {58, 15200}, {56, 15220}, {55, 15240}, {51, 15260}, {53, 15280}, {61, 15320}, {64, 15340}, {64, 15360}, {66, 15380}, {61, 15400}, {59, 15420}, {61, 15440}, {59, 15480}, {61, 15500}, {64, 15520}, {66, 15540}, {61, 15560}, {59, 15580}, {61, 15600}, {59, 15640}, {61, 15660}, {59, 15680}, {57, 15700}, {56, 15720}, {52, 15740}, {54, 15760}, {52, 15800}, {54, 15820}, {56, 15840}, {57, 15860}, {59, 15880}, {61, 15900}, {54, 15920}, {61, 15960}, {64, 15980}, {64, 16000}, {66, 16020}, {61, 16040}, {59, 16060}, {61, 16080}, {59, 16120}, {61, 16140}, {64, 16160}, {66, 16180}, {61, 16200}, {59, 16220}, {61, 16240}, {59, 16280}, {61, 16300}, {59, 16320}, {57, 16340}, {56, 16360}, {52, 16380}, {54, 16400}, {52, 16440}, {54, 16460}, {56, 16480}, {57, 16500}, {59, 16520}, {61, 16540}, {54, 16560}, {61, 16600}, {64, 16620}, {64, 16640}, {66, 16660}, {61, 16680}, {59, 16700}, {61, 16720}, {59, 16760}, {61, 16780}, {64, 16800}, {66, 16820}, {61, 16840}, {59, 16860}, {61, 16880}, {59, 16920}, {61, 16940}, {59, 16960}, {57, 16980}, {56, 17000}, {52, 17020}, {54, 17040}, {52, 17080}, {54, 17100}, {56, 17120}, {57, 17140}, {59, 17160}, {61, 17180}, {54, 17200}, {61, 17240}, {64, 17260}, {64, 17280}, {66, 17300}, {61, 17320}, {59, 17340}, {61, 17360}, {59, 17400}, {61, 17420}, {64, 17440}, {66, 17460}, {61, 17480}, {59, 17500}, {61, 17520}, {66, 17560}, {68, 17580}, {69, 17600}, {68, 17620}, {66, 17640}, {64, 17660}, {61, 17680}, {59, 17720}, {61, 17740}, {59, 17760}, {57, 17780}, {56, 17800}, {52, 17820}, {54, 17840}};
const uint8_t motor_control[] = {0x09, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0c, 0x08};
uint8_t motor_control_phase = 0;

bool isLeapYear(uint16_t year);
void getTimeFromTimestamp(struct Time *time, uint64_t timestamp, uint32_t timeZone);
void updateSolarTermsTimestamp(uint64_t solarTermsTimestamp[], uint16_t year);
void addTenMilliseconds(struct Time *time);
void minusTenMilliseconds(struct Time *time);

char convertNumberToChar(uint8_t number);
uint8_t getSegmentDisplayControlWord(char character);

void segmentDisplayBlink(char *segmentDisplayCharacter, uint64_t counter, uint8_t blinkDigitByte);
void LEDBlink(uint64_t counter, uint8_t blinkDigitByte);
const uint16_t noteFrequency[] = {0, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62, 66, 70, 74, 78, 83, 88, 93, 98, 104, 110, 117, 124, 131, 139, 147, 156, 165, 175, 186, 197, 208, 221, 234, 248, 263, 278, 295, 312, 331, 351, 371, 393, 417, 442, 468, 496, 525, 556, 590, 625, 662, 701, 743, 787, 834, 883, 936, 992, 1051, 1113, 1179, 1249, 1324, 1402, 1486, 1574, 1668, 1767, 1872, 1983, 2101, 2226, 2358, 2499, 2647, 2804, 2971, 3148, 3335, 3533, 3744, 3966, 4202, 4452, 4717, 4997, 5294, 5609, 5943, 6296, 6670, 7067, 7487, 7932, 8404, 8904, 9433, 9994, 10588, 11218, 11885, 12592, 13341, 14134, 14974, 15865};

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
	peripheralDeviceOutput.lightLevel = NUMBERS_OF_LIGHT_LEVELS - 1;
	peripheralDeviceOutput.motorClockwise = true;
	peripheralDeviceOutput.motorCycle = 0;

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

			if (peripheralDeviceOutput.lightLevel > 0)
				flash_seg(i, peripheralDeviceOutput.segmentDisplayControlWord[i]);
			if (peripheralDeviceOutput.motorCycle > 0)
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, motor_control[8 - i]);
		}
		if (peripheralDeviceOutput.motorCycle > 0)
			peripheralDeviceOutput.motorCycle--;
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
	Delay((uint32_t)800 * peripheralDeviceOutput.lightLevel);
	// I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~((uint8_t)(1 << display_index)));
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)(0));
	Delay((uint32_t)800 * (NUMBERS_OF_LIGHT_LEVELS - 1 - peripheralDeviceOutput.lightLevel));
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Enable PortF
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
		;										 //Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); //Enable PortJ
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
		; //Wait for the GPIO moduleJ ready

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3); //Set PF0 as Output pin
																							   //Set PF0 as Output pin
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);							   //Set the PJ0,PJ1 as input pin
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

	static struct Time time;
	static uint64_t timestampInUTC = 162309904000; //单位为0.01s
	uint32_t timestampInUTCHighBit = (timestampInUTC >> 32) & 0xffffffff, timestampInUTCLowBit = timestampInUTC & 0xffffffff;

	static struct Time countdownTime;
	static uint64_t countdownTimestamp;

	static struct Time alarmTime[NUMBER_OF_ALARMS];
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
		uint8_t doSleep;
		uint8_t doPlayMusic;
	};
	static struct Mode mode = {MASTER_MODE_BOOT, TIME_MODE_DISPLAY, CALENDER_MODE_DISPLAY, COUNTDOWN_MODE_PAUSE, ALARM_MODE_DISPLAY};

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
	static uint16_t keypadHoldCount[8] = {0}, buttonHoldCount[8] = {0};

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
			keypadHoldCount[i] = (currentKeypadState[i] && previousKeypadState[i]) ? keypadHoldCount[i] + 1 : 0;
			keypadHold[i] = keypadHoldCount[i] > 0 && (keypadHoldCount[i] % 50 == 0);
			previousKeypadState[i] = currentKeypadState[i];
		}
		for (i = 0; i < 2; i++)
		{
			currentButtonState[i] = ((~peripheralDeviceInput.buttonStateByte) >> i) & 1;
			buttonPressed[i] = currentButtonState[i] && !previousButtonState[i];
			buttonHoldCount[i] = (currentButtonState[i] && previousButtonState[i]) ? buttonHoldCount[i] + 1 : 0;
			buttonHold[i] = buttonHoldCount[i] > 0 && (buttonHoldCount[i] % 50 == 0);
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
	{
		timestampInUTC++;
		addTenMilliseconds(&time);
		if (time.tenMillisecond == 0)
		{

			peripheralDeviceOutput.motorClockwise = true;
			peripheralDeviceOutput.motorCycle = 10;
		}
	}
	// getTimeFromTimestamp(&time, timestampInUTC, 8);

	//倒计时控制.
	if (mode.countdown == COUNTDOWN_MODE_FORWARD && counter % (SYSTICK_FREQUENCY * 10 / 1000) == 0) //0.01s
	{
		if (countdownTimestamp == 0)
			mode.countdown = COUNTDOWN_MODE_TIMEOUT;
		else
		{
			countdownTimestamp--;
			minusTenMilliseconds(&countdownTime);
		}
	}

	//闹钟控制.
	for (i = 0; i < NUMBER_OF_ALARMS; i++)
	{
		// getTimeFromTimestamp(&alarmTime[i], alarmTimestamp[i], 0);

		if (alarmTime[i].hour == time.hour && alarmTime[i].minute == time.minute && alarmTime[i].second == time.second && mode.master != MASTER_MODE_BOOT)
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

	if (buttonHold[0])
		peripheralDeviceOutput.lightLevel = (peripheralDeviceOutput.lightLevel + NUMBERS_OF_LIGHT_LEVELS - 1) % NUMBERS_OF_LIGHT_LEVELS;
	if (buttonHold[1])
		peripheralDeviceOutput.lightLevel = (peripheralDeviceOutput.lightLevel + 1) % NUMBERS_OF_LIGHT_LEVELS;
	//输入输出
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
			segmentDisplayCharacter[6] = convertNumberToChar(time.tenMillisecond % 100 / 10);
			segmentDisplayCharacter[7] = convertNumberToChar(time.tenMillisecond % 10);

			if (mode.time == TIME_MODE_SET)
			{
				if (keypadPressed[0])
					timeSelectedDigit = (timeSelectedDigit + 5) % 6;
				if (keypadPressed[2])
					timeSelectedDigit = (timeSelectedDigit + 1) % 6;
				if (keypadPressed[6])
				{
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
					getTimeFromTimestamp(&time, timestampInUTC, 8);
				}
				if (keypadPressed[1])
				{
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
					getTimeFromTimestamp(&time, timestampInUTC, 8);
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
				{
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
					getTimeFromTimestamp(&time, timestampInUTC, 8);
				}
				if (keypadPressed[1])
				{
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
					getTimeFromTimestamp(&time, timestampInUTC, 8);
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
			segmentDisplayCharacter[6] = convertNumberToChar(countdownTime.tenMillisecond % 100 / 10);
			segmentDisplayCharacter[7] = convertNumberToChar(countdownTime.tenMillisecond % 10);

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

				if (keypadPressed[6])
				{
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
					countdownTimestamp %= 86400 * 100;
					getTimeFromTimestamp(&countdownTime, countdownTimestamp, 0);
				}
				if (keypadPressed[1])
				{
					countdownTimestamp += 86400 * 100;
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
					getTimeFromTimestamp(&countdownTime, countdownTimestamp, 0);
				}

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

				if (keypadPressed[6])
				{
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
					alarmTimestamp[alarmOrdinalNumber] %= 86400 * 100;
					getTimeFromTimestamp(&alarmTime[alarmOrdinalNumber], alarmTimestamp[alarmOrdinalNumber], 0);
				}
				if (keypadPressed[1])
				{
					alarmTimestamp[alarmOrdinalNumber] += 86400 * 100;
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
					alarmTimestamp[alarmOrdinalNumber] %= 86400 * 100;
					getTimeFromTimestamp(&alarmTime[alarmOrdinalNumber], alarmTimestamp[alarmOrdinalNumber], 0);
				}

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
			getTimeFromTimestamp(&time, timestampInUTC, 8);
			getTimeFromTimestamp(&countdownTime, countdownTimestamp, 0);
			for (i = 0; i < NUMBER_OF_ALARMS; i++)
				getTimeFromTimestamp(&alarmTime[i], alarmTimestamp[i], 0);
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
	else if (mode.doPlayMusic)
	{

		if (counter % (SYSTICK_FREQUENCY * 10 / 1000) == 0)
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

	if (UARTMessageReceived)
	{
		if (strncasecmp(msg, "set", strlen("set")) == 0)
		{
			if (strncasecmp(msg + strlen("set") + 1, "timestamp", strlen("timestamp")) == 0)
			{
				uint16_t positionInMessage = strlen("set timestamp ");
				uint64_t timestampFromMessage = 0;
				while (msg[positionInMessage] != '\0')
				{
					timestampFromMessage = timestampFromMessage * (uint64_t)10 + (uint64_t)(msg[positionInMessage++] - '0');
				}

				char tmp[100];
				sprintf(tmp, "Updating current timestamp to:%llu(times ten milliseconds)", timestampFromMessage);
				UARTStringPut(tmp);

				timestampInUTC = timestampFromMessage;
				getTimeFromTimestamp(&time, timestampInUTC, 8);
			}
		}
		else if (strncmp(msg, "MUSIC", strlen("MUSIC")) == 0)
		{
			if (strncmp(msg + strlen("MUSIC") + 1, "PLAY", strlen("PLAY")) == 0)
			{
				noteTime = 0;
				mode.doPlayMusic = true;
			}
			else if (strncmp(msg + strlen("MUSIC") + 1, "STOP", strlen("STOP")) == 0)
			{
				
				mode.doPlayMusic = false;
				noteTime = 0;
				peripheralDeviceOutput.beepFrequency = 0;
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
	char const characterSet[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'b', 'B', 'C', 'd', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P', 'o', 'O', 'Q', 'R', 'T', 'S', 'U', 'V', 'W', 'X', 'Y', 'Z', '.', '-', '_', ' ', '\0'};
	uint8_t const controlWordSet[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x7c, 0x39, 0x5E, 0x5e, 0x79, 0x71, 0x3d, 0x76, 0x0d, 0x72, 0x38, 0x55, 0x54, 0x73, 0x5c, 0x5c, 0x6f, 0x50, 0x78, 0x6d, 0x3e, 0x1c, 0x6a, 0x1d, 0x6e, 0x49, 0x80, 0x40, 0x08, 0x00};
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
	time->tenMillisecond = timestamp % 100;
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
void addTenMilliseconds(struct Time *time)
{

	time->tenMillisecond++;
	if (time->tenMillisecond >= 100)
	{
		time->tenMillisecond -= 100;
		time->second++;
		if (time->second >= 60)
		{
			time->second -= 60;
			time->minute++;
			if (time->minute >= 60)
			{
				time->minute -= 60;
				time->hour++;
				if (time->hour >= 24)
				{
					time->hour -= 24;
					time->day++;
					if (time->day >= (isLeapYear(time->year) ? daysInLeapYear : daysInYear))
					{
						time->day -= (isLeapYear(time->year) ? daysInLeapYear : daysInYear);
						time->year++;
						if (time->day >= (isLeapYear(time->year) ? daysInMonthInLeapYear[time->month] : daysInMonth[time->month]))
						{
							time->day -= (isLeapYear(time->year) ? daysInMonthInLeapYear[time->month] : daysInMonth[time->month]);
							time->month++;
						}
					}
				}
			}
		}
	}
}
void minusTenMilliseconds(struct Time *time)
{
	if (time->tenMillisecond == 0)
	{
		time->tenMillisecond = 99;
		if (time->second == 0)
		{
			time->second = 59;
			if (time->minute == 0)
			{
				time->minute = 59;
				if (time->hour == 0)
				{
					time->hour = 23;
					if (time->day == 0)
					{
						if (time->month == 0)
						{
							if (time->year == 0)
								return;
							time->year--;
							time->month = 11;
						}
						else
						{
							time->month--;
						}
						time->day = (isLeapYear(time->year) ? daysInMonthInLeapYear[time->month] : daysInMonth[time->month]) - 1;
					}
					else
						time->day--;
				}
				else
					time->hour--;
			}
			else
				time->minute--;
		}
		else
			time->second--;
	}
	else
		time->tenMillisecond--;
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
