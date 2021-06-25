
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
	{{34, 21}, {39, 21}, {34, 65}, {39, 65}, {34, 108}, {39, 108}, {34, 141}, {39, 141}, {34, 163}, {39, 163}, {34, 195}, {39, 195}, {34, 239}, {39, 239}, {34, 282}, {39, 282}, {34, 369}, {39, 369}, {34, 413}, {39, 413}, {34, 456}, {39, 456}, {34, 489}, {39, 489}, {34, 510}, {39, 510}, {34, 543}, {39, 543}, {34, 586}, {39, 586}, {34, 630}, {39, 630}, {34, 717}, {39, 717}, {34, 760}, {39, 760}, {34, 804}, {39, 804}, {34, 836}, {39, 836}, {34, 858}, {39, 858}, {34, 891}, {39, 891}, {34, 934}, {39, 934}, {34, 978}, {39, 978}, {34, 1065}, {39, 1065}, {34, 1108}, {39, 1108}, {34, 1152}, {39, 1152}, {34, 1184}, {39, 1184}, {34, 1206}, {39, 1206}, {34, 1239}, {39, 1239}, {34, 1282}, {39, 1282}, {88, 1347}, {86, 1349}, {87, 1349}, {84, 1350}, {85, 1350}, {83, 1352}, {82, 1353}, {81, 1354}, {80, 1354}, {79, 1355}, {78, 1357}, {77, 1357}, {76, 1358}, {74, 1360}, {75, 1360}, {72, 1361}, {73, 1361}, {71, 1363}, {70, 1364}, {69, 1364}, {68, 1365}, {67, 1366}, {66, 1368}, {65, 1368}, {64, 1369}, {34, 1423}, {39, 1423}, {34, 1445}, {39, 1445}, {37, 1456}, {39, 1467}, {34, 1510}, {39, 1510}, {34, 1532}, {39, 1532}, {37, 1543}, {39, 1554}, {34, 1597}, {39, 1597}, {34, 1619}, {39, 1619}, {37, 1630}, {39, 1641}, {39, 1673}, {42, 1684}, {42, 1717}, {44, 1728}, {34, 1771}, {39, 1771}, {34, 1793}, {39, 1793}, {37, 1804}, {39, 1815}, {34, 1858}, {39, 1858}, {34, 1880}, {39, 1880}, {37, 1891}, {39, 1902}, {34, 1945}, {39, 1945}, {34, 1967}, {39, 1967}, {37, 1978}, {39, 1989}, {42, 2021}, {44, 2032}, {39, 2065}, {42, 2076}, {34, 2119}, {39, 2119}, {34, 2141}, {39, 2141}, {37, 2152}, {39, 2163}, {34, 2206}, {39, 2206}, {34, 2228}, {39, 2228}, {37, 2239}, {39, 2250}, {34, 2293}, {39, 2293}, {34, 2315}, {39, 2315}, {37, 2326}, {39, 2336}, {39, 2369}, {42, 2380}, {42, 2413}, {44, 2423}, {34, 2467}, {39, 2467}, {34, 2489}, {39, 2489}, {37, 2500}, {39, 2510}, {34, 2554}, {39, 2554}, {34, 2576}, {39, 2576}, {37, 2586}, {39, 2597}, {34, 2641}, {39, 2641}, {34, 2663}, {39, 2663}, {37, 2673}, {39, 2684}, {44, 2695}, {42, 2710}, {44, 2724}, {42, 2739}, {39, 2753}, {42, 2768}, {51, 2782}, {53, 2804}, {54, 2826}, {56, 2847}, {58, 2869}, {63, 2913}, {61, 2934}, {58, 2956}, {51, 3000}, {58, 3043}, {56, 3065}, {54, 3086}, {53, 3108}, {51, 3130}, {53, 3152}, {54, 3173}, {56, 3195}, {58, 3217}, {56, 3260}, {54, 3282}, {53, 3304}, {51, 3326}, {53, 3347}, {54, 3369}, {53, 3391}, {51, 3413}, {50, 3434}, {53, 3456}, {51, 3478}, {53, 3500}, {54, 3521}, {56, 3543}, {58, 3565}, {63, 3608}, {61, 3630}, {58, 3652}, {51, 3695}, {58, 3739}, {56, 3760}, {54, 3782}, {53, 3804}, {51, 3826}, {53, 3847}, {54, 3869}, {56, 3891}, {58, 3913}, {56, 3956}, {54, 3978}, {53, 4000}, {54, 4043}, {56, 4086}, {58, 4130}, {46, 4173}, {51, 4173}, {53, 4195}, {54, 4217}, {56, 4239}, {54, 4260}, {58, 4260}, {63, 4304}, {61, 4326}, {54, 4347}, {58, 4347}, {46, 4391}, {51, 4391}, {58, 4434}, {56, 4456}, {54, 4478}, {53, 4500}, {47, 4521}, {51, 4521}, {53, 4543}, {54, 4565}, {56, 4586}, {54, 4608}, {58, 4608}, {56, 4652}, {54, 4673}, {49, 4695}, {53, 4695}, {51, 4717}, {53, 4739}, {54, 4760}, {49, 4782}, {53, 4782}, {51, 4804}, {50, 4826}, {53, 4847}, {46, 4869}, {51, 4869}, {53, 4891}, {54, 4913}, {56, 4934}, {54, 4956}, {58, 4956}, {63, 5000}, {61, 5021}, {54, 5043}, {58, 5043}, {46, 5086}, {51, 5086}, {58, 5130}, {56, 5152}, {54, 5173}, {53, 5195}, {47, 5217}, {51, 5217}, {53, 5239}, {54, 5260}, {56, 5282}, {54, 5304}, {58, 5304}, {56, 5347}, {54, 5369}, {53, 5391}, {54, 5434}, {56, 5478}, {58, 5521}, {61, 5565}, {63, 5586}, {58, 5608}, {56, 5630}, {54, 5652}, {58, 5652}, {56, 5695}, {58, 5717}, {61, 5739}, {63, 5760}, {58, 5782}, {56, 5804}, {53, 5826}, {58, 5826}, {56, 5869}, {58, 5891}, {56, 5913}, {54, 5934}, {53, 5956}, {49, 5978}, {46, 6000}, {51, 6000}, {49, 6043}, {51, 6065}, {53, 6086}, {54, 6108}, {56, 6130}, {58, 6152}, {46, 6173}, {51, 6173}, {58, 6217}, {61, 6239}, {61, 6260}, {63, 6282}, {58, 6304}, {56, 6326}, {54, 6347}, {58, 6347}, {56, 6391}, {58, 6413}, {61, 6434}, {63, 6456}, {58, 6478}, {56, 6500}, {53, 6521}, {58, 6521}, {56, 6565}, {58, 6586}, {56, 6608}, {54, 6630}, {53, 6652}, {49, 6673}, {46, 6695}, {51, 6695}, {49, 6739}, {51, 6760}, {53, 6782}, {54, 6804}, {56, 6826}, {58, 6847}, {46, 6869}, {51, 6869}, {58, 6913}, {70, 6913}, {61, 6934}, {73, 6934}, {61, 6956}, {73, 6956}, {63, 6978}, {75, 6978}, {58, 7000}, {70, 7000}, {56, 7021}, {68, 7021}, {58, 7043}, {70, 7043}, {56, 7086}, {68, 7086}, {58, 7108}, {70, 7108}, {61, 7130}, {73, 7130}, {63, 7152}, {75, 7152}, {58, 7173}, {70, 7173}, {56, 7195}, {68, 7195}, {58, 7217}, {70, 7217}, {56, 7260}, {68, 7260}, {58, 7282}, {70, 7282}, {56, 7304}, {68, 7304}, {54, 7326}, {66, 7326}, {53, 7347}, {65, 7347}, {49, 7369}, {61, 7369}, {51, 7391}, {63, 7391}, {49, 7434}, {61, 7434}, {51, 7456}, {63, 7456}, {53, 7478}, {65, 7478}, {54, 7500}, {66, 7500}, {56, 7521}, {68, 7521}, {58, 7543}, {70, 7543}, {51, 7565}, {63, 7565}, {58, 7608}, {70, 7608}, {61, 7630}, {73, 7630}, {61, 7652}, {73, 7652}, {63, 7673}, {75, 7673}, {58, 7695}, {70, 7695}, {56, 7717}, {68, 7717}, {58, 7739}, {70, 7739}, {56, 7782}, {68, 7782}, {58, 7804}, {70, 7804}, {61, 7826}, {73, 7826}, {63, 7847}, {75, 7847}, {58, 7869}, {70, 7869}, {56, 7891}, {68, 7891}, {58, 7913}, {70, 7913}, {63, 7956}, {75, 7956}, {65, 7978}, {77, 7978}, {66, 8000}, {78, 8000}, {65, 8021}, {77, 8021}, {63, 8043}, {75, 8043}, {61, 8065}, {73, 8065}, {58, 8086}, {70, 8086}, {56, 8130}, {68, 8130}, {58, 8152}, {70, 8152}, {56, 8173}, {68, 8173}, {54, 8195}, {66, 8195}, {53, 8217}, {65, 8217}, {49, 8239}, {61, 8239}, {51, 8260}, {63, 8260}, {58, 8304}, {61, 8326}, {61, 8347}, {63, 8369}, {58, 8391}, {56, 8413}, {58, 8434}, {56, 8478}, {58, 8500}, {61, 8521}, {63, 8543}, {58, 8565}, {56, 8586}, {58, 8608}, {56, 8652}, {58, 8673}, {56, 8695}, {54, 8717}, {53, 8739}, {49, 8760}, {51, 8782}, {49, 8826}, {51, 8847}, {53, 8869}, {54, 8891}, {56, 8913}, {58, 8934}, {51, 8956}, {58, 9000}, {61, 9021}, {61, 9043}, {63, 9065}, {58, 9086}, {56, 9108}, {58, 9130}, {56, 9173}, {58, 9195}, {61, 9217}, {63, 9239}, {58, 9260}, {56, 9282}, {58, 9304}, {56, 9347}, {58, 9369}, {56, 9391}, {54, 9413}, {53, 9434}, {49, 9456}, {51, 9478}, {49, 9521}, {51, 9543}, {53, 9565}, {54, 9586}, {56, 9608}, {58, 9630}, {51, 9652}, {58, 9695}, {70, 9695}, {61, 9717}, {73, 9717}, {61, 9739}, {73, 9739}, {63, 9760}, {75, 9760}, {58, 9782}, {70, 9782}, {56, 9804}, {68, 9804}, {58, 9826}, {70, 9826}, {56, 9869}, {68, 9869}, {58, 9891}, {70, 9891}, {61, 9913}, {73, 9913}, {63, 9934}, {75, 9934}, {58, 9956}, {70, 9956}, {56, 9978}, {68, 9978}, {58, 10000}, {70, 10000}, {56, 10043}, {68, 10043}, {58, 10065}, {70, 10065}, {56, 10086}, {68, 10086}, {54, 10108}, {66, 10108}, {53, 10130}, {65, 10130}, {49, 10152}, {61, 10152}, {51, 10173}, {63, 10173}, {49, 10217}, {61, 10217}, {51, 10239}, {63, 10239}, {53, 10260}, {65, 10260}, {54, 10282}, {66, 10282}, {56, 10304}, {68, 10304}, {58, 10326}, {70, 10326}, {51, 10347}, {63, 10347}, {58, 10391}, {70, 10391}, {61, 10413}, {73, 10413}, {61, 10434}, {73, 10434}, {63, 10456}, {75, 10456}, {58, 10478}, {70, 10478}, {56, 10500}, {68, 10500}, {58, 10521}, {70, 10521}, {56, 10565}, {68, 10565}, {58, 10586}, {70, 10586}, {61, 10608}, {73, 10608}, {63, 10630}, {75, 10630}, {58, 10652}, {70, 10652}, {56, 10673}, {68, 10673}, {58, 10695}, {70, 10695}, {63, 10739}, {75, 10739}, {65, 10760}, {77, 10760}, {66, 10782}, {78, 10782}, {65, 10804}, {77, 10804}, {63, 10826}, {75, 10826}, {61, 10847}, {73, 10847}, {58, 10869}, {70, 10869}, {56, 10913}, {68, 10913}, {58, 10934}, {70, 10934}, {56, 10956}, {68, 10956}, {54, 10978}, {66, 10978}, {53, 11000}, {65, 11000}, {49, 11021}, {61, 11021}, {51, 11043}, {63, 11043}, {34, 11163}, {39, 11163}, {34, 11184}, {39, 11184}, {37, 11195}, {39, 11206}, {34, 11250}, {39, 11250}, {34, 11271}, {39, 11271}, {37, 11282}, {39, 11293}, {34, 11336}, {39, 11336}, {34, 11358}, {39, 11358}, {37, 11369}, {39, 11380}, {39, 11413}, {42, 11423}, {42, 11456}, {44, 11467}, {34, 11510}, {39, 11510}, {34, 11532}, {39, 11532}, {37, 11543}, {39, 11554}, {34, 11597}, {39, 11597}, {34, 11619}, {39, 11619}, {37, 11630}, {39, 11641}, {34, 11684}, {39, 11684}, {34, 11706}, {39, 11706}, {37, 11717}, {39, 11728}, {42, 11760}, {44, 11771}, {39, 11804}, {42, 11815}, {34, 11858}, {39, 11858}, {34, 11880}, {39, 11880}, {37, 11891}, {39, 11902}, {34, 11945}, {39, 11945}, {34, 11967}, {39, 11967}, {37, 11978}, {39, 11989}, {34, 12032}, {39, 12032}, {34, 12054}, {39, 12054}, {37, 12065}, {39, 12076}, {39, 12108}, {42, 12119}, {42, 12152}, {44, 12163}, {34, 12206}, {39, 12206}, {34, 12228}, {39, 12228}, {37, 12239}, {39, 12250}, {34, 12293}, {39, 12293}, {34, 12315}, {39, 12315}, {37, 12326}, {39, 12336}, {34, 12380}, {39, 12380}, {34, 12402}, {39, 12402}, {37, 12413}, {39, 12423}, {44, 12434}, {42, 12449}, {44, 12463}, {42, 12478}, {39, 12492}, {42, 12507}, {51, 12521}, {53, 12543}, {54, 12565}, {56, 12586}, {58, 12608}, {63, 12652}, {61, 12673}, {58, 12695}, {51, 12739}, {58, 12782}, {56, 12804}, {54, 12826}, {53, 12847}, {51, 12869}, {53, 12891}, {54, 12913}, {56, 12934}, {58, 12956}, {56, 13000}, {54, 13021}, {53, 13043}, {51, 13065}, {53, 13086}, {54, 13108}, {53, 13130}, {51, 13152}, {50, 13173}, {53, 13195}, {51, 13217}, {53, 13239}, {54, 13260}, {56, 13282}, {58, 13304}, {63, 13347}, {61, 13369}, {58, 13391}, {51, 13434}, {58, 13478}, {56, 13500}, {54, 13521}, {53, 13543}, {51, 13565}, {53, 13586}, {54, 13608}, {56, 13630}, {58, 13652}, {56, 13695}, {54, 13717}, {53, 13739}, {54, 13782}, {56, 13826}, {58, 13869}, {46, 13913}, {51, 13913}, {53, 13934}, {54, 13956}, {56, 13978}, {54, 14000}, {58, 14000}, {63, 14043}, {61, 14065}, {54, 14086}, {58, 14086}, {46, 14130}, {51, 14130}, {58, 14173}, {56, 14195}, {54, 14217}, {53, 14239}, {47, 14260}, {51, 14260}, {53, 14282}, {54, 14304}, {56, 14326}, {54, 14347}, {58, 14347}, {56, 14391}, {54, 14413}, {49, 14434}, {53, 14434}, {51, 14456}, {53, 14478}, {54, 14500}, {49, 14521}, {53, 14521}, {51, 14543}, {50, 14565}, {53, 14586}, {46, 14608}, {51, 14608}, {53, 14630}, {54, 14652}, {56, 14673}, {54, 14695}, {58, 14695}, {63, 14739}, {61, 14760}, {54, 14782}, {58, 14782}, {46, 14826}, {51, 14826}, {58, 14869}, {56, 14891}, {54, 14913}, {53, 14934}, {47, 14956}, {51, 14956}, {53, 14978}, {54, 15000}, {56, 15021}, {54, 15043}, {58, 15043}, {56, 15086}, {54, 15108}, {53, 15130}, {54, 15173}, {56, 15217}, {58, 15260}, {61, 15304}, {63, 15326}, {58, 15347}, {56, 15369}, {54, 15391}, {58, 15391}, {56, 15434}, {58, 15456}, {61, 15478}, {63, 15500}, {58, 15521}, {56, 15543}, {53, 15565}, {58, 15565}, {56, 15608}, {58, 15630}, {56, 15652}, {54, 15673}, {53, 15695}, {49, 15717}, {46, 15739}, {51, 15739}, {49, 15782}, {51, 15804}, {53, 15826}, {54, 15847}, {56, 15869}, {58, 15891}, {46, 15913}, {51, 15913}, {58, 15956}, {61, 15978}, {61, 16000}, {63, 16021}, {58, 16043}, {56, 16065}, {54, 16086}, {58, 16086}, {56, 16130}, {58, 16152}, {61, 16173}, {63, 16195}, {58, 16217}, {56, 16239}, {53, 16260}, {58, 16260}, {56, 16304}, {58, 16326}, {56, 16347}, {54, 16369}, {53, 16391}, {49, 16413}, {46, 16434}, {51, 16434}, {49, 16478}, {51, 16500}, {53, 16521}, {54, 16543}, {56, 16565}, {58, 16586}, {46, 16608}, {51, 16608}, {58, 16652}, {70, 16652}, {61, 16673}, {73, 16673}, {61, 16695}, {73, 16695}, {63, 16717}, {75, 16717}, {58, 16739}, {70, 16739}, {56, 16760}, {68, 16760}, {58, 16782}, {70, 16782}, {56, 16826}, {68, 16826}, {58, 16847}, {70, 16847}, {61, 16869}, {73, 16869}, {63, 16891}, {75, 16891}, {58, 16913}, {70, 16913}, {56, 16934}, {68, 16934}, {58, 16956}, {70, 16956}, {56, 17000}, {68, 17000}, {58, 17021}, {70, 17021}, {56, 17043}, {68, 17043}, {54, 17065}, {66, 17065}, {53, 17086}, {65, 17086}, {49, 17108}, {61, 17108}, {51, 17130}, {63, 17130}, {49, 17173}, {61, 17173}, {51, 17195}, {63, 17195}, {53, 17217}, {65, 17217}, {54, 17239}, {66, 17239}, {56, 17260}, {68, 17260}, {58, 17282}, {70, 17282}, {51, 17304}, {63, 17304}, {58, 17347}, {70, 17347}, {61, 17369}, {73, 17369}, {61, 17391}, {73, 17391}, {63, 17413}, {75, 17413}, {58, 17434}, {70, 17434}, {56, 17456}, {68, 17456}, {58, 17478}, {70, 17478}, {56, 17521}, {68, 17521}, {58, 17543}, {70, 17543}, {61, 17565}, {73, 17565}, {63, 17586}, {75, 17586}, {58, 17608}, {70, 17608}, {56, 17630}, {68, 17630}, {58, 17652}, {70, 17652}, {63, 17695}, {75, 17695}, {65, 17717}, {77, 17717}, {66, 17739}, {78, 17739}, {65, 17760}, {77, 17760}, {63, 17782}, {75, 17782}, {61, 17804}, {73, 17804}, {58, 17826}, {70, 17826}, {56, 17869}, {68, 17869}, {58, 17891}, {70, 17891}, {56, 17913}, {68, 17913}, {54, 17934}, {66, 17934}, {53, 17956}, {65, 17956}, {49, 17978}, {61, 17978}, {51, 18000}, {63, 18000}, {59, 18043}, {62, 18065}, {62, 18086}, {64, 18108}, {59, 18130}, {57, 18152}, {59, 18173}, {57, 18217}, {59, 18239}, {62, 18260}, {64, 18282}, {59, 18304}, {57, 18326}, {59, 18347}, {57, 18391}, {59, 18413}, {57, 18434}, {55, 18456}, {54, 18478}, {50, 18500}, {52, 18521}, {50, 18565}, {52, 18586}, {54, 18608}, {55, 18630}, {57, 18652}, {59, 18673}, {52, 18695}, {59, 18739}, {62, 18760}, {62, 18782}, {64, 18804}, {59, 18826}, {57, 18847}, {59, 18869}, {57, 18913}, {59, 18934}, {62, 18956}, {64, 18978}, {59, 19000}, {57, 19021}, {59, 19043}, {57, 19086}, {59, 19108}, {57, 19130}, {55, 19152}, {54, 19173}, {50, 19195}, {52, 19217}, {50, 19260}, {52, 19282}, {54, 19304}, {55, 19326}, {57, 19347}, {59, 19369}, {52, 19391}, {59, 19434}, {71, 19434}, {62, 19456}, {74, 19456}, {62, 19478}, {74, 19478}, {64, 19500}, {76, 19500}, {59, 19521}, {71, 19521}, {57, 19543}, {69, 19543}, {59, 19565}, {71, 19565}, {57, 19608}, {69, 19608}, {59, 19630}, {71, 19630}, {62, 19652}, {74, 19652}, {64, 19673}, {76, 19673}, {59, 19695}, {71, 19695}, {57, 19717}, {69, 19717}, {59, 19739}, {71, 19739}, {57, 19782}, {69, 19782}, {59, 19804}, {71, 19804}, {57, 19826}, {69, 19826}, {55, 19847}, {67, 19847}, {54, 19869}, {66, 19869}, {50, 19891}, {62, 19891}, {52, 19913}, {64, 19913}, {50, 19956}, {62, 19956}, {52, 19978}, {64, 19978}, {54, 20000}, {66, 20000}, {55, 20021}, {67, 20021}, {57, 20043}, {69, 20043}, {59, 20065}, {71, 20065}, {52, 20086}, {64, 20086}, {59, 20130}, {71, 20130}, {62, 20152}, {74, 20152}, {62, 20173}, {74, 20173}, {64, 20195}, {76, 20195}, {59, 20217}, {71, 20217}, {57, 20239}, {69, 20239}, {59, 20260}, {71, 20260}, {57, 20304}, {69, 20304}, {59, 20326}, {71, 20326}, {62, 20347}, {74, 20347}, {64, 20369}, {76, 20369}, {59, 20391}, {71, 20391}, {57, 20413}, {69, 20413}, {59, 20434}, {71, 20434}, {64, 20478}, {76, 20478}, {66, 20500}, {78, 20500}, {67, 20521}, {79, 20521}, {66, 20543}, {78, 20543}, {64, 20565}, {76, 20565}, {62, 20586}, {74, 20586}, {59, 20608}, {71, 20608}, {57, 20652}, {69, 20652}, {59, 20673}, {71, 20673}, {57, 20695}, {69, 20695}, {55, 20717}, {67, 20717}, {54, 20739}, {66, 20739}, {50, 20760}, {62, 20760}, {52, 20782}, {64, 20782}, {45, 20869}, {45, 20891}, {47, 20902}, {45, 20923}, {47, 20934}, {45, 20956}, {45, 20978}, {47, 20989}, {45, 21010}, {47, 21021}, {45, 21043}, {45, 21065}, {47, 21076}, {45, 21097}, {47, 21108}, {45, 21130}, {45, 21152}, {47, 21163}, {45, 21184}, {47, 21195}, {45, 21217}, {45, 21239}, {47, 21250}, {45, 21271}, {47, 21282}, {45, 21304}, {45, 21326}, {47, 21336}, {45, 21358}, {47, 21369}, {45, 21391}, {45, 21413}, {47, 21423}, {45, 21445}, {47, 21456}, {45, 21478}, {45, 21500}, {47, 21510}, {45, 21532}, {47, 21543}};

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
	else
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
		peripheralDeviceOutput.beepFrequency = 0;
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
