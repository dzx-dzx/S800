
#include <stdint.h>
#include <stdbool.h>
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
const uint16_t solarTermsIn2021[] = {0};

bool isLeapYear(uint16_t year);
void getTimeFromTimestamp(struct Time *time, uint64_t timestamp, uint32_t timeZone);

char convertNumberToChar(uint8_t number);
uint8_t getSegmentDisplayControlWord(char character);

void segmentDisplayBlink(char *segmentDisplayCharacter, uint64_t counter, uint8_t blinkDigitByte);
void beepPlay(uint32_t *beepFrequency, uint64_t counter, uint16_t *beepSequence);

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

	while (1)
	{

		//I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT0,0xff);
		for (i = 0; i < 8; i++)
		{
			flash_seg(i, peripheralDeviceOutput.segmentDisplayControlWord[i]);
		}

		PWMOutputState(PWM0_BASE, (PWM_OUT_7_BIT), peripheralDeviceOutput.beepFrequency != 0);

		if (peripheralDeviceOutput.beepFrequency != 0)
		{
			PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32SysClock / peripheralDeviceOutput.beepFrequency);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ui32SysClock / peripheralDeviceOutput.beepFrequency / 3);
		}

		//在读写操作同时存在的情况下,似乎有时读得数据会变为0x0.取代直接读取按钮状态的代码,下列代码防止错误数据的传入.
		tempKeyboardStateByte = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		if (tempKeyboardStateByte != 0x0)
			peripheralDeviceInput.keyPadStateByte = tempKeyboardStateByte;
		peripheralDeviceInput.buttonStateByte = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1 | GPIO_PIN_0);
	}
}
void flash_seg(uint8_t display_index, uint8_t control_word)
{
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, control_word);
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)(1 << display_index));
	Delay(2000);
	// I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~((uint8_t)(1 << display_index)));
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)(0));
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

	struct Time countdownTime;
	static uint64_t countdownTimestamp;

	struct Time alarmTime[NUMBER_OF_ALARMS];
	static uint64_t alarmTimestamp[NUMBER_OF_ALARMS];

	struct Mode
	{
		uint8_t master;
		uint8_t time;
		uint8_t calender;
		uint8_t countdown;
		uint8_t alarm;
	};
	static struct Mode mode = {
		MASTER_MODE_CALENDER,
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
		peripheralDeviceInput.UARTMessageReceiveFinishedCountdown++;
	else if (peripheralDeviceInput.UARTMessageReceiveFinishedCountdown == 100)
	{
		peripheralDeviceInput.UARTMessageReceiveFinishedCountdown = 1926;
		UARTMessageReceived = true;
		peripheralDeviceInput.UARTMessageTail = peripheralDeviceInput.UARTMessage;
	}

	//Keypad及按钮状态.
	if (counter % (SYSTICK_FREQUENCY * 20 / 1000) == 0) //20ms
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

	//主模式切换.
	if (buttonPressed[0])
		mode.master = (mode.master + NUMBER_OF_MASTER_MODES - 1) % NUMBER_OF_MASTER_MODES;
	if (buttonPressed[1])
		mode.master = (mode.master + 1) % NUMBER_OF_MASTER_MODES;
	UARTCharPut(UART0_BASE, convertNumberToChar(mode.master));
	//
	switch (mode.master)
	{
	case MASTER_MODE_TIME:
	{

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
		if (keypadPressed[5])
		{
			mode.time = CALENDER_MODE_SET;
			calenderSelectedDigit = 7;
		}
		else if (keypadPressed[7])
			mode.time = CALENDER_MODE_DISPLAY;
		segmentDisplayCharacter[0] = convertNumberToChar(time.year % 10000 / 1000);
		segmentDisplayCharacter[1] = convertNumberToChar(time.year % 1000 / 100);
		segmentDisplayCharacter[2] = convertNumberToChar(time.year % 100 / 10);
		segmentDisplayCharacter[3] = convertNumberToChar(time.year % 10);
		segmentDisplayCharacter[4] = convertNumberToChar((time.month + 1) % 100 / 10);
		segmentDisplayCharacter[5] = convertNumberToChar((time.month + 1) % 10);
		segmentDisplayCharacter[6] = convertNumberToChar((time.day + 1) % 100 / 10);
		segmentDisplayCharacter[7] = convertNumberToChar((time.day + 1) % 10);

		if (mode.time == CALENDER_MODE_SET)
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
					timestampInUTC -= (isLeapYear(time.year) ? daysInMonthInLeapYear[time.month] : daysInMonth[time.month]) * 86400 * 100; //认为时间真正增加一月.
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
		segmentDisplayCharacter[0] = 'A';
		segmentDisplayCharacter[1] = 'B';
		segmentDisplayCharacter[2] = 'C';
		segmentDisplayCharacter[3] = 'D';
		segmentDisplayCharacter[4] = 'E';
		segmentDisplayCharacter[5] = 'F';
		segmentDisplayCharacter[6] = 'G';
		segmentDisplayCharacter[7] = 'H';
	}
	break;
	case MASTER_MODE_COUNTDOWN:
	{

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
	default:
		break;
	}
	for (i = 0; i < 8; i++)
	{
		peripheralDeviceOutput.segmentDisplayControlWord[i] = getSegmentDisplayControlWord(segmentDisplayCharacter[i]);
		segmentDisplayCharacter[i] = ' ';
	}

	if (mode.alarm == ALARM_MODE_RINGING)
		peripheralDeviceOutput.beepFrequency = 2000;
	else
		peripheralDeviceOutput.beepFrequency = 0;

	if (UARTMessageReceived)
	{
		if (strncmp(msg, "set", 3))
		{
			;
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
	time->second = timestamp / 100 % 60;
	time->minute = timestamp / 100 % 3600 / 60;
	time->hour = timestamp / 100 % 86400 / 3600 + timeZone;

	time->day = timestamp / 100 / 86400;
	if (time->hour >= 24)
	{
		time->hour -= 24;
		time->day++;
	}
	time->year = 1970;
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
