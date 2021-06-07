
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
#define MASTER_MODE_BOOT 3
#define NUMBER_OF_MASTER_MODES 3 //排除启动模式.
#define TIME_MODE_DISPLAY 0
#define TIME_MODE_SET 1

void Delay(uint32_t value);
void UARTStringPut(const char *cMessage);

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);

void S800_GPIO_Init(void);
void S800_I2C0_Init(void);
void S800_Int_Init(void);
void S800_UART_Init(void);

void flash_seg(uint8_t display_index, uint8_t control_word);
uint32_t ui32SysClock;

struct PeripheralDeviceInput
{
	uint8_t keyPadStateByte;
	uint8_t buttonStateByte;
	char UARTMessage[100];
	char *UARTMessageTail;
} peripheralDeviceInput;

struct PeripheralDeviceOutput
{
	uint8_t segmentDisplayControlWord[8];
	uint8_t LEDDisplayByte;
} peripheralDeviceOutput;

struct Date
{
	uint8_t year, month, day, hour, minute, second;
} date;
uint8_t display_mode = 0;

bool isLeapYear(uint16_t year);

char convertNumberToChar(uint8_t number);
uint8_t getSegmentDisplayControlWord(char character);

int main(void)
{
	uint8_t i = 0;

	uint8_t tempKeyboardStateByte = 0xff;

	//use internal 16M oscillator, HSI
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ | SYSCTL_OSC_INT | SYSCTL_USE_OSC), 16000000);
	peripheralDeviceInput.keyPadStateByte = tempKeyboardStateByte;

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	S800_Int_Init();

	while (1)
	{

		//I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT0,0xff);
		for (i = 0; i < 8; i++)
		{
			flash_seg(i, peripheralDeviceOutput.segmentDisplayControlWord[i]);
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

	struct Time
	{
		uint16_t year;
		uint16_t month;
		uint16_t day;
		uint16_t hour;
		uint16_t minute;
		uint16_t second;
	};
	struct Time time;
	static uint64_t timeInUTC; //单位为0.01s
	const uint16_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	const uint16_t daysInMonthInLeapYear[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	const uint16_t daysInYear = 365;
	const uint16_t daysInLeapYear = 366;

	static uint32_t counter;
	struct Mode
	{
		uint8_t master;
		uint8_t time;
		// uint8_t calender;
	};
	static struct Mode mode = {
		MASTER_MODE_TIME,
		TIME_MODE_DISPLAY};
	static uint8_t timeModeSelectedDigit;
	static char segmentDisplayCharacter[8];

	static uint8_t previousKeypadState[8];
	static uint8_t previousButtonState[2];
	uint8_t currentKeypadState[8] = {0};
	uint8_t currentButtonState[2] = {0};
	uint16_t keypadPressed[8] = {0}, keypadHold[8] = {0};
	uint16_t buttonPressed[2] = {0}, buttonHold[2] = {0};

	counter++;

	//Keypad按钮状态.
	if (counter % (SYSTICK_FREQUENCY * 50 / 1000) == 0) //50ms
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
		timeInUTC++;
	time.second = timeInUTC / 100 % 60;
	time.minute = timeInUTC / 100 % 3600 / 60;
	time.hour = timeInUTC / 100 % 86400 / 3600 + 8;

	time.day = timeInUTC / 100 / 86400;
	if (time.hour >= 24)
	{
		time.hour -= 24;
		time.day++;
	}
	time.year = 1970;
	time.month = 1;

	while (time.day > (isLeapYear(time.year) ? daysInLeapYear : daysInYear))
	{
		time.day -= (isLeapYear(time.year) ? daysInLeapYear : daysInYear);
		time.year++;
	}
	while (time.day > (isLeapYear(time.year) ? daysInMonthInLeapYear[time.month] : daysInMonth[time.month]))
	{
		time.day -= (isLeapYear(time.year) ? daysInMonthInLeapYear[time.month] : daysInMonth[time.month]);
		time.month++;
	}

	if (buttonPressed[0])
		mode.master = (mode.master + NUMBER_OF_MASTER_MODES - 1) % NUMBER_OF_MASTER_MODES;
	if (buttonPressed[1])
		mode.master = (mode.master + 1) % NUMBER_OF_MASTER_MODES;

	//
	switch (mode.master)
	{
	case MASTER_MODE_TIME:
	{

		if (keypadPressed[5])
			mode.time = TIME_MODE_SET;
		else if (keypadPressed[7])
			mode.time = TIME_MODE_DISPLAY;
		segmentDisplayCharacter[0] = convertNumberToChar(time.hour / 10);
		segmentDisplayCharacter[1] = convertNumberToChar(time.hour % 10);
		segmentDisplayCharacter[2] = convertNumberToChar(time.minute / 10);
		segmentDisplayCharacter[3] = convertNumberToChar(time.minute % 10);
		segmentDisplayCharacter[4] = convertNumberToChar(time.second / 10);
		segmentDisplayCharacter[5] = convertNumberToChar(time.second % 10);
		segmentDisplayCharacter[6] = convertNumberToChar(timeInUTC % 100 / 10);
		segmentDisplayCharacter[7] = convertNumberToChar(timeInUTC % 10);

		if (mode.time == TIME_MODE_SET)
		{
			if (keypadPressed[0])
				timeModeSelectedDigit = (timeModeSelectedDigit + 5) % 6;
			if (keypadPressed[2])
				timeModeSelectedDigit = (timeModeSelectedDigit + 1) % 6;
			if (keypadPressed[6])
				switch (timeModeSelectedDigit)
				{
				case 0:
					timeInUTC += 36000 * 100;
					break;
				case 1:
					timeInUTC += 3600 * 100;
					break;
				case 2:
					timeInUTC += 600 * 100;
					break;
				case 3:
					timeInUTC += 60 * 100;
					break;
				case 4:
					timeInUTC += 10 * 100;
					break;
				case 5:
					timeInUTC += 1 * 100;
					break;
				default:
					break;
				}
			if (keypadPressed[1])
				switch (timeModeSelectedDigit)
				{
				case 0:
					timeInUTC -= 36000 * 100;
					break;
				case 1:
					timeInUTC -= 3600 * 100;
					break;
				case 2:
					timeInUTC -= 600 * 100;
					break;
				case 3:
					timeInUTC -= 60 * 100;
					break;
				case 4:
					timeInUTC -= 10 * 100;
					break;
				case 5:
					timeInUTC -= 1 * 100;
					break;
				default:
					break;
				}
			if (counter % (SYSTICK_FREQUENCY * 200 / 1000) < (SYSTICK_FREQUENCY * 200 / 1000) / 2)
				segmentDisplayCharacter[timeModeSelectedDigit] = ' ';
		}
	}
	break;
	case MASTER_MODE_CALENDER:
	{
		segmentDisplayCharacter[0] = '1';
		segmentDisplayCharacter[1] = '9';
		segmentDisplayCharacter[2] = '2';
		segmentDisplayCharacter[3] = '6';
		segmentDisplayCharacter[4] = '0';
		segmentDisplayCharacter[5] = '8';
		segmentDisplayCharacter[6] = '1';
		segmentDisplayCharacter[7] = '7';
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
	default:
		break;
	}
	for (i = 0; i < 8; i++)
		peripheralDeviceOutput.segmentDisplayControlWord[i] = getSegmentDisplayControlWord(segmentDisplayCharacter[i]);
}
void UART0_Handler(void)
{

	uint32_t ulStatus;
	ulStatus = UARTIntStatus(UART0_BASE, true);
	UARTIntClear(UART0_BASE, ulStatus);

	// if (ulStatus == UART_INT_RX)
	// {
	// 	tail = msg;
	// }
	// else if (ulStatus == UART_INT_RT)
	// {
	// 	tail = tail;
	// }

	// while (UARTCharsAvail(UART0_BASE))
	// {
	// 	*tail++ = UARTCharGetNonBlocking(UART0_BASE);
	// }
	// *tail = '\0';
	// receiveFinishedCountdown = 0;
	// UARTStringPut(msg);
	// UARTStringPut("\n");
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
