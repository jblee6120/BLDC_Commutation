#include "stm32f4xx.h"   // Compiler symbols에 STM32F407xx를 선언하면 stm32f407xx.h를 include 해준다.
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "monitor.h"  // Full speed USB 기반의 고속 monitoring을 위한 header file

void SystemClock_Config(void);
void Error_Handler(void);

void RCC_Configuration();
void DWT_Init();             // DWT initialization 함수
void DWT_us_Delay(uint32_t us); // 인자의 단위는 microseconds
void DWT_ms_Delay(uint32_t ms); // 인자의 단위는 microseconds
void UsbPutString(char *s);
void PutStringTest();

void MCO_Out();               // MCO를 통해 clock을 출력하는 함수
void GpioBitTest();           // GPIO 특정 bit를 Set, Clear 하는 것을 학습할 수 있는 함수r interrupt를 사용하는 방법을 학습하는 함수
void GpioWordTest();          // GPIO의 여러 bit를 한번에 읽고 쓰는 것을 학습할 수 있는 함수
void USART3_Configuration();  // USART3를 사용하기 위한 설정(configuration) 함수
void TX3_Char(char);          // USART3를 이용하여 char 하나를 전송하는 함수
void TX3_PutString(char *);   // USART3를 이용하여 string data를 전송하는 함수
void USART3_Test();           // USART3를 설정해서 data를 PC로 전송하는 것을 학습할 수 있는 함수
void SineSpeedTest();         // sinf 함수를 계산하는데 소요되는 시간을 비교하는 것을 학습할 수 있는 함수
void SysTickInterruptTest();  // SysTick timer interrupt를 사용하는 방법을 학습할 수 있는 함수
int VCP_read(void *pBuffer, int size); // USB buffer로부터 data를 읽어들이는 함수
void EchoTest();              // 사용자가 PC에서 USB로 전송한 글자를 다시 loop back으로 되돌려보내 monitor에 display 하는 함수
void FsMonitoringTest();      // USB Full speed를 이용한 monitoring test
void DacTest();               // DAC를 학습할 수 있는 함수
void AdcTest();               // ADC 사용을 학습할 수 있는 함수
void AdcInjectedTest();       // Injected ADC의 사용을 학습할 수 있는 함수
void Timer6_InterruptTest();  // Timer6를 이용하여 timer interrupt를 사용하는 방법을 학습하는 함수
void Timer1_EncoderTest();    // Timer1(Advanced-control timer)의 Encoder counter 기능을 학습하는 함수
void Timer2_EncoderTest();    // Timer2(32-bit General-purpose timer)의 Encoder counter 기능을 학습하는 함수
void Timer3_EncoderTest();    // Timer3(16-bit General-purpose timer)의 Encoder counter 기능을 학습하는 함수
void ThreePhasePwmTest();     // 3상 PWM을 생성하는 방법을 학습하는 함수. Timer1과 Timer8을 이용할 수 있음.
void ExtInterruptTest();      // External interrupt를 사용하는 방법을 학습하는 함수
void PwmInputCaptureTest();   // PWM 신호의 주파수 및 duty를 input capture를 이용하여 측정하는 것을 학습하는 함수
void SpiTest();               // SPI 통신을 학습할 수 있는 함수
void TimerDmaTest();          // Timer DMA를 활용하는 방법을 학습할 수 있는 함수


// 6 Step commutation의 기동동작(Start up)만 수행하는 코드를 실습하는 파일.
// ------------------------------------------------------------------
//    STM32407VE         DRV8302-based Inverter Board
// ------------------------------------------------------------------
//       PE13           WH 하단 FET gate 신호 (GPIO)
//       PE12      	    WL 상단 FET gate 신호 (PWM)
//       PE11           VH 하단 FET gate 신호 (GPIO)
//       PE10           VL 상단 FET gate 신호 (PWM)
//       PE 9           UH 하단 FET gate 신호 (GPIO)
//       PE 8           UL 상단 FET gate 신호 (PWM)
//
//       PA15(ENC)         Encoder A
//       PB3 (ENC)         Encoder B
//
//       PC14	              Switch1
//       PC15	              Switch2
//
//       PA 3            U 상 Back EMF
//       PA 5            V 상 Back EMF
//       PA 6            W 상 Back EMF
//
//       PC10                EN_GATE
// ------------------------------------------------------------------
//    STM32F407VE        Peripheral Board
// ------------------------------------------------------------------
//      5V                   5V
//      GND                  GND
//      PC0(ADC)             ANA1
// ------------------------------------------------------------------
// BLDC를 연결할 때 BL5057 model (no gear, 뒷꽁무니 encoder 달려 있음)의 경우
// 하양 -- U,  파랑 -- V,  갈색 -- W
// 로 연결한다.
// 실험 방법
// 1. ANA1을  반시계방향으로 최대로 돌려 놓은 후 Reset을 누른다.
// 2. Switch1(PC14)를 누르면 Align 방식의 BLDC Sensorless 제어를 수행한다.
// 3. Switch2(PC15)을 누르면 정지한다.
// ------------------------------------------------------------------

#define PE8_AF (0x10 << GPIO_MODER_MODER8_Pos)
#define PE9_AF (0x10 << GPIO_MODER_MODER9_Pos)
#define PE10_AF (0x10 << GPIO_MODER_MODER10_Pos)
#define PE11_AF (0x10 << GPIO_MODER_MODER11_Pos)
#define PE12_AF (0x10 << GPIO_MODER_MODER12_Pos)
#define PE13_AF (0x10 << GPIO_MODER_MODER13_Pos)

#define PE8_GPIO (0x01 << GPIO_MODER_MODER8_Pos)
#define PE9_GPIO (0x01 << GPIO_MODER_MODER9_Pos)
#define PE10_GPIO (0x01 << GPIO_MODER_MODER10_Pos)
#define PE11_GPIO (0x01 << GPIO_MODER_MODER11_Pos)
#define PE12_GPIO (0x01 << GPIO_MODER_MODER12_Pos)
#define PE13_GPIO (0x01 << GPIO_MODER_MODER13_Pos)

#define PE8_GPIO_HIGH (0x1 << GPIO_ODR_OD8_Pos)
#define PE9_GPIO_HIGH (0x1 << GPIO_ODR_OD9_Pos)
#define PE10_GPIO_HIGH (0x1 << GPIO_ODR_OD10_Pos)
#define PE11_GPIO_HIGH (0x1 << GPIO_ODR_OD11_Pos)
#define PE12_GPIO_HIGH (0x1 << GPIO_ODR_OD12_Pos)
#define PE13_GPIO_HIGH (0x1 << GPIO_ODR_OD13_Pos)

#define PE8_GPIO_LOW (0x0 << GPIO_ODR_OD8_Pos)
#define PE9_GPIO_LOW (0x0 << GPIO_ODR_OD9_Pos)
#define PE10_GPIO_LOW (0x0 << GPIO_ODR_OD10_Pos)
#define PE11_GPIO_LOW (0x0 << GPIO_ODR_OD11_Pos)
#define PE12_GPIO_LOW (0x0 << GPIO_ODR_OD12_Pos)
#define PE13_GPIO_LOW (0x0 << GPIO_ODR_OD13_Pos)

#define MODER1	(PE8_GPIO | PE9_GPIO | PE10_AF | PE11_AF | PE12_GPIO | PE13_GPIO)	// UH(PE9)=01, UL(PE8)=01, VH(PE11)=10, VL(PE10)=10, WH(PE13)=01, WL(PE12)=01
#define MODER2	(PE8_GPIO | PE9_GPIO | PE10_GPIO | PE11_GPIO | PE12_AF | PE13_AF)	// UH(PE9)=01, UL(PE8)=01, VH(PE11)=01, VL(PE10)=01, WH(PE13)=10, WL(PE12)=10
#define MODER3	(PE8_GPIO | PE9_GPIO | PE10_GPIO | PE11_GPIO | PE12_AF | PE13_AF)	// UH(PE9)=01, UL(PE8)=01, VH(PE11)=01, VL(PE10)=01, WH(PE13)=10, WL(PE12)=10
#define MODER4	(PE8_AF | PE9_AF | PE10_GPIO | PE11_GPIO | PE12_GPIO | PE13_GPIO)	// UH(PE9)=10, UL(PE8)=10, VH(PE11)=01, VL(PE10)=01, WH(PE13)=01, WL(PE12)=01
#define MODER5	(PE8_AF | PE9_AF | PE10_GPIO | PE11_GPIO | PE12_GPIO | PE13_GPIO)	// UH(PE9)=10, UL(PE8)=10, VH(PE11)=01, VL(PE10)=01, WH(PE13)=01, WL(PE12)=01
#define MODER6	(PE8_GPIO | PE9_GPIO | PE10_AF | PE11_AF | PE12_GPIO | PE13_GPIO)	// UH(PE9)=01, UL(PE8)=01, VH(PE11)=10, VL(PE10)=10, WH(PE13)=01, WL(PE12)=01
#define MODER56 (PE8_AF | PE9_AF | PE10_AF | PE11_AF | PE12_GPIO | PE13_GPIO)

#define GPIO1 	(PE8_GPIO_HIGH | PE9_GPIO_LOW | PE10_GPIO_LOW | PE11_GPIO_LOW | PE12_GPIO_LOW | PE13_GPIO_LOW) // UH(PE9)=0, UL(PE8)=1, VH(PE11)=X, VL(PE10)=X, WH(PE13)=0, WL(PE12)=0
#define GPIO2 	(PE8_GPIO_HIGH | PE9_GPIO_LOW | PE10_GPIO_LOW | PE11_GPIO_LOW | PE12_GPIO_LOW | PE13_GPIO_LOW) // UH(PE9)=0, UL(PE8)=1, VH(PE11)=0, VL(PE10)=0, WH(PE13)=X, WL(PE12)=X
#define GPIO3 	(PE8_GPIO_LOW | PE9_GPIO_LOW | PE10_GPIO_HIGH | PE11_GPIO_LOW | PE12_GPIO_LOW | PE13_GPIO_LOW) // UH(PE9)=0, UL(PE8)=0, VH(PE11)=0, VL(PE10)=1, WH(PE13)=X, WL(PE12)=X
#define GPIO4 	(PE8_GPIO_LOW | PE9_GPIO_LOW | PE10_GPIO_HIGH | PE11_GPIO_LOW | PE12_GPIO_LOW | PE13_GPIO_LOW) // UH(PE9)=X, UL(PE8)=X, VH(PE11)=0, VL(PE10)=1, WH(PE13)=0, WL(PE12)=0
#define GPIO5 	(PE8_GPIO_LOW | PE9_GPIO_LOW | PE10_GPIO_LOW | PE11_GPIO_LOW | PE12_GPIO_HIGH | PE13_GPIO_LOW) // UH(PE9)=X, UL(PE8)=X, VH(PE11)=0, VL(PE10)=0, WH(PE13)=0, WL(PE12)=1
#define GPIO6 	(PE8_GPIO_LOW | PE9_GPIO_LOW | PE10_GPIO_LOW | PE11_GPIO_LOW | PE12_GPIO_HIGH | PE13_GPIO_LOW) // UH(PE9)=0, UL(PE8)=0, VH(PE11)=X, VL(PE10)=X, WH(PE13)=0, WL(PE12)=1
#define GPIO56	(PE8_GPIO_LOW | PE9_GPIO_LOW | PE10_GPIO_LOW | PE11_GPIO_LOW | PE12_GPIO_HIGH | PE13_GPIO_LOW)

#define STARTUP_DELAY_MULTIPLIER 200
#define EDGE_RISING 0
#define EDGE_FALLING 1
#define ADC_ZC_THRESHOLD 77

unsigned int *pPwmPattern;
unsigned int *pGpioPattern;
unsigned int *pADMUXTable;


char data[256];
extern uint8_t UserTxBufferFS[];  // USB Transmit Buffer
extern uint8_t UserRxBufferFS[];  // USB Receive Buffer
extern int RxPosition, RxSize;    // USB Receive에 사용되는 변수들
extern char ReadDone;
extern USBD_HandleTypeDef hUsbDeviceFS; // usb_device.c에 정의되어 있음
extern ToHost toSimulink;



void StartMotor();
void ExtInterrupt_Init();

//----------------------------------------------------------------------------------
// 아래의 delay table은 Matlab을 이용해서 계산하였으며 기본이 되는 알고리즘은
// 아래의 참고문헌을 참조하였다.
// Reference : David Austin, Generate stepper-motor speed profiles
// in real time, EE Times-India, Jan 2005, pp.1-5.
// 아래에 나와있는 숫자의 단위는 100 [us]이다.
// 따라서 이 sketch file이 있는 folder 안에 m-fiel script가 있는데
// 거기서 구한 counter 값을 200 으로 나누어준뒤 정수로 환산한 값이
// 바로 아래의 table에 나온 값이다. 200 으로 나누어주는 이유는 counter의
// clock period가 0.5 us 이기때문에 그것을 200 개 반복하면 100 us 가 되기
// 때문이다.
// 아래의 table에 나와 있는 data는 0.2초 동안 30 RPS에 도달하기 위한 선형
// 가속도를 생성하기 위한 delay 값이다.
//----------------------------------------------------------------------------------
//#define STARTUP_NUM_COMMUTATIONS 18  // 요기 아래 나와있는 배열의 크기를 나타낸다. 0.2초에 30 RPS에 도달하는 경우
//#define STARTUP_NUM_COMMUTATIONS 27  // 요기 아래 나와있는 배열의 크기를 나타낸다. 0.3초에 30 RPS에 도달하는 경우
#define STARTUP_NUM_COMMUTATIONS 45  // 요기 아래 나와있는 배열의 크기를 나타낸다. 0.5초에 30 RPS에 도달하는 경우
//#define STARTUP_NUM_COMMUTATIONS 90  // 요기 아래 나와있는 배열의 크기를 나타낸다. 1초에 30 RPS에 도달하는 경우
//#define STARTUP_NUM_COMMUTATIONS 180  // 요기 아래 나와있는 배열의 크기를 나타낸다. 2초에 30 RPS에 도달하는 경우
//----------------------------------------------------------------------------------
// startupDelays[]는 이 sketch가 저장된 folder안에 있는 Matlab script를 이용하여
// 생성한 것이다. 도데체 무엇인지 궁금하다면 역시 Prof. Lee의 hand-written note를
// 참조해야 한다.
//----------------------------------------------------------------------------------
//unsigned int startupDelays[] = {471, 195, 150, 126, 111, 101,  93,  86,  81, 76, 73, 70, 67, 64, 62, 60, 58, 56};  // 0.2초에 30 RPS에 도달하는 경우
//unsigned int startupDelays[] = {577,239,184,155,136,123,113,105,99,94,89,85,82,79,76,73,71,69,67,65,64,62,61,60,58,57,56}; // 0.3초에 30 RPS에 도달하는 경우
unsigned int startupDelays[] = {745,309,237,200,176,159,146,136,128,121,115,110,105,101,98,95,92,89,87,84,82,80,79,77,75,74,72,71,70,69,67,66,65,64,63,63,62,61,60,59,59,58,57,57,56}; // 0.5초에 30 RPS에 도달하는 경우
//unsigned int startupDelays[] = {1054,437,335,282,249,225,207,193,181,171,163,155,149,143,138,134,130,126,123,119,116,114,111,109,106,104,102,101,99,97,95,94,92,91,90,88,87,86,85,84,83,82,81,80,79,78,77,76,76,75,74,73,73,72,71,71,70,70,69,68,68,67,67,66,66,65,65,64,64,63,63,62,62,61,61,61,60,60,59,59,59,58,58,58,57,57,57,56,56,56}; // 1초에 30 RPS에 도달하는 경우에 도달하는 경우
//unsigned int startupDelays[] = {1491,617,474,399,352,318,293,272,256,242,230,220,211,203,196,189,184,178,173,169,165,161,157,154,151,148,145,142,140,137,135,133,131,129,127,125,123,122,120,119,117,116,114,113,112,111,109,108,107,106,105,104,103,102,101,100,99,98,97,97,96,95,94,94,93,92,91,91,90,89,89,88,88,87,86,86,85,85,84,84,83,83,82,82,81,81,80,80,79,79,78,78,77,77,77,76,76,75,75,75,74,74,74,73,73,73,72,72,72,71,71,71,70,70,70,69,69,69,68,68,68,68,67,67,67,67,66,66,66,65,65,65,65,65,64,64,64,64,63,63,63,63,62,62,62,62,62,61,61,61,61,61,60,60,60,60,60,59,59,59,59,59,58,58,58,58,58,58,57,57,57,57,57,57,56,56,56,56,56,56}; // 2초에 30 RPS에 도달하는 경우에 도달하는 경우
//----------------------------------------------------------------------
// PWM pattern과 GPIO pattern의 배열을 만들어 두자. Forward와 Backward에
// 너무 민감해 할 필요는 없다. Forward를 사용했을때 돌아가는 방향이
// 그냥 Forward direction이고 Backward를 사용했을때 돌아가는 방향이
// Backward direction이라고 생각하면 된다.
//----------------------------------------------------------------------

unsigned int pwmPatternsForward[] = {
		MODER1,
		MODER2,
		MODER3,
		MODER4,
		MODER5,
		MODER6
};

unsigned int pwmPatternsBackward[] = {
		MODER1,
		MODER2,
		MODER3,
		MODER4,
		MODER5,
		MODER6
};

unsigned int gpioPatternsForward[]= {
		GPIO1,
		GPIO2,
		GPIO3,
		GPIO4,
		GPIO5,
		GPIO6
};

unsigned int gpioPatternsBackward[] ={
		GPIO4,
		GPIO3,
		GPIO2,
		GPIO1,
		GPIO6,
		GPIO5
};

unsigned int CommutationStep2Hall[6] = {5,1,3,2,6,4};
unsigned int CommutationStep2Pattern[6] = {4,3,2,1,6,5};

volatile unsigned char zcPolarity;
volatile uint32_t filteredTimeSinceCommutation;
volatile uint32_t timeSinceCommutation;

volatile unsigned char nextCommutationStep;
volatile uint16_t voltage_ADC;

float control_freq;
long control_us_period;
float ts;
float time = 0.0;


unsigned int p;
int main(void)
{
	RCC_Configuration();

	//-------------------------------------------------------------------------------------
	// SystemCoreClock 변수의 갱신. 이것이 제대로 동작하려면 system_stm32f407xx.c의 제일 위에 있는 HSE_VALUE를
	// 8000000 으로 변경해 주어야 한다. 이것을 안해 주고 default 값인 25000000을 사용했다가 DWT_Delay()가 제대로
	// 동작하지 않는 문제가 발생했다. debugging 하느라 한참 고생함.
	//-------------------------------------------------------------------------------------
	SystemCoreClockUpdate();


	DWT_Init();            // DWT initialization
	MCO_Out();             // MCO를 통해 (분주된) clock을 출력하는 함수. PA8 사용
	MX_USB_DEVICE_Init();  // USB를 사용하려면 이 line을 활성화 해야 한다.


	TIM1_Init();

	while(GPIOA->IDR & 0x04 == 0x04);

	GPIOD->ODR |= GPIO_ODR_OD13;

	StartMotor();

	while(1)
	{

	}
}


void RCC_Configuration()
{
	// Flash memory Prefetch buffer, wait state 설정
    FLASH->ACR |= (uint32_t)FLASH_ACR_ICEN;    // Instruction cache enable
    FLASH->ACR |= (uint32_t)FLASH_ACR_DCEN;    // Data cache enable
    FLASH->ACR |= (uint32_t)FLASH_ACR_PRFTEN;  // Prefetch enable
    FLASH->ACR &= ~((uint32_t)FLASH_ACR_LATENCY);
    FLASH->ACR |= (0x5U << FLASH_ACR_LATENCY_Pos);  // (0x5)가   five wait states에 해당

    //----------------------------------------------------------
    // HSEBYP의 사용에 대해
    // HSE crystal oscillator bypassed with external clock
    // X-TAL이 아닌 그냥 oscillator를 사용할 경우 이 option을 사용한다. 하지만
    // Core407V의 경우 X-TAL을 사용하므로 이 option을 사용하면 안된다.
    //----------------------------------------------------------
	//RCC->CR |= (uint32_t)RCC_CR_HSEBYP;  //
    RCC->CR |= (uint32_t)RCC_CR_HSEON;   // Enable HSE
    while((RCC->CR & (uint32_t)RCC_CR_HSERDY) == 0); // Wait till HSE is ready

    // SYSCLK, HCLK, PCLK2, PCLK1 configuration
    RCC->CFGR &= ~((uint32_t)RCC_CFGR_HPRE);    // AHB Prescaler=1. 즉 SYSCLK not divided -> 168 MHz
    RCC->CFGR &= ~((uint32_t)RCC_CFGR_PPRE2);   // APB2 Prescaler=2. AHB clock divided by 2 -> 84 MHz (PCLK2)
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR &= ~((uint32_t)RCC_CFGR_PPRE1);   // APB1 Prescaler=4. AHB clock divided by 4 -> 42 MHz (PCLK1)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

    //-------------------------------------------------------------------------
    // STM32F303K에서는 PLL을 설정하기 위해 RCC_CFGR을 사용하였다.
    // 하지만 STM32F407에서는 PLL을 설정하기 위한 Register인 RCC_PLLCFGR이 별도로 있다.
    // PLL을 설정하기 위해 PLL의 source 설정, HSE의 분주비 설정, PLL multiplication 설정
    // 등은 모드 PLL을 off 시킨 후에야 설정할 수 있다.
    // PLL의 source로 HSE를 선택할 것이다. 이것은 PLLCFGR의 22번 bit를 1로 설정하면된다.
    //
    // SYSCLK = HSE*PLLN/PLLM/PLLP = 8*168/4/2 = 168 MHz
    // PLL48CLK = HSE*PLLN/PLLM/PLLQ = 8*168/4/7 = 48 MHz
    //--------------------------------------------------------------------
    RCC->CR &= ~((uint32_t)RCC_CR_PLLON);     // 일단 PLL을 Off시킨다.
    while((RCC->CR & RCC_CR_PLLRDY) == 1){;}  // PLL이 off 될때까지 기다린다.

    // PLL Source 설정
    RCC->PLLCFGR &= ~((uint32_t)RCC_PLLCFGR_PLLSRC);
    RCC->PLLCFGR |= ((uint32_t)0x1<<RCC_PLLCFGR_PLLSRC_Pos);  // PLL Source = HSE clock

    // PLLN 설정
    RCC->PLLCFGR &= ~((uint32_t)RCC_PLLCFGR_PLLN);  // 168
    RCC->PLLCFGR |= ((uint32_t)168 << RCC_PLLCFGR_PLLN_Pos);  // PLLN = 168

    // PLLM 설정
    RCC->PLLCFGR &= ~((uint32_t)RCC_PLLCFGR_PLLM);
    RCC->PLLCFGR |= ((uint32_t)0x4 << RCC_PLLCFGR_PLLM_Pos);  // PLLM = 4 [주] stm32f407xx.h에 이 부분 오류가 있다.

    // PLLP 설정
    RCC->PLLCFGR &= ~((uint32_t)RCC_PLLCFGR_PLLP);
    RCC->PLLCFGR |= ((uint32_t)0x0 << RCC_PLLCFGR_PLLP_Pos);  // PLLP = 2로 설정하려면 해당 부분에 0b00을 써주어야 한다.

    // PLLQ 설정
    RCC->PLLCFGR &= ~((uint32_t)RCC_PLLCFGR_PLLQ);
    RCC->PLLCFGR |= ((uint32_t)0x7 << RCC_PLLCFGR_PLLQ_Pos);  // PLLQ = 7

    RCC->CFGR &= ~((uint32_t)RCC_CFGR_MCO1);
    RCC->CFGR |= ((uint32_t)0x3<< RCC_CFGR_MCO1_Pos);     // MCO=0b11으로 설정해서 PLL clock이 MCO에 출력되도록 한다.

	RCC->CFGR &= ~((uint32_t)RCC_CFGR_MCO1PRE);
	RCC->CFGR |=((uint32_t)0x7 << RCC_CFGR_MCO1PRE_Pos);  // 111을 써 주어야 division by 5가 된다.
	                                                      // MCO prescaler = division by 5, PLL일 경우 168/5=33.6 MHz를 출력하게 된다.

    RCC->CR |= RCC_CR_PLLON;  // Enable PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0){;}   // Wait till PLL is ready

    RCC->CFGR &= ~((uint32_t)RCC_CFGR_SW);
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    // Select PLL as system clock source

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL){;}
}


//-----------------------------------------------------------
// MCO_Out()
// MCO에 clock을 출력하도록 하는 함수
// RCC_Configuration()을 수행하지 않은채 이 함수를 사용하면 PA8을
// MCO1으로 설정하여 내부의 HSI는 16 MHz가 MCO를 통해 출력된다.
// 만약 RCC_Configuration()을 수행한 다음에 이 함수를 사용하면
// PLL 출력의 1/5에 해당하는 33.6 MHz를 PA8을 통해 볼 수 있다.
//-----------------------------------------------------------
void MCO_Out()
{
	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOAEN;    // Port A에 clock 공급
	GPIOA->MODER &= ~((uint32_t)GPIO_MODER_MODER8);
	GPIOA->MODER |= (0x2U << GPIO_MODER_MODER8_Pos);  // MCO의 경우 alternate function. PA8의 Mode를 따라서 0x2로 설정해야 함.
	GPIOA->AFR[1] &= ~((uint32_t)GPIO_AFRH_AFRH0);    // AF0는 0000 이므로 네 bit를 모두 0으로 처리한다.
	GPIOA->OSPEEDR &= ~((uint32_t)GPIO_OSPEEDER_OSPEEDR8);
	GPIOA->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED8_Pos);  // Very high speed일 경우 11 (=0x3)로 설정
}


//--------------------------------------------------------------------
// The definitive guide to ARM Cortex-M3 and Cortex-M4 processors 라는
// 책의 page 464에 보면 CoreDebug->DEMCR에 대한 설명을 찾아볼 수 있다.
// DEMCR : Debug exception and monitor control register
// DEMCR의 24번 bit가 TRCENA 인데 DWT, ETM, ITM, TPIU를 사용하려면 TRCENA bit가
// 1로 set 되어야만 한다.
//--------------------------------------------------------------------
void DWT_Init(void)
{
	// DWT를 사용하기 위해서는 먼저 CoreDebug->DEMCR의 24번 bit인 TRCENA를 1로
	// set 한다. 그 다음 DWT의 CYCCNT를 0으로 clear 시키고 counter를 enable
	// 시키면 cycle counter를 동작시킬 수 있다.
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;  // Reset the counter
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // Enable the counter
	}
}


void DWT_us_Delay(uint32_t us) // microseconds
{
	uint32_t tp = DWT->CYCCNT + us * (SystemCoreClock/1000000)-1;  // SystemCoreClock 계산이 잘못 되고 있다. Bug 수정 필요
	while(!((tp-DWT->CYCCNT) & 0x80000000));
}

void DWT_ms_Delay(uint32_t ms)
{
	uint32_t tp = DWT->CYCCNT + ms*1000* (SystemCoreClock/1000000)-1;
	while(!((tp-DWT->CYCCNT) & 0x80000000));
}
//---------------------------------------------------------
// GPIO 특정 bit를 Set, Clear 하는 것을 학습할 수 있는 함수.
// 여기서는 PD12를 GPIO output으로 설정한 후 set, clear를 수행한다.
// PD12를 사용하는 이유는 Core407V를 test하기 위해 ECL에서 만든 interface
// board에 PD12를 LED에 연결해 놓았기 때문이다.
//---------------------------------------------------------
void GpioBitTest()
{
	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIODEN;    // Port D에 clock 공급

	GPIOD->MODER &= ~((uint32_t)GPIO_MODER_MODER12);
	GPIOD->MODER |= (0x1U << GPIO_MODER_MODER12_Pos);  // Output의 경우 01 (=0x1)로 설정해야 함

	GPIOD->OSPEEDR &= ~((uint32_t)GPIO_OSPEEDER_OSPEEDR12);
	GPIOD->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED12_Pos);  // Very high speed일 경우 11 (=0x3)로 설정

	while(1)
	{
		GPIOD->BSRR = GPIO_BSRR_BS12;   // PD12 Set
		DWT_us_Delay(500000);  // 이 시간을 변경해 가면서 oscilloscope로 DWT_us_Delay의 정확성을 검사해볼 수 있다.
		GPIOD->BSRR = GPIO_BSRR_BR12;   // PD12 Clear
		DWT_us_Delay(500000);
	}
}

//-----------------------------------------------------------
// PA0, PA1, P2를 입력으로 설정하여 Switch input을 읽어들이고
// PD13, PD14, PD15를 출력으로 설정하여 LED를 On/Off 하도록 한다.
// Core407V를 test하기 위한 interface board를 사용하도록 한다.
// PA0에 연결된 switch가 pressed 되면 PD13에 연결된 LED가 On
// PA1에 연결된 switch가 pressed 되면 PD14에 연결된 LED가 On
// PA2에 연결된 switch가 pressed 되면 PD15에 연결된 LED가 On 되는 방식이다.
// switch를 release하게 되면 해당 LED는 Off 된다.
//-----------------------------------------------------------
void GpioWordTest()
{
	uint32_t data;

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOAEN; // Port A에 clock 공급
	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIODEN; // Port D에 clock 공급

	// 여기서는 일괄로 설정하는 방식을 사용해보자.
	GPIOD->MODER &= 0x03FFFFFF; // PD13, PD14, PD15를 출력으로 설정 (즉 01로 설정)
	GPIOD->MODER |= 0x54000000;
	GPIOD->OSPEEDR &= 0x03FFFFFF; // PD13, PD14, PD15의 speed를 High speed (즉, 11로 설정)
	GPIOD->OSPEEDR |= 0xFC000000;

	GPIOA->MODER &= 0x000000C0; // PA0, PA1, PA2 input으로 설정 (즉 00로 설정). 입력으로 설정할 경우 속도설정은 해당사항이 없다.
	//GPIOA->PUPDR |= 0x00000000; // Nothing (00), Pull (01), Pulldown(10)을 설정할 경우 활성화하고 알맞게 사용한다.


	// Switch 입력을 읽어서 그 정보를 GPIO 출력에 반영한다.
	while(1)
	{
		data = GPIOA->IDR;  // data를 읽어들인다.
		GPIOD->ODR = ((~data) & 0x0000000F)<<13 ;  // invert 및 masking 한 후 출력한다.
	}
}




//------------------------------------------------------
// USART3를 Configure 하기 위해 필요한 절차는 다음과 같다.
// 먼저 TX, RX에 해당하는 GPIO pin을 USART 기능으로 설정해준다.
// 여기서는 USART3_TX로는 PC10 (AF7)을, USART3_RX로는 PC11 (AF7)을
// 사용한다. USART기능은 Alternate function중 AF7에 해당한다.
// PC10과 PC11를 각각 Tx와 Rx로 사용해야 한다. 필요한 기능이 USART이지만 GPIO pin을
// 이용해야 하므로 GPIOC에 clock을 enable 시켜야 한다. USART를 사용한다고
// USART clock만 enable 시키면 동작하지 않는다.
// USART3로 공급되는 clock의 source를 정해줄 수 있다.
// STM32F407에서는 USART3의 clock으로 APB1 clock(즉 PCLK1)을 사용하도록 설정한다.
// RCC_Configuration에서 APB1 clock을 42 MHz로 설정해 놓았던 것을 기억하자.
// USART3의 clock을 enable 시킨다.
// Baudrate, data bit length 등과 같은 설정을 해준다.
// 이를 위해 USART3의 CR1, CR2, CR3, BRR 등의 register를 조작한다.
// 이것들을 조작하기 위해서 항상 CR1의 UE(USART Enable)을 0으로 만든다음
// 필요한 설정을 하고 모든 것이 완료되면 다시 UE를 1로 만들면 USART가 동작한다.
// USART1, USART7는 PCLK2를 clock source로 사용
// 나머지 USART는 PCLK1을 clock source로 사용 (Reference manual page 985)
//------------------------------------------------------
void USART3_Configuration()
{
	// PC10는 USART3_TX로, PC11는 USART_RX로 설정하자.
	// USART로 사용하겠지만 GPIOC의 pin을 사용하므로 일단 GPIOC로 가는 clock을 Enable
	// 시켜야 한다. 이거 안했다가 하루 정도 날렸다.

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOCEN;   // GPIOC clock enable

	//----------------------------------------------------------------------
	// PC10과 PC11를 alternate function AF7 (USART3_TX와 USART3_RX)로 설정하자.
	// 참고로 PC10과 PC11는 GPIOx_AFRH에서 설정하는데, 이것은 ARF[1]에 해당한다.
	//----------------------------------------------------------------------
	GPIOC->MODER &= ~((uint32_t)GPIO_MODER_MODER10);   // PC10을 Alternate function으로 설정.
	GPIOC->MODER |= (0x2U << GPIO_MODER_MODER10_Pos);  // Alternate function의 경우 10 (=0x2)로 설정해야 함
	GPIOC->AFR[1] &= ~((uint32_t)GPIO_AFRH_AFRH2);     // AF7 할당, 즉 USART3_TX로 할당.
	GPIOC->AFR[1] |= (0x7U << GPIO_AFRH_AFSEL10_Pos);  // AF7은 0111 (=0x7)

	GPIOC->MODER &= ~((uint32_t)GPIO_MODER_MODER11);   // PC11을 Alternate function으로 설정.
	GPIOC->MODER |= (0x2U << GPIO_MODER_MODER11_Pos);  // Alternate function의 경우 10 (=0x2)로 설정해야 함
	GPIOC->AFR[1] &= ~((uint32_t)GPIO_AFRH_AFRH3);     // AF7 할당, 즉 USART3_RX로 할당.
	GPIOC->AFR[1] |= (0x7U << GPIO_AFRH_AFSEL11_Pos);  // AF7은 0111 (=0x7)

	// STM32F407에서는 USART3의 clock source로 PCLK1을 사용
	RCC->APB1ENR |= ((uint32_t)RCC_APB1ENR_USART3EN);   // USART3 clock enable

	//------------------------------------------------------------------------------------
	// USART3의 CR1, CR2, CR3, BRR 등을 이용하여 USART3의 parameter를 설정해준다.
	// CR1을 통해서는 start bit, data bits를 설정할 수 있는데 reset value로 1 start bit, 8 data bits로
	// 설정되어 있다. stop bit의 설정은 CR2에서 하도록 되어 있다.
	//------------------------------------------------------------------------------------
	USART3->CR1 = 0x00000000;  // USART3->CR1의 UE를 0으로 설정하면 USART3가 disable 됨. 일단 UE=0으로 만든다.
	USART3->CR1 |= USART_CR1_TE|USART_CR1_RE; // TE, RE (Transmit Enable, Receive Enable)
	USART3->CR1 &= ~USART_CR1_OVER8; // oversampling by 16. 이 bit는 USART가 disabled 되어 있을 때만 write가 가능하다.
	                                // reset value에 의해 1 start bit, 8 data bits 설정이 되어 있다.
	USART3->CR2 = 0x00000000;    // 1 stop bit 로 설정.
	USART3->CR3 = 0x00000000;
	//----------------------------------------------------------
	// USART3->BRR 설정에 주의해야 한다.
	// 만약 115200 bps로 baud rate를 설정하려 한다면
	// 42000000/115200 = 364.5833 이 된다. 이 값에 해당하는 BRR을 만들어주어야
	// 하는데 BRR의 하위 16 bit가 USARTDIV를 구성하는데 이것은 12-bit Mantisa와
	// 4-bit Fractional bit로 구성된다.
	// Baud rate = fck/(8*(2-OVER8)*USARTDIV)
	// 위의 설정에서 OVER8=0으로 설정했으므로 USARTDIV = fck/(8*(2-OVER8)*baudrate)
	// 이므로 USARTDIV = 42000000/(8*(2-0)*115200) = 22.7865 가 된다.
	// Mantisa = 22 (=0x16), Fraction = round(0.7865*16)=12 (=0xC)
	// 가 되므로 USARTDIV = 0x16C로 설정해야 한다.

	//----------------------------------------------------------
	USART3->BRR = 0x16C; //
	USART3->CR1 |= USART_CR1_UE; // UE=1, 즉 USART를 Enable 시킨다.
}


// USART3를 이용하여 char 한개를 전송하는 함수. polling 기반의 전송
void TX3_Char(char data)
{
	while((USART3->SR & USART_SR_TXE)==0); // Transmit data register가 empty 될때까지 기다린다.
	USART3->DR = data; // Transmit data
}

// USART3를 이용하여 string data를 전송하는 함수
void TX3_PutString(char *s)
{
	while (*s != '\0')
	{
		TX3_Char(*s);
		s++;
	}
}

//----------------------------------------------------------
// USART3를 이용하여 PC로 data를 전송하는 함수
// 이것을 test 해 보려면 USB to Serial 3.3V UART convert cable이
// 필요하다. 이 cable의 RXD (노랑색)을 PC10 (USART3_TX)에,
// TXD(주황색)를 PC11 (USART_RX)에 연결하고 GND도 연결해준다.
// Program이 실행되면 Arduino의 Serial monitor나 Serial Plot 등을
// 이용하여 data가 PC로 제대로 전송되어 오는지를 확인할 수 있다.
//----------------------------------------------------------
void USART3_Test()
{
	char str[64];      // character buffer
	float time = 0.0;  // simulation time
	float dt = 0.01;   // sample time

	USART3_Configuration();

	while(1)
	{
		//----------------------------------------------------------------
		// arm_sin_f32는 float 형에 대해 수행하고, sin()은 double 형 data에 대해
		// 값을 계산하므로 그 결과는 다르다. sin()을 이용하여 구한 값이 좀더 정확하다. 하지만
		// 시간이 좀더 걸릴 것이다.
		//----------------------------------------------------------------
		sprintf(str, "%1.10f  %1.10f \n", arm_sin_f32(2*3.14*time), (float)cos(2*3.14*time));
		time+=dt;
		TX3_PutString(str);
		DWT_us_Delay(10000);  // 10 ms
	}
}


//-----------------------------------------------------------
// UsbPutString : USB를 통해 string data를 전송하는 함수
//-----------------------------------------------------------
void UsbPutString(char *s)
{
	uint32_t length;
	length = strlen(s);
	memcpy(UserTxBufferFS, s, length);
	CDC_Transmit_FS(UserTxBufferFS, length);
}


//----------------------------------------------
// USB를 이용하여 string data를 PC로 전송하는 예제
// 이 함수의 동작을 확인하기 위해서는 PC에서 예를 들면
// Arduino IDE에서 Serial monitor 창을 활성화시켜야지만
// 동작이 되는 것을 알 수 있다.
//----------------------------------------------
void PutStringTest()
{
	static int cnt = 0;
	char str[64];

	while(1)
	{
		DWT_ms_Delay(500);
		cnt++;

		sprintf(str, "hello = %d\n",cnt);
		UsbPutString(str);
	}
}

//----------------------------------------------------
// VCP_read : Virtual Comport read 함수
// 이 함수는 internet에 있는 내용을 참조로 하여
// 작성한 함수이다. 관련 internet 주소는 다음과 같다.
// http://visualgdb.com/tutorials/arm/stm32/usb
//----------------------------------------------------
int VCP_read(void *pBuffer, int size)
{
    if (!ReadDone)
        return 0;

    int remaining = RxSize - RxPosition;
    int todo = MIN(remaining, size);
    if (todo <= 0)
        return 0;

    memcpy(pBuffer, UserRxBufferFS + RxPosition, todo);
    RxPosition += todo;
    if (RxPosition >= RxSize)
    {
        ReadDone = 0;
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }

    return todo;
}

//--------------------------------------------------------------------
// EchoTest()
// 사용자가 PC에서 보낸 글자를 다시 loop back으로 되돌려보내 monitor에 display 시킨다.
// 이 함수를 이용하여 Test를 해 보려면 PC 쪽에서는 Arduino IDE의 serial monitor 창을
// open 시킨다음 전송할 data를 쓴 다음 전송 button을 누르면 전송한 data가 다시
// Serial monitor 창에 display 되는 것을 볼 수 있다. 꼭 Arduino의 Serial monitor
// 일 필요는 없지만 Serial monitor가 가장 편하다고 생각된다. 여기서 Serial monitor 창의
// baudrate 설정은 신경쓸 필요가 없다. 이것은 UART 통신이 아니라 USB 통신이므로 Serial
// monitor 창에서 설정한 baudrate는 아무런 영향을 끼치지 못한다.
//--------------------------------------------------------------------
void EchoTest()
{
	int length;
	while(1)  // while-loop로 구성되므로 계속해서 loopback display를 반복한다.
	{
		DWT_ms_Delay(10); // 10 ms 정도의 delay를 삽입해 준다.

		length = VCP_read(&data, 254);  // 254는 256에서 두 글자 뺀 숫자임.

		if(length!=0)
		{
			memcpy(UserTxBufferFS, data, length);
			UserTxBufferFS[length]='\n';
			UserTxBufferFS[length+1]='\r';
			CDC_Transmit_FS(UserTxBufferFS, length+2);
		}
	}
}

//------------------------------------------------------
// Full speed USB를 이용한 data monitoring 예제
// 이 예제를 돌릴때 PC의 Simulink에서는 SerialFromTarget으로 구성된
// Simulink model이 쌍으로 실행되고 있어야 한다.
//------------------------------------------------------
void FsMonitoringTest()
{
	uint32_t SampleTimeCycle;  // sample time에 해당하는 instruction cycle의 수
	uint32_t start_time;
	double time = 0.0;  // simulation time
	double dt = 0.01;   // sample time
	SampleTimeCycle = (uint32_t)(SystemCoreClock*dt);  // system core clock = 168e6
	double my_sin, my_cos;  // PC로 전송하고 싶은 data의 변수 이름

	// 여기서 부터는 realtime monitoring을 위한 설정부분
  	connectData(&toSimulink, SS_DOUBLE, 0, (void *)&my_sin);  // my_sin을 0번 channel에 연결
	connectData(&toSimulink, SS_DOUBLE, 1, (void *)&my_cos);  // my_cos을 1번 channel에 연결
  	toSimulink.ModCntr = 0;            // ModCntr 초기화
	toSimulink.ModNumber = 1;
  	toSimulink.SendHoldFlag = 0;  // SendHoldFlag을 false(=0)로 초기화

  	start_time = DWT->CYCCNT;

	while(1)
	{
		my_sin = sin(2*3.14*time);
		my_cos = cos(2*3.14*time);
		time+=dt;

		SetBit(toSimulink.UpdateDoneFlag, 0);   // Channel 0 data is now updated
		SetBit(toSimulink.UpdateDoneFlag, 1);   // Channel 1 data is now updated
		ProcessToHostBlock();  // High-speed USB를 통해 data를 PC로 전송한다.

		while(!((start_time-DWT->CYCCNT) & 0x80000000));
		start_time += SampleTimeCycle;
	}
}

//---------------------------------------------------------------
// Core407V interface board의 LED에 연결되어 있는
// PD15를 GPIO 출력으로 설정한 뒤 SysTick interrupt를
// 이용하여 일정한 주기에 맞쳐 LED를 On/Off 시켜보자. LED가 너무 빨리 On/Off
// 될 경우에는 육안으로 확인이 안되므로 PD15를 Oscilloscope로 확인해보면
// SysTick interrupt의 정상동작 여부를 확인해 볼 수 있다.
//---------------------------------------------------------------
// SysTick timer는 24-bit timer로써 주로 운영체제나 시스템 프로그램이 사용핟록
// 만들어졌으며 down counting을 한다. Cortex M 계열의 core에 포함된 counter로써
// 제조사와 관계없이 무조건 존재한다. SysTick->LOAD에 24 bit의 정수만을
// load 할 수 있으므로 긴 주기의 interrupt를 만들기 곤란할 수 있다.
// SysTick의 clock source로는 external clock과 processor clock을
// 사용할 수 있는데 external clock을 선택하면 AHB clock인 HCLK의 8분주인
// HCLK/8의 주파수를 갖는 clock이 external clock에 공급되어 사용되고
// processor clock을 사용하면 그냥 HCLK가 사용된다. 여기서는 SysTick의
// clock source를 HCLK/8에 해당하는  외부 clock을 사용하도록 설정한다.
//---------------------------------------------------------------
void SysTickInterruptTest()
{
	double ts = 0.001; // sample time
	double SysTickClockSpeed = 168000000.0/8;   // HCLK/8
	uint32_t load_value = (uint32_t)(SysTickClockSpeed*ts)-1;

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIODEN;      // Port D에 clock 공급

	GPIOD->MODER &= ~((uint32_t)GPIO_MODER_MODER15);   // PB7를 출력으로 설정하자.
	GPIOD->MODER |= (0x1U << GPIO_MODER_MODER15_Pos);  // Output의 경우 01 (=0x1)로 설정해야 함
	GPIOD->OSPEEDR &= ~((uint32_t)GPIO_OSPEEDER_OSPEEDR15);
	GPIOD->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED15_Pos);  // Very high speed일 경우 11 (=0x3)로 설정

	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;  // SysTick의 clock source로 external clock (HCLK/8)을 사용한다.
	//SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;  // SysTick의 clock source로 HCLK를 사용한다.
	SysTick->LOAD = load_value;  // 위에서 계산한  Reload value를 LOAD register에 write 한다.
	SysTick->VAL = 0;     // 임의의 값을 write하면 VAL을 clear 시키면서 동시에 SysTick->CTRL의 COUNTFLAG도 0으로 clear 된다.
	SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk);  // interrupt enable 시키고 counter도 enable 시킨다.

	while(1); // 무한 loop를 돈다. 필요한 일들은 SysTick interrupt에서 이루어진다.
}


//------------------------------------------------------------------------
// Arduino Due의 경우 Interrupt status register를 읽어 들여야지
// flag value가 clear 되었다. 따라서 interrupt service routine에서
// 제일 처음 해야 할 일이 Status register를 읽어들이는 일이었다. 그리고 해당
// flag의 값이 1인 interrupt의 service를 해주는 방식이었다. 하지만
// STM32 계열의 u-controller는 특별히 그럴 필요가 없이 interrupt service routine을
// 마치고 나면 하드웨어에 의해 해당 flag이 clear되는 방식인 것 같다. 따라서
// Arduino Due와 같은 절차가 필요없는 것으로 일단은 생각된다.
//------------------------------------------------------------------------
void SysTick_Handler()
{
	static int flag=0;

	if(flag==0)
	{
		GPIOD->BSRR = 0x00008000;   // PD15 HIGH;
		flag = 1;
	}
	else
	{
		GPIOD->BSRR = 0x80000000;   // PD15 LOW;
		flag = 0;
	}
}


//-----------------------------------------------------------
// sinf과 cosf을 각각 math library를 이용한 경우와 arm_math에서
// 제공하는 함수 arm_sin_f32와 arm_cos_f32를 이용한 경우에 소요되는 시간의
// 차이를 비교하기 위한 함수이다. 정확한 시간 비교를 위해 compile 할 때
// optimization level을 -O, 즉 no optimization으로 설정하여
// 실험하기로 한다. 각각을 5번씩 수행한 후 소요된 시간을 5로 나누어주는 방식으로
// 시간을 측정해 보았다. Oscilloscope로 측정한 결과는 다음과 같다.
//
// arm_sin_f32() : 1회 수행시 약 0.46 [us]
// sinf() : 1회 수행시 2.08 [us].
// 2.08/0.46 = 4.5217 따라서 약 4.5배 정도 더 빠름
//
// 하지만 sin() 함수는 double을 인자로 받고 arm_sin_f32()는 float를
// 인자로 받기 때문에 계산의 공정성을 위하여 sinf() 함수를 사용하였다.
//-----------------------------------------------------------
void SineSpeedTest()
{
	float sin_value;
	float time = 3.3;  // -0.32를 사용하면 짧게 걸리고 다른 양의 값을 쓰면 더 오래 거리는 것을 확인하였음.

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOAEN;    // Port A에 clock 공급

	GPIOA->MODER &= ~((uint32_t)GPIO_MODER_MODER0);   // PA0를 output으로 설정
	GPIOA->MODER |= (0x1U << GPIO_MODER_MODER0_Pos);  // Output의 경우 01 (=0x1)로 설정해야 함

	GPIOA->OSPEEDR &= ~((uint32_t)GPIO_OSPEEDER_OSPEEDR0);
	GPIOA->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED0_Pos);  // Very high speed일 경우 11 (=0x3)로 설정

	while(1)
	{
		GPIOA->BSRR = 0x00000001;   // PA0 HIGH;
		sin_value = arm_sin_f32(time);
		sin_value = arm_sin_f32(time);
		sin_value = arm_sin_f32(time);
		sin_value = arm_sin_f32(time);
		sin_value = arm_sin_f32(time);
		DWT_us_Delay(10000);
		GPIOA->BSRR = 0x00010000;   // PA0 LOW;
		sin_value = sinf(time);
		sin_value = sinf(time);
		sin_value = sinf(time);
		sin_value = sinf(time);
		sin_value = sinf(time);
		DWT_us_Delay(10000);
	}
}


void Error_Handler(void)
{
  while(1){}
}

//---------------------------------------------------------
// CubeMx로 할 때는 SystemClock_Config() 함수에 잔뜩 내용이 있지만
// 우리의 경우는 그 일을 RCC_Configuration()에서 다 하므로 필요없음
// compile 오류가 나는 것을 막기 위해 텅빈 함수로 만들어 버렸음.
//---------------------------------------------------------
void SystemClock_Config(void)
{
}



//-------------------------------------------------------------------
// DAC 기능은 Pin의 alternate function이 아니라, additional function이다.
// ADC, DAC와 같은 additional function으로 Pin을 사용하고 싶을 때에는 analog mode로
// Pin의 mode를 설정해야 한다.
// STM32F407의 경우에는 2개의 DAC channel이 존재한다. 다음과 같이 사용하기로 한다.
// PA4 : DAC_OUT1
// PA5 : DAC_OUT2
//-------------------------------------------------------------------
void DacTest()
{
	double sample_time = 0.001;
	double time=0.0;
	double sin_value, cos_value;

	uint32_t MicrosSampleTime = (uint32_t)(sample_time*1e6);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // GPIOA clock enable
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;     // DAC clock enable

	// PA4, PA5를 DAC 기능을 수행하도록 analog mode로 설정하자.
	GPIOA->MODER &= (~GPIO_MODER_MODER4 & ~GPIO_MODER_MODER5);
	GPIOA->MODER |= ((0x3 << GPIO_MODER_MODER4_Pos)|(0x3 << GPIO_MODER_MODER5_Pos));  // Analog mode는 11 (=0x3)로 설정해야 함.

	//---------------------------------------------------------------
	// DAC를 사용하기 위한 설정을 해보자. 주의할 사항은
	// 각각의 channel 별로 enable을 해주어야 한다는 것이다.
	//---------------------------------------------------------------
	DAC->CR |= DAC_CR_EN1;  // DAC의 channel 1 enable
	DAC->CR |= DAC_CR_EN2;  // DAC의 channel 2 enable

	while(1)
	{
		sin_value = sin(100*time);
		cos_value = cos(100*time);

		DAC->DHR12R1 = (uint16_t)(sin_value*2047.0+2048.0);
		DAC->DHR12R2 = (uint16_t)(cos_value*2047.0+2048.0);

		time += sample_time;

		DWT_us_Delay(MicrosSampleTime);
	}
}


//-----------------------------------------------------------------
// 가장 단순한 형태로 ADC를 사용하는 방법을 학습해보자. 즉 사용자가 software적인
// 방법으로 start-of-conversion 신호를 주고 변환이 완료되었는지를 검사한 후
// 변환 data를 읽어오는 방식이다. STM32F407V의 경우 ADC1, ADC2, ADC3가 있는데
// PA가 주로 input 역할을 하며 들어온 신호는 모두 ADC1, ADC2, ADC3에 입력을
// 전달할 수 있는 구조이다. 간단하게 정리해보면 다음과 같다.
//--------------------------------------------------
// Pin Name    ADC channel
//--------------------------------------------------
//  PA0        ADC1_IN0, ADC2_IN0, ADC3_IN0
//  PA1        ADC1_IN1, ADC2_IN1, ADC3_IN1
//  PA2        ADC1_IN2, ADC2_IN2, ADC3_IN2
//  PA3        ADC1_IN3, ADC2_IN3, ADC3_IN3
//  PA4        ADC1_IN4, ADC2_IN4, DAC_OUT1
//  PA5        ADC1_IN5, ADC2_IN5, DAC_OUT2
//  PA6        ADC1_IN6, ADC2_IN6
//  PA7        ADC1_IN7, ADC2_IN7
//  ...
//--------------------------------------------------
// 이 예제에서는 PA4과 PA5를 사용하여 Peripherhal board의 potentiometer 출력을
// AD 변환한 후 PC로 전송한 후 Arduino의 Serial Plotter를 이용하여 확인하는 것으로
// 한다. PA4 = ADC1_IN4, PA5 = ADC1_IN5
// 확인은 Peripheral board의 Potentiometer 출력인 ANA1, ANA2와 연결하여
// Serial Plotter를 통해 하는 것으로 한다.
// ADC에서는 single mode/continuous mode/discontinuous mode 등의 의미를
// 잘 이해해야 한다.
//-----------------------------------------------------------------
// 이 함수를 test할 때는 USART3를 이용하여 PC로 data를 전송하므로
// USB to Serial 3.3V UART convert cable이 필요하다.
// 이 cable의 RXD (노랑색)을 PC10 (USART3_TX)에,
// TXD(주황색)를 PC11 (USART_RX)에 연결하고 GND도 연결해준다.
// Program이 실행되면 Arduino의 Serial monitor나 Serial Plot 등을
// 이용하여 data가 PC로 제대로 전송되어 오는지를 확인할 수 있다.
//----------------------------------------------------------
void AdcTest()
{
	char str[64];
	uint16_t data1, data2;

	USART3_Configuration();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock enable
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // ADC1 clock enable

	// PA4, PA5를 ADC 기능을 수행하도록 analog mode로 설정하자.
	GPIOA->MODER &= (~GPIO_MODER_MODER4 & ~GPIO_MODER_MODER5);
	GPIOA->MODER |= ((0x3 << GPIO_MODER_MODER4_Pos)|(0x3 << GPIO_MODER_MODER5_Pos));  // Analog mode는 11 (=0x3)로 설정해야 함.

	// ADC clock 설정. 참고로 CCR이라는 register는 ADC Common에 있다.
	ADC->CCR  &= ~ADC_CCR_ADCPRE;
	ADC->CCR |= (0x0 << ADC_CCR_ADCPRE_Pos);  // 00 : PCLK2 divided by 2 => 84 MHz/2 = 42 MHz

	// channel 4, 5의 sampling time을 56 cycle로 설정
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP4;
	ADC1->SMPR2 &= (0x3 << ADC_SMPR2_SMP4_Pos); // 011 : 56 cycles
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP5;
	ADC1->SMPR2 &= (0x3 << ADC_SMPR2_SMP5_Pos); // 011 : 56 cycles

	// Regular channel sequence length를 2로 설정한다. PA4, PA5 2개
	ADC1->SQR1 &= ~ADC_SQR1_L;
	ADC1->SQR1 |= (0x1 << ADC_SQR1_L_Pos);  // 0001 (=0x1): 2 conversions

	ADC1->SQR3 &= ~ADC_SQR3_SQ1;
	ADC1->SQR3 |= (0x4 << ADC_SQR3_SQ1_Pos); // 1st conversion : IN4
	ADC1->SQR3 &= ~ADC_SQR3_SQ2;
	ADC1->SQR3 |= (0x5 << ADC_SQR3_SQ2_Pos); // 2nd conversion : IN5

	// right-align, 12-bit resolution, single mode 등으로 설정한다. 사실 이것은 default 설정이므로 안해줘도 된다.
	ADC1->CR1 |= ADC_CR1_SCAN;  // Scan mode enabled
	ADC1->CR1 &= ~ADC_CR1_RES;
	ADC1->CR1 |= (0x0 << ADC_CR1_RES_Pos);  // 00 (=0x0 ) 12-bit resolution
	ADC1->CR2 &= ~ADC_CR2_ALIGN; // Right alignment
	ADC1->CR2 |= ADC_CR2_EOCS;   // EOC bit is set at the end of each regular conversion.
	ADC1->CR2 &= ~ADC_CR2_CONT;  // Single mode : Start하면 여러개의 channel을 차례로 AD 변환한다.
	ADC1->CR2 |= ADC_CR2_ADON;   // ADC On

	while(1)
	{
		ADC1->CR2 |= ADC_CR2_SWSTART;  // AD Software Start

		//-----------------------------------------------------------
		// 변환 하고자하는 channel의 수가 2개이다. 따라서 ADSTART가 일단
		// 내려지만 SQ1, SQ2에 정의된 channel이 차례로 변환된다.
		// single mode에서는 ADSTART 한번만 주면 알아서 2개의 data가
		// 변환된다. 따라서 사용자는 EOC를 잘 감시하고 있다가 변환된 값을 받아와야 된다.
		// 만약 discontinuous mode로 변환을 하게 되면 ADSTART를 하면 2개의
		// channel중 첫번째 data만 변환되고 두번째 data는 변환되지 않고 기다린다.
		// 사용자는 EOC를 감시한 후 EOC=1 이 되면 변환 data를 읽어오면 된다. 두번째
		// data를 변환하기 위해서는 ADSTART를 또 주어야 한다. 이 점이 바로
		// single mode와 discontinuous mode의 차잇점이다.
		//-----------------------------------------------------------
		while((ADC1->SR & ADC_SR_EOC)==0);
		data1 = ADC1->DR;
		while((ADC1->SR & ADC_SR_EOC)==0);
		data2 = ADC1->DR;
		sprintf(str, "%d %d\n", data1, data2);
		TX3_PutString(str);
		DWT_us_Delay(10000);  // 0.01 second delay
	}
}


//-----------------------------------------------------------------
// Injected channel을 이용하여 ADC를 수행하는 방법을 학습해보자.
// Injected channel을 이용하게 되면 트리거 신호에 의해
// 지정된 4개 channel 까지를 모두 AD 변환할 수 있고 그 결과값이
// 별도로 할당된 resister에 저장되게 된다.
// 이 예제에서는  다음과 같은 4개의 channel을 Injected channel로 설정하기로 한다.
// PC0는 ADC2의  Channel 10으로 할당하고 Regular conversion도 같이 시키도록 한다.
//--------------------------------------------------
// Pin Name    ADC channel  (datasheet page 45~57 참조)
//--------------------------------------------------
//  PA0        ADC1_IN0, ADC2_IN0, ADC3_IN0 (Injected channel)  - ANA1과 연결
//  PA3        ADC1_IN3, ADC2_IN3, ADC3_IN3 (Injected channel)  - ANA2와 연결
//  PA5        ADC1_IN5, ADC2_IN5, DAC_OUT2 (Injected channel)  - JOYX와 연결
//  PA6        ADC1_IN6, ADC2_IN6           (Injected channel)  - JOYY와 연결
//
//  PC0        ADC2_IN10  (Regular channel) - ANA1을 여기에도 연결. PA0에 연결된 것 잠깐 떼서 여기 연결할 것.
//
// ADC1은 channel number는 0번, 3번, 5번, 6번
// ADC2는 channel number 10번을 사용할 것이다.
// 이 함수를 사용하기 위해서는 5개의 analog signal을 공급해야 한다.
// Peripheral board의 ANA1, AN2, JOYX, JOY를 inject channel의 입력으로
// 사용하면 된다. Peripheral board에는 analog signal이 4개 밖에 나오지 않으므로
// Regular channel에 공급할 analog signal은 ANA1을 같이 이용하면 되겠다.
// USART3을 사용하여 data를 전송하므로 FTDI Serial TTL-232 USB cable을 이용
// 해야 한다. PC에서는 Serial Plot을 실행해서 전송되온 data가 제대로 display 되는지
// 확인하도록 한다.
//------------------------------------------------------------------
void AdcInjectedTest()
{
	char str[128];
	uint16_t data[5];

	USART3_Configuration();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock enable (PA를 사용하므로)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // GPIOC clock enable (PC를 사용하므로)

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // ADC1 clock enable
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;   // ADC2 clock enable

	// 여기서는 일괄로 설정하는 방식을 사용해보자.
	// PA0, PA3, PA5, PA6를 ADC 기능을 수행하도록 analog mode로 설정하자.
	// Analog mode는 11 (=0x3)로 설정해야 함.
	GPIOA->MODER &= 0xFFFFC33C; // PA0, PA3, PA5, PA6을 Analog mode로 설정 (즉 11로 설정)
	GPIOA->MODER |= 0x00003CC3;

	GPIOC->MODER &= 0xFFFFFFFC;  // PC0를 Analog mode
	GPIOC->MODER |= 0x00000003;  // 0x3 = 11, 즉 analog mode로 설정

	// ADC clock 설정. 참고로 CCR이라는 register는 ADC Common에 있다.
	ADC->CCR  &= ~ADC_CCR_ADCPRE;
	ADC->CCR |= (0x0 << ADC_CCR_ADCPRE_Pos);  // 00 : PCLK2 divided by 2 => 84 MHz/2 = 27 MHz

	// channel 0, 3, 5, 6의 sampling time을 56 cycle로 설정
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;
	ADC1->SMPR2 |= (0x3 << ADC_SMPR2_SMP0_Pos); // 011 : 56 cycles
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP3;
	ADC1->SMPR2 |= (0x3 << ADC_SMPR2_SMP3_Pos); // 011 : 56 cycles
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP5;
	ADC1->SMPR2 |= (0x3 << ADC_SMPR2_SMP5_Pos); // 011 : 56 cycles
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP6;
	ADC1->SMPR2 |= (0x3 << ADC_SMPR2_SMP6_Pos); // 011 : 56 cycles

	// Total injected channel number = 4, 순서는 channel 0, channel 3, channel 5, channel 6
	//ADC1->JSQR = 0x00331460; // 이렇게 단 한줄로 할 수 있다. 아니면 요기 아래에 나온 것처럼 한다.
	ADC1->JSQR &= ~ADC_JSQR_JL;
	ADC1->JSQR |= (0x3 << ADC_JSQR_JL_Pos);  // 0x3 (=11) 4 conversions.
	ADC1->JSQR &= ~ADC_JSQR_JSQ1;
	ADC1->JSQR |= (0x0 << ADC_JSQR_JSQ1_Pos );  // JSQ1에는 channel 0 할당
	ADC1->JSQR &= ~ADC_JSQR_JSQ2;
	ADC1->JSQR |= (0x3 << ADC_JSQR_JSQ2_Pos );  // JSQ2에는 channel 3 할당
	ADC1->JSQR &= ~ADC_JSQR_JSQ3;
	ADC1->JSQR |= (0x5 << ADC_JSQR_JSQ3_Pos );  // JSQ3에는 channel 5 할당
	ADC1->JSQR &= ~ADC_JSQR_JSQ4;
	ADC1->JSQR |= (0x6 << ADC_JSQR_JSQ4_Pos );  // JSQ4에는 channel 6 할당

	//ADC2의 channel 10의 sampling time을 56 cycle로 설정
	ADC2->SMPR1 &= ~ADC_SMPR1_SMP10;
	ADC2->SMPR1 |= (0x3 << ADC_SMPR1_SMP10_Pos); // 011 : 56 cycles


	// Regular channel sequence length를 1로 설정한다. PC0 (ADC2_IN10) 1개
	ADC2->SQR1 &= ~ADC_SQR1_L;
	ADC2->SQR1 |= (0x0 << ADC_SQR1_L_Pos);  // 0000 (=0x0): 1 conversions

	ADC2->SQR3 &= ~ADC_SQR3_SQ1;
	ADC2->SQR3 |= (0xA << ADC_SQR3_SQ1_Pos); // 1st conversion : IN10

	// right-align, 12-bit resolution, single mode 등으로 설정한다. 사실 이것은 default 설정이므로 안해줘도 된다.
	ADC1->CR1 |= ADC_CR1_SCAN;  // Scan mode enabled
	ADC1->CR1 &= ~ADC_CR1_RES;
	ADC1->CR1 |= (0x0 << ADC_CR1_RES_Pos);  // 00 (=0x0 ) 12-bit resolution
	ADC1->CR2 &= ~ADC_CR2_ALIGN; // Right alignment
	ADC1->CR2 &= ~ADC_CR2_EOCS;  // EOC bit is set at the end of each sequence of regular conversion
	ADC1->CR2 &= ~ADC_CR2_CONT;  // Single conversion mode
	ADC1->CR2 |= ADC_CR2_ADON;   // ADC On

	// right-align, 12-bit resolution, single mode 등으로 설정한다. 사실 이것은 default 설정이므로 안해줘도 된다.
	ADC2->CR1 |= ADC_CR1_SCAN;  // Scan mode enabled
	ADC2->CR1 &= ~ADC_CR1_RES;
	ADC2->CR1 |= (0x0 << ADC_CR1_RES_Pos);  // 00 (=0x0 ) 12-bit resolution
	ADC2->CR2 &= ~ADC_CR2_ALIGN; // Right alignment
	ADC2->CR2 |= ADC_CR2_EOCS;   // EOC bit is set at the end of each regular conversion.
	ADC2->CR2 &= ~ADC_CR2_CONT;  // Single mode : Start하면 여러개의 channel을 차례로 AD 변환한다.
	ADC2->CR2 |= ADC_CR2_ADON;   // ADC On

	while(1)
	{
		ADC1->CR2 |= ADC_CR2_JSWSTART;       // AD Software Start
		while(!(ADC1->SR & ADC_SR_JEOC));    // Wait for JEOC

		ADC2->CR2 |= ADC_CR2_SWSTART;        // AD Software Start
		while(!(ADC2->SR & ADC_SR_EOC));     // Wait for EOC

		data[0] = ADC1->JDR1 & 0x0FFF;
		data[1] = ADC1->JDR2 & 0x0FFF;
		data[2] = ADC1->JDR3 & 0x0FFF;
		data[3] = ADC1->JDR4 & 0x0FFF;
		data[4] = ADC2->DR & 0x0FFF;

		sprintf(str, "%d %d %d %d %d\n", data[0],data[1],data[2],data[3], data[4]);
		TX3_PutString(str);
		DWT_us_Delay(30000);  // 20 ms
	}
}


//----------------------------------------------------------------
// Timer 6는 16-bit up counter로써 timer type은 basic timer이다.
// TIM1, TIM8은 Advanced timer로써 3상 PWM을 생성할 수 있고 encoder counting
// 기능이 있다.TIM2,TIM3,TIM4,TIM5는 General purpose timer로써
// encoder counting 기능이 있어 요긴하다.
// 따라서 timer interrupt를 구현할 일이 있으면 유용성면에서 좀 떨어지는
// TIM6를 이용하여 구현하는 것이 resource를 알뜰하게 쓰는 방법이 될 것이다.
// TIM6의 경우 16-bit timer이기 때문에 prescaler value와 ARR value를
// 잘 정해야지 원하는 interrupt 주기를 만들 수 있다. 이것을 잘못 선택하면 너무
// 짧거나 혹은 긴 interrupt 주기는 만들 수 없음을 유의하자.
//----------------------------------------------------------------
void Timer6_InterruptTest()
{
	double sample_time = 0.001;

	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOAEN;    // Port A에 clock 공급
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // TIM6 clock enable

	// PA3를 GPIO 출력, Very high speed로 설정하자.
	GPIOA->MODER &= ~((uint32_t)GPIO_MODER_MODER3);
	GPIOA->MODER |= (0x1U << GPIO_MODER_MODER3_Pos);  // Output의 경우 01 (=0x1)로 설정해야 함
	GPIOA->OSPEEDR &= ~((uint32_t)GPIO_OSPEEDR_OSPEED3);
	GPIOA->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED3_Pos);  // Very high speed일 경우 11 (=0x3)로 설정

	//--------------------------------------------------------------------------------------
	// TIM6에 대한 설정을 수행하자. 설정해주어야 할 것은 Prescaler 값, ARR(Auto-reload register)의 값
	// 참고로 TIM6는 APB1에 속하는데, Timer의 경우 APB1 peripheral과 달리 APB1의 prescaler 값이
	// 1보다 큰 경우 PCLK의 x2에 해당하는 clock을 입력받는다. RCC_Configuration()에서
	// APB1 Prescaler=4으로 설정했기 때문에 HCLK divided by 4가 되어 42MHz이다. 그런데
	// Timer의 경우 prescaler value=4는 1보다 큰 값이므로 PCLK의 x2가 되어 42MHz x 2 = 84MHz의 clock을
	// 사용하게 되는 것에 유의하자.
	//---------------------------------------------------------------------------------------
	TIM6->PSC = 839;   //  108 MHz/(839+1) = 100 KHz
	TIM6->ARR = (uint16_t)(100000*sample_time)-1;  // ARR의 16-bit register 이다. 100000 -> 100 KHz
	TIM6->CNT = 0;   // counter vale = 0 으로 초기화
	TIM6->DIER |= TIM_DIER_UIE; // Update interrupt enable.
	TIM6->CR1 |= TIM_CR1_ARPE;   // ARPE=1: TIMx_ARR register is buffered.
	TIM6->CR1 |= TIM_CR1_URS;   // URS=1: Only counter overflow/underflow generates an update interrupt or DMA request if enabled.

	//-----------------------------------------------------------------------------
	// NVIC 설정을 수행하자. TIM6_DAC1_IRQn=54 이다. 이것을 확인하려면 STM32F407의 reference manual
	// 의 page374를 보면 된다. 0~31까지가 ISER[0]을 이용하여 enable 시키고
	// 32~63 까지는 ISER[1], 그리고 64~81까지는 ISER[2]를 이용하여 enable 시킨다. STM32F303K8의
	// 경우 마지막 interrupt number가 81 이다. 54-32 = 22 이므로 54번 Interrupt의 경우
	// ISER[1]의 22번 bit를 1로 만들면 된다.
	// 0000_0000__0100_0000_0000_0000_0000_0000 = 0x00400000
	// ISER을 이용하는 방법은 정말로 원초적인 방법이고 CMSIS의 core_cm4.h에 이것을 위한 inline function
	// NVIC_EnableIRQ(IRQn_TypeIRQn)을 사용하는 것이 좀더 편리한 방법이라 할 수 있다.
	//-----------------------------------------------------------------------------
	NVIC->ISER[1] |= 0x00400000;  // Enable (54) TIM6 interrupt
	// NVIC_EnableIRQ(TIM6_DAC_IRQn); // 위의 code 대신 이렇게 처리할 수도 있다. TIM6_DAC_IRQn는 stm32f407xx.h에 정의

	TIM6->CR1 |= TIM_CR1_CEN;     // TIM6 enable.

	while(1);
}

//----------------------------------------------------------------
// Timer6 interrupt test의 설정에 의해 enable 된 interrupt handler.
//----------------------------------------------------------------
void TIM6_DAC_IRQHandler()  // IRQ Handler의 이름은 startup_stm32f407xx.s 에서 찾아볼 수 있다.
{
	static int flag=0;

	//-----------------------------------------------------------------------------
	// Clear pending bit of TIM6 interrupt. Status register는 0을 write 해서 clear 시킨다.
	// 이것을 안해주면 pending interrupt가 계속 있는 셈이므로 interrupt request가 계속 발생하게 되는
	// 난감한 현상이 발생한다. 따라서 pending bit를 반드시 clear 시켜주도록 하자.
	//-----------------------------------------------------------------------------
	TIM6->SR = 0x0000;

	if(flag==0)
	{
		GPIOA->BSRR = 0x00000008;   // PA3 HIGH;
		flag = 1;
	}
	else
	{
		GPIOA->BSRR = 0x00080000;   // PA3 LOW;
		flag = 0;
	}
}

//-----------------------------------------------------------------------
// TIM1은 Advanced timer 이고 16-bit counter 이다. Encoder counter를 처리하기
// 위한 Pin 은 다음고 같다.
//
// TIM1_CH1 : PE9  (AF1) --> A상 연결  [주] AF1 = Alternate Function 1
// TIM1_CH2 : PE11 (AF1) --> B상 연결
//
// [주] PA8, PA9도 TIM1_CH1, TIM1_CH2이지만, 이렇게 할 경우 PA9을 사용하게 되므로
//     Full speed USB를 사용할 수 없게 되는 문제가 발생한다. 이런 이유로 Pin을
//     PE9과 PE11을 할당하였다.
//
// Timer1은 APB2에 속한 timer 이다. Timer1의 clock은 APB2 Prescaler가 1이 아닐 경우
// PCLK2의 x2에 해당하는 clock이 공급된다.
//
// Pololu motor의 encoder를 연결하여 Test 해보자. 결선은 다음과 같다
//------------------------------
// STM32F407      Pololu motor
//------------------------------
// PE9             A (Yellow)
// PE11            B (White)
// 3.3V            3.3V (Blue)
// GND             Encoder GND (Green)
//----------------------------------------
// 실행결과는 Serial Plot을 이용하여 확인해볼 수 있다. 또는 Arudino의 Serial monitor를
// 이용하여 text 형태로 확인해 볼 수도 있다
//-----------------------------------------------------------------------
void Timer1_EncoderTest()
{
	char str[64];
	int16_t cnt;  // encoder 값을 signed integer로 받아오도록 하자. 그래야지 양/음의 방향이 생긴다.

	USART3_Configuration();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; // GPIOE clock enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // TIM1 clock enable

	// PE9, PE11을 Alternate function으로 설정한다.
	GPIOE->MODER &= (~GPIO_MODER_MODER9 & ~GPIO_MODER_MODER11);
	GPIOE->MODER |= ((0x2 <<GPIO_MODER_MODER9_Pos)|(0x2<<GPIO_MODER_MODER11_Pos)); // 10 (=0x2) : alternate function

	//----------------------------------------------------------
	// PE9, PE11은 AFR[1]에서 설정해야 한다. AFR[1]에서는 1번과 3번에 해당한다.
	// TIM1_CH1과 TIM1_CH2는 AF1에 해당한다. (datasheet page 58 참조)
	//----------------------------------------------------------
	GPIOE->AFR[1] &= (~GPIO_AFRH_AFSEL9 & ~GPIO_AFRH_AFSEL11); // AF1 할당, AF1는 0001 (=0x1)
	GPIOE->AFR[1] |= ((0x1 << GPIO_AFRH_AFSEL9_Pos)|(0x1 << GPIO_AFRH_AFSEL11_Pos));  //  AF1 할당, AF1는 0001 (=0x1)

	//------------------------------------------------------------------------
	// TIM1을 Encoder counter 기능을 수행하도록 설정해보자.
	// 먼저 Encoder counter는 slave mode로 동작할 때 얻는 기능이다. 따라서 SMCR을 조작해야 한다.
	//------------------------------------------------------------------------
	TIM1->SMCR &= ~TIM_SMCR_SMS;  // SMS는 모두 3-bit로 구성
	TIM1->SMCR |= (0x3 << TIM_SMCR_SMS_Pos); // 011 (=0x3) : Encoder mode 3

	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM1->CCMR1 |= (0x1 << TIM_CCMR1_CC1S_Pos); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1
	TIM1->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM1->CCMR1 |= (0x1 << TIM_CCMR1_CC2S_Pos); // 01 (=0x1) : CC2 channel is configured as input, IC2 is mapped on TI2

	TIM1->CCMR1 &= ~TIM_CCMR1_IC1F;
	TIM1->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4
	TIM1->CCMR1 &= ~TIM_CCMR1_IC2F;
	TIM1->CCMR1 |= (0x2 << TIM_CCMR1_IC2F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4

	TIM1->CCER &= ~TIM_CCER_CC1P;  // CC1P=0, non-inverted/rising edge. The circuit is sensitive to TIxFP1 rising edge
	TIM1->CCER &= ~TIM_CCER_CC2P;  // CC2P=0, non-inverted/rising edge.


	TIM1->CNT = 0;  // Counter를 0으로 clear

	TIM1->CR1 |= TIM_CR1_CEN; // TIM1을 enable 시킨다.

	while(1)
	{
		cnt = TIM1->CNT;
		//sprintf(str, "%d\n", TIM1->CNT);
		sprintf(str, "%d\n", cnt);
		TX3_PutString(str);
		DWT_us_Delay(10000);  // 0.01 second delay
	}
}


//-----------------------------------------------------------------------
// TIM2는 General purpose timer이고 32 bit counter 이다. TIM2는 APB1에 포함된
// Peripheral 이다. Encoder counter를 처리하기 위한 Pin 은 다음과 같이 설정하기로 한다.
//
// TIM2_CH1 : PA0 --> A상 연결
// TIM2_CH2 : PA1 --> B상 연결
//
// Timer2는 APB1의 clock인 PCLK1을 사용하는데 만약에 APB1의 prescaler가 1보다 크면
// PCLK1 x 2에 해당하는 clock을 받게 된다. RCC_Configuration에서 APB1 prescaler를 4로
// 설정했으므로 PCLK1은 42 MHz가 되고 그것의 2배인 84 MHz의 clock이 Timer2에 사용되게 된다.
//
// Pololu motor의 encoder를 연결하여 Test 해보자. 결선은 다음과 같다
//------------------------------
// STM32F407      Pololu motor
//------------------------------
// PE9             A (Yellow)
// PE11            B (White)
// 3.3V            3.3V (Blue)
// GND             Encoder GND (Green)
//----------------------------------------
// 실행결과는 Serial Plot을 이용하여 확인해볼 수 있다. 또는 Arudino의 Serial monitor를
// 이용하여 text 형태로 확인해 볼 수도 있다
//-----------------------------------------------------------------------
void Timer2_EncoderTest()
{
	char str[64];
	int32_t cnt;  // encoder 값을 32-bit signed integer로 받아오도록 하자. 그래야지 양/음의 방향이 생긴다

	USART3_Configuration();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // TIM2 clock enable

	// PA0, PA1를 Alternate function으로 설정한다.
	GPIOA->MODER &= (~GPIO_MODER_MODER0 & ~GPIO_MODER_MODER1);
	GPIOA->MODER |= ((0x2 <<GPIO_MODER_MODER0_Pos)|(0x2<<GPIO_MODER_MODER1_Pos)); // 10 (=0x2) : alternate function

	//----------------------------------------------------------
	// PA0, PA1는 AFR[0]에서 설정해야 한다. AFR[0]에서는 0번과 1번에 해당한다.
	// TIM2_CH1과 TIM2_CH2는 AF1에 해당한다.
	//----------------------------------------------------------
	GPIOA->AFR[0] &= (~GPIO_AFRL_AFSEL0 & ~GPIO_AFRL_AFSEL1); // AF1 할당, AF1는 0001 (=0x1)
	GPIOA->AFR[0] |= ((0x1 << GPIO_AFRL_AFSEL0_Pos)|(0x1 << GPIO_AFRL_AFSEL1_Pos));  //  AF1 할당, AF1는 0001 (=0x1)

	//------------------------------------------------------------------------
	// TIM2을 Encoder counter 기능을 수행하도록 설정해보자.
	// 먼저 Encoder counter는 slave mode로 동작할 때 얻는 기능이다. 따라서 SMCR을 조작해야 한다.
	//------------------------------------------------------------------------
	TIM2->SMCR &= ~TIM_SMCR_SMS;  // SMS는 모두 3-bit로 구성
	TIM2->SMCR |= (0x3 << TIM_SMCR_SMS_Pos); // 011 (=0x3) : Encoder mode 3

	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM2->CCMR1 |= (0x1 << TIM_CCMR1_CC1S_Pos); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1
	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM2->CCMR1 |= (0x1 << TIM_CCMR1_CC2S_Pos); // 01 (=0x1) : CC2 channel is configured as input, IC2 is mapped on TI2

	TIM2->CCMR1 &= ~TIM_CCMR1_IC1F;
	TIM2->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4
	TIM2->CCMR1 &= ~TIM_CCMR1_IC2F;
	TIM2->CCMR1 |= (0x2 << TIM_CCMR1_IC2F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4

	TIM2->CCER &= ~TIM_CCER_CC1P;  // CC1P=0, non-inverted/rising edge. The circuit is sensitive to TIxFP1 rising edge
	TIM2->CCER &= ~TIM_CCER_CC2P;  // CC2P=0, non-inverted/rising edge.

	TIM2->CNT = 0;  // Counter를 0으로 clear

	TIM2->CR1 |= TIM_CR1_CEN; // TIM1을 enable 시킨다.

	while(1)
	{
		cnt = TIM2->CNT;
		sprintf(str, "%d\n", cnt);
		TX3_PutString(str);
		DWT_us_Delay(10000);  // 0.01 second delay
	}
}


//-----------------------------------------------------------------------
// TIM3는 General purpose timer이고 16 bit counter 이다. Encoder counter를 처리하기
// 위한 Pin 은 다음고 같다.
//
// TIM3_CH1 : PA6 --> A상 연결 (Datasheet의  page 58 참조)
// TIM3_CH2 : PA7 --> B상 연결
//
// Timer3는 APB1의 clock인 PCLK1을 사용하는데 만약에 APB1의 prescaler가 1보다 크면
// PCLK1 x 2에 해당하는 clock을 받게 된다. RCC_Configuration에서 APB1 prescaler를 4로
// 설정했으므로 PCLK1은 42 MHz가 되고 그것의 2배인 84 MHz의 clock이 Timer3에 사용되게 된다.
//
// Pololu motor의 encoder를 연결하여 Test 해보자. 결선은 다음과 같다
//------------------------------
// STM32F407      Pololu motor
//------------------------------
// PE9             A (Yellow)
// PE11            B (White)
// 3.3V            3.3V (Blue)
// GND             Encoder GND (Green)
//----------------------------------------
// 실행결과는 Serial Plot을 이용하여 확인해볼 수 있다. 또는 Arudino의 Serial monitor를
// 이용하여 text 형태로 확인해 볼 수도 있다
//-----------------------------------------------------------------------
void Timer3_EncoderTest()
{
	char str[64];
	int16_t cnt;  // encoder 값을 16-bit signed integer로 받아오도록 하자. 그래야지 양/음의 방향이 생긴다.
	USART3_Configuration();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;   // TIM3 clock enable

	// PA6, PA7를 Alternate function으로 설정한다.
	GPIOA->MODER &= (~GPIO_MODER_MODER6 & ~GPIO_MODER_MODER7);
	GPIOA->MODER |= ((0x2 <<GPIO_MODER_MODER6_Pos)|(0x2<<GPIO_MODER_MODER7_Pos)); // 10 (=0x2) : alternate function

	//----------------------------------------------------------
	// PA6, PA7는 AFR[0]에서 설정해야 한다. AFR[0]에서는 6번과 7번에 해당한다.
	// TIM3_CH1과 TIM3_CH2는 AF2에 해당한다.
	//----------------------------------------------------------
	GPIOA->AFR[0] &= (~GPIO_AFRL_AFSEL6 & ~GPIO_AFRL_AFSEL7); // AF2 할당, AF2는 0010 (=0x2)
	GPIOA->AFR[0] |= ((0x2 << GPIO_AFRL_AFSEL6_Pos)|(0x2 << GPIO_AFRL_AFSEL7_Pos));  //  AF2 할당, AF2는 0010 (=0x2)

	//------------------------------------------------------------------------
	// TIM3을 Encoder counter 기능을 수행하도록 설정해보자.
	// 먼저 Encoder counter는 slave mode로 동작할 때 얻는 기능이다. 따라서 SMCR을 조작해야 한다.
	//------------------------------------------------------------------------
	TIM3->SMCR &= ~TIM_SMCR_SMS;  // SMS는 모두 3-bit로
	TIM3->SMCR |= (0x3 << TIM_SMCR_SMS_Pos); // 011 (=0x3) : Encoder mode 3

	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM3->CCMR1 |= (0x1 << TIM_CCMR1_CC1S_Pos); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1
	TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM3->CCMR1 |= (0x1 << TIM_CCMR1_CC2S_Pos); // 01 (=0x1) : CC2 channel is configured as input, IC2 is mapped on TI2

	TIM3->CCMR1 &= ~TIM_CCMR1_IC1F;
	TIM3->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4
	TIM3->CCMR1 &= ~TIM_CCMR1_IC2F;
	TIM3->CCMR1 |= (0x2 << TIM_CCMR1_IC2F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4

	TIM3->CCER &= ~TIM_CCER_CC1P;  // CC1P=0, non-inverted/rising edge. The circuit is sensitive to TIxFP1 rising edge
	TIM3->CCER &= ~TIM_CCER_CC2P;  // CC2P=0, non-inverted/rising edge.

	TIM3->CNT = 0;  // Counter를 0으로 clear

	TIM3->CR1 |= TIM_CR1_CEN; // TIM1을 enable 시킨다.

	while(1)
	{
		cnt = TIM3->CNT;
		sprintf(str, "%d\n", cnt);
		TX3_PutString(str);
		DWT_us_Delay(10000);  // 0.01 second delay
	}
}

//----------------------------------------------------------
// TIM1은 Advanced timer로써 3상 PWM을 생성할 수 있다.
// Complementary PWM을 생성해보는 방법을 학습해보자. 더불어 dead-time도
// 추가하는 방법을 살펴보기로 한다. STM32F407에서 3상 PWM을 생성하기 위한 Pin은
// 다음과 같이 정하기로 한다.
// PE8  --> TIM1_CH1N (AF1)
// PE9  --> TIM1_CH1  (AF1)
// PE10 --> TIM1_CH2N (AF1)
// PE11 --> TIM1_CH2  (AF1)
// PE12 --> TIM1_CH3N (AF1)
// PE13 --> TIM1_CH3  (AF1)
//
// 또다른 조합으로 다음과 같은 Pin 사용도 생각해 볼 수 있다. 하지만 아래의 조합은
// PA9을 사용해야 하는데 이렇게 되면  Full speed USB를 사용할 수 없게 된다.
// 위에서 제시한 첫번째 Pin 조합을 사용하는 것으로 한다.
//
// PA8  --> TIM1_CH1  (AF1)  => PA8은 MCO1으로 사용할 수 있으므로 안쓰는 게 좋다.
// PA9  --> TIM1_CH2  (AF1)  => PA9를 사용하면 USB를 사용할 수 없게 된다.
// PA10 --> TIM1_CH3  (AF1)
// PA7  --> TIM1_CH1N (AF1)
// PB0  --> TIM1_CH2N (AF1)
// PB1  --> TIM1_CH3N (AF1)
//
// Timer1은 APB2에 속한 peripheral이다. APB2의 prescaler를 1이 아닌 2로 설정했으므로
// APB2 timer에 공급되는 clock은 PCLK2 (=84MHz)의 x2가 168 MHz의 clock이 공급된다.
// 여기서는 Center-aligned 방식의 PWM을 사용하고 ARR의 값을 4200 으로 설정하여
// PWM의 주파수는 168 MHz / (4200*2) = 20 KHz 가 되는 것으로 설정한다.
//----------------------------------------------------------
void ThreePhasePwmTest()
{
	uint16_t ccr1_value, ccr2_value;
	double sample_time = 0.001;
	double time = 0.0;
	uint32_t MicrosSampleTime = (uint32_t)(sample_time*1e6);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;  // GPIOE clock enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   // TIM1 clock enable

	// PE8, PE9, PE10, PE11, PE12, PE13을 alternate function으로 설정하자.
	GPIOE->MODER &= (~GPIO_MODER_MODER8 & ~GPIO_MODER_MODER9 & ~GPIO_MODER_MODER10);
	GPIOE->MODER &= (~GPIO_MODER_MODER11 & ~GPIO_MODER_MODER12 & ~GPIO_MODER_MODER13);
	GPIOE->MODER |= ((0x2 <<GPIO_MODER_MODER8_Pos)|(0x2<<GPIO_MODER_MODER9_Pos));    // 10 (=0x2) : alternate function
	GPIOE->MODER |= ((0x2 <<GPIO_MODER_MODER10_Pos)|(0x2<<GPIO_MODER_MODER11_Pos));  // 10 (=0x2) : alternate function
	GPIOE->MODER |= ((0x2 <<GPIO_MODER_MODER12_Pos)|(0x2 <<GPIO_MODER_MODER13_Pos)); // 10 (=0x2) : alternate function

	// GPIO 속도를 High speed로 설정하자. 사실 이 부분이 필요한지 안한지는 잘 모르겠다. comment 처리해도 PWM은 잘 생성되더라...
	GPIOE->OSPEEDR &= (~GPIO_OSPEEDR_OSPEED8 & ~GPIO_OSPEEDR_OSPEED9 & ~GPIO_OSPEEDR_OSPEED10);
	GPIOE->OSPEEDR &= (~GPIO_OSPEEDR_OSPEED11 & ~GPIO_OSPEEDR_OSPEED12 & ~GPIO_OSPEEDR_OSPEED13);
	GPIOE->OSPEEDR |= ((0x3U << GPIO_OSPEEDR_OSPEED8_Pos)|(0x3U << GPIO_OSPEEDR_OSPEED9_Pos));  // High speed일 경우 11 (=0x3)로 설정
	GPIOE->OSPEEDR |= ((0x3U << GPIO_OSPEEDR_OSPEED10_Pos)|(0x3U << GPIO_OSPEEDR_OSPEED11_Pos));  // High speed일 경우 11 (=0x3)로 설정
	GPIOE->OSPEEDR |= ((0x3U << GPIO_OSPEEDR_OSPEED12_Pos)|(0x3U << GPIO_OSPEEDR_OSPEED13_Pos));  // High speed일 경우 11 (=0x3)로 설정

	//----------------------------------------------------------
	// PE8, PE9, PE10, PE11, PE12, PE13은 AFR[1]에서 설정해야 한다.
	// AFR[1]에서는 0번, 1번, 2번, 3번, 4, 5번에 해당한다.
	// PWM과 관련해서는 모두 AF1에 해당한다.
	//----------------------------------------------------------
	GPIOE->AFR[1] &= (~GPIO_AFRH_AFSEL8 & ~GPIO_AFRH_AFSEL9 & ~GPIO_AFRH_AFSEL10);
	GPIOE->AFR[1] &= (~GPIO_AFRH_AFSEL11 & ~GPIO_AFRH_AFSEL12 & ~GPIO_AFRH_AFSEL13);
	GPIOE->AFR[1] |= ((0x1 << GPIO_AFRH_AFSEL8_Pos)|(0x1 << GPIO_AFRH_AFSEL9_Pos)|(0x1 << GPIO_AFRH_AFSEL10_Pos));  //  AF1 할당, AF1는 0001 (=0x1)
	GPIOE->AFR[1] |= ((0x1 << GPIO_AFRH_AFSEL11_Pos)|(0x1 << GPIO_AFRH_AFSEL12_Pos)|(0x1 << GPIO_AFRH_AFSEL13_Pos));  //  AF1 할당, AF1는 0001 (=0x1)

	// 여기서부터 TIM1을 설정해보자.
	TIM1->PSC = 0x0;   // Prescale 설정 PSC=0, The counter clock frequency (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1). 즉 no division에 해당함.
	TIM1->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered
	TIM1->CR1 &= ~TIM_CR1_CMS;
	TIM1->CR1 |= (0x3 << TIM_CR1_CMS_Pos); // 11 (=0x3) : Center-aligned mode 3. Output compare interrupt flag이 counter가 up, down일 때 모두 set 되는 방식
	TIM1->ARR =4200;   // PWM frequency는 168 MHz/(2*4200) = 20 KHz

	// Channel 1, 2, 3를 PWM mode로 설정한다.
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);    // Capture/compare output 1, 2, 3를 enable
	TIM1->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE); // Capture/compare complementary output 1, 2, 3을 enable
	TIM1->CCER &= (~TIM_CCER_CC1P & ~TIM_CCER_CC2P & ~TIM_CCER_CC3P); // CC1P=0, CC2P=0, CC3P=0 : Capture/compare output 1,2,3을 모두 active high로 설정
	TIM1->CCER &= (~TIM_CCER_CC1NP & ~TIM_CCER_CC2NP & ~TIM_CCER_CC3NP); // CC1NP=0, CC2NP=0, CC3NP=0, Capture/compare complementary output 1, 2, 3을 모두 active high로 설정

	TIM1->CCMR1 &= (~TIM_CCMR1_CC1S & ~TIM_CCMR1_CC2S); // CC1S=0, CC2S=0, CC1 channel, CC 2 channel are configured as output
	TIM1->CCMR2 &= ~TIM_CCMR2_CC3S; // CC3S=0 : CC3 channel is configured as output

	TIM1->CCMR1 &= (~TIM_CCMR1_OC1M & ~TIM_CCMR1_OC2M);
	TIM1->CCMR1 |= ((0x6 << TIM_CCMR1_OC1M_Pos)|(0x6 << TIM_CCMR1_OC2M_Pos));  // OC1M=0110 (=0x6) PWM mode 1, OC2M=0110
	TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM1->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos);  // OC3M=0110 (=0x6) : PWM mode 1

	//--------------------------------------------------------------------------------
	// Idle state 값을 설정하자. idle state라는 것은 MOE (Master Output Enable)이 0 이 되었을 때
	// PWM pin의 출력값을 뜻한다. 이 값들은 MOE=0 이 되고 dead-time 만큼 시간이 지난후에 출력된다.
	//--------------------------------------------------------------------------------
	TIM1->CR2 |= (TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3);       // OC1, OC2, OC3의 idle state는 High로 설정
	TIM1->CR2 &= (~TIM_CR2_OIS1N & ~TIM_CR2_OIS2N & ~TIM_CR2_OIS3N); // OC1N, OC2N, OC3N의 idle state는 LOW로 설정

	TIM1->CR1 &= ~TIM_CR1_CKD;
	TIM1->CR1 |= (0x0 << TIM_CR1_CKD_Pos); // 00: t_DTS = tck_INT, 01:t_DTS=2*tck_INT, 10:t_DTS=4*tck_INT

	TIM1->BDTR &= ~TIM_BDTR_OSSI; // OSSI=0 : MOE=0일 경우 OC/OCN outputs are disabled-> Hi-Z state가 됨.
	                              // 만약 OSSI=1 이면 MOE=0 일 때 deadtime 만큼 지난 후 idle level로 지정한 값으로 forced 된다.
	TIM1->BDTR &= ~TIM_BDTR_DTG;
	TIM1->BDTR |= (10 << TIM_BDTR_DTG_Pos);  // dead-time=10*dts=59.52 [ns]

	TIM1->CNT = 0;   // Counter를 0으로 clear

	TIM1->CCR1 = 2100;  // 2100은 4200의 절반에 해당하는 값
	TIM1->CCR2 = 2100;
	TIM1->CCR3 = 2199;

	TIM1->CR1 |= TIM_CR1_CEN;   // TIM1 enable.
	TIM1->BDTR |= TIM_BDTR_MOE; // MOE=1 : Main output enable

	while(1)
	{
		ccr1_value = (uint16_t)(sin(2*time)*2100+2100);
		ccr2_value = (uint16_t)(cos(2*time)*2100+2100);

		TIM1->CCR1 = ccr1_value;
		TIM1->CCR2 = ccr2_value;
		TIM1->CCR3 = 2100;

		time += sample_time;   // time 갱신
		DWT_us_Delay(MicrosSampleTime);  // sample time 만큼 기다린다.
	}
}

//-------------------------------------------------------------------------
// External interrupt를 이용하여 Software encoder counting을 수행하는 예제를 학습해보자.
// PA0와 PA1에 external interrupt를 설정해보자. 4체배 방식으로 encoder reading을
// 수행해야 하므로 interrupt는 rising 및 falling edge에 걸릴 수 있도록 하자.
//
// 결선 :  Encoder의 A상과 B상을 PA0와 PA1에 연결한다.
//---------------------------------------
// STM32F407      Pololu motor
//---------------------------------------
// PA0             A (Yellow)
// PA1             B (White)
// 3.3V            3.3V (Blue)
// GND             Encoder GND (Green)
//---------------------------------------
// External interrupt가 잘 설정되어 동작하는지 확인하기 위해 PA2, PA3를 GPIO 출력으로
// 설정하여 interrupt가 걸릴때마다 PA2, PA3의 출력을 toggle 시키도록 했다.
// 여기서 Encoder를 Software적인 방법으로 처리하기 위해 사용한 방법은 Prof. Lee의
// PPT 강의노트를 참조하여 구현하였다.
//-------------------------------------------------------------------------
uint32_t indx;      // changeMtx의 index value
int32_t count = 0;  // Encoder pulse의 counting value
int32_t changeMtx[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Encoder change matrix

void ExtInterruptTest()
{
	char str[64];
	USART3_Configuration();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // GPIOA clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   // SYSCFG clock enable.

	GPIOA->MODER &= (~GPIO_MODER_MODER0 & ~GPIO_MODER_MODER1); // PA0, PA1을 input으로 설정한다.

	GPIOA->MODER &= (~GPIO_MODER_MODER2 & ~GPIO_MODER_MODER3); // PA2, PA3를 출력으로 설정한다.
	GPIOA->MODER |= ((0x1 << GPIO_MODER_MODER2_Pos) | (0x1 << GPIO_MODER_MODER3_Pos)); // 01 (=0x1): output
	GPIOA->OSPEEDR &= (~GPIO_OSPEEDR_OSPEED2 & ~GPIO_OSPEEDR_OSPEED3); // PA2, PA3를 High speed로 설정한다.
	GPIOA->OSPEEDR |= ((0x3U <<GPIO_OSPEEDR_OSPEED2_Pos)|(0x3U <<GPIO_OSPEEDR_OSPEED3_Pos));  // High speed일 경우 11 (=0x3)로 설정

	// PA0와 PA1을 external interrupt의 source로 설정한다.
	SYSCFG->EXTICR[0] &= (~SYSCFG_EXTICR1_EXTI0 & ~SYSCFG_EXTICR1_EXTI1); // EXTIx[3:0]=0000 : PA[x]가 source input임.

	EXTI->IMR |= (EXTI_IMR_MR0 | EXTI_IMR_MR1); // Enable EXTI0, EXTI1 interrupt
	/// rising edge, falling edge 둘다에 interrupt가 걸리도록 설정하자.
	EXTI->RTSR |= (EXTI_RTSR_TR0 | EXTI_RTSR_TR1); // TRx=1: Rising trigger enabled (for Event and Interrupt) for input line
	EXTI->FTSR |= (EXTI_FTSR_TR0 | EXTI_FTSR_TR1); // TRx=1: Falling trigger enabled (for Event and Interrupt) for input line.

	/// External interrupt를 enable 시키기 앞서서 indx value의 초기값을 읽어들이자.
	indx = (GPIOA->IDR & 0x00000003); // PA0, PA1을 읽어들인다.
	indx <<= 2;  // 2-bit shift left 시켜서 방금 읽은 2-bit의 값을 previous value로 만들어 놓는다. [주] 2번, 3번 bit가 previous value 이다.

	NVIC->ISER[0] = 0x000000C0;  // EXTI0, EXTI1 interrupt enable. [주] EXTI0_IRQn = 6, EXTI1_IRQn = 7
	// NVIC_EnableIRQ(EXTI0_IRQn); // 위의 code 대신 이렇게 처리할 수도 있다. EXTI0_IRQn, EXTI1_IRQn는 stm32f407xx.h에 정의
	// NVIC_EnableIRQ(EXTI1_IRQn);
	while(1)
	{
		sprintf(str, "%d\n", count);
		TX3_PutString(str);
		DWT_us_Delay(10000);  // 0.01 second delay
	}
}

void EXTI0_IRQHandler() // EXTI0_IRQHandler의 IRQn은 6번
{
	static int flag = 0;

	EXTI->PR = 0x00000001;  // Clear pending bit of EXTI0. [주] PR은 1을 write함으로써 clear하는 방식

	// PA0, PA1을 읽어 들여 indx의 current value 부분을 갱신하자.
	// 참고로 current value 부분은 0번, 1번 bit 임.
	indx &= ~0x00000003;
	indx |= (GPIOA->IDR & 0x00000003); // 아래 두 bit 만 살리고 나머지는 모두 clear 시킬 것.

	count += changeMtx[indx];  // counter 값을 갱신한다.
	indx <<= 2;   // current value를 2-bit 만큼 shift 시켜서 previous value로 만든다.
	indx &= 0x0000000F;  // indx[3:0]이 중요하므로 나머지 부분은 모드 clear 할 것

	// 요기 아래 부분은 interrupt가 제대로 걸리는지 GPIO로 출력하여 검사하기 위한 부분이다.
	if(flag==0)
	{
		GPIOA->BSRR = GPIO_BSRR_BS_2;  // bit 2을 set
		flag = 1;
	}
	else
	{
		GPIOA->BSRR = GPIO_BSRR_BR_2;  // bit 2을 clear
		flag = 0;
	}
}

void EXTI1_IRQHandler()  // EXTI0_IR1Handler의 IRQn은 6번
{
	static int flag = 0;

	EXTI->PR = 0x00000002;  // Clear pending bit of EXTI1. [주] PR은 1을 write함으로써 clear하는 방식

	// PA0, PA1을 읽어 들여 indx의 current value 부분을 갱신하자.
	// 참고로 current value 부분은 0번, 1번 bit 임.
	indx &= ~0x00000003;
	indx |= (GPIOA->IDR & 0x00000003); // 아래 두 bit 만 살리고 나머지는 모두 clear 시킬 것.

	count += changeMtx[indx];  // counter 값을 갱신한다.
	indx <<= 2;   // current value를 2-bit 만큼 shift 시켜서 previous value로 만든다.
	indx &= 0x0000000F;  // indx[3:0]이 중요하므로 나머지 부분은 모드 clear 할 것

	// 요기 아래 부분은 interrupt가 제대로 걸리는지 GPIO로 출력하여 검사하기 위한 부분이다.
	if(flag==0)
	{
		GPIOA->BSRR = GPIO_BSRR_BS_3;  // bit 3을 set
		flag = 1;
	}
	else
	{
		GPIOA->BSRR = GPIO_BSRR_BR_3;  // bit 3을 clear
		flag = 0;
	}
}


//-------------------------------------------------------------------
// TIM1_CH3을 이용하여 PWM을 생성하고 TIM2를 이용하여 PWM 신호의 주파수와 duty를 측정하는
// 예제를 구성해보자.
// 사용하는 Pin은 다음과 같다.
// PWM 생성 : PA10 (TIM1_CH3)
// Input capture : PA1 (TIM2_CH2)
// PA10을 PA1에 연결해야 함.
// [주] TIM1_CH1 대신 TIM1_CH3를 이용하는 것은 TIM1_CH1이 PA8에 연결되어 있는데
// PA8의 AF1(Alternate Function 1)이 MCO1이기 때문에 MCO1으로 출력하는 test
// program과의 충돌을 막기 위해 TIM1_CH3를 사용하는 것이다. 또한 TIM1_CH2의 경우 PA9에
// 연결되어 있는 것인데 Full speed USB를 사용하려면 PA9를 통해 전원을 공급해야 하므로
// TIM_CH2도 사용하지 않고 사용해도 다른 program에 영향을 주지않을 TIM1_CH3를 사용하기로
// 한다.
// 이 예제와 관련해서는 Mango Story 3 : ARM Cortex-M3 시스템 프로그래밍 완전정복 II
// 를 참조하면 도움이 될 것이다. 그 책에 나온 내용을 토대로 예제를 작성하였다.
// [주]
// duty, period를 정확히 측정하기 위해서 PWM 발생은 edge-aligned 방식이 더 좋은 것으로
// 생각된다. center-aligned 방식의 경우 실제로 측정해 보면 counter value 기준으로
// 1 정도의 jitter가 발생하는 경우가 계속 발생해서 정보를 PWM으로 보낼경우 noise가
// 추가되는 현상이 발생한다. 이에 비해 edge-aligned 방식은 noise 없이 정확하게
// capture를 하는 것을 확인하였다.
//-------------------------------------------------------------------
double Duty;  // 측정을 통해 얻은 PWM의 duty
double Freq;  // 측정을 통해 얻은 PWM의 주파수
uint32_t IC1Value, IC2Value, IC2_PrevValue;
uint32_t duty_cnt_g, period_cnt_g; // 첨자 g는 global 변수를 나타내기 위해 사용해 보았음

void PwmInputCaptureTest()
{
	char str[64];
	USART3_Configuration();  // Serial 통신을 사용하기 위해 USART2 설정

	// 사용하고자 하는 Peripheral에 clock을 enable 시켜주자.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // TIM1 clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // TIM2 clock enable.

	///---------------------------------------------------------------------------
	/// GPIO를 설정하자. PA10 (TIM1_CH3), PA1 (TIM2_CH2), PA3 (GPIO)와 같이 설정해야 한다.
	///---------------------------------------------------------------------------
	/// 제일 먼저 PA3를 출력으로 설정하자. PA3를 사용하는 이유는 TIM2 interrupt가 동작하는지 확인하기 위한 용도이다.
	GPIOA->MODER &= ~GPIO_MODER_MODER3; // PA3을 output으로 설정한다.
	GPIOA->MODER |= (0x1 << GPIO_MODER_MODER3_Pos); // 01 (=0x1): output

	// 두번째로 PA10 (TIM1_CH3)을 PWM을 출력하도록 설정하자. 먼저 PA10을 alternate function으로 설정하자.
	GPIOA->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER |= (0x2 <<GPIO_MODER_MODER10_Pos); // 10 (=0x2) : alternate function

	/// PA10는 AFR[1]에서 설정해야 한다. AFR[1]에서는 2번에 해당한다. 그리고 PWM은 AF1에 해당한다.
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10 ;             //
	GPIOA->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL10_Pos);  //  AF1 할당, AF1는 0001 (=0x1)

	// 마지막으로 PA1을 Alternate function (TIM2_CH2)으로 설정한다.
	GPIOA->MODER &= ~GPIO_MODER_MODER1;
	GPIOA->MODER |= (0x2<<GPIO_MODER_MODER1_Pos); // 10 (=0x2) : alternate function

	/// PA1는 AFR[0]에서 설정해야 한다. AFR[0]에서는 1번에 해당한다. TIM2_CH2는 AF1에 해당한다.
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1;
	GPIOA->AFR[0] |= (0x1 << GPIO_AFRL_AFSEL1_Pos);  //  AF1 할당, AF1는 0001 (=0x1)

	/// 아래 부분은 불필요한 것 같은 comment 처리했다. 지워도 아무 상관없는 것 같다.
	//GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1;              // PA1를 High speed로 설정한다.
	//GPIOA->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED1_Pos);  // High speed일 경우 11 (=0x3)로 설정

	//---------------------------------------------------------------------------
	// 여기서부터 TIM1을 설정해보자. TIM1은 PWM을 출력하도록 설정한다. PWM은 center-aligned,
	// edge-aligned 방식이 있는데 여기서는 둘다 test 해볼 수 있도록 coding 해 놓았다.
	// TIM1은 APB2 peripheral이고 APB2 prescaler가 2이므로 PCLK2 (=84MHz)의 x2에
	// 해당하는 168MHz의 clock이 TIM1에 공급된다. TIM2가 84MHz clock을 사용하므로 TIM1과
	// TIM2의 clock을 일치시키기 위하여 Prescaler 값을 1로 설정하도록 하자.
	//----------------------------------------------------------------------------
	TIM1->PSC = 0x1;   // Prescale 설정 PSC=1, The counter clock frequency (CK_CNT) is equal to
	                   // fCK_PSC / (PSC[15:0] + 1). 즉 division by 2에 해당
	TIM1->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered

#if 0
	// Center-aligned PWM 방식
	TIM1->CR1 &= ~TIM_CR1_CMS;
	TIM1->CR1 |= (0x3 << TIM_CR1_CMS_Pos); // 11 (=0x3) :   .
	                                       // Output compare interrupt flag이 counter가 up, down일 때 모두 set 되는 방식
	TIM1->ARR = 2100;                       // PWM frequency는 84 MHz/(2*2100) = 20 KHz
#endif

#if 1
	// 이번엔 Edge-aligned 방식을 사용하도록 설정해보자.
	TIM1->CR1 &= ~TIM_CR1_CMS;  // CMS = 00 : Edge-aligned mode. CMS=Center-aligned mode selection
	TIM1->CR1 &= ~TIM_CR1_DIR;  // DIR=0 : Upcounter
	TIM1->ARR = 4199;           // PWM frequency는 84MHz MHz/(4199+1) = 20 KHz
#endif

	// Channel 3를 PWM mode로 설정한다.
	TIM1->CCER |= TIM_CCER_CC3E;    // Capture/compare output 1를 enable
	TIM1->CCER &= ~TIM_CCER_CC3P;   // CC3P=0 : Capture/compare output 3을 active high로 설정
	TIM1->CCMR2 &= ~TIM_CCMR2_CC3S; // CC3S=0 : CC3 channel is configured as output
	TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM1->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos);  // OC3M=0110 (=0x6) PWM mode 1

	//-------------------------------------------------------------------------------
	// PWM mode 1 : upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
	// else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0’) as long as
	// TIMx_CNT>TIMx_CCR1 else active (OC1REF=’1’).
	//-------------------------------------------------------------------------------

	//--------------------------------------------------------------------------------
	// Idle state 값을 설정하자. idle state라는 것은 MOE (Master Output Enable)이 0 이 되었을 때
	// PWM pin의 출력값을 뜻한다. 이 값들은 MOE=0 이 되고 dead-time 만큼 시간이 지난후에 출력된다.
	// 요기 아래부분은 무슨 의미인지 좀더 검토해야 할 것 같다.
	//--------------------------------------------------------------------------------
	TIM1->CR2 |= TIM_CR2_OIS3;    // OC3의 idle state는 High로 설정
	TIM1->BDTR &= ~TIM_BDTR_OSSI; // OSSI=0 : MOE=0일 경우 OC/OCN outputs are disabled-> Hi-Z state가 됨.
	                              // 만약 OSSI=1 이면 MOE=0 일 때 deadtime 만큼 지난 후 idle level로 지정한 값으로 forced 된다.
	                              // 따라서 위에서 idle state를 High로 지정했지만 MOE=0일 경우 Hi-Z로 됨.

	//------------------------------------------------------------------------------------
	// 여기서부터 TIM2를 설정해보자. TIM2는 32-bit timer이다. TIM2는 APB1에 속해서 PCLK1을 사용하는데
	// RCC_Configuration에서 Prescale value=4로 해 놓았기 때문에 PCLK1(=42MHz) x 2, 즉 84 MHz의
	// clock speed 를 가지게 된다.
	//------------------------------------------------------------------------------------
	TIM2->PSC = 0x0;   // Prescale 설정 PSC=0, The counter clock frequency (CK_CNT) is equal to
	                   // fCK_PSC / (PSC[15:0] + 1). 즉 no division에 해당함.

	//-------------------------------------------------------------------------------
	// PA1에 PWM 신호(PA10)를 연결시킨다. 내부적인 연결은 다음과 같이 되어야 한다.
	//
	// PA0 (TIM2_CH1)    ------IC1
	//                   |
	// PA1 (TIM2_CH2)--------- IC2
	//
	// 그림에서 보듯이 PA1에서 입력된 신호가 IC1, IC2 모두에 연결되어야 한다.
	// 즉 IC1은 TI2에 연결되어야 하고 IC2도 TI2에 연결한다. datasheet를 보면 IC1에 TI2가 연결될 때에는
	// TI2FP1으로 나타낸다. F는 filtered 된 신호를 나타내고 P는 polarity를 나타낸다.
	// TI2가 IC2에 연결되면 TI2FP2와 같이 신호의 이름이 정해진다.
	// IC1과 IC2에 둘다 TI2, 즉 TIM2_CH2인 PA1의 값이 사용되므로 Pin PA0는 사용하지 않는다. 따라서
	// 선의 연결이 필요없다.
	// 다음과 같은 PWM 파형을 생각해보자.
	//
	//         -------------        ----------
	//         |           |        |
	// ---------           |---------
	//
	// IC1은 falling edge를 검출하여 capture하고 IC2는 rising edge를 검출하여 capture하도록
	// polarity를 설정해야 한다. 또한 CC2 interrupt를 이용하여 capture된 data인 CCR1과 CCR2의
	// data를 이용하여 주파수와 duty를 계산하도록 설정해야 한다.
	//-------------------------------------------------------------------------------
	TIM2->CCER &= (~TIM_CCER_CC1E & ~TIM_CCER_CC2E);  // Capture/Compare 1, 2 output을 disable 시킨다.
	// capture/compare selection을 설정하기 위한 CC1S, CC2S 등의 조작은 CCER의 CC1E, CC2E등을 disable 시킨 상태에서만
	// 할 수 있다. 위의 code에서 CC1E=0, CC2E=0으로 disable 시켜놓은 이유는 아래의 code에서 CC1S, CC2S 등을 조작하기
	// 위해서이다. CC1S, CC2S를 통해서 IC1, IC2에 연결될 TIx의 신호를 설정한다. 여기서는 PA1으로 입력되는 신호, 즉 TI2가
	// IC1과 IC2에 연결되어야 한다. 이때 연결되는 신호의 polarity는 요 아래 나온것처럼 CC1NP/CC1P로 구성된 2-bit의 조합을
	// 통해 설정한다.
	TIM2->CCMR1 &= (~TIM_CCMR1_CC1S & ~TIM_CCMR1_CC2S);
	TIM2->CCMR1 |= (0x2 << TIM_CCMR1_CC1S_Pos);  // 10: CC1 channel is configured as input, IC1 is mapped on TI2
	TIM2->CCMR1 |= (0x1 << TIM_CCMR1_CC2S_Pos);  // 01: CC2 channel is configured as input, IC2 is mapped on TI2
	TIM2->CCMR1 &= (~TIM_CCMR1_IC1F & ~TIM_CCMR1_IC2F); // Input capture 1, 2의 filter를 사용하지 않는다. default설정이므로 comment처리해도 됨

	//-------------------------------------------------------
	// CC1은 falling edge를 검출, CC2는 rising edge를 검출하도록 설정.
	// edge의 polarity는 (CC1NP/CC1P)로 구성된 2-bit의 조합을 통해서 설정한다.
	// 각각의 조합에 대해 다음과 같은 설정이 이루어진다.
	// 00 : rising edge
	// 01 : falling edge
	// 10 : reserved
	// 11 : both edge
	//
	// CC1을 falling edge로 설정하려면 (CC1NP=0, CC1P=1)
	// CC2를 rising edge로 설정하려면 (CC2NP=0, CC2P=0)
	// 으로 해야 한다. default로 CC1NP=0, CC2NP=0으로 되어 있으므로 편의를
	// 위하여 여기서는 CC1P, CC2P만 조작할 수도 있으나, CC1NP, CC2NP가 0이
	// 아닌 것으로 설정되어 있을 경우를 대비해 두 bit를 모두 조작하도록 한다.
	//-------------------------------------------------------
	TIM2->CCER &= ~TIM_CCER_CC1NP; // CC1NP=0, CC1P=1 : The circuit is sensitive to TIxFP1 falling edge
	TIM2->CCER |= TIM_CCER_CC1P;
	TIM2->CCER &= ~TIM_CCER_CC2NP; // CC2NP=0, CC2P=0 : The circuit is sensitive to TIxFP2 rising edge
	TIM2->CCER &= ~TIM_CCER_CC2P;
	TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);  // Capture/Compare 1, 2 output을 enable 시킨다.

	//--------------------------------------------------------------------------------
	// 아래부분은 Slave mode control을 이용하여 TIM2의 CNT를 IC2의 rising edge일 때 clear 시키도록
	// 설정하는 것이다. 이렇게 되면 PWM의 주기와 duty가 일정할 때 CCR1, CCR2가 계속 증가하지 않고 일정한 값을
	// 가지게 된다. 하지만 Prof. Lee의 PPT 강의노트
	// (d:\user\lys\lecture\임베디드시스템설계\SingleBoardComputer\Nucleo\ 아래에
	// 있는 Nucleo(Timer).pptx를 참조할 것)에서 보듯이 이런 방식으로 duty, freq를 측정할 경우
	// 약간의 오차가 발생하게 된다. TIM2의 CNT가 IC2의 rising edge인 순간에 바로 0으로 clear 되는 것이
	// 아니라 약 3 clock 만큼 시간이 지난 후에 0으로 clear 된다. 이로 인해 freq, duty의 측정에 약간의
	// 오차가 발생한다. 따라서 Slave mode control을 이용하면 약간의 편리함은 있지만 측정오차가 생기는
	// 단점이 있다. 아래의 Slave mode control을 사용하고자 하면 #if 0 ~ #endif를 #if 1 ~ #endif로
	// 수정하여야 한다. 그리고 저기 아래의 TIM2의 interrupt handler 안의 부분도 수정해 주어야 한다.
	// TIM2_IRQHandler에는 Slave mode control을 이용하는 경우에 있어 Freq, duty를 계산하는 code와
	// Slave mode control을 사용하지 않는 경우에 Freq, Duty를 계산하는 code가 있다.
	// Slave mode control을 이용하지 않게 되면 TIM2의 CNT는 계속해서 Upcounting을 반복하게 된다.
	//--------------------------------------------------------------------------------
#if 0  // Slave mode control을 사용하려면 #if 0를 #if 1로 변경할 것.
	// SMCR : Slave mode control register
	TIM2->SMCR &= ~TIM_SMCR_TS;
	TIM2->SMCR |= (0x6 << TIM_SMCR_TS_Pos);  // Trigger selection, 110 : Filtered Timer Input 2 (TI2FP2)

	TIM2->SMCR &= ~TIM_SMCR_SMS;
	TIM2->SMCR |= (0x4 << TIM_SMCR_SMS_Pos); // 0100 : reset Mode - Rising edge of the selected trigger input (TRGI)
	                                         // reinitializes the counter and generates an update of the registers.
	                                         // 이 경우 Trigger input은 TI2FP2로 설정되어 있다.
	TIM2->SMCR |= TIM_SMCR_MSM;  // Master/slave mode enable.
#endif

	TIM2->DIER |= TIM_DIER_CC2IE;  // 1 : CC2 interrupt enable 시킨다.(CC2=Capture/Compare 2)

	// TIM1을 가동시키자.
	TIM1->CNT = 0;              // Counter를 0으로 clear
	TIM1->CR1 |= TIM_CR1_CEN;   // TIM1 enable.
	TIM1->BDTR |= TIM_BDTR_MOE; // MOE=1 : Main output enable

	// TIM2를 가동시키자.
	TIM2->CNT = 0;              // Counter를 0으로 clear
	TIM2->CR1 |= TIM_CR1_CEN;   // TIM2 enable.

	TIM1->CCR3 = 2100;          // PWM duty를 조정하기 위해서 이 값을 변경하면 된다. PA10은 TIM1_CH3 임에 유의한다.

	NVIC->ISER[0] = 0x10000000; // TIM2 interrupt를 enable 시킨다. [주] TIM2_IRQn = 28
	//NVIC_EnableIRQ(TIM2_IRQn); // 위의 code 대신 이렇게 처리할 수도 있다. TIM2_IRQn는 stm32f407xx.h에 정의

	while(1)
	{
		sprintf(str, "%f  %f  %d  %d\n", Duty, Freq, duty_cnt_g, period_cnt_g);
		TX3_PutString(str);
		DWT_us_Delay(1000000);
	}
}

void TIM2_IRQHandler()
{
	//----------------------------------------------------
	// TIM2 에서는 PWM input에 대한 capture interrupt를
	// 시행한다. 이 경우 rising edge일 때 interrupt에 진입한다.
	//    -------     -------
	//    |     |     |     |
	//  ---     -------     -------
	//    0    CCR1  CCR2
	//----------------------------------------------------
	static int flag = 0;
	static int first_flag = 1;


	uint32_t period_cnt, duty_cnt;

	TIM2->SR = ~TIM_SR_CC2IF; // CC2IF bit를 clear 한다. 0을 write함으로써 clear할 수 있는 register 임.
	//TIM2->SR = 0;           // Pending bit를 모두 지우고 싶다면 이렇게 할 것.


	//-----------------------------------------------------------------
	// PA3를 Toggle 시킴으로써 TIM2 interrupt가 제대로 걸리는지를 확인할 수 있도록 하자
	//-----------------------------------------------------------------
	if(flag==0)
	{
		GPIOA->BSRR = GPIO_BSRR_BS_3;  // bit 3을 set
		flag = 1;
	}
	else
	{
		GPIOA->BSRR = GPIO_BSRR_BR_3;  // bit 3을 clear
		flag = 0;
	}

	// 요기 바로 아래 부분은 Slave mode control을 사용하지 않는 경우 활성화해주어야 한다.
#if 1
	if(first_flag)
	{
		IC2_PrevValue = TIM2->CCR2;
		Freq = 0;
		Duty = 0;
		first_flag = 0;
	}
	else
	{
		IC1Value = TIM2->CCR1;
		IC2Value = TIM2->CCR2;

		if(IC2_PrevValue>IC2Value) period_cnt = IC2Value-IC2_PrevValue + 0xFFFFFFFF;  // 32-bit counter이므로
		else period_cnt = IC2Value-IC2_PrevValue;

		if(IC2_PrevValue>IC1Value) duty_cnt = IC1Value - IC2_PrevValue + 0xFFFFFFFF;
		else duty_cnt = IC1Value - IC2_PrevValue;

		IC2_PrevValue = IC2Value;

		Duty = (double)(duty_cnt*100)/(double)period_cnt;
		Freq = 84000000/(double)period_cnt;
		duty_cnt_g = duty_cnt;
		period_cnt_g = period_cnt;
	}
#endif


	// 요기 아래 부분은 Slave mode control을 사용하는 경우 활성화 해주어야 한다.
#if 0
	//-----------------------------------------------------------------
	// Capture된 register 값을 이용하여 PWM의 주파수와 duty를 계산해보자.
	//-----------------------------------------------------------------
	IC2Value = TIM2->CCR2;  // Period에 해당하는 값이 CCR2에 저장됨
	period_cnt_g = IC2Value;

	if(IC2Value!=0)
	{
		IC1Value = TIM2->CCR1;  // falling edge일때의 counter 값
		duty_cnt_g = IC1Value;
		Duty = (double)(IC1Value*100)/(double)IC2Value;
		Freq = 84000000/(double)IC2Value;  // TIM2가 84MHz
	}
	else
	{
		Duty = 0;
	}
#endif
}

//---------------------------------------------------------------------
// SPI 통신을 이용하여 Magnetic Encoder인 AS5145B를 interface 하는 방법을 학습해보자.
// STM32F407의 SPI 관련 Pin은 다음과 같다.
//
// PB3 SPI1_SCLK (AF5)
// PB4 SPI1_MISO (AF5)
// PB5 SPI1_MOSI (AF5)
//
// AS5145B의 Chip Select는 PB0를 GPIO 설정하여 발생시키기로 한다. STM32F407의 경우
// NSS Pin이 있어 하드웨어적인 방법으로 CS를 생성할 수 있다. 하지만 이 예제에서는 NSS를 사용하지
// 않고 Software적인 방법으로 CS를 생성하게끔 설정할 것이다. (하드웨어적인 방법으로 NSS를
// 제어하는 방식을 실행해 보려고 했으나 실패했고 시간 관계상 GPIO로 Chip Select를 만들어
// 주는 방식을 먼저 구현하였다. 나중에 시간이 되면 NSS를 하드웨어적으로 제어하여 발생하는 예제를
// 만들어 보도록 하자.)
// STM32F407과의 연결을 위해 AS5145B에서 연결해주어야 하는 Pin은
// 3.3V, GND, CSn, CLK, D0 이다. 참고로 AS5145B는 Absolute magnetic
// encoder로써 resolution은 4096으로 12-bit data이다. STM32F407의 경우 SPI
// 통신에서 8-bit data frame format과 16-bit data frame format을 지원하는데
// AS5145B를 interface 하기 위해서는 16-bit data frame format을 선택해야 한다.
// 하지만 들어오는 data중 유의미한 12-bit의 data는 16-bit의 상위 12-bit에 채워지므로
// 우리가 필요한 12-bit를 얻어내기 위해서는 하위 4-bit를 clear 시키고, 4-bit 만큼
// shift right 연산을 수행해야 한다.
//
// 결선정보는 다음과 같다.
//-----------------------------------------
// STM32F407           AS5145B
// PB3 (SPI1_SCLK)       CLK
// PB4 (SPI1_MISO)       D0
// PB0                   CSn
//-----------------------------------------
// 위를 보면 PB5(SPI1_MOSI)는 결선하지 않아도 됨을 알 수 있다.
//
// SPI1은 APB2 peripheral로써 clock은 PCLK2 (=84 MHz)를 공급받는다.
//---------------------------------------------------------------------
void SpiTest()
{
	char str[64];
	uint16_t cnt;

	USART3_Configuration();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // GPIOA clock enable, PA4를 사용하기 위해서 GPIOA clock 필요
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   // GPIOB clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;    // SPI clock enable

	// PB3, PB4, PB5를 SPI 기능을 수행하도록 설정하자. 여기서 SPI 기능은 AF5에 해당한다.
	GPIOB->MODER &= (~GPIO_MODER_MODER3 & ~GPIO_MODER_MODER4 & ~GPIO_MODER_MODER5);
	GPIOB->MODER |= ((0x2U << GPIO_MODER_MODER3_Pos)|(0x2U << GPIO_MODER_MODER4_Pos));  // Alternate function의 경우 10 (=0x2)로 설정해야 함
	GPIOB->MODER |= (0x2U << GPIO_MODER_MODER5_Pos);  // Alternate function의 경우 10 (=0x2)로 설정해야 함
	GPIOB->AFR[0] &= ~0x00FFF000;   // 3,4,5번 Pin의 alternate function 기능 clear.
	GPIOB->AFR[0] |= 0x00555000;    // AF5 할당.

	// PA4를 SPI1_NSS로 하여 hardware적으로 생성해보려 했으나 실패했다.
	// 아래에 comment 처리한 부분은 바로 PA4를 이용한 NSS 생성을 시도하려고 작업했던 code이다.
/*
 	GPIOA->MODER &= ~GPIO_MODER_MODER4;
	GPIOA->MODER |= (0x2U << GPIO_MODER_MODER4_Pos);  // Alternate function의 경우 10 (=0x2)
	GPIOA->AFR[0] &= ~0x000F0000;   // 4번 Pin의 alternate function 기능 clear.
	GPIOA->AFR[0] |= 0x00050000;    // AF5 할당.
*/

	// PB0를 GPIO 출력으로 설정하고 Speed는 High speed로 하자.
	GPIOB->MODER &= ~GPIO_MODER_MODER0;
	GPIOB->MODER |= (0x1 << GPIO_MODER_MODER0_Pos);  // 01 (=0x1)의 GPIO 출력
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0;
	GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED0_Pos); // 11 (=0x3)이 Very high speed, 00 Low speed, 01 Medium speed, 10 High speed

	GPIOB->BSRR = 0x00000001;   // PB0 HIGH, Chip select를 High로 설정

	// 여기서부터 SPI 관련 설정이 시작된다.
	SPI1->CR1 |= SPI_CR1_CPHA;   // CPHA=1
	SPI1->CR1 &= ~ SPI_CR1_CPOL; // CPOL=0
	SPI1->CR1 |= SPI_CR1_MSTR;   // Master Mode로 설정
	SPI1->CR1 &= ~SPI_CR1_BR;    // Baud rate를 설정해보자.
	SPI1->CR1 |= (0x6 << SPI_CR1_BR_Pos);  // Baud rate를 f_PCLK/128로 분주설정. 84MHz/128 = 656250 Hz.

	SPI1->CR1 |= SPI_CR1_SSM;   // bit SSM=1
	SPI1->CR1 |= SPI_CR1_SSI;   // bit SSI=1
	SPI1->CR2 &= ~SPI_CR2_SSOE; // bit SSOE=0, 즉 SS output을 disable 시킨다.

	/// 이 아래의 code도 역시 PA4를 SPI1_NSS로 사용하려고 작업했던 code 이다. 동작하지 않으므로
	/// 큰 의미는 없다고 하겠다.
	///SPI1->CR2 |= SPI_CR2_SSOE; // bit SSOE=1, 즉 SS output을 enable 시킨다.

	//----------------------------------------------------------------
	// data size를 설정하자. STM32F407에서는 8-bit data frame format과
	// 16-bit data frame format을 지원한다. CR1 register의 DFF bit에
	// 0을 쓰면 8-bit, 1을 쓰면 16-bit data frame format이다.
	// 우리가 interface하려는 AS5145B는 12-bit data를 사용한다. 따라서
	// 16-bit data frame을 사용하여 data를 받게 되면 AS5145B의 12-bit data가
	// 16-bit중 상위 12-bit를 차지하게 되어 masking을 통해 하위 4-bit를 clear시킨뒤에
	// 4-bit만큼 shift right를 해 주어야 한다.
	//----------------------------------------------------------------
	SPI1->CR1 &= ~SPI_CR1_DFF;    // DFF bit를 clear 시킨다.
	SPI1->CR1 |= (0x1 << SPI_CR1_DFF_Pos);  // 0: 8-bit data frame format, 1:16-bit data frame format

	SPI1->CR1 |= SPI_CR1_SPE;   // SPI를 enable 시킨다.

	while(1)
	{
		GPIOB->BSRR = 0x00010000;   // PB0 (CS)를 Low로 설정

		SPI1->DR = 0x0000;  // dummy data writing
		while((SPI1->SR & 0x0003)!=0x0003); // data가 수신될 때까지 기다린다.
		                                    // 0x0003의 의미는 Tx Buffer empty 이면서 Rx buffer는 not empty를 뜻함.
		cnt = SPI1->DR;  // data를 읽어들인다.

		GPIOB->BSRR = 0x00000001;   // PB0 (CS)를 High로 설정

		cnt &= 0xFFF0;  // 상위 12-bit만을 남기고 하위 4-bit는 clear 시킨다.
		cnt >>= 4;      // 4-bit 만큼 shirt right 연산을 수행해 우리가 필요한 12-bit data를 얻어낸다.
		sprintf(str, "%d\n", cnt);
		TX3_PutString(str);
		DWT_us_Delay(10000);
	}
}

//-----------------------------------------------------------------------------
// Timer의 PWM 생성을 DMA를 이용하여 이루어지도록 하는 방법을 학습할 수 있는 함수
// 여기서 Timer는 TIM1을 사용하고 PWM 출력은 TIM_CH1을 통해 이루어지도록 한다.
// 이때 DMA request는 Timer1의 update event에 의해 발생하므로 TIM1_UP을
// 담당하는 DMA인 DMA2의 channel 6를 사용하도록 한다.
// TIM_CH1은 PE9, TIM_CH1N은 PE8을 이용하기로 한다. 둘다 AF1(Alternate function 1)이다.
//-----------------------------------------------------------------------------
void TimerDmaTest()
{
	// 먼저 GPIO PE8, PE9을 위한 GPIO 설정을 하기로 하자.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;  // GPIOE clock enable

	// PE8, PE9를 alternate function으로 설정하자.
	GPIOE->MODER &= (~GPIO_MODER_MODER8 & ~GPIO_MODER_MODER9);
	GPIOE->MODER |= ((0x2 <<GPIO_MODER_MODER8_Pos)|(0x2<<GPIO_MODER_MODER9_Pos));    // 10 (=0x2) : alternate function

	// GPIO 속도를 High speed로 설정하자. 사실 이 부분이 필요한지 안한지는 잘 모르겠다. comment 처리해도 PWM은 잘 생성되더라...
	GPIOE->OSPEEDR &= (~GPIO_OSPEEDR_OSPEED8 & ~GPIO_OSPEEDR_OSPEED9);
	GPIOE->OSPEEDR |= ((0x3U << GPIO_OSPEEDR_OSPEED8_Pos)|(0x3U << GPIO_OSPEEDR_OSPEED9_Pos));  // High speed일 경우 11 (=0x3)로 설정

	//----------------------------------------------------------
	// PE8, PE9 AFR[1]에서 설정해야 한다.
	// PWM과 관련해서는 모두 AF1에 해당한다.
	//----------------------------------------------------------
	GPIOE->AFR[1] &= (~GPIO_AFRH_AFSEL8 & ~GPIO_AFRH_AFSEL9);
	GPIOE->AFR[1] |= ((0x1 << GPIO_AFRH_AFSEL8_Pos)|(0x1 << GPIO_AFRH_AFSEL9_Pos)); //  AF1 할당, AF1는 0001 (=0x1)

	// 여기서부터 TIM1에 대한 설정을 하도록 한다.
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   // TIM1 clock enable

	TIM1->PSC = 0x0;   // Prescale 설정 PSC=0, The counter clock frequency (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1). 즉 no division에 해당함.
	TIM1->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered
	TIM1->CR1 &= ~TIM_CR1_DIR; // DIR=0 downcounter, 참고로 DIR=1은 upcounter이다.
	TIM1->CR1 |= TIM_CR1_DIR; // Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered
	TIM1->CR1 &= ~TIM_CR1_CMS;
	TIM1->CR1 |= (0x0 << TIM_CR1_CMS_Pos); // 00 (=0x0) : Edge-aligned mode.
	TIM1->ARR = 1119;   // PWM frequency는 168 MHz/(1119+1) = 150 KHz

	// Channel 1을 PWM mode로 설정한다.
	TIM1->CCER &= (uint16_t)~TIM_CCER_CC1E; // Disable the channel 1
	TIM1->CCER |= TIM_CCER_CC1E;    // Capture/compare output 1를 enable
	TIM1->CCER |= TIM_CCER_CC1NE;   // Capture/compare complementary output 1을 enable
	TIM1->CCER &= ~TIM_CCER_CC1P;   // CC1P=0 Capture/compare output 1을 active high로 설정
	TIM1->CCER &= ~TIM_CCER_CC1NP;  // CC1NP=0 Capture/compare complementary output 1을 active high로 설정
	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S; // CC1S=0  CC 1 channel are configured as output
	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);  // OC1M=0110 (=0x6) PWM mode 1

	//--------------------------------------------------------------------------------
	// Idle state 값을 설정하자. idle state라는 것은 MOE (Master Output Enable)이 0 이 되었을 때
	// PWM pin의 출력값을 뜻한다. 이 값들은 MOE=0 이 되고 dead-time 만큼 시간이 지난후에 출력된다.
	//--------------------------------------------------------------------------------
	TIM1->CR2 |= (TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3);       // OC1, OC2, OC3의 idle state는 High로 설정
	TIM1->CR2 &= (~TIM_CR2_OIS1N & ~TIM_CR2_OIS2N & ~TIM_CR2_OIS3N); // OC1N, OC2N, OC3N의 idle state는 LOW로 설정

	TIM1->CR1 &= ~TIM_CR1_CKD;
	TIM1->CR1 |= (0x0 << TIM_CR1_CKD_Pos); // 00: t_DTS = tck_INT, 01:t_DTS=2*tck_INT, 10:t_DTS=4*tck_INT

	TIM1->BDTR &= ~TIM_BDTR_OSSI; // OSSI=0 : MOE=0일 경우 OC/OCN outputs are disabled-> Hi-Z state가 됨.
	                              // 만약 OSSI=1 이면 MOE=0 일 때 deadtime 만큼 지난 후 idle level로 지정한 값으로 forced 된다.
	TIM1->BDTR &= ~TIM_BDTR_DTG;
	TIM1->BDTR |= (10 << TIM_BDTR_DTG_Pos);  // dead-time=10*dts=59.52 [ns]

	TIM1->CNT = 0;   // Counter를 0으로 clear

	TIM1->CCR1 = 2100;  // 2100은 4200의 절반에 해당하는 값
	TIM1->CCR2 = 2100;
	TIM1->CCR3 = 2199;

	TIM1->CR1 |= TIM_CR1_CEN;   // TIM1 enable.
	TIM1->BDTR |= TIM_BDTR_MOE; // MOE=1 : Main output enable

}

//------------------------------------------------------------------
//		STM32F407VE            Inverter Board(ECL ESC)
//------------------------------------------------------------------
//       PE13           UH 상단 FET gate 신호 (PWM)
//       PE12      	    UL 하단 FET gate 신호 (PWM/GPIO)
//       PE11           VH 상단 FET gate 신호 (PWM)
//       PE10           VL 하단 FET gate 신호 (PWM/GPIO)
//       PE 9           WH 상단 FET gate 신호 (PWM)
//       PE 8           WL 하단 FET gate 신호 (PWM/GPIO)
//------------------------------------------------------------------
//----------------------------------------------------------
// TIM1은 Advanced timer로써 3상 PWM을 생성할 수 있다.
// Complementary PWM을 생성해보는 방법을 학습해보자. 더불어 dead-time도
// 추가하는 방법을 살펴보기로 한다. STM32F407에서 3상 PWM을 생성하기 위한 Pin은
// 다음과 같이 정하기로 한다.
// PE8  --> TIM1_CH1N (AF1) -> UL
// PE9  --> TIM1_CH1  (AF1) -> UH
// PE10 --> TIM1_CH2N (AF1) -> VL
// PE11 --> TIM1_CH2  (AF1) -> VH
// PE12 --> TIM1_CH3N (AF1) -> WL
// PE13 --> TIM1_CH3  (AF1) -> WH
//
// 또다른 조합으로 다음과 같은 Pin 사용도 생각해 볼 수 있다. 하지만 아래의 조합은
// PA9을 사용해야 하는데 이렇게 되면  Full speed USB를 사용할 수 없게 된다.
// 위에서 제시한 첫번째 Pin 조합을 사용하는 것으로 한다.
//
// PA8  --> TIM1_CH1  (AF1)  => PA8은 MCO1으로 사용할 수 있으므로 안쓰는 게 좋다.
// PA9  --> TIM1_CH2  (AF1)  => PA9를 사용하면 USB를 사용할 수 없게 된다.
// PA10 --> TIM1_CH3  (AF1)
// PA7  --> TIM1_CH1N (AF1)
// PB0  --> TIM1_CH2N (AF1)
// PB1  --> TIM1_CH3N (AF1)
//
// Timer1은 APB2에 속한 peripheral이다. APB2의 prescaler를 1이 아닌 2로 설정했으므로
// APB2 timer에 공급되는 clock은 PCLK2 (=84MHz)의 x2가 168 MHz의 clock이 공급된다.
// 여기서는 Center-aligned 방식의 PWM을 사용하고 ARR의 값을 4200 으로 설정하여
// PWM의 주파수는 168 MHz / (4200*2) = 20 KHz 가 되는 것으로 설정한다.
//----------------------------------------------------------

void TIM1_Init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;  // GPIOE clock enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   // TIM1 clock enable

	// PE8, PE9, PE10, PE11, PE12, PE13을 alternate function으로 설정하자.
	GPIOE->MODER &= (~GPIO_MODER_MODER8 & ~GPIO_MODER_MODER9 & ~GPIO_MODER_MODER10);
	GPIOE->MODER &= (~GPIO_MODER_MODER11 & ~GPIO_MODER_MODER12 & ~GPIO_MODER_MODER13);
	GPIOE->MODER |= ((0x2 << GPIO_MODER_MODER8_Pos) | (0x2 << GPIO_MODER_MODER9_Pos)); // 10 (=0x2) : alternate function
	GPIOE->MODER |= ((0x2 << GPIO_MODER_MODER10_Pos) | (0x2 << GPIO_MODER_MODER11_Pos)); // 10 (=0x2) : alternate function
	GPIOE->MODER |= ((0x2 << GPIO_MODER_MODER12_Pos) | (0x2 << GPIO_MODER_MODER13_Pos)); // 10 (=0x2) : alternate function

	// GPIO 속도를 High speed로 설정하자. 사실 이 부분이 필요한지 안한지는 잘 모르겠다. comment 처리해도 PWM은 잘 생성되더라...
	GPIOE->OSPEEDR &= (~GPIO_OSPEEDR_OSPEED8 & ~GPIO_OSPEEDR_OSPEED9 & ~GPIO_OSPEEDR_OSPEED10);
	GPIOE->OSPEEDR &= (~GPIO_OSPEEDR_OSPEED11 & ~GPIO_OSPEEDR_OSPEED12 & ~GPIO_OSPEEDR_OSPEED13);
	GPIOE->OSPEEDR |= ((0x3U << GPIO_OSPEEDR_OSPEED8_Pos) | (0x3U << GPIO_OSPEEDR_OSPEED9_Pos)); // High speed일 경우 11 (=0x3)로 설정
	GPIOE->OSPEEDR |= ((0x3U << GPIO_OSPEEDR_OSPEED10_Pos) | (0x3U << GPIO_OSPEEDR_OSPEED11_Pos)); // High speed일 경우 11 (=0x3)로 설정
	GPIOE->OSPEEDR |= ((0x3U << GPIO_OSPEEDR_OSPEED12_Pos) | (0x3U << GPIO_OSPEEDR_OSPEED13_Pos)); // High speed일 경우 11 (=0x3)로 설정

	//----------------------------------------------------------
	// PE8, PE9, PE10, PE11, PE12, PE13은 AFR[1]에서 설정해야 한다.
	// AFR[1]에서는 0번, 1번, 2번, 3번, 4, 5번에 해당한다.
	// PWM과 관련해서는 모두 AF1에 해당한다.
	//----------------------------------------------------------
	GPIOE->AFR[1] &= (~GPIO_AFRH_AFSEL8 & ~GPIO_AFRH_AFSEL9 & ~GPIO_AFRH_AFSEL10);
	GPIOE->AFR[1] &= (~GPIO_AFRH_AFSEL11 & ~GPIO_AFRH_AFSEL12 & ~GPIO_AFRH_AFSEL13);
	GPIOE->AFR[1] |= ((0x1 << GPIO_AFRH_AFSEL8_Pos) | (0x1 << GPIO_AFRH_AFSEL9_Pos) | (0x1 << GPIO_AFRH_AFSEL10_Pos)); // AF1 할당, AF1는 0001 (=0x1)
	GPIOE->AFR[1] |= ((0x1 << GPIO_AFRH_AFSEL11_Pos) | (0x1 << GPIO_AFRH_AFSEL12_Pos) | (0x1 << GPIO_AFRH_AFSEL13_Pos)); // AF1 할당, AF1는 0001 (=0x1)

	// 여기서부터 TIM1을 설정해보자.
	TIM1->PSC = 0x0; // Prescale 설정 PSC=0, The counter clock frequency (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1). 즉 no division에 해당함.
	TIM1->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered
	TIM1->CR1 &= ~TIM_CR1_CMS;
	TIM1->CR1 |= (0x3 << TIM_CR1_CMS_Pos); // 11 (=0x3) : Center-aligned mode 3. Output compare interrupt flag이 counter가 up, down일 때 모두 set 되는 방식
	TIM1->ARR = 4200;   			// PWM frequency는 168 MHz/(2*4200) = 20 KHz

	// Channel 1, 2, 3를 PWM mode로 설정한다.
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E); // Capture/compare output 1, 2, 3를 enable
	TIM1->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE); // Capture/compare complementary output 1, 2, 3을 enable
	TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P); // CC1P=0, CC2P=0, CC3P=0 : Capture/compare output 1,2,3을 모두 active high로 설정
	TIM1->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP); // CC1NP=0, CC2NP=0, CC3NP=0, Capture/compare complementary output 1, 2, 3을 모두 active high로 설정

	TIM1->CCMR1 &= (~TIM_CCMR1_CC1S & ~TIM_CCMR1_CC2S); // CC1S=0, CC2S=0, CC1 channel, CC 2 channel are configured as output
	TIM1->CCMR2 &= ~TIM_CCMR2_CC3S; // CC3S=0 : CC3 channel is configured as output

	TIM1->CCMR1 &= (~TIM_CCMR1_OC1M & ~TIM_CCMR1_OC2M);
	TIM1->CCMR1 |= ((0x6 << TIM_CCMR1_OC1M_Pos) | (0x6 << TIM_CCMR1_OC2M_Pos)); // OC1M=0110 (=0x6) PWM mode 1, OC2M=0110
	TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM1->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos); // OC3M=0110 (=0x6) : PWM mode 1

	TIM1->CCMR2 &= ~TIM_CCMR2_CC4S; //CC4를 output 모드로 설정
	TIM1->CCMR2 |= (0x06 << TIM_CCMR2_OC4M_Pos); //CC4를 PWM 모드로 설정

	//--------------------------------------------------------------------------------
	// Idle state 값을 설정하자. idle state라는 것은 MOE (Master Output Enable)이 0 이 되었을 때
	// PWM pin의 출력값을 뜻한다. 이 값들은 MOE=0 이 되고 dead-time 만큼 시간이 지난후에 출력된다.
	//--------------------------------------------------------------------------------
	TIM1->CR2 |= (TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3); // OC1, OC2, OC3의 idle state는 High로 설정
	TIM1->CR2 &= (~TIM_CR2_OIS1N & ~TIM_CR2_OIS2N & ~TIM_CR2_OIS3N); // OC1N, OC2N, OC3N의 idle state는 LOW로 설정

	TIM1->CR2 |= TIM_CR2_OIS4; //실제로 CC4 출력이 나가는지 확인하기 위해서 하는 것이다. 나중에는 삭제할 예정.

	TIM1->CR1 &= ~TIM_CR1_CKD;
	TIM1->CR1 |= (0x0 << TIM_CR1_CKD_Pos); // 00: t_DTS = tck_INT, 01:t_DTS=2*tck_INT, 10:t_DTS=4*tck_INT

	TIM1->BDTR &= ~TIM_BDTR_OSSI; // OSSI=0 : MOE=0일 경우 OC/OCN outputs are disabled-> Hi-Z state가 됨.
								  // 만약 OSSI=1 이면 MOE=0 일 때 deadtime 만큼 지난 후 idle level로 지정한 값으로 forced 된다.
	TIM1->BDTR &= ~TIM_BDTR_DTG;
	TIM1->BDTR |= (0 << TIM_BDTR_DTG_Pos);  // dead-time=10*dts=59.52 [ns]

	TIM1->CNT = 0;	 // Counter를 0으로 clear

	TIM1->CCER |= TIM_CCER_CC4E; // capture/compare ouput CC4를 enable.
	TIM1->CCER &= ~TIM_CCER_CC4P; //CC4를 active high로 설정

	//Compare match Interrupt Enable(TIM1_CH4=CC4에 해당. RCR을 사용한 인터럽트 대신 Compare match를 이용한 교수님 방식을 활용한다.)
	TIM1->DIER |= TIM_DIER_CC4IE; // Capture/Compare 4 interrupt를 enable 시킨다.
	TIM1->SR &= ~TIM_SR_CC4IF; // Capture/Compare 4 interrupt flag를 초기화 시킨다.
	NVIC_DisableIRQ(TIM1_CC_IRQn); //NVIC Enable.

	TIM1->CCR1 = 2100; 	 					 // 2100은 4200의 절반에 해당하는 값
	TIM1->CCR2 = 2100;
	TIM1->CCR3 = 2100;

	TIM1->CCR4 = 1; // Compare match시 사용되는 CCR값. 원래 1 or ARR-1을 선택함에 따라 서로 다른 타이밍에서 compare match가 발생하게 된다.
					   // 그렇게 하지 않고, 0 or ARR을 사용하게 되면, Injected ADC의 Auto-Trigger 신호가 생성되지 않는다.(TIM1_CH4가 펄스 파형으로 나와야, ADC 트리거 신호가 되기 때문임)

	TIM1->CR1 |= TIM_CR1_CEN;   			 // TIM1 enable.
	TIM1->BDTR |= TIM_BDTR_MOE; 			 // MOE=1 : Main output enable
}

//------------------------------------------------------------------
//     STM32F407VE   Peripheral Board
//------------------------------------------------------------------
//     PC14(GPIO)         SW1
//     PC15(GPIO)         SW2
//------------------------------------------------------------------
void ExtInterrupt_Init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    // GPIOC clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   // SYSCFG clock enable.

	GPIOC->MODER &= ~(GPIO_MODER_MODER14 | GPIO_MODER_MODER15); // PC14, PC15을 input으로 설정한다.

	// PC14와 PC15을 external interrupt의 source로 설정한다.
	// EXTI14: PC14, EXTI15: PC15
	// STM32F407 Reference Manual p291 참고
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI14 | SYSCFG_EXTICR4_EXTI15);// EXTIx[3:0]=0000 : PC[x]가 source input임.

	SYSCFG->EXTICR[3] |= (0x2 << SYSCFG_EXTICR4_EXTI14_Pos) | (0x2 << SYSCFG_EXTICR4_EXTI15_Pos);// EXTIx[3:0]=0010 : PC[x]가 source input임.

	EXTI->IMR |= (EXTI_IMR_MR14 | EXTI_IMR_MR15); // Enable EXTI6, EXTI7 interrupt, EXTI11 interrupt

	// rising edge, falling edge 둘다에 interrupt가 걸리도록 설정하자.
	EXTI->RTSR |= (EXTI_RTSR_TR14 | EXTI_RTSR_TR15); // TRx=1: Rising trigger enabled (for Event and Interrupt) for input line

	EXTI->FTSR |= (EXTI_FTSR_TR14 | EXTI_FTSR_TR15); // TRx=1: Falling trigger enabled (for Event and Interrupt) for input line.

	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void GPIO_Init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODER2); //PA2를 입력으로 설정

	GPIOD->MODER &= ~(GPIO_MODER_MODER13);
	GPIOD->MODER |= (0x01 << GPIO_MODER_MODER13_Pos); //PD13을 출력으로 설정 LED를 이용한 상태확인
}

void StartMotor()
{
	int i;

#if 1
	pPwmPattern = pwmPatternsForward;
	pGpioPattern = gpioPatternsForward;
//	pADMUXTable = admuxTableForward;
#endif

#if 0
	pPwmPattern = pwmPatternForward;
	pGpioPattern = gpioPatternForward;
	pADMUXTable = admuxTableForward;
#endif

	NVIC_DisableIRQ(TIM1_CC_IRQn); //NVIC Disable.
	NVIC_DisableIRQ(TIM2_IRQn); //NVIC Disable.
	NVIC_DisableIRQ(TIM5_IRQn); //NVIC Disable.

	//PWM 초기 duty ratio를 결정한다.(최대 전압의 1/8 DUTY RATIO)
	TIM1->CCR1 = 525;
	TIM1->CCR2 = 525;
	TIM1->CCR3 = 525;

//	NVIC_EnableIRQ(TIM1_CC_IRQn); //NVIC Disable.
//	NVIC_EnableIRQ(TIM2_IRQn); //NVIC Disable.
//	NVIC_EnableIRQ(TIM5_IRQn); //NVIC Disable.

	DWT_us_Delay(5000);

	nextCommutationStep = 0;

	for(int i=0; i<STARTUP_NUM_COMMUTATIONS; i++)
	{
		//Commutation 방식의 변경.
		GPIOE->ODR = *(pGpioPattern + nextCommutationStep);
		GPIOE->MODER = *(pPwmPattern + nextCommutationStep);

		DWT_us_Delay(startupDelays[i]);

		//ADC MUX 설정하는 부분이 STM 버전으로 들어가야 함.

		zcPolarity = nextCommutationStep & 0x01;

		nextCommutationStep++;

		if(nextCommutationStep>=6) nextCommutationStep = 0;
	}

	filteredTimeSinceCommutation = startupDelays[STARTUP_NUM_COMMUTATIONS - 1]*STARTUP_DELAY_MULTIPLIER/2;

	//motor start up motion 끝.
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	GPIOD->ODR = ~(GPIO_ODR_OD13);
	//Compare match value = CCRx 변경하고 인터럽트를 활성화하는 부분.
}














