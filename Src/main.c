#include "stm32l4xx_hal.h"
#include "PID_GRANDO_F.h"
#include "gpio.h"

#define F_APB1 48000000								//Hz
#define RLSB0C 65535.*(1000./4022.)						//LSB at 1kOhm
#define Talpha 0.00375								//1/K
#define T0 35									//Start/end temperature
#define Tsmin 100								//Preheat lower temperature
#define Tsmax 150								//Preheat upper temperature
#define Tl 183									//Maintatence temperature
#define Tp 235									//Peak temperature
#define Thist 0.5								//Temperature deviation
#define tstart 0								//Start time
#define tsp tstart+120								//Start to preheat time
#define tprm tsp+90								//Preheat to maintenance time
#define tmp tprm+40								//Maintenance to peak time
#define tpk1 tmp+((105-20)/2)							//Peak start time
#define tpk2 tpk1+20								//Peak end time
#define tpm tpk2+((105-20)/2)							//Peak to maintenance time
#define tend tpm+60								//Maintenance to end time
#define cycfreq 200								//Cycle frequency
#define WITH_REG 20								//PID synchronization cycle
#define T5MIN 60000								//Number of 5 ms cycles to reach the goal of 5 mins
#define Toff 0									//Power off state goal temperature

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void max_read (void);
char SPItrx (char tbyte);
void Delay_us (uint16_t time);
void max_write (uint8_t addr, uint8_t *src, uint8_t len);
void UARTTX (uint16_t *src, uint16_t len);
//void COMPinit (void);

size_t __write(int, const unsigned char *, size_t);

size_t __write(int i, const unsigned char *src, size_t len){
	
	unsigned char *pSrc = (unsigned char *)src;				//Data to transfer
	unsigned char *pEnd = pSrc + len;
	while(pSrc < pEnd){
		while ((USART2->ISR & USART_ISR_TXE) && (USART2->ISR & USART_ISR_TC) == 0) __NOP();//Check the flags
		USART2->TDR = *pSrc;						//Transmit data
		pSrc++;
	}
	
	return len;
}

typedef struct {
	uint8_t POWER			:1;
	uint8_t READY			:1;
	uint8_t PROFILE			:1;
	uint8_t DOOR			:1;
	uint8_t OVERHEAT		:1;
	uint8_t END			:1;
	uint8_t FIRE			:1;
	uint8_t HEATER			:1;
} statusFlags_type;

typedef struct {
	uint8_t filterSelect		:1;					///< 1 = 50Hz, 0 = 60Hz
	uint8_t faultStatusClear	:1;					///< 1 = Clear (auto-clear)
	uint8_t faultDetectionCnf	:2;					///< Fault-Detection Cycle Control Bits
	uint8_t threeWire		:1;					///< 1 = 3-wire RTD, 0 = 2-wire or 4-wire
	uint8_t oneShot			:1;					///< 1 = 1-shot (auto-clear)
	uint8_t convMode		:1;					///< mode 1 = Auto, 0 = Normally off
	uint8_t VBIAS			:1;					///< 1 = ON, 0 = OFF
} maxConf_type;

typedef struct {
	uint8_t __reserv		:2;	
	uint8_t OVUV			:1;					///< Overvoltage/undervoltage fault
	uint8_t RTDIN_L085      	:1;					///< RTDIN- < 0.85 x VBIAS (FORCE- open)
	uint8_t REFIN_L085		:1;					///< REFIN- < 0.85 x VBIAS (FORCE- open)
	uint8_t REFIN_H085		:1;					///< REFIN- > 0.85 x VBIAS
	uint8_t RTD_L			:1;					///< RTD Low Threshold
	uint8_t RTD_H			:1;					///< RTD High Threshold
} maxFault_type;

typedef struct {
	maxConf_type			confReg;
	uint8_t				RTDH;
	uint8_t				RTDL;
	uint8_t				HFltThrH;
	uint8_t				HFltThrL;
	uint8_t				LFltThrH;
	uint8_t				LFltThrL;
	maxFault_type			faultReg;
} maxRegMap_type;

typedef enum {
	statusOk = 0,
	openDoor,
	overHeat
}status_type;

typedef struct {
	uint16_t			Tgi;
	uint16_t			Tmi;
	status_type			com;
} uarttx_type;

uarttx_type		uartTXMap;
maxRegMap_type		maxRegMap;
PID_GRANDO_F_CONTROLLER pidHeater;
statusFlags_type	statusFlags;

static float Tmeas, Tgoal, dT;
static int tper, tout;

#define LED_OPEN_DOOR() GPIOE->BSRR |= GPIO_BSRR_BS8				//OPEN door
#define LED_CLOSE_DOOR() GPIOE->BSRR |= GPIO_BSRR_BR8				//CLOSE door

int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	initGpios();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	
	PID_GRANDO_F_init(&pidHeater);
	
	RCC->APB1ENR1	|= RCC_APB1ENR1_TIM4EN;					//Enable clock for delay timer
	RCC->APB1ENR1	|= RCC_APB1ENR1_TIM3EN;					//Enable clock for sync timer
	TIM3->PSC	= F_APB1/1000000+1;
	TIM3->ARR	= 1000000/cycfreq;					//Period
	TIM3->EGR	|= TIM_EGR_UG;
	TIM3->CR1	|= TIM_CR1_OPM;
	
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB;
	EXTI->IMR1	|= EXTI_IMR1_IM10;					//Enable interrupt
	EXTI->RTSR1	|= EXTI_RTSR1_RT10;					//Rising edge
	EXTI->FTSR1	|= EXTI_FTSR1_FT10;					//Falling edge
	NVIC_SetPriority(EXTI15_10_IRQn, 15);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	maxRegMap.confReg.filterSelect = 1;
	maxRegMap.confReg.faultStatusClear = 1;
	maxRegMap.confReg.faultDetectionCnf = 0;
	maxRegMap.confReg.threeWire = 0;
	maxRegMap.confReg.oneShot = 0;
	maxRegMap.confReg.convMode = 0;
	maxRegMap.confReg.VBIAS = 1;
	max_write(0, (uint8_t*)&maxRegMap.confReg, 1);				//Configure
	Delay_us(10000);
	maxRegMap.confReg.convMode = 1;
	max_write(0, (uint8_t*)&maxRegMap.confReg, 1);
	
	uint8_t b1Cnt, b2Cnt, varcycfreq;
	uint16_t RLSB, dRLSB, Tu, Td, tuart;
	uint32_t vartstart, vartsp, vartprm, vartmp, vartpk1, vartpk2, vartpm, vartend;
	uint32_t tact, tpre, tper, tdb, tgen, tdnt;
	
	varcycfreq = cycfreq;
	vartstart = tstart;
	vartstart = vartstart*varcycfreq;
	vartsp = tsp;
	vartsp = vartsp*varcycfreq;
	vartprm = tprm;
	vartprm = vartprm*varcycfreq;
	vartmp = tmp;
	vartmp = vartmp*varcycfreq;
	vartpk1 = tpk1;
	vartpk1 = vartpk1*varcycfreq;
	vartpk2 = tpk2;
	vartpk2 = vartpk2*varcycfreq;
	vartpm = tpm;
	vartpm = vartpm*varcycfreq;
	vartend = tend;
	vartend = vartend*varcycfreq;
		
	tact = 0;
	uartTXMap.Tmi = 0;
	uartTXMap.Tgi = 0;
	uartTXMap.com = statusOk;
	tuart = 0;
	tper = 0;
	tout = 0;
	tpre = 0;
	tgen = 0;
	Tgoal = Toff;
	tdnt = 0;
	
	while (1) {
		while ((TIM3->CR1 & TIM_CR1_CEN) != 0) __NOP();				//Syncronize
		TIM3->CR1	|= TIM_CR1_CEN;						//Set the timer
		max_read();								//Get register data from MAX
		RLSB = maxRegMap.RTDL | maxRegMap.RTDH<<8;
		dRLSB = RLSB - ((int)RLSB0C);
		Tmeas = dRLSB / (Talpha*RLSB0C);
		//Clear OV/UV flag
		if (maxRegMap.faultReg.OVUV) {
			maxRegMap.confReg.faultStatusClear = 1;
			max_write (0, (uint8_t*)&maxRegMap.confReg, 1);
		}
		//If RTD is open/short
		if ((maxRegMap.faultReg.RTD_H || maxRegMap.faultReg.RTD_L) != 0) {
			maxRegMap.confReg.faultStatusClear = 1;
			max_write (0, (uint8_t*)&maxRegMap.confReg, 1);
			statusFlags.POWER = 0;
		}
		//Sensing the ON button
		if ((gppin_get (GP_Button_ON)) == 0) {
			b1Cnt++;
		} else if (b1Cnt != 0) {
			if ((b1Cnt > 2) && (statusFlags.POWER == 0)) {
				statusFlags.POWER = 1;
				statusFlags.END = 0;
				pidHeater.param.Kp = 0.125;
				pidHeater.param.Ki = 0;
				pidHeater.param.Kd = 0;
			}
			b1Cnt = 0;
		}
		if (statusFlags.POWER == 1) {
			tgen++;
			//Sensing the START button
			if ((gppin_get (GP_Button_START)) == 0) {			//Sense the button
				b2Cnt++;
			} else if (b2Cnt != 0) {					//If pressed
				if ((b2Cnt > 2) && (statusFlags.READY == 1)) {		//Anti-bounce
					statusFlags.PROFILE = 1;			//Enable the regulator & set the profile flag
					statusFlags.READY = 0;				//Clear the ready flag
					pidHeater.param.Kp = 1;				//Set the P coefficient
					pidHeater.param.Ki = 0.0005;
					pidHeater.param.Kd = 0.5;
				}
				b2Cnt = 0;
			}
			//Regulator configuration
			if (statusFlags.PROFILE != 0) {			
				if ((tact >= vartstart) && (tact <= vartsp)) {		//Time intervals
					tdb = vartstart;
					tper = vartsp - vartstart;
					Tu = Tsmin;
					Td = T0;
				}
				if ((tact >= vartsp) && (tact <= vartprm)) {
					tdb = vartsp;
					tper = vartprm - vartsp;
					Tu = Tsmax;
					Td = Tsmin;
				}
				if ((tact >= vartprm) && (tact <= vartmp)) {
					tdb = vartprm;
					tper = vartmp - vartprm;
					Tu = Tl;
					Td = Tsmax;
				}
				if ((tact >= vartmp) && (tact <= vartpk1)) {
					tdb = vartmp;
					tper = vartpk1 - vartmp;
					Tu = Tp;
					Td = Tl;
				}
				if ((tact >= vartpk1) && (tact <= vartpk2)) {
					tdb = vartpk1;
					tper = vartpk2 - vartpk1;
					Tu = Td = Tp;
				}
				if ((tact >= vartpk2) && (tact <= vartpm)) {
					tdb = vartpk2;
					tper = vartpm - vartpk2;
					Tu = Tl;
					Td = Tp;
				}
				if ((tact >= vartpm) && (tact <= vartend)) {
					tdb = vartpm;
					tper = vartend - vartpm;
					Tu = T0;
					Td = Tl;
				}
				Tgoal = ((float)(tact-tdb)/tper)*(Tu - Td) + Td;	//Calculate the goal temperature
				tact++;							//Thermoprofile counter
				if (tact > vartend) {
					statusFlags.PROFILE = 0;			//Stop condition
					statusFlags.END = 1;
					statusFlags.POWER = 0;
					statusFlags.DOOR = 0;
					statusFlags.OVERHEAT = 0;
					Tgoal = Toff;
					tact = 0;
					tgen = 0;
					tdnt = 0;
				}
			} else {
				Tgoal = T0;						//Preheat temperature
				tact = 0;
			}
			//Delta T calculation
			dT = Tmeas - Tgoal;
			//Preheat threshold
			if ((tpre >= T5MIN) && (statusFlags.READY == 0)) {
				statusFlags.READY = 1;
				tpre = 0;
			} else if ((statusFlags.PROFILE == 0) && (statusFlags.READY == 0) && (dT > -5) && (dT < 5)) {
				tpre++;
			} else if ((statusFlags.PROFILE == 0) && (statusFlags.READY == 0) && (tpre > 0)) {
				tpre++;
			}
			//Downtime
			if (statusFlags.READY != 0) tdnt++;
			//FIRE
			if (Tmeas > 260) {
				statusFlags.FIRE = 1;
				statusFlags.DOOR = 1;
				statusFlags.END = 1;
			}
			//Door
			if (((dT - Thist) > 1) && (statusFlags.POWER != 0)) {
				statusFlags.DOOR = 1;
			} else if ((dT + Thist) < 1) {
				statusFlags.DOOR = 0;
			}
			//Overheat
			if (((dT - Thist) > 10) && (statusFlags.POWER != 0)) {
				statusFlags.OVERHEAT = 1;
			} else if ((dT + Thist) < 10){
				statusFlags.OVERHEAT = 0;
			}
			//Data transmition via UART
			uartTXMap.Tmi = (int)Tmeas;					//Integer for UART TX
			uartTXMap.Tgi = (int)Tgoal;
			tuart++;
			if (tuart > varcycfreq) {					//Transmit data via UART once per 1 s
				tuart = 0;
				uartTXMap.com = statusOk;				//None
				if (statusFlags.DOOR != 0) uartTXMap.com = openDoor;	//Open door
				if (statusFlags.OVERHEAT != 0) uartTXMap.com = overHeat;//Overheat
				printf ("%u, %u, %u, %u, %u, %u \r", tgen/200, tdnt/200, tact/200, uartTXMap.Tgi, uartTXMap.Tmi, uartTXMap.com);
			}
		}
		(statusFlags.POWER	!=0)?(gppin_set (GP_LED_POWER)):(gppin_reset (GP_LED_POWER));
		(statusFlags.READY	!=0)?(gppin_set (GP_LED_READY)):(gppin_reset (GP_LED_READY));
		(statusFlags.PROFILE	!=0)?(gppin_set (GP_LED_PROFILE)):(gppin_reset (GP_LED_PROFILE));
		(statusFlags.DOOR	!=0)?(gppin_set (GP_LED_DOOR)):(gppin_reset (GP_LED_DOOR));
		(statusFlags.OVERHEAT	!=0)?(gppin_set (GP_LED_OVERHEAT)):(gppin_reset (GP_LED_OVERHEAT));
		(statusFlags.END	!=0)?(gppin_set (GP_LED_END)):(gppin_reset (GP_LED_END));
		if (statusFlags.FIRE) {
			NVIC_DisableIRQ(EXTI15_10_IRQn);
			gppin_set (GP_LED_FIRE);
			gppin_reset (GP_TRIAC_EN);
			while (1) __NOP ();
		}
	}
}

__irq void EXTI15_10_IRQHandler (void) {
	//Check the interrupt flag!
	if ((gppin_get (GP_PHASE_SYNC)) != 0) {						//Rising front
		if (tper == 0) {
			pidHeater.term.Ref = Tgoal;					//Goal temperature
			pidHeater.term.Fbk = Tmeas;					//Measured temperature
			PID_GRANDO_F_FUNC(&pidHeater);					//Calc output value
			tout = (int)(pidHeater.term.Out + 0.5);				//Output width
			if (tout > 0) {
				gppin_set (GP_TRIAC_EN);				//Enable the triac
				gppin_set (GP_LED_HEATER);				//Display this on LED
			}
		}
		tper++;
	} else {									//Falling front
		if (tper >= tout) {							//Rewrite it to start at the rising front!
			gppin_reset (GP_TRIAC_EN);					//Disable the triac
			gppin_reset (GP_LED_HEATER);					//Turn off LED
		}
		if (tper >= (WITH_REG - 1)){
			tper = 0;
		}
	}
	EXTI->PR1	= EXTI_PR1_PIF10;						//Clear the flag
}

//UART transmitter routine
void UARTTX (uint16_t *src, uint16_t len) {
	uint16_t *pSrc = src;								//Data to transfer
	uint16_t *pEnd = pSrc + len;
	while(pSrc < pEnd){
		while ((USART2->ISR & USART_ISR_TXE) && (USART2->ISR & USART_ISR_TC) == 0) __NOP();//Check the flags
		USART2->TDR = *pSrc;							//Transmit data
		pSrc++;
	}
}

//Read register file routine from MAX31865
void max_read (void) {
	uint8_t *dst, n;
	
	dst = (uint8_t*)&maxRegMap;
	for(n = 0; n < 8; n++){
		gppin_reset(GP_MAX31865_CSn);						//Open transfer
		SPItrx(n);								//Set address
		*dst = SPItrx(0);							//Transmit data
		gppin_set(GP_MAX31865_CSn);						//Close transfer
		dst++;
	}
}

//Write routine for MAX31865
void max_write (uint8_t addr, uint8_t *src, uint8_t len){
	uint8_t *pSrc = src;								//Data to transfer
	uint8_t *pEnd = pSrc + len;
	while(pSrc < pEnd){
		gppin_reset(GP_MAX31865_CSn);						//Open transfer
		SPItrx(addr | 0x80);							//Set address
		SPItrx(*pSrc);								//Transmit data
		gppin_set(GP_MAX31865_CSn);						//Close transfer
		pSrc++;
		addr++;
	}
}

//SPI tranceiver handler
char SPItrx (char tbyte) {
	char i, mosi, miso, rbyte;
	//Transfer cycle
	for (i = 8; i >= 1; i--) {
		mosi = (tbyte & 0x80)>>7;
		tbyte = tbyte<<1;
		gppin_set(GP_MAX31865_MOSI);						//MOSI
		if (mosi == 0) gppin_reset(GP_MAX31865_MOSI);
		gppin_set(GP_MAX31865_CLK);						//CLK
		Delay_us(1);
		miso = (GPIOC->IDR & 1);						//MISO
		rbyte = rbyte<<1;							//Received byte
		rbyte |= miso;
		gppin_reset(GP_MAX31865_CLK);
		Delay_us(1);
	}
	return rbyte;
}

//Delay in microseconds
void Delay_us (uint16_t time) {
	TIM4->PSC	= F_APB1/1000000+1;
	TIM4->ARR	= time;
	TIM4->EGR	|= TIM_EGR_UG;
	TIM4->CR1	|= TIM_CR1_OPM;
	TIM4->CR1	|= TIM_CR1_CEN;
	while ((TIM4->CR1 & TIM_CR1_CEN) != 0) __NOP();
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
