/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
/* USER CODE BEGIN Includes */
#include "configure.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

osThreadId defaultTaskHandle;
osThreadId analyzeHandle;
osThreadId i2cTransHandle;
osMessageQId qI2cTransHandle;
osMessageQId qAnalyzeComHandle;
osMailQId qAnalyzeConfHandle;
//osSemaphoreId semI2cBusyHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint16_t outOverload[3] = { GPIO_PIN_12 , GPIO_PIN_11, GPIO_PIN_10};	//номера GPIOA выводов, сигнализирующих о перегрузки по току каналов питания МАД
static uint16_t bufData[NUM_ELEM_BUF_DATA] = {1};	//буфер данных, заполняемый показаниями АЦП
static uint16_t* addrBegBufData = bufData;	//адрес начала буфера данных
static uint16_t* addrMeanBufData = bufData + NUM_ELEM_BUF_DATA/2;	//адрес середины буфера данных
static uint16_t* addrEndBufData = bufData + NUM_ELEM_BUF_DATA;	//адрес последнего элемента буфера данных
static struct Sample* addrStopPoint;	//адрес следующей точки остановки мониторинговой программы
static struct Sample* addrCurPoint;	//адрес текущего положения мониторинговой программы
static uint32_t countOfLead;	//счётчик опережения. Используется для определния насколько сегментов мониторинг данных отстал от их заполнения
struct StatusMonitData stMonitData;	//структура, управляющая режимом сбора данных
static struct SecConfigure bufConf;	//буфер, в котором сохраняется полученное по I2C новое значение структуры безопасности
static uint32_t bufCom;	//буфер, в котором сохраняется полученная по I2C очередная команда
static uint32_t identBlockDataComForI2c;	//идентификатор блока данных, пришедшего по i2c
static uint32_t bufI2cTrans = 1;
static bool i2c_is_trans = true;	//флаг режима работы i2c, true - передатчик
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void Analyze(void const * argument);
void I2cTrans(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void initMonitorSwitchParams(struct MonitSwitchModeParams* pMonitSwMPs, struct SecConfigure* pConf); //инициализация pMonitSwMPs в соответствие с новой конфигурацией безопасности
void initTracking(struct MonitSwitchModeParams* pMonitSwMPs, struct SecConfigure* pConf);	//предварительная подготовка перед стартом мониторинга
enum SWITCH_MODE checkCurrentMads(uint16_t val, const struct MCur* pMCur, struct MonitSwitchMode* pMonit);	//управление переключением режима безопасности параметра ток МАД
enum SWITCH_MODE checkVoltage(uint16_t val, const struct Volt* pVol, struct MonitSwitchMode* pMonit);	//управление переключением режима безопасности параметра напряжение
void monitoringSample(const struct Sample* pSample, const struct SecConfigure* pConf, struct MonitSwitchModeParams* mSwitchModePs);	//мониторинг одного отсчёта
void startTracking(void);	//применяется при старте/рестарте процедуры отслеживания параметров питания
void sendMesOverrunAdc(void);	//сообщить мониторинговой подпрограмме о перегрузке АЦП
void sendMesSpiNotTime(void);	//сообщить мониторинговой подпрограмме о том, что передача данных  в SPI порт не успевает вовремя закончится при текущей скорости потока
void setStMonitData(struct StatusMonitData st); //установить новое значение структуры, управляющей режимом сбора данных
void enableSpiTrans(void);	//разрешение передавать данные по spi
void disableSpiTrans(void);	//запрет передавать данные по spi
void countOfLeadSetZero(void);	//обнуление счётчика опережения
void countOfLeadInc(void);	//инкрементирование счётчика опережения
void countOfLeadDec(void); //декрементировать счётчик опережения
uint32_t countOfLeadRead(void); //прочитать счётчик опережения
struct Sample* getCurAddrDmaAdc(void);	//сообщает текущий адрес заполнения DMA ADC
void setStopPointAtBegSegment(void);	//установить точку останова в начало следующего сегмента
void setTestPinPB1(void);
void setTestPinPA8(void);
void temp_setSecConfigure(struct SecConfigure* sec);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef User_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint16_t *pData, uint16_t Size) {	//передача данных в SPI используя DMA без прерываний
	if((hdma_spi1_tx.Instance->CNDTR != 0)	//проверка, что SPI передача данных уже не активна
			|| ((hspi->Instance->SR & SPI_SR_FTLVL) != SPI_FRLVL_EMPTY)
			|| ((hspi->Instance->SR & SPI_FLAG_BSY) !=  RESET ) )
		return HAL_ERROR;
	__HAL_DMA_DISABLE(&hdma_spi1_tx);
	/* Configure DMA Channel data length */
	hdma_spi1_tx.Instance->CNDTR = Size;
	/* Configure DMA Channel source address */
	hdma_spi1_tx.Instance->CMAR = (uint32_t)pData;
	__HAL_DMA_ENABLE(&hdma_spi1_tx);
	return HAL_OK;
}

void ADCIntCallback(void) {
	if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) {
		sendMesOverrunAdc();
	}
}

void getInfoBufConf(uint8_t** addr, size_t* pSize) {
	*addr = (uint8_t*)&bufConf;
	*pSize = sizeof(bufConf);
}

void getInfoBufCom(uint8_t** addr, size_t* pSize){
	*addr = (uint8_t*)&bufCom;
	*pSize = sizeof(bufCom);
}

void setIdentBlockDataComForI2c(uint32_t ident) {
	identBlockDataComForI2c = ident;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	countOfLeadInc();
	if(stMonitData.isTransDataToSpi != true)
		return;
	if(User_SPI_Transmit_DMA(&hspi1, addrMeanBufData, NUM_ELEM_BUF_DATA/2) == HAL_ERROR) {
		sendMesSpiNotTime();
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	countOfLeadInc();
	if(stMonitData.isTransDataToSpi != true)
		return;
	if(User_SPI_Transmit_DMA(&hspi1, addrBegBufData, NUM_ELEM_BUF_DATA/2) == HAL_ERROR) {
		sendMesSpiNotTime();
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(i2c_is_trans == false)
		HAL_I2C_Slave_Receive_IT(hi2c, (uint8_t*)&bufCom, sizeof(bufCom));
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	struct SecConfigure* pConf;
	if(hi2c->XferSize == 0) {
		switch(identBlockDataComForI2c) {
		case I2C_ADDR_COM:
			osMessagePut(qAnalyzeComHandle, bufCom, 0);
			break;
		case I2C_ADDR_CONF:
			pConf = (struct SecConfigure*)osMailAlloc(qAnalyzeConfHandle, 0);
			if(pConf != NULL) {
				*pConf = bufConf;
				osMailPut(qAnalyzeConfHandle, pConf);
			}
		}
	}
	i2c_is_trans = true;
}
/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of semI2cBusy */
	//osSemaphoreDef(semI2cBusy);
	//semI2cBusyHandle = osSemaphoreCreate(osSemaphore(semI2cBusy), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */

	/* definition and creation of analyze */
	osThreadDef(analyze, Analyze, osPriorityNormal, 0, 256);
	analyzeHandle = osThreadCreate(osThread(analyze), NULL);

	/* definition and creation of i2cTrans */
	osThreadDef(i2cTrans, I2cTrans, osPriorityAboveNormal, 0, 128);
	i2cTransHandle = osThreadCreate(osThread(i2cTrans), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of qI2cTrans */
	osMessageQDef(qI2cTrans, 20, uint32_t);
	qI2cTransHandle = osMessageCreate(osMessageQ(qI2cTrans), NULL);

	/* definition and creation of qAnalyzeCom */
	osMessageQDef(qAnalyzeCom, 10, uint32_t);
	qAnalyzeComHandle = osMessageCreate(osMessageQ(qAnalyzeCom), NULL);

	/* definition and creation of qAnalyzeConf */
	osMailQDef(qAnalyzeConf, 1, struct SecConfigure);
	qAnalyzeConfHandle = osMailCreate(osMailQ(qAnalyzeConf), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */


	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 5;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = OVR_DATA_PRESERVED;
	HAL_ADC_Init(&hadc1);

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 2;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 3;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 4;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x0000020B;
	hi2c1.Init.OwnAddress1 = I2C_ADDR_CONF;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLED;
	hi2c1.Init.OwnAddress2 = I2C_ADDR_COM;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	HAL_I2C_Init(&hi2c1);

	/**Configure Analogue filter
	 */
	HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	hspi1.Init.CRCPolynomial = 10;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
	HAL_SPI_Init(&hspi1);
	__HAL_SPI_ENABLE(&hspi1);

}

/** 
 * Enable DMA controller clock
 */
void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	//HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 6, 0);
	//HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__GPIOF_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : IN_Pin */
	GPIO_InitStruct.Pin = IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OUT3_Pin OUT2_Pin OUT1_Pin */
	GPIO_InitStruct.Pin = OUT3_Pin|OUT2_Pin|OUT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure test GPIO pins */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void initMonitorSwitchParams(struct MonitSwitchModeParams* pMonitSwMPs, struct SecConfigure* pConf) {
	union I2cMes mes;
	for(int i = 0; i < 3; i++) {
		pMonitSwMPs->mCur[i].isSecMode = true;
		pMonitSwMPs->mCur[i].whileInOthMode = pConf->mads[i].time_overh;
	}
	pMonitSwMPs->v_48.isSecMode = false;
	pMonitSwMPs->v_48.whileInOthMode = pConf->v_48.time_overh;
	pMonitSwMPs->v_300.isSecMode = false;
	pMonitSwMPs->v_300.whileInOthMode = pConf->v_300.time_overh;
	for(int i = 0; i < 3; i++)
		HAL_GPIO_WritePin(GPIOA, outOverload[i], GPIO_PIN_RESET);
	mes.frame.id = MES_VOLT_300V_TO_UNSEC_MODE;
	mes.frame.arg = 0;
	osMessagePut(qI2cTransHandle, mes.blob, 0);
	mes.frame.id = MES_VOLT_48V_TO_UNSEC_MODE;
	osMessagePut(qI2cTransHandle, mes.blob, 0);
}

void initTracking(struct MonitSwitchModeParams* pMonitSwMPs, struct SecConfigure* pConf) {
	HAL_ADC_Stop_DMA(&hadc1);
	addrStopPoint = (struct Sample*)addrBegBufData;
	addrCurPoint = (struct Sample*)addrBegBufData;
	initMonitorSwitchParams(pMonitSwMPs, pConf);
	struct StatusMonitData st;
	st.isTransDataToSpi = false;
	st.isValidConfig = true;
	setStMonitData(st);
	countOfLead = 0;	//обнуление счётчика опережения
	while((hdma_spi1_tx.Instance->CNDTR != 0)	//ожидание завершения активности SPI передачи данных
			|| ((hspi1.Instance->SR & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY)
			|| ((hspi1.Instance->SR & SPI_FLAG_BSY) !=  RESET ) ) {
		osDelay(3);
	}
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)addrCurPoint, NUM_ELEM_BUF_DATA);
}

enum SWITCH_MODE checkCurrentMads(uint16_t val, const struct MCur* pMCur, struct MonitSwitchMode* pMonit) {
	bool is_SecValue = (val <= pMCur->max);
	if (is_SecValue == pMonit->isSecMode) {
		pMonit->whileInOthMode = pMCur->time_overh;
	} else {
		if(--pMonit->whileInOthMode <= 0) {
			pMonit->isSecMode = is_SecValue;
			pMonit->whileInOthMode = pMCur->time_overh;
			if(is_SecValue)
				return MOVE_TO_SEC_MODE;
			else
				return MOVE_TO_UNSEC_MODE;
		}
	}
	return NO_CHANGE;
}

enum SWITCH_MODE checkVoltage(uint16_t val, const struct Volt* pVol, struct MonitSwitchMode* pMonit) {
	bool is_SecValue = (val <= pVol->max && val >= pVol->min);
	if (is_SecValue == pMonit->isSecMode) {
		pMonit->whileInOthMode = pVol->time_overh;
	} else {
		if(--pMonit->whileInOthMode <= 0) {
			pMonit->isSecMode = is_SecValue;
			pMonit->whileInOthMode = pVol->time_overh;
			if(is_SecValue)
				return MOVE_TO_SEC_MODE;
			else
				return MOVE_TO_UNSEC_MODE;
		}
	}
	return NO_CHANGE;
}

void monitoringSample(const struct Sample* pSample, const struct SecConfigure* pConf, struct MonitSwitchModeParams* mSwitchModePs) {
	static enum SWITCH_MODE swMode;
	static union I2cMes mes;
	//проверка токов потребления МАД на перегрузку
	for(int i = 0; i < 3; i++){
		swMode = checkCurrentMads(pSample->iMad[i], &pConf->mads[i], &mSwitchModePs->mCur[i]);
		switch(swMode) {
		case MOVE_TO_SEC_MODE:
			HAL_GPIO_WritePin(GPIOA, outOverload[i], GPIO_PIN_SET);
			break;
		case MOVE_TO_UNSEC_MODE:
			HAL_GPIO_WritePin(GPIOA, outOverload[i], GPIO_PIN_RESET);
			break;
		case NO_CHANGE:
			break;
		}
	}
	//проверка мониторируемого напряжения 48В на соответствие безопасному режиму
	swMode = checkVoltage(pSample->u48v, &pConf->v_48, &mSwitchModePs->v_48);
	switch(swMode) {
	case MOVE_TO_SEC_MODE:
		mes.frame.id = MES_VOLT_48V_TO_SEC_MODE;
		mes.frame.arg = pSample->u48v;
		osMessagePut(qI2cTransHandle, mes.blob, 0);
		break;
	case MOVE_TO_UNSEC_MODE:
		mes.frame.id = MES_VOLT_48V_TO_UNSEC_MODE;
		mes.frame.arg = pSample->u48v;
		osMessagePut(qI2cTransHandle, mes.blob, 0);
		break;
	case NO_CHANGE:
		break;
	}
	//проверка мониторируемого напряжения 300В на соответствие безопасному режиму
	swMode = checkVoltage(pSample->u300v, &pConf->v_300, &mSwitchModePs->v_300);
	switch(swMode) {
	case MOVE_TO_SEC_MODE:
		mes.frame.id = MES_VOLT_300V_TO_SEC_MODE;
		mes.frame.arg = pSample->u300v;
		osMessagePut(qI2cTransHandle, mes.blob, 0);
		break;
	case MOVE_TO_UNSEC_MODE:
		mes.frame.id = MES_VOLT_300V_TO_UNSEC_MODE;
		mes.frame.arg = pSample->u300v;
		osMessagePut(qI2cTransHandle, mes.blob, 0);
		break;
	case NO_CHANGE:
		break;
	}
}

void startTracking(void) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)addrBegBufData, NUM_ELEM_BUF_DATA);
}

void sendMesOverrunAdc(void) {
	union I2cMes mes;
	mes.frame.id = MES_OVR_ADC;
	osMessagePut(qAnalyzeComHandle, mes.blob, 0);
}

void sendMesSpiNotTime(void) {
	union I2cMes mes;
	mes.frame.id = MES_SPI_TRANS_NOT_TIME;
	osMessagePut(qAnalyzeComHandle, mes.blob, 0);
}

void setStMonitData(struct StatusMonitData st) {
	taskENTER_CRITICAL();
	stMonitData = st;
	taskEXIT_CRITICAL();
}

void enableSpiTrans(void) {
	taskENTER_CRITICAL();
	stMonitData.isTransDataToSpi = true;
	taskEXIT_CRITICAL();
}

void disableSpiTrans(void) {
	taskENTER_CRITICAL();
	stMonitData.isTransDataToSpi = false;
	taskEXIT_CRITICAL();
}

void countOfLeadSetZero(void) {
	taskDISABLE_INTERRUPTS();
	countOfLead = 0;
	taskENABLE_INTERRUPTS();
}

void countOfLeadInc(void) {
	taskDISABLE_INTERRUPTS();
	if(countOfLead < MAX_VAL_COUNT_LEAD)
		++countOfLead;
	taskENABLE_INTERRUPTS();
}

void countOfLeadDec(void) {
	taskDISABLE_INTERRUPTS();
	if(countOfLead > 0)
		--countOfLead;
	taskENABLE_INTERRUPTS();
}

uint32_t countOfLeadRead(void) {
	return countOfLead;
}

struct Sample* getCurAddrDmaAdc(void) {
	uint32_t n = NUM_ELEM_BUF_DATA - hdma_adc1.Instance->CNDTR;
	return (struct Sample*)addrBegBufData + n/5;
}

void setStopPointAtBegSegment(void) {
	if(addrCurPoint < (struct Sample*)addrMeanBufData)
		addrStopPoint = (struct Sample*)addrMeanBufData;
	else if(addrCurPoint < (struct Sample*)addrEndBufData)
		addrStopPoint = (struct Sample*)addrEndBufData;
	else {
		addrCurPoint = (struct Sample*)addrBegBufData;
		addrStopPoint = (struct Sample*)addrMeanBufData;
	}
}

void setTestPinPB1(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

void setTestPinPA8(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void temp_setSecConfigure(struct SecConfigure* sec) {
	sec->v_48.max = 65535;
	sec->v_48.min = 0;
	sec->v_48.time_overh = 10;
	sec->v_300.max = 65535;
	sec->v_300.min = 0;
	sec->v_300.time_overh = 10;
	for(int i = 0; i < 3; i++) {
		sec->mads[i].max = 65535;
		sec->mads[i].time_overh = 10;
	}
	stMonitData.isValidConfig = true;
}
/* USER CODE END 4 */

/* Analyze function */
void Analyze(void const * argument)
{
	/* USER CODE BEGIN Analyze */
	static struct SecConfigure confSecure;
	static struct MonitSwitchModeParams mSwitchModePs;
	static union I2cMes mes;
	static uint32_t tick;
	static osEvent event;
	static struct SecConfigure* pConf;
	countOfLeadSetZero();
	tick = osKernelSysTick() + osKernelSysTickMicroSec(TIME_TRANS_REQ_CONFIGURE);
	stMonitData.isValidConfig = false;

	//код, тестирующий начало мониторинга
	/*temp_setSecConfigure(&confSecure);
	initTracking(&mSwitchModePs, &confSecure);
	enableSpiTrans();
	startTracking();*/

	for(;;) {
		//чтение из очереди конфигурации
		event = osMailGet(qAnalyzeConfHandle, 0);
		if(event.status == osEventMail) {
			pConf = (struct SecConfigure*)event.value.p;
			confSecure = *pConf;
			osMailFree(qAnalyzeConfHandle, pConf);
			initTracking(&mSwitchModePs, &confSecure);
			startTracking();
			setTestPinPA8();
		}
		//проверка находится ли программа в начале процедуры запуска
		if(!stMonitData.isValidConfig) {
			if(osKernelSysTick() >= tick) {
				tick = tick + osKernelSysTickMicroSec(TIME_TRANS_REQ_CONFIGURE);
				mes.frame.id = MES_GET_CONFIGURE;
				mes.frame.arg = TIME_TRANS_REQ_CONFIGURE/1000;
				osMessagePut(qI2cTransHandle, mes.blob, 0);
			}
			continue;
		}
		//чтение сообщений из очереди команд
		event = osMessageGet (qAnalyzeComHandle, 0);
		if(event.status == osEventMessage) {
			mes.blob = event.value.v;
			switch(mes.frame.id) {
			case MES_OVR_ADC:
			case MES_SPI_TRANS_NOT_TIME:
				initTracking(&mSwitchModePs, &confSecure);
				osMessagePut(qI2cTransHandle, mes.blob, 0);
				break;
			case MES_DISABLE_TRANS_MONIT_DATA:
				disableSpiTrans();
				break;
			case MES_ENABLE_TRANS_MONIT_DATA:
				enableSpiTrans();
				setTestPinPB1();
				break;
			case MES_START_MONIT_DATA:
				startTracking();
				break;
			}
		}
		//проверка достиг ли счётчик адресов начала полубуфера
		if(addrCurPoint == (struct Sample*)addrBegBufData || addrCurPoint == (struct Sample*)addrMeanBufData)
			countOfLeadDec();
		//проверка достигнута ли точка останова
		if(addrCurPoint >= addrStopPoint) {
			switch(countOfLeadRead()) {
			case 0:
				addrStopPoint = getCurAddrDmaAdc();
				break;
			case 1:
				setStopPointAtBegSegment();
				break;
			case 2:
				if(addrCurPoint > getCurAddrDmaAdc()) {
					addrStopPoint = (struct Sample*)addrEndBufData;
					break;
				}
				/* no break */
			case 3:
				mes.frame.id = MES_MONIT_NOT_TIME;
				osMessagePut(qI2cTransHandle, mes.blob, 0);
				taskENTER_CRITICAL();
				addrStopPoint = getCurAddrDmaAdc();
				if(addrStopPoint < (struct Sample*)addrMeanBufData)
					addrCurPoint = (struct Sample*)addrBegBufData;
				else
					addrCurPoint = (struct Sample*)addrMeanBufData;
				countOfLeadSetZero();
				taskEXIT_CRITICAL();
				break;
			}

		} else {
			monitoringSample(addrCurPoint, &confSecure, &mSwitchModePs);
			++addrCurPoint;
		}
	}
	/* USER CODE END Analyze */
}

/* I2cTrans function */
void I2cTrans(void const * argument)
{
	/* USER CODE BEGIN I2cTrans */
	/* Infinite loop */
	static osEvent event;
	HAL_StatusTypeDef status;
	for(;;)
	{
		//чтение сообщений из очереди команд
		event = osMessageGet (qI2cTransHandle, osWaitForever);
		if(event.status == osEventMessage) {
			do {
				while(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == SET || i2c_is_trans == false)
					osDelay(2);
				bufI2cTrans = event.value.v;
				status = HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_ADDR_HOST_MCU, (uint8_t*)&bufI2cTrans, 4);
			} while(status != HAL_OK);
		}
	}
	/* USER CODE END I2cTrans */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_9) {
		i2c_is_trans = false;
		if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != SET)
			HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&bufCom, sizeof(bufCom));
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if(hi2c == &hi2c1)
		i2c_is_trans = true;
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
