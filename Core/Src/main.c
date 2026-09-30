/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "ssd1306.h"      // Ekran kütüphanesi
#include "ssd1306_fonts.h" // Yazı tipleri
#include <stdio.h>        // sprintf fonksiyonu için
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

const float ADC_MAX_VALUE = 4095.0f; // STM32F103C8T6'nin 12-bit çözünürlüğü (2^12 - 1)
const float VREF = 3.3f;            // besleme gerilimi

const float R_TOP = 1410000.0f; 	//R1+R2+R3 veya R4+R5+R6 (Kol direnci)
const float R_BOTTOM = 100000.0f;	//R7 = 100k
const float Rp = R_TOP + R_BOTTOM ; //Şasiye bağlanan eşdeğer ölçüm direnci


char lcd_buffer[20];	// Ekrana yazı yazdırmak için geçici hafıza

float insulation_resistance = 0.0f;

// --- Non-blocking ölçüm state machine ---
typedef enum {
    ISO_STATE_STARTUP_BEEP, // Açılış sesi (non-blocking bekleme)
    ISO_STATE_SELECT_HVP,   // HV+ hattını seç
    ISO_STATE_READ_HVP,     // Settle süresini bekle ve HV+ oku
    ISO_STATE_SELECT_HVN,   // HV- hattını seç
    ISO_STATE_READ_HVN,     // Settle süresini bekle ve HV- oku
    ISO_STATE_PROCESS,      // Hesapla, ekrana yaz, CAN gönder
    ISO_STATE_IDLE          // Bir sonraki döngüye kadar bekle
} IsoState_t;

static IsoState_t iso_state = ISO_STATE_STARTUP_BEEP;
static uint32_t   state_timer = 0;
#define STARTUP_BEEP_MS 200u  // Açılış buzzer süresi
#define ADC_SETTLE_MS   50u   // Röle/mux anahtarlaması sonrası bekleme süresi
#define CYCLE_IDLE_MS   50u   // Döngüler arası bekleme (CAN yükünü sınırlamak için)
#define HV_PRESENT_THRESHOLD_V 10.0f // Bu değerin altı: HV yok sayılır
#define SHORT_CIRCUIT_RATIO    0.2f  // current_v_system'in bu oranını aşan fark: fiziksel arıza
#define MIN_V_DIFF_V           0.05f // Direnç hesap. bölen alt sınırı (aşırı büyük değer önleme)

float v_plus;
float v_minus;

#define BATTERY_MAX_VOLTAGE   72.0f
#define ISO_LIMIT_OHM_PER_V   100.0f //Volt başına düşen minimum izolasyon direnci

// Rlimit = Vbat,max × 100 Ω/V
#define ISO_RESISTANCE_LIMIT  (BATTERY_MAX_VOLTAGE * ISO_LIMIT_OHM_PER_V) // 8400 Ω

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
float read_and_calculate_voltage(ADC_HandleTypeDef* hadc);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	CAN_TxHeaderTypeDef TxHeader = {0}; // CAN Mesaj Başlığı (Kime gidecek, kaç bayt vs.)
	uint32_t TxMailbox;           // Gönderim için posta kutusu
	uint8_t TxData[8];            // Gönderilecek 8 baytlık veri paketi

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */


  CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

  ssd1306_Init(); // Ekranı donanımsal olarak başlat
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();


  // 2. CAN Modülünü Başlat (BU ÇOK ÖNEMLİ!)
    // Bunu yapmazsan while döngüsündeki CAN mesajları gitmez.
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler(); // Başlatılamazsa hata döngüsüne girsin
    }

    // 3. Açılış Sesi (Opsiyonel - "Cihaz Açıldı" Sesi)
    // ESKİ KOD: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);  <-- ARTIK BU YANLIŞ!
    // YENİ KOD:
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); // Buzzer Öt
    state_timer = HAL_GetTick(); // STARTUP_BEEP state'i bunu okuyup 200ms sonra söndürecek


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	switch (iso_state)
	{
	case ISO_STATE_STARTUP_BEEP:
		if (HAL_GetTick() - state_timer >= STARTUP_BEEP_MS)
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); // Buzzer Sus
			iso_state = ISO_STATE_SELECT_HVP;
		}
		break;

	case ISO_STATE_SELECT_HVP:
		// 1. HV+ Testi (PB0 aktif)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		state_timer = HAL_GetTick();
		iso_state = ISO_STATE_READ_HVP;
		break;

	case ISO_STATE_READ_HVP:
		if (HAL_GetTick() - state_timer >= ADC_SETTLE_MS)
		{
			v_plus = read_and_calculate_voltage(&hadc1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			iso_state = ISO_STATE_SELECT_HVN;
		}
		break;

	case ISO_STATE_SELECT_HVN:
		// 2. HV- Testi (PB1 aktif)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		state_timer = HAL_GetTick();
		iso_state = ISO_STATE_READ_HVN;
		break;

	case ISO_STATE_READ_HVN:
		if (HAL_GetTick() - state_timer >= ADC_SETTLE_MS)
		{
			v_minus = read_and_calculate_voltage(&hadc1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			iso_state = ISO_STATE_PROCESS;
		}
		break;

	case ISO_STATE_PROCESS:
	{
		float current_v_system = v_plus + v_minus;

		// 3. İzolasyon Direnci Hesaplama
		//Tam formül için v_plus ve v_minus farkına bakılır.
		float v_diff = fabsf(v_plus - v_minus);

		/* Kısa devre / sert kaçak durumunda insulation_resistance hesaplanmaz */
		if (current_v_system < HV_PRESENT_THRESHOLD_V)
		{
			/* HV yok -> ölçüm geçersiz */
			insulation_resistance = 0.0f;
		}
		else if (v_diff > (SHORT_CIRCUIT_RATIO * current_v_system))
		{
			/* Fiziksel kısa devre / ciddi dengesizlik */
			insulation_resistance = 0.0f;
		}
		else
		{
			/* Normal durumda izolasyon direnci hesaplanır */
			if (v_diff < MIN_V_DIFF_V)
				v_diff = MIN_V_DIFF_V;

			insulation_resistance = Rp * ((current_v_system / v_diff) - 1.0f);
		}

		// V+ Değerini Yaz
		ssd1306_SetCursor(2, 0);
		sprintf(lcd_buffer, "HV+ : %.1f V", v_plus);
		ssd1306_WriteString(lcd_buffer, Font_7x10, White);

		// V- Değerini Yaz
		ssd1306_SetCursor(2, 12);
		sprintf(lcd_buffer, "HV- : %.1f V", v_minus);
		ssd1306_WriteString(lcd_buffer, Font_7x10, White);

		// İzolasyon Durumunu Yaz
		ssd1306_SetCursor(2, 24);
		if (insulation_resistance == 0.0f)
		{
			sprintf(lcd_buffer, "Res: SHORT");
		}
		else if (insulation_resistance >= 1000000.0f)
			sprintf(lcd_buffer, "Res: %.2f MOhm", insulation_resistance / 1000000.0f);
		else
			sprintf(lcd_buffer, "Res: %.1f kOhm", insulation_resistance / 1000.0f);

		ssd1306_WriteString(lcd_buffer, Font_7x10, White);

		// 4. Buzzer Kontrolü
		/* 1- Fiziksel kısa devre / ciddi dengesizlik */
		if (current_v_system > HV_PRESENT_THRESHOLD_V && v_diff > (SHORT_CIRCUIT_RATIO * current_v_system))
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		}
		/* 2- İzolasyon direnci hatası */
		else if (insulation_resistance < ISO_RESISTANCE_LIMIT)
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AKS_OUT_GPIO_Port, AKS_OUT_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AKS_OUT_GPIO_Port, AKS_OUT_Pin, GPIO_PIN_RESET);
		}

		// ==========================================
		// CAN BUS VERİ GÖNDERİMİ
		// ==========================================
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;
		TxHeader.StdId = 0x101;
		TxHeader.DLC = 5;                // 4 Byte Direnç + 1 Byte Durum

		memcpy(TxData, (uint8_t*)&insulation_resistance, 4);

		uint8_t status_code = 0;         // 0: Normal, 1: Düşük Direnç, 2: Kısa Devre/Hata
		if (insulation_resistance == 0.0f) status_code = 2;
		else if (insulation_resistance < ISO_RESISTANCE_LIMIT) status_code = 1;
		else status_code = 0;
		TxData[4] = status_code;

		ssd1306_SetCursor(2, 52);
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
		{
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				ssd1306_WriteString("CAN: ERROR ", Font_7x10, White);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			else
			{
				ssd1306_WriteString("CAN: OK    ", Font_7x10, White);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			}
		}
		else
		{
			ssd1306_WriteString("CAN: BUSY  ", Font_7x10, White);
		}
		ssd1306_UpdateScreen();

		state_timer = HAL_GetTick();
		iso_state = ISO_STATE_IDLE;
		break;
	}

	case ISO_STATE_IDLE:
		if (HAL_GetTick() - state_timer >= CYCLE_IDLE_MS)
		{
			iso_state = ISO_STATE_SELECT_HVP;
		}
		break;
	}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|AKS_OUT_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 AKS_OUT_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|AKS_OUT_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//ADC OKUMA FONKSIYONU

 float read_and_calculate_voltage(ADC_HandleTypeDef* h_adc) {
    uint32_t adc_raw_sum = 0;
    const int samples = 16; // 16 örnek alarak gürültüyü filtreleyelim

    for(int i = 0; i < samples; i++) {
        HAL_ADC_Start(h_adc);
        if(HAL_ADC_PollForConversion(h_adc, 10) == HAL_OK) {
            adc_raw_sum += HAL_ADC_GetValue(h_adc);
        }
        HAL_ADC_Stop(h_adc);
    }

    float adc_avg = (float)adc_raw_sum / (float)samples;

    // Pindeki voltajı bulur
    float Vout = (adc_avg / ADC_MAX_VALUE) * VREF;

    // Gerilim Bölme Oranı (Şemadaki R1+R2+R3 ve R7)
    float Ratio = (R_TOP + R_BOTTOM) / R_BOTTOM;

    return Vout * Ratio;
}

/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Güvenlik kritik cihaz: init hatasında sessizce kilitlenmek yerine
     buzzer ve AKS uyarı çıkışını aktif ederek arızayı bildir (fail-safe). */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AKS_OUT_GPIO_Port, AKS_OUT_Pin, GPIO_PIN_SET);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
