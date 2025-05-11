#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"

/* Constants for thermistor calculations */
#define RT0 10000    // Thermistor resistance at 25°C
#define B 7500       // Beta constant
#define VCC 3.3      // Supply voltage
#define R 10000      // Voltage divider resistor
#define T0 298.15    // Reference temperature in Kelvin (25°C)
#define NUM_SENSORS 2 // Number of thermistor sensors - adjusted to match pin diagram

/* Handle structures */
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
USART_HandleTypeDef husart1; // Keep USART_HandleTypeDef
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

/* Global variables */
float temperature[NUM_SENSORS];
volatile uint32_t errorCount = 0;
char uartBuffer[128]; // Buffer for UART messages

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_Init(void); // Keep original name
void readSensors(void);
void calculateTemperatures(float *Tmax, float *Tmin, float *avg);
void Error_Handler(void);
void UART_Print(char* message);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_CAN_Init();
    MX_USART1_Init();

    /* Print startup message */
    UART_Print("Temperature Module Starting...\r\n");
    UART_Print("Using Thermistor Beta Model for Temperature Calculation\r\n");

    /* Start CAN peripheral with error checking */
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        UART_Print("ERROR: CAN initialization failed\r\n");
        Error_Handler();
    } else {
        UART_Print("CAN Bus Initialized\r\n");
    }

    /* Configure CAN filter to accept all messages */
    CAN_FilterTypeDef canFilterConfig;
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0;
    canFilterConfig.FilterIdLow = 0;
    canFilterConfig.FilterMaskIdHigh = 0;
    canFilterConfig.FilterMaskIdLow = 0;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK) {
        UART_Print("ERROR: CAN filter configuration failed\r\n");
        Error_Handler();
    }

    /* Main loop */
    while (1) {
        /* Toggle LED to indicate system is running */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        /* Read sensor data */
        readSensors();

        float Tmax, Tmin, avg;
        calculateTemperatures(&Tmax, &Tmin, &avg);

        /* Send temperature data via UART */
        sprintf(uartBuffer, "Temperatures: ");
        UART_Print(uartBuffer);

        for (int i = 0; i < NUM_SENSORS; i++) {
            sprintf(uartBuffer, "Sensor %d: %.2f°C | ", i+1, temperature[i]);
            UART_Print(uartBuffer);
        }

        sprintf(uartBuffer, "\r\nSummary - Max: %.2f°C | Min: %.2f°C | Avg: %.2f°C | Errors: %lu\r\n",
                Tmax, Tmin, avg, errorCount);
        UART_Print(uartBuffer);

        /* Prepare CAN message */
        TxHeader.ExtId = 0x1839F380;
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.DLC = 8;

        /* Pack data - using multiplication to preserve decimal places */
        int16_t maxTemp = (int16_t)(Tmax * 10);
        int16_t minTemp = (int16_t)(Tmin * 10);
        int16_t avgTemp = (int16_t)(avg * 10);

        TxData[0] = (uint8_t)(maxTemp >> 8);
        TxData[1] = (uint8_t)(maxTemp & 0xFF);
        TxData[2] = (uint8_t)(minTemp >> 8);
        TxData[3] = (uint8_t)(minTemp & 0xFF);
        TxData[4] = (uint8_t)(avgTemp >> 8);
        TxData[5] = (uint8_t)(avgTemp & 0xFF);
        TxData[6] = NUM_SENSORS;
        TxData[7] = errorCount > 255 ? 255 : (uint8_t)errorCount;

        /* Transmit CAN message with retry mechanism */
        uint8_t retryCount = 0;
        HAL_StatusTypeDef canStatus;

        do {
            canStatus = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

            if (canStatus != HAL_OK) {
                retryCount++;
                sprintf(uartBuffer, "CAN TX Retry %d/3\r\n", retryCount);
                UART_Print(uartBuffer);
                HAL_Delay(10);
            }
        } while (canStatus != HAL_OK && retryCount < 3);

        if (canStatus == HAL_OK) {
            UART_Print("CAN message sent successfully\r\n");
        } else {
            UART_Print("ERROR: Failed to send CAN message\r\n");
            Error_Handler();
        }

        /* Print separator for readability */
        UART_Print("----------------------------------------\r\n");

        HAL_Delay(1000);  // 1 second delay between measurements
    }
}

void readSensors(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adcChannels[NUM_SENSORS] = {ADC_CHANNEL_1, ADC_CHANNEL_2}; // PA1, PA2

    for (int i = 0; i < NUM_SENSORS; i++) {
        /* Configure ADC channel */
        sConfig.Channel = adcChannels[i];
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            sprintf(uartBuffer, "ERROR: ADC channel %d config failed\r\n", i+1);
            UART_Print(uartBuffer);
            temperature[i] = 25.0f; // Default safe value
            errorCount++;
            continue;
        }

        /* Start ADC conversion */
        HAL_ADC_Start(&hadc1);

        /* Check for conversion completion */
        if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
            sprintf(uartBuffer, "ADC %d conversion timeout\r\n", i+1);
            UART_Print(uartBuffer);
            temperature[i] = 25.0f;
            errorCount++;
            HAL_ADC_Stop(&hadc1);
            continue;
        }

        uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        /* Debug raw ADC output */
        sprintf(uartBuffer, "ADC%d Raw: %lu | ", i+1, adcValue);
        UART_Print(uartBuffer);

        /* Check for open circuit (ADC near VCC) or short circuit (ADC near 0) */
        if (adcValue > 4000) { // Open circuit (thermistor disconnected)
            sprintf(uartBuffer, "OPEN CIRCUIT | ");
            UART_Print(uartBuffer);
            temperature[i] = 25.0f; // Default value
            errorCount++;
            continue;
        }
        else if (adcValue < 10) { // Short circuit (thermistor shorted)
            sprintf(uartBuffer, "SHORT CIRCUIT | ");
            UART_Print(uartBuffer);
            temperature[i] = 25.0f;
            errorCount++;
            continue;
        }

        /* Calculate temperature */
        float voltage = (float)adcValue * VCC / 4095.0f;
        float resistance = R * (VCC - voltage) / voltage; // Fixed formula

        sprintf(uartBuffer, "V: %.3fV | R: %.0fΩ | ", voltage, resistance);
        UART_Print(uartBuffer);

        /* Only calculate if resistance is valid */
        if (resistance > 100 && resistance < 100000) { // Reasonable range for NTC
            float steinhart;
            steinhart = log(resistance / RT0) / B;    // ln(R/R0)/B
            steinhart += 1.0f / T0;                   // + (1/T0)
            steinhart = 1.0f / steinhart;             // Invert
            temperature[i] = steinhart - 273.15f;      // Convert to °C

            /* Sanity check */
            if (temperature[i] < -40.0f || temperature[i] > 125.0f) {
                sprintf(uartBuffer, "INVALID RANGE (%.1f°C) | ", temperature[i]);
                UART_Print(uartBuffer);
                temperature[i] = 25.0f;
                errorCount++;
            }
        }
        else {
            sprintf(uartBuffer, "INVALID RESISTANCE | ");
            UART_Print(uartBuffer);
            temperature[i] = 25.0f;
            errorCount++;
        }
    }
    UART_Print("\r\n");
}

void calculateTemperatures(float *Tmax, float *Tmin, float *avg) {
    *Tmax = temperature[0];
    *Tmin = temperature[0];
    *avg = temperature[0];

    for (int i = 1; i < NUM_SENSORS; i++) {
        if (temperature[i] > *Tmax) *Tmax = temperature[i];
        if (temperature[i] < *Tmin) *Tmin = temperature[i];
        *avg += temperature[i];
    }

    *avg /= NUM_SENSORS;
}

void UART_Print(char* message) {
    HAL_USART_Transmit(&husart1, (uint8_t*)message, strlen(message), 100); // Use USART function

    // Also send to ITM for debug console
    for (int i = 0; i < strlen(message); i++) {
        ITM_SendChar(message[i]);
    }
}

static void MX_ADC1_Init(void) {
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* Calibrate ADC before use */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_CAN_Init(void) {
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 9;  // 72MHz/(9*(1+13+2)) = 250kbps standard CAN baud rate
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART1_Init(void) {
    husart1.Instance = USART1;
    husart1.Init.BaudRate = 115200;
    husart1.Init.WordLength = USART_WORDLENGTH_8B;
    husart1.Init.StopBits = USART_STOPBITS_1;
    husart1.Init.Parity = USART_PARITY_NONE;
    husart1.Init.Mode = USART_MODE_TX_RX;
    husart1.Init.CLKPolarity = USART_POLARITY_LOW;
    husart1.Init.CLKPhase = USART_PHASE_1EDGE;
    husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;

    if (HAL_USART_Init(&husart1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /* Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* Initialize the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;                  // Enable HSE
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;              // Enable PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;      // Set HSE as PLL source
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;              // Multiply HSE by 9 (for 8 MHz, gives 72 MHz)

    /* Initialize the RCC Oscillator */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();  // Handle error
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1, and PCLK2 clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as system clock
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;       // Set AHB clock divider
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;        // Set APB1 clock divider
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;        // Set APB2 clock divider

    /* Initialize the CPU, AHB and APB clocks */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();  // Handle error
    }

    /* Configure ADC clock source */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;  // 72MHz/6 = 12MHz ADC clock
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    /* Enable the ITM peripheral */
    ITM->LAR = 0xC5ACCE55;
    ITM->TER = 0xFFFFFFFF;
}

void Error_Handler(void) {
    errorCount++;

    /* Send error message */
    UART_Print("*** ERROR DETECTED ***\r\n");

    /* Visual feedback for errors */
    for(int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}
