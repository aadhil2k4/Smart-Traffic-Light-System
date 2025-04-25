#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>

#define TRIG_PIN1 0       // PA0 for Sensor 1 Trigger
#define ECHO_PIN1 1       // PA1 for Sensor 1 Echo
#define TRIG_PIN2 6       // PA6 for Sensor 2 Trigger
#define ECHO_PIN2 7       // PA7 for Sensor 2 Echo
#define RED_LED_PIN1 12   // PB12 for Sensor 1 Red LED
#define YELLOW_LED_PIN1 13 // PB13 for Sensor 1 Yellow LED
#define GREEN_LED_PIN1 14  // PB14 for Sensor 1 Green LED
#define RED_LED_PIN2 8    // PA8 for Sensor 2 Red LED
#define YELLOW_LED_PIN2 11 // PA11 for Sensor 2 Yellow LED
#define GREEN_LED_PIN2 12  // PA12 for Sensor 2 Green LED
#define TEMP_SENSOR_PIN 9  // PB9 for Temperature Sensor
#define PA0_EXTI_LINE (1 << 0) // EXTI Line for PA0

// Bluetooth
void USART1_Init(void);
void USART1_Write(char ch);
void USART1_WriteString(const char* str);

#define TEMP_THRESHOLD 1
uint32_t flag = 0;
uint32_t distance1;  // Distance from Sensor 1
uint32_t distance2;  // Distance from Sensor 2
uint32_t temperature; // Temperature reading

void printit(uint32_t temperature, const char* lane1_status, const char* lane2_status);
void Delay(uint32_t time);
void GPIO_Config(void);
void Timer_Config(void);
void ADC_Config(void);
uint32_t Measure_Distance1(void);
uint32_t Measure_Distance2(void);
uint32_t Read_Temperature(void);
void Update_Traffic_Lights(void);
void NVIC_Config(void);

void NVIC_Config(void) {
    EXTI->IMR |= PA0_EXTI_LINE;
    EXTI->FTSR |= PA0_EXTI_LINE;
    NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & PA0_EXTI_LINE) {
        EXTI->PR |= PA0_EXTI_LINE;
        GPIOA->ODR &= ~((1U << RED_LED_PIN2) | (1U << YELLOW_LED_PIN2) | (1U << GREEN_LED_PIN2));
    }
}

int main(void) {
    SystemInit();
    GPIO_Config();
    Timer_Config();
    ADC_Config();
    USART1_Init();
    NVIC_Config();

    uint32_t lastDistanceUpdateTime = 0;

    while (1) {
        uint32_t current_time = TIM2->CNT; // Assuming TIM2 as time base

        if ((current_time - lastDistanceUpdateTime) >= 10000) { // Update every 10 seconds
            distance1 = Measure_Distance1();
            distance2 = Measure_Distance2();
            lastDistanceUpdateTime = current_time;

            temperature = Read_Temperature();
            Update_Traffic_Lights();  // Traffic light update now also prints status and temperature
        }

        Delay(10000); // Delay for the loop (if needed)
    }
}

void GPIO_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    GPIOA->MODER |= (1U << (TRIG_PIN1 * 2)) | (1U << (TRIG_PIN2 * 2));
    GPIOA->OTYPER &= ~((1U << TRIG_PIN1) | (1U << TRIG_PIN2));
    GPIOA->OSPEEDR |= (3U << (TRIG_PIN1 * 2)) | (3U << (TRIG_PIN2 * 2));
    GPIOA->PUPDR &= ~((3U << (TRIG_PIN1 * 2)) | (3U << (TRIG_PIN2 * 2)));
    GPIOA->MODER &= ~((3U << (ECHO_PIN1 * 2)) | (3U << (ECHO_PIN2 * 2)));
    GPIOB->MODER |= (1U << (RED_LED_PIN1 * 2)) | (1U << (YELLOW_LED_PIN1 * 2)) | (1U << (GREEN_LED_PIN1 * 2));
    GPIOB->OTYPER &= ~((1U << RED_LED_PIN1) | (1U << YELLOW_LED_PIN1) | (1U << GREEN_LED_PIN1));
    GPIOB->OSPEEDR |= (3U << (RED_LED_PIN1 * 2)) | (3U << (YELLOW_LED_PIN1 * 2)) | (3U << (GREEN_LED_PIN1 * 2));
    GPIOA->MODER |= (1U << (RED_LED_PIN2 * 2)) | (1U << (YELLOW_LED_PIN2 * 2)) | (1U << (GREEN_LED_PIN2 * 2));
    GPIOA->OTYPER &= ~((1U << RED_LED_PIN2) | (1U << YELLOW_LED_PIN2) | (1U << GREEN_LED_PIN2));
    GPIOA->OSPEEDR |= (3U << (RED_LED_PIN2 * 2)) | (3U << (YELLOW_LED_PIN2 * 2)) | (3U << (GREEN_LED_PIN2 * 2));
    GPIOB->MODER |= (3U << (TEMP_SENSOR_PIN * 2));
}

void Timer_Config(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}

    FLASH->ACR = FLASH_ACR_LATENCY_0WS;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 84 - 1;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void ADC_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = 0;
    ADC1->SQR3 = 17;
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint32_t Read_Temperature(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;         // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));     // Wait for end of conversion
    return ADC1->DR;                      // Return ADC data register value
}

void Delay(uint32_t time) {
    time *= 1000;
    while (time--) {
        __NOP();
    }
}

uint32_t Measure_Distance1(void) {
    uint32_t start, end, pulse_width;
    EXTI->IMR &= ~PA0_EXTI_LINE;
    GPIOA->ODR &= ~(1U << TRIG_PIN1);
    Delay(2);
    GPIOA->ODR |= (1U << TRIG_PIN1);
    Delay(10);
    GPIOA->ODR &= ~(1U << TRIG_PIN1);
    EXTI->IMR |= PA0_EXTI_LINE;
    
    while (!(GPIOA->IDR & (1U << ECHO_PIN1)));
    start = TIM2->CNT;
    while (GPIOA->IDR & (1U << ECHO_PIN1));
    end = TIM2->CNT;
    pulse_width = (end > start) ? (end - start) : (0xFFFFFFFF - start + end);
    return (pulse_width * 0.0343) / 2;
}

uint32_t Measure_Distance2(void) {
    uint32_t start, end, pulse_width;
    GPIOA->ODR &= ~(1U << TRIG_PIN2);
    Delay(2);
    GPIOA->ODR |= (1U << TRIG_PIN2);
    Delay(10);
    GPIOA->ODR &= ~(1U << TRIG_PIN2);
    while (!(GPIOA->IDR & (1U << ECHO_PIN2)));
    start = TIM2->CNT;
    while (GPIOA->IDR & (1U << ECHO_PIN2));
    end = TIM2->CNT;
    pulse_width = (end > start) ? (end - start) : (0xFFFFFFFF - start + end);
    return (pulse_width * 0.0343) / 2;
}

// Update the printit function to print lane status
void printit(uint32_t temperature, const char* lane1_status, const char* lane2_status) {
    char buffer[100];
    sprintf(buffer, "Temperature: %d C\nLane1: %s\nLane2: %s\n", temperature, lane1_status, lane2_status);
    USART1_WriteString(buffer);
}

void Update_Traffic_Lights(void) {
    const char *lane1_status, *lane2_status;

    // Turn on Yellow LEDs on both lanes for 5 seconds
    GPIOB->ODR |= (1U << YELLOW_LED_PIN1);
    GPIOA->ODR |= (1U << YELLOW_LED_PIN2);
    GPIOB->ODR &= ~((1U << RED_LED_PIN1) | (1U << GREEN_LED_PIN1));
    GPIOA->ODR &= ~((1U << RED_LED_PIN2) | (1U << GREEN_LED_PIN2));
    Delay(5000);

    GPIOB->ODR &= ~(1U << YELLOW_LED_PIN1);
    GPIOA->ODR &= ~(1U << YELLOW_LED_PIN2);

    // Determine which lane should be green based on distance
    if (distance1 < distance2) {
        // Lane 1 Green, Lane 2 Red
        GPIOB->ODR |= (1U << GREEN_LED_PIN1);
        GPIOB->ODR &= ~(1U << RED_LED_PIN1);
        GPIOA->ODR |= (1U << RED_LED_PIN2);
        GPIOA->ODR &= ~(1U << GREEN_LED_PIN2);

        lane1_status = "Green";
        lane2_status = "Red";
    } else {
        // Lane 2 Green, Lane 1 Red
        GPIOA->ODR |= (1U << GREEN_LED_PIN2);
        GPIOA->ODR &= ~(1U << RED_LED_PIN2);
        GPIOB->ODR |= (1U << RED_LED_PIN1);
        GPIOB->ODR &= ~(1U << GREEN_LED_PIN1);

        lane1_status = "Red";
        lane2_status = "Green";
    }

    // Print temperature and lane statuses via Bluetooth
    printit(temperature, lane1_status, lane2_status);
}

void USART1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (2U << 18);
    GPIOA->AFR[1] |= (7U << 4);
    USART1->BRR = 0x683;
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_UE;
}

void USART1_Write(char ch) {
    while (!(USART1->SR & USART_SR_TXE)) {}
    USART1->DR = (ch & 0xFF);
}

void USART1_WriteString(const char* str) {
    while (*str) {
        USART1_Write(*str++);
    }
}