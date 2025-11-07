/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    Assignment.c
 * @brief   Application entry point.
 */
 #include <stdio.h>
 #include "board.h"
 #include "peripherals.h"
 #include "pin_mux.h"
 #include "clock_config.h"
 #include "fsl_debug_console.h"
 /* TODO: insert other include files here. */

 #include "duration_array.h"
 #include "freq_array.h"
 #include "FreeRTOS.h"
 #include "task.h"
 #include "semphr.h"

 /* TODO: insert other definitions and declarations here. */

 /*
  * @brief   Application entry point.
  */

 // LED pin numbers
 #define BUZZER_PIN 1 // PTB1

 /*
  * @brief   Application entry point.
  */

 // Tune array length calculation
 #define ARRAY_LENGTH (sizeof(duration_array) / sizeof(duration_array[0]))

 typedef enum {
   TOO_CLOSE,
   CLOSE,
   SAFE
 } DistanceMode_t;

 DistanceMode_t distanceMode = SAFE;
 SemaphoreHandle_t xDistanceModeMutex;

 /* Delay Function */
 static void delay(volatile uint32_t nof) {
   while(nof!=0) {
   __asm("NOP");
   nof--;
   }
 }

 // Configure the MCG Internal Reference Clock
 void setMCGIRClk() {

   MCG->C1 &= ~MCG_C1_CLKS_MASK;
   // Choose MCG clock source of 01 for LIRC
   // and set IRCLKEN to 1 to enable LIRC
   MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

   // Set IRCS to 1 to choose 8 MHz clock
   MCG->C2 |= MCG_C2_IRCS_MASK;

   // Choose FCRDIV of 0 for divisor of 1
   MCG->SC &= ~MCG_SC_FCRDIV_MASK;
   MCG->SC |= MCG_SC_FCRDIV(0b0);

   // Choose LIRC_DIV2 of 0 for divisor of 1
   MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
   MCG->MC |= MCG_MC_LIRC_DIV2(0b0);

 }

 void setTPMClock(){

   // Set MCGIRCLK
   setMCGIRClk();

   // Choose MCGIRCLK (8 MHz)
   SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
   SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);

 }

 // Buzzer: PTB1 TPM1 CH 1 ALT3

 void initPWM() {

   // Turn on clock gating to TPM1
   SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

   // Set up TPM1
   // Turn off TPM1 and clear the prescalar field
   TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

   // Set prescalar of 128
   TPM1->SC |= TPM_SC_PS(0b000);

   // Select centre-aligned PWM mode
   TPM1->SC |= TPM_SC_CPWMS_MASK;

   // Initialize count to 0
   TPM1->CNT = 0;

   // Turn on clock gating to Port B
   // Set up the clock gating
   SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK);

   // Set the pin multiplexors
   PORTB->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;

   // PWM
   PORTB->PCR[BUZZER_PIN] |= PORT_PCR_MUX(0b11);

 //  // Set pin to output
 //  GPIOB->PDDR |= (1 << BUZZER_PIN);

   // High drive mode (Louder)
 //  PORTB->PCR[BUZZER_PIN] |= PORT_PCR_DSE_MASK;

   TPM1->CONTROLS[1].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK);
   TPM1->CONTROLS[1].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1));
   TPM1->CONTROLS[1].CnV=(int)((0.5)*(double)TPM1->MOD);
 }

 long calculateModForFreq(int freq) {
     long base_clk_freq_hz = 8000000L; // 8 MHz
     int prescalar_value = 1;

     if (freq == 0) return 1;

     // Use floating-point division to maintain precision
     long mod_value = (long)((double)base_clk_freq_hz / (2.0 * (double)prescalar_value * (double)freq)) - 1L;
     printf("MOD value: %ld\r\n", mod_value);

     return (mod_value < 1) ? 1 : (int)mod_value;
 }

 void start_buzzer_pwm_with_freq(int freq) {
   int mod = calculateModForFreq(freq);
   // Set up TPM1
   // Turn off TPM1 and clear the prescalar field
   TPM1->SC &= ~(TPM_SC_CMOD_MASK);

   // Initialize count to 0
   TPM1->CNT = 0;

   TPM1->MOD = mod;

   TPM1->CONTROLS[1].CnV=(int)((0.5)*(double)TPM1->MOD);

   // Start PWM
   TPM1->SC |= TPM_SC_CMOD(0b01);
 }

 void stopPWM() {
   TPM1->SC &= ~TPM_SC_CMOD_MASK;
 }

 //void buzzerTask(void *p) {
 //	int i = 0;
 //	for (;;) {
 //		PRINTF("START BUZZER TASK");
 //		int delay = 500;
 //		if (xSemaphoreTake(xDistanceModeMutex, portMAX_DELAY) == pdTRUE) {
 //			if (distanceMode == TOO_CLOSE) {
 //				start_buzzer_pwm_with_freq(freq_array[i]);
 //				delay = (int)((double)duration_array[i]/3);
 //				PRINTF("%f", delay);
 //				i = (i + 1) % ARRAY_LENGTH;
 //			} else {
 //				stopPWM();
 //				i = 0;
 //			}
 //			xSemaphoreGive(xDistanceModeMutex);
 //		}
 //		PRINTF("END BUZZER TASK");
 //		vTaskDelay(pdMS_TO_TICKS(delay));
 //	}
 //}

 TickType_t xNoteEndTime = 0;

 void buzzerTask(void *p) {
     int i = 0;
     for (;;) {
       PRINTF("START BUZZER TASK");

         // Check Distance Mode and Start Next Note ---
         if (xTaskGetTickCount() >= xNoteEndTime && xSemaphoreTake(xDistanceModeMutex, 0) == pdTRUE) { // Use a short wait, not MAX_DELAY
             if (distanceMode == TOO_CLOSE) {
                 // Calculate next note duration in ticks
                 TickType_t xDurationTicks = pdMS_TO_TICKS((int)((double)duration_array[i] * 0.3));

                 start_buzzer_pwm_with_freq(freq_array[i]);

                 // Set the time when this note should stop
                 xNoteEndTime = xTaskGetTickCount() + xDurationTicks;

                 i = (i + 1) % ARRAY_LENGTH;
             } else {
                 stopPWM();
                 xNoteEndTime = 0; // Reset timer
                 i = 0;
             }
             xSemaphoreGive(xDistanceModeMutex);
         }

         // Ensure the task yields to let the scheduler run
         PRINTF("END BUZZER TASK");
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 }

 //void modeSwitchTask(void *p) {
 //	for (;;) {
 //		PRINTF("START SWITCH TASK");
 //		if (xSemaphoreTake(xDistanceModeMutex, portMAX_DELAY) == pdTRUE) {
 //			if (distanceMode == TOO_CLOSE) {
 //				distanceMode = SAFE;
 //			} else if (distanceMode == SAFE) {
 //				distanceMode = CLOSE;
 //			} else if (distanceMode == CLOSE) {
 //				distanceMode = TOO_CLOSE;
 //			}
 //			xSemaphoreGive(xDistanceModeMutex);
 //			PRINTF("Dist mode is %d\r\n", distanceMode);
 //		}
 //		PRINTF("END SWITCH TASK");
 //		vTaskDelay(pdMS_TO_TICKS(1000));
 //	}
 //}

 int main(void) {

     /* Init board hardware. */
     BOARD_InitBootPins();
     BOARD_InitBootClocks();
     BOARD_InitBootPeripherals();
 #ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
     /* Init FSL debug console. */
     BOARD_InitDebugConsole();
 #endif
     setTPMClock();
     initPWM();

     xDistanceModeMutex = xSemaphoreCreateMutex();

     distanceMode = TOO_CLOSE;
 //
 //    while(1) {
 //
 //    	for (int i = 0; i < ARRAY_LENGTH; i++) {
 //    		start_buzzer_pwm_with_freq(freq_array[i]);
 //    		delay((int)((double)duration_array[i]/1000*DELAY));
 //    	}
 //    	stopPWM();
 //
 //    }


     if (xTaskCreate(buzzerTask, "BuzzerTask", configMINIMAL_STACK_SIZE + 200, NULL, 1, NULL) != pdPASS) {
       PRINTF("BuzzerTask init fail.\r\n");
     } else {
       PRINTF("BuzzerTask init success.\r\n");
     }

 //    if (xTaskCreate(modeSwitchTask, "ModeSwitchTask", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL) != pdPASS) {
 //		PRINTF("ModeSwitchTask init fail.\r\n");
 //	} else {
 //		PRINTF("ModeSwitchTask init success.\r\n");
 //	}



     vTaskStartScheduler();

     return 0;
 }

