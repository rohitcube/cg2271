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
 #include "button.h"
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

#define BUZZER_PIN 1 // PTB1

#define UART_TX_PTE22   22     // UART2 TX pin (PTE22)
#define UART_RX_PTE23   23     // UART2 RX pin (PTE23)

#define ADC_SE0			0
#define ADC_SE0_PIN		20
#define ADC_SE4a		4
#define ADC_SE4_PIN		21

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

 static int threshold = 1000;
 SemaphoreHandle_t xThresholdMutex;


 /* --- UART Configuration Definitions --- */
 #define BAUD_RATE 9600

 // Priority is set to be safe for FreeRTOS ISR usage, 0 being highest priority
 #define UART_MAX_SYSCALL_PRIORITY 128
 #define MAX_MSG_LEN    64     // Maximum expected message length

 /* --- Threshold Definitions for ESP32 Messaging --- */
 // These strings must exactly match what the ESP32 sends, including the newline.
 #define T1_MESSAGE "T1\r\n" // Object >= 500mm away
 #define T2_MESSAGE "T2\r\n" // Object >= 100mm away
 #define NO_OBJECT_MESSAGE "SAFE\r\n" // Object > 500mm or not present

 // FreeRTOS Queue parameters
 #define QLEN  5
 QueueHandle_t rx_queue;

 // Structure to hold a received message string for passing through the queue
 typedef struct {
   char message[MAX_MSG_LEN];
 } TMessage;

 // Global buffer for UART transmission (accessed by sendMessage and ISR)
 static char send_buffer[MAX_MSG_LEN];

 /* --- Hardware Initialization --- */

 /**
  * @brief Initializes UART2 for bidirectional communication using register level access.
  *
  * Configures the UART clocking, sets up both RX (PTE23) and TX (PTE22) pin
  * multiplexing, sets the baud rate, and enables the RX interrupt, receiver,
  * and transmitter.
  */
 void initUART2_Bidirectional(uint32_t baud_rate)
 {
   // Disable interrupt while configuring
   NVIC_DisableIRQ(UART2_FLEXIO_IRQn);

   // Enable clock to UART2 and PORTE (where UART2 pins are located)
   SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
   SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

   // Ensure the Receiver and Transmitter are disabled before configuration
   UART2->C2 &= ~((UART_C2_RE_MASK) | (UART_C2_TE_MASK));

   // Configure PTE22 (TX pin) for UART2 function (MUX=4)
   PORTE->PCR[UART_TX_PTE22] &= ~PORT_PCR_MUX_MASK;
   PORTE->PCR[UART_TX_PTE22] |= PORT_PCR_MUX(4);

   // Configure PTE23 (RX pin) for UART2 function (MUX=4)
   PORTE->PCR[UART_RX_PTE23] &= ~PORT_PCR_MUX_MASK;
   PORTE->PCR[UART_RX_PTE23] |= PORT_PCR_MUX(4);

   // --- Baud Rate Calculation ---
   // Assumes a fixed bus clock for simplicity based on typical MCX configuration
   uint32_t bus_clk = CLOCK_GetBusClkFreq();
   // SBR = (Bus Clock) / (16 * Baud Rate)
   uint32_t sbr = (bus_clk + (baud_rate * 8)) / (baud_rate * 16);

   // Set SBR. Bits 8 to 12 in BDH, 0-7 in BDL. MUST SET BDH FIRST!
   UART2->BDH &= ~UART_BDH_SBR_MASK;
   UART2->BDH |= ((sbr >> 8) & UART_BDH_SBR_MASK);
   UART2->BDL = (uint8_t) (sbr &0xFF);
   // Disable loop mode and parity (Standard 8N1 configuration)
   UART2->C1 &= ~UART_C1_LOOPS_MASK; // Disable Loop Mode
   UART2->C1 &= ~UART_C1_RSRC_MASK; // Receiver Source Select (for Loop Mode)
   UART2->C1 &= ~UART_C1_PE_MASK;    // No Parity
   UART2->C1 &= ~UART_C1_M_MASK;    // 8-bit mode

   // Enable RX interrupt (RIE)
   UART2->C2 |= UART_C2_RIE_MASK;

   // Enable the receiver (RE) and transmitter (TE)
   UART2->C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);

   // Configure and enable NVIC for UART2 interrupt
   // Lower number means higher priority. Priorities must be <= configMAX_SYSCALL_INTERRUPT_PRIORITY
   NVIC_SetPriority(UART2_FLEXIO_IRQn, UART_MAX_SYSCALL_PRIORITY);
   NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
   NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
 }

 /**
  * @brief Copies a message to the transmit buffer and enables the TX interrupt
  * to start the transmission process in the ISR.
  *
  * @param message The null-terminated string to send.
  */
 void sendMessage(char *message) {
   // Copy message to the global buffer
   strncpy(send_buffer, message, MAX_MSG_LEN - 1);

   // Enable the Transmit Interrupt (TIE) to signal TDRE in the ISR
   // This starts the transmission process
   UART2->C2 |= UART_C2_TIE_MASK;
   UART2->C2 |= UART_C2_TE_MASK;
 }


 /* --- Interrupt Service Routine (ISR) --- */

 void UART2_FLEXIO_IRQHandler(void)
 {
   // Pointers for the receive and send processes
   static int recv_ptr = 0;
   static int send_ptr = 0;
   char rx_data;
   // Local buffer to hold the incoming message fragment
   static char recv_buffer[MAX_MSG_LEN];

   // --- 1. TRANSMIT Logic (TDRE: Transmit Data Register Empty) ---
   // This check handles the TIE flag being set
   if(UART2->S1 & UART_S1_TDRE_MASK) {
     // Check if we reached the null terminator
     if (send_buffer[send_ptr] == '\0') {
       send_ptr = 0;

       // Disable the transmit interrupt (TIE) to stop sending
       UART2->C2 &= ~UART_C2_TIE_MASK;

       // Disable the transmitter
       UART2->C2 &= ~UART_C2_TE_MASK;
     } else {
       // Write the next character to the data register
//       PRINTF("%c\r\n", send_buffer[send_ptr]);
       UART2->D = send_buffer[send_ptr++];
     }
   }

   // --- 2. RECEIVE Logic (RDRF: Receive Data Register Full) ---
   // This check handles the RIE flag being set
   if(UART2->S1 & UART_S1_RDRF_MASK)
   {

     TMessage msg;
   rx_data = UART2->D;
   recv_buffer[recv_ptr++] = rx_data;
 //    PRINTF("Char received: %d in %d\r\n", rx_data, recv_ptr-1);

     if(rx_data == '\n') {
     // Copy over the string
     BaseType_t hpw;
     recv_buffer[recv_ptr]='\0';
     strncpy(msg.message, recv_buffer, MAX_MSG_LEN);
 //		PRINTF("Received Message: %s\r\n", msg.message);
     xQueueSendFromISR(rx_queue, (void *)&msg, &hpw);
 //		PRINTF("Add to Queue success: %d\r\n", xQueueSendFromISR(rx_queue, (void *)&msg, &hpw));

     portYIELD_FROM_ISR(hpw);
     recv_ptr = 0;
   }

 //      msg.message[MAX_MSG_LEN - 1] = '\0'; // Double-check null termination for safety

   }
 }

 /* --- FreeRTOS Tasks --- */
 /**
  * @brief Processes received ultrasonic distance messages from ESP32.
  * This task waits for threshold messages posted by the UART ISR and triggers
  * the alarm based on the message content.
  */
	static void ultrasonicDataProcessorTask(void *p) {
		while(1) {
			PRINTF("Ultrasonic Data Processor Task Running...\r\n");
			TMessage msg;
			// Wait indefinitely for a message from the UART ISR queue
			if (xQueueReceive(rx_queue, (TMessage *) &msg, portMAX_DELAY) == pdTRUE) {
					PRINTF("RX Raw: %s", msg.message); // Print raw message for debugging
					if (xSemaphoreTake(xDistanceModeMutex, portMAX_DELAY) == pdTRUE) {
						PRINTF("Semaphore Acquired");
						// Compare received message to known threshold strings
						if (strcmp(msg.message, T1_MESSAGE) == 0) {
							PRINTF("-> T1 Threshold Detected (>= 500mm).\r\n");
							distanceMode = CLOSE;
						} else if (strcmp(msg.message, T2_MESSAGE) == 0) {
							PRINTF("-> T2 Threshold Detected (>= 100mm).\r\n");
							distanceMode = TOO_CLOSE;
						} else if (strcmp(msg.message, NO_OBJECT_MESSAGE) == 0) {
							PRINTF("-> No Object Detected.\r\n");
							distanceMode = SAFE;
						} else {
							PRINTF("-> Unknown message format received. Message: %s\r\n", msg.message);
						}
						xSemaphoreGive(xDistanceModeMutex);
					} else {
						PRINTF("FAILED TO ACQUIRE SEMAPHORE");
					}
			}
			vTaskDelay(100);
		}
	}

 /**
  * @brief Polls the ESP32 periodically for new data.
  * This task triggers the transmission by calling sendMessage().
  */
 static void pollingTask(void *p) {
   const char *poll_message = "POLL_DATA";
   while(1) {
     PRINTF("Polling Task Running. Sending '%s'\r\n", poll_message);
     // Send the polling message. This enables the TX interrupt.
     sendMessage((char *)poll_message);

     // Wait for 2 seconds (2000 milliseconds) before polling again
     vTaskDelay(pdMS_TO_TICKS(500));
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

 TickType_t xNoteEndTime = 0;

 void buzzerTask(void *p) {
	 int i = 0;
	 for (;;) {
		 // Check Distance Mode and Start Next Note ---
		 if (xTaskGetTickCount() >= xNoteEndTime && xSemaphoreTake(xDistanceModeMutex, 0) == pdTRUE) { // Use a short wait, not MAX_DELAY
			 if (distanceMode == TOO_CLOSE || isButtonPressed()) {
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
		 vTaskDelay(pdMS_TO_TICKS(10));
	 }
 }

 void initADC() {
 	// Configure interrupt
 	NVIC_DisableIRQ(ADC0_IRQn);
 	NVIC_ClearPendingIRQ(ADC0_IRQn);

 	// Enable clock gating to ADC0
 	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

 	// Enable clock gating to PTE
 	// This is done when we initialize the PWM
 	// but we want to make our function self-contained
 	// so we do it again
 	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

 	// Set PTE20 and PTE21 (ADC0_SE0 and ADC_SE4a) to ADC
 	PORTE->PCR[ADC_SE0_PIN] &= ~PORT_PCR_MUX_MASK;
 	PORTE->PCR[ADC_SE0_PIN] |= PORT_PCR_MUX(0);

 	PORTE->PCR[ADC_SE4_PIN] &= ~PORT_PCR_MUX_MASK;
 	PORTE->PCR[ADC_SE4_PIN] |= PORT_PCR_MUX(0);


 	// Configure the ADC
 	// Enable ADC interrupt
 	ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;

 	// Select single-ended ADC
 	ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
 	ADC0->SC1[0] |= ADC_SC1_DIFF(0b0);

 	// Set 12 bit conversion
 	ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
 	ADC0->CFG1 |= ADC_CFG1_MODE(0b11);

 	// Use software trigger
 	ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;

 	// Use VALTH and VALTL
 	ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
 	ADC0->SC2 |= ADC_SC2_REFSEL(0b01);

 	// Don't use averaging
 	ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;
 	ADC0->SC3 |= ADC_SC3_AVGE(0);

 	// Switch off continuous conversion.
 	ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
 	ADC0->SC3 |= ADC_SC3_ADCO(0);

 	// Highest priority
 	NVIC_SetPriority(ADC0_IRQn, 0);
 	NVIC_EnableIRQ(ADC0_IRQn);

 }

 int result[2];

 void startADC(int channel) {
 	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
 	ADC0->SC1[0] |= ADC_SC1_ADCH(channel);
 }

 void ADC0_IRQHandler(){
 	static int turn=0;

 	NVIC_ClearPendingIRQ(ADC0_IRQn);
 	if(ADC0->SC1[0] & ADC_SC1_COCO_MASK) {
 		// Read the result into result[turn]
 		result[turn] = ADC0->R[0];/* Statement to read result */
 		turn = 1 - turn;
 		if(turn == 0) {
 			// Call startADC to convert PTE20
 			startADC(ADC_SE0);
 		} else {
 			//Call startADC to convert PTE21
 			startADC(ADC_SE4a);
 		}
 	}
 }

void joystickTask(void *p) {
	for (;;) {
		if (xSemaphoreTake(xThresholdMutex, 0) == pdTRUE) {
		// Joystick pushed up, increment threshold to max of 2000 (2m)
			if (result[0] > 40000) {
				threshold = threshold + 30 > 2000 ? 2000 : threshold + 30;
			// Joystick pushed down, decrement threshold to max of 300 (30cm)
			} else if (result[0] < 20000) {
				threshold = threshold - 30 < 300 ? 300 : threshold - 30;
			}
			xSemaphoreGive(xThresholdMutex);
			PRINTF("Threshold: %d\r\n", threshold);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

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
	 initUART2_Bidirectional(BAUD_RATE);
	 initButton();
	 initADC();

	 startADC(ADC_SE0);

     // Create the FreeRTOS message queue for received data (QLEN messages, each of size TMessage)
   rx_queue = xQueueCreate(QLEN, sizeof(TMessage));
   if (rx_queue == NULL) {
     PRINTF("Error: Failed to create FreeRTOS Queue.\r\n");
   }

	 xDistanceModeMutex = xSemaphoreCreateMutex();
	 xThresholdMutex = xSemaphoreCreateMutex();

	 distanceMode = TOO_CLOSE;


	 if (xTaskCreate(buzzerTask, "BuzzerTask", configMINIMAL_STACK_SIZE + 200, NULL, 1, NULL) != pdPASS) {
		 PRINTF("BuzzerTask init fail.\r\n");
	 } else {
		 PRINTF("BuzzerTask init success.\r\n");
	 }

	 if (xTaskCreate(joystickTask, "JoystickTask", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL) != pdPASS) {
		 PRINTF("JoystickTask init fail.\r\n");
   } else {
     PRINTF("JoystickTask init success.\r\n");
   }

	 if (xTaskCreate(pollingTask, "PollingTask", configMINIMAL_STACK_SIZE + 200, NULL, 3, NULL) != pdPASS) {
		 PRINTF("pollingTask init fail.\r\n");
   } else {
     PRINTF("pollingTask init success.\r\n");
   }

	 if (xTaskCreate(ultrasonicDataProcessorTask, "DataProcessor", configMINIMAL_STACK_SIZE + 200, NULL, 4, NULL) != pdPASS) {
		 PRINTF("pollingTask init fail.\r\n");
   } else {
     PRINTF("pollingTask init success.\r\n");
   }

 //    if (xTaskCreate(modeSwitchTask, "ModeSwitchTask", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL) != pdPASS) {
 //		PRINTF("ModeSwitchTask init fail.\r\n");
 //	} else {
 //		PRINTF("ModeSwitchTask init success.\r\n");
 //	}

     vTaskStartScheduler();

     return 0;
 }

