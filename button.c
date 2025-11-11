/*
 * @file    button.c
 * @brief   External button interrupt for buzzer control
 */

#include "button.h"
#include "fsl_device_registers.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"

#define BUTTON_PORT PORTA
#define BUTTON_GPIO GPIOA
#define BUTTON_PIN  4
#define BUTTON_IRQ  PORTA_IRQn
#define BUTTON_MASK (1 << BUTTON_PIN)

static volatile bool buttonPressed = false;

void PORTA_IRQHandler(void) {
    /* Clear pending interrupt flag */
    NVIC_ClearPendingIRQ(BUTTON_IRQ);

    /* Check if PTA4 caused interrupt */
    if (BUTTON_PORT->ISFR & BUTTON_MASK) {
        /* Active low: pressed = 0, released = 1 */
        if ((BUTTON_GPIO->PDIR & BUTTON_MASK) == 0) {
            buttonPressed = true;
            PRINTF("Button pressed (IRQ)\r\n");
        } else {
            buttonPressed = false;
            PRINTF("Button released (IRQ)\r\n");
        }

        /* Clear ISFR by writing 1 to corresponding bit */
        BUTTON_PORT->ISFR |= BUTTON_MASK;
    }
}

void initButton(void) {
    NVIC_DisableIRQ(PORTA_IRQn);
    
    /* Enable clock to Port A */
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    /* Configure PTA4 as GPIO with pull-up resistor */
    BUTTON_PORT->PCR[BUTTON_PIN] &= ~PORT_PCR_MUX_MASK;
    BUTTON_PORT->PCR[BUTTON_PIN] |= PORT_PCR_MUX(1);   /* ALT1 = GPIO */
    BUTTON_PORT->PCR[BUTTON_PIN] |= (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);

    /* Configure as input */
    BUTTON_GPIO->PDDR &= ~BUTTON_MASK;

    /* Configure interrupt on both edges */
    BUTTON_PORT->PCR[BUTTON_PIN] &= ~PORT_PCR_IRQC_MASK;
    BUTTON_PORT->PCR[BUTTON_PIN] |= PORT_PCR_IRQC(0b1011);  /* Interrupt on both edges */

    /* Enable NVIC interrupt for PORTA */
    NVIC_ClearPendingIRQ(BUTTON_IRQ);
    NVIC_SetPriority(BUTTON_IRQ, 3);
    NVIC_EnableIRQ(BUTTON_IRQ);

    PRINTF("Button initialized on PTA4 (IRQ mode)\r\n");
}

bool isButtonPressed(void) {
    return buttonPressed;
}

