#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xscugic.h"
#include "pwm_controller.h"
#include "xparameters.h"

#define TMRCTR_1_DEVICE_ID        XPAR_TMRCTR_1_DEVICE_ID
#define TMRCTR_0_DEVICE_ID        XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_1_INTERRUPT_ID     XPAR_FABRIC_TMRCTR_1_VEC_ID
#define TMRCTR_0_INTERRUPT_ID     XPAR_FABRIC_TMRCTR_0_VEC_ID

//XPAR_TMRCTR_1_DEVICE_ID

// Global timer instances
XTmrCtr Timer0;
XTmrCtr Timer1;

int main()
{
    pwm_control pwm0;
    pwm_control pwm1;

    // 1. Initialize hardware timers
    if (PWMtimerInit(&Timer0, 0) != XST_SUCCESS) {
        xil_printf("Timer 0 initialization failed\r\n");
        return XST_FAILURE;
    }

    if (PWMtimerInit(&Timer1, 1) != XST_SUCCESS) {
        xil_printf("Timer 1 initialization failed\r\n");
        return XST_FAILURE;
    }

    // 2. Create PWM control instances
    if (PWMcontrolInit(&pwm0, &Timer0, 0, 1000000, 99) != XST_SUCCESS) {
        xil_printf("PWM0 init failed\r\n");
        return XST_FAILURE;
    }

    if (PWMcontrolInit(&pwm1, &Timer1, 1, 1000000, 50) != XST_SUCCESS) {
        xil_printf("PWM1 init failed\r\n");
        return XST_FAILURE;
    }

    while(1) {
        // 3. Start PWMs
        PWMstart(&pwm0);
        PWMstart(&pwm1);

        // Simple delay
        for (int i = 0; i < 10000000; i++);

        // You can add additional control logic here
        // For example:
        // PWMsetPeriod(&pwm0, 2000000);
        // PWMstop(&pwm0);
    }

    return 0;
}
