// main.c

#include "xparameters.h"
#include "xsysmon.h"
#include "xgpio.h"
#include "xtmrctr.h"
#include "xscugic.h"
#include "xil_types.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xil_io.h"
//#include "setup.h"
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

//GPIO
#define AXI_GPIO_DEVICE_ID    XPAR_AXI_GPIO_0_DEVICE_ID
#define AXI_GPIO_INTR_ID      XPAR_FABRIC_AXI_GPIO_0_IP2INTC_IRPT_INTR
#define INTC_DEVICE_ID        XPAR_SCUGIC_SINGLE_DEVICE_ID
#define GPIOpriority			  0xA0
//ADC
#define XADC_DEVICE_ID XPAR_XADC_WIZ_0_DEVICE_ID
#define XADC_SEQ_CHANNELS 0x2020000
#define XADC_CHANNELS 0x2020000
#define Test_Bit(VEC,BIT) ((VEC&(1<<BIT))!=0)
//PWM
//#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID
#define INTC_HANDLER            XScuGic_InterruptHandler
//#define TMRCTR_0_INTERRUPT_ID     XPAR_FABRIC_TMRCTR_0_VEC_ID
#define TMRCTR_3_DEVICE_ID        XPAR_TMRCTR_3_DEVICE_ID
#define TMRCTR_2_DEVICE_ID        XPAR_TMRCTR_2_DEVICE_ID
#define TMRCTR_1_DEVICE_ID	      XPAR_TMRCTR_1_DEVICE_ID
#define TMRCTR_0_DEVICE_ID        XPAR_TMRCTR_0_DEVICE_ID

#define TMRCTR_INTERRUPT_ID     XPAR_FABRIC_AXI_TIMER_0_INTERRUPT_INTR

#define TMRCTR_1_0                0            /* Timer 0 ID */
#define TMRCTR_1_1                1            /* Timer 1 ID */
#define TMRCTR_0_0                0            /* Timer 0 ID */
#define TMRCTR_0_1                1            /* Timer 1 ID */

#define TMR1Address		0x42810000
#define TMR2Address     0x42820000
#define TMR3Address     0x42830000

XGpio AxiGpio;
XScuGic Intc;
XSysMon Xadc;
XTmrCtr TmrCtr2Instance; /* Timer counter instance */
XTmrCtr TmrCtr1Instance; /* Timer counter instance */
XTmrCtr TmrCtr0Instance; /* Timer counter instance */
XTmrCtr TmrCtr3Instance; /* Timer counter instance */

u64 calcHighTime(u64 Period, u32 DutyCycle);
u64 calcPeriod(u32 Frequency);
void GpioIntrHandler(void *InstancePtr);
int ADCtoDC(u16 RawData);
int GpioSetup(void);
void Xadc_Init(XSysMon *InstancePtr, u32 DeviceId);
int SetupInterruptSystem(Xil_ExceptionHandler GpioIntrHandler);
int TmrCtrPwmExample(XTmrCtr *InstancePtr, u16 DeviceId, u32 CounterAddress,
		u64 Period, u64 HighTime);

int TmrCtrCapture(XScuGic *IntcInstancePtr, XTmrCtr *InstancePtr, u16 DeviceId,
		u16 IntrId);
void TimerCounterHandler_0(void *CallBackRef, u8 TmrCtrNumber);
//static void TimerCounterHandler_1(void *CallBackRef, u8 TmrCtrNumber);
int TmrCtrSetupIntrSystem(XScuGic *IntcInstancePtr, XTmrCtr *InstancePtr,
		u16 DeviceId, u16 IntrId);

volatile bool ADCModeSwitch;
volatile bool enableFlag;

static int PeriodTimerHit = FALSE;
static int HighTimerHit = FALSE;
static u32 capture0, capture1, pulsewidth;
float distance;
bool ResetFlag = 0;

u8 Channel;
u16 RawData;
u32 USPeriod = 20000000;
u32 USHighTime = 15000;

//u32 USPeriod = 3000000;        // 60ms in microseconds
//u32 USHighTime = 500;         // 10us high time

int main() {

	// TODO:
	// add LCD Display changes
	// add SERVO and PMOD LED to adc mode 1
	// add RGB LED for estop, softstop, and go mode
	// add logic for displaying distance on lcd, only take new data every __ seconds so it isnt hard to read

	int DCDutyCycle;
	int State = 0;
	int Status;
	u64 DCHighTime;
	u32 DCFrequency = 600;        // Example period value (adjust as needed)
	u64 DCPeriod = calcPeriod(DCFrequency);
//	u32 USPeriod = 3000000;        // 60ms in microseconds
//	u32 USHighTime = 500;         // 10us high time

	u64 BzPeriod1 = 1000000;
	u64 BzPeriod3 = 300000;

	Status = TmrCtrCapture(&Intc, &TmrCtr0Instance,
	TMRCTR_0_DEVICE_ID, TMRCTR_INTERRUPT_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("Tmrctr Capture Example Failed\r\n");
		return XST_FAILURE;
	}

	if (GpioSetup() != XST_SUCCESS) {
		xil_printf("GPIO setup failed!\r\n");
		return XST_FAILURE;
	}
	if (SetupInterruptSystem(GpioIntrHandler) != XST_SUCCESS) {
		xil_printf("Interrupt setup failed!\r\n");
		return XST_FAILURE;
	}
	Xadc_Init(&Xadc, XADC_DEVICE_ID);

	Xil_ExceptionEnable();

	printf("Cora XADCinterrupt Demo Initialized! \r\n");

	TmrCtrPwmExample(&TmrCtr1Instance, TMRCTR_1_DEVICE_ID,
	TMR1Address, USPeriod, USHighTime);

	TmrCtrPwmExample(&TmrCtr3Instance, TMRCTR_3_DEVICE_ID,
	TMR3Address, 0, BzPeriod1 / 2);

	while (1) {
		if (ResetFlag == 1) {
			Status = TmrCtrCapture(&Intc, &TmrCtr0Instance,
			TMRCTR_0_DEVICE_ID, TMRCTR_INTERRUPT_ID);
			if (Status != XST_SUCCESS) {
				xil_printf("Tmrctr Capture Example Failed\r\n");
				return XST_FAILURE;
			}

			if (GpioSetup() != XST_SUCCESS) {
				xil_printf("GPIO setup failed!\r\n");
				return XST_FAILURE;
			}
			if (SetupInterruptSystem(GpioIntrHandler) != XST_SUCCESS) {
				xil_printf("Interrupt setup failed!\r\n");
				return XST_FAILURE;
			}
			Xadc_Init(&Xadc, XADC_DEVICE_ID);

			Xil_ExceptionEnable();

			printf("System Reset! \r\n");

			TmrCtrPwmExample(&TmrCtr1Instance, TMRCTR_1_DEVICE_ID,
			TMR1Address, USPeriod, USHighTime);

			TmrCtrPwmExample(&TmrCtr3Instance, TMRCTR_3_DEVICE_ID,
			TMR3Address, 0, BzPeriod1 / 2);

			ResetFlag = 0;
		}
		distance = (float) pulsewidth * 0.020 / 148;  //distance in inches
		printf("Distance %5.2F inches \r\n", distance);
		if (!enableFlag) {
			if (distance < 5) {
				printf("ESTOP \r\n");
				Xil_Out32(TMR3Address + 4, BzPeriod3);
				Xil_Out32(TMR3Address + 16 + 4, BzPeriod3 / 2);
				Xil_Out32(TMR2Address + 16 + 4, 0);// turn off motor
				// ADD servo stop
			} else {
				if (distance < 15) {
					printf("softstop \r\n");
					Xil_Out32(TMR3Address + 4, BzPeriod1);
					Xil_Out32(TMR3Address + 16 + 4, BzPeriod1 / 2);
				} else {
					Xil_Out32(TMR3Address + 4, 0);
				}
				switch (State) {
				case 0:
					RawData = XSysMon_GetAdcData(&Xadc, Channel);
					State = 1;
				case 1:
					Channel = 17;
					RawData = XSysMon_GetAdcData(&Xadc, Channel);
//					xil_printf("Channel 17 = %u\r\n", RawData);
					if (ADCModeSwitch == 1) {
						State = 2;
						ADCModeSwitch = 0;
					}
					break;
				case 2:
//				Xil_Out32(TMR1Address + 16 + 4, USHighTime);
					RawData = XSysMon_GetAdcData(&Xadc, Channel);
					DCDutyCycle = ADCtoDC(RawData);
					TmrCtrPwmExample(&TmrCtr2Instance, TMRCTR_2_DEVICE_ID,
					TMR2Address, DCPeriod, DCHighTime);
					State = 3;
				case 3:
					Channel = 25;
					RawData = XSysMon_GetAdcData(&Xadc, Channel);
					DCDutyCycle = ADCtoDC(RawData);
					DCHighTime = calcHighTime(DCPeriod, DCDutyCycle);
					Xil_Out32(TMR2Address + 16 + 4, DCHighTime);
//					xil_printf("Channel 25 = %u\r\n", DCDutyCycle);
					if (ADCModeSwitch == 1) {
//						Xil_Out32(TMR1Address + 16 + 4, USHighTime);
						State = 0;
						ADCModeSwitch = 0;
					}
					break;
				default:
					break;
				}
			}
		} else {
			Xil_Out32(TMR3Address + 4, 0);
		}
	}
}

int ADCtoDC(u16 RawData) {
	return 0.0012 * RawData + 34;
}

void GpioIntrHandler(void *InstancePtr) {
	XGpio *GpioPtr = (XGpio *) InstancePtr;

	// Clear interrupt first
	XGpio_InterruptClear(GpioPtr, 1);  // Clear Channel 1 interrupt
	// Set Duty Cycle to 0, "off"
	Xil_Out32(TMR2Address + 16 + 4, 0);

	int value = XGpio_DiscreteRead(GpioPtr, 1);
	xil_printf("interrupt Triggered\r\n");

	switch (value) {
	case 1:
		//        xil_printf("Button 1 pressed! %u Reseting \r\n", value);
		ResetFlag = 1;
		break;
	case 2:
		//    	xil_printf("Button 2 pressed! %u Switching \r\n", value);
		ADCModeSwitch = 1;
		break;
	case 4:
		enableFlag = !enableFlag;
		if (enableFlag) {
			Xil_Out32(TMR1Address + 16 + 4, 0);
		} else {
			Xil_Out32(TMR1Address + 16 + 4, USHighTime);

		}

		//        	xil_printf("Button 4 pressed! %u Enabling \r\n", value);
		break;
	}
}

u64 calcPeriod(u32 Frequency) {
	u64 Period;
	u32 ClockFrequency = XPAR_TMRCTR_0_CLOCK_FREQ_HZ; // Typically 100MHz in Zynq
	Period = ClockFrequency / Frequency;
	return Period;
}
u64 calcHighTime(u64 Period, u32 DutyCycle) {
	u64 HighTime;
	HighTime = (Period * DutyCycle) / 100;
	return HighTime;
}

int GpioSetup() {
	XGpio_Config *ConfigPtr;
	ConfigPtr = XGpio_LookupConfig(AXI_GPIO_DEVICE_ID);  // Fixed typo
	if (ConfigPtr == NULL)
		return XST_FAILURE;

	int Status = XGpio_Initialize(&AxiGpio, AXI_GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	XGpio_SetDataDirection(&AxiGpio, 1, 0x1); // Y18 input (Channel 1, Bit 0)
	XGpio_SetDataDirection(&AxiGpio, 2, 0x0); // W14 output (Channel 2, Bit 0)
	return XST_SUCCESS;
}

void Xadc_Init(XSysMon *InstancePtr, u32 DeviceId) {
	XSysMon_Config *ConfigPtr;
	ConfigPtr = XSysMon_LookupConfig(DeviceId); // find the device and return the correct config struct address
	XSysMon_CfgInitialize(InstancePtr, ConfigPtr, ConfigPtr->BaseAddress); // Initialize the system struct

	//Disable the Channel Sequencer before configuring the Sequence registers
	XSysMon_SetSequencerMode(InstancePtr, XSM_SEQ_MODE_SAFE);
	//Disable all alarms
	XSysMon_SetAlarmEnables(InstancePtr, 0x0);
	//Set averaging for all channels to 16 samples, averages 16 values to reduce noise , good practice even though it is disabled later
	XSysMon_SetAvg(InstancePtr, XSM_AVG_16_SAMPLES);
	//skip setting the differential input mode for each input -- default Unipolar

	//Set 6ADCCLK acquisition time in all channels, 10 clk cycles for 1, 4 for 0
	XSysMon_SetSeqAcqTime(InstancePtr, XADC_SEQ_CHANNELS);
	//Disable averaging in all channels
	XSysMon_SetSeqAvgEnables(InstancePtr, XADC_SEQ_CHANNELS);
	//Enable all Channels
	XSysMon_SetSeqChEnables(InstancePtr, XADC_SEQ_CHANNELS);
	//Set the ADCCLK frequency equal to 1/32 of the system clock
	XSysMon_SetAdcClkDivisor(InstancePtr, 32);
	//Enable Calibration to fix hardware level errors
	XSysMon_SetCalibEnables(InstancePtr,
	XSM_CFR1_CAL_ADC_GAIN_OFFSET_MASK | XSM_CFR1_CAL_ADC_GAIN_OFFSET_MASK);
	//Enable the channel sequencer in continuous sequencer cycling mode
	XSysMon_SetSequencerMode(InstancePtr, XSM_SEQ_MODE_CONTINPASS);
}

int SetupInterruptSystem(Xil_ExceptionHandler GpioIntrHandler) {
	int Status;
	XScuGic_Config *IntcConfig;

	// Initialize exception handling first
	Xil_ExceptionInit();

	// Look up and initialize interrupt controller
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (IntcConfig == NULL) {
		xil_printf("ERROR: Interrupt config lookup failed\r\n");
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(&Intc, IntcConfig,
			IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("ERROR: Interrupt controller init failed\r\n");
		return XST_FAILURE;
	}

	// Register interrupt controller handler
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler) XScuGic_InterruptHandler, &Intc);

	// Setup GPIO interrupt (higher priority = 0x10)
	Status = XScuGic_Connect(&Intc, AXI_GPIO_INTR_ID, GpioIntrHandler,
			&AxiGpio);
	if (Status != XST_SUCCESS) {
		xil_printf("ERROR: Failed to connect GPIO interrupt\r\n");
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(&Intc, AXI_GPIO_INTR_ID, GPIOpriority, 0x3); // Rising edge

	// Enable interrupts
	XScuGic_Enable(&Intc, AXI_GPIO_INTR_ID);
	XGpio_InterruptClear(&AxiGpio, 1);
	XGpio_InterruptEnable(&AxiGpio, 1);
	XGpio_InterruptGlobalEnable(&AxiGpio);

	// Enable exceptions last
//    Xil_ExceptionEnable();

	xil_printf("Interrupt system setup complete\r\n");
	return XST_SUCCESS;
}

int TmrCtrPwmExample(XTmrCtr *TmrCtrInstancePtr, u16 DeviceId,
		u32 CounterAddress, u64 Period, u64 HighTime) {
	int Status;

	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly. Timer0 is used for self test
	 */
	Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, TMRCTR_1_0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	u32 pwmmasks = XTC_CSR_DOWN_COUNT_MASK | XTC_CSR_AUTO_RELOAD_MASK;

	// Reset both timers
	Xil_Out32(CounterAddress, 0);
	Xil_Out32(CounterAddress + 16, 0);

	// Configure timer 0 (period)
	Xil_Out32(CounterAddress, pwmmasks);
	Xil_Out32(CounterAddress + 4, Period);

	// Configure timer 1 (duty cycle)
	Xil_Out32(CounterAddress + 16, pwmmasks);
	Xil_Out32(CounterAddress + 16 + 4, HighTime);

	// Enable PWM mode and start both timers
	pwmmasks |= XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK
			| XTC_CSR_ENABLE_ALL_MASK;
	Xil_Out32(CounterAddress, pwmmasks);
	Xil_Out32(CounterAddress + 16, pwmmasks);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function demonstrates the use of tmrctr PWM APIs.
 *
 * @param	IntcInstancePtr is a pointer to the Interrupt Controller
 *		driver Instance
 * @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance
 * @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
 *		xparameters.h
 * @param	IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_INTERRUPT_INTR
 *		value from xparameters.h
 *
 * @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE
 *
 * @note		none.
 *
 *****************************************************************************/
int TmrCtrCapture(XScuGic *IntcInstancePtr, XTmrCtr *TmrCtrInstancePtr,
		u16 DeviceId, u16 IntrId) {

	int Status;

	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly. Timer0 is used for self test
	 */
	Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, TMRCTR_0_0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the timer counter to the interrupt subsystem such that
	 * interrupts can occur
	 */
	Status = TmrCtrSetupIntrSystem(IntcInstancePtr, TmrCtrInstancePtr, DeviceId,
			IntrId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handler for the timer counter that will be called from the
	 * interrupt context when the timer expires
	 */
	XTmrCtr_SetHandler(TmrCtrInstancePtr, TimerCounterHandler_0,
			TmrCtrInstancePtr);

//	u32 masks = XTC_CSR_ENABLE_ALL_MASK | XTC_CSR_ENABLE_INT_MASK | XTC_CSR_AUTO_RELOAD_MASK |
//			XTC_CSR_EXT_CAPTURE_MASK | XTC_CSR_CAPTURE_MODE_MASK;
	Xil_Out32(0x42800000, 0x459); //Start timer0 with Capture capability and interrupts.
	Xil_Out32(0x42800010, 0x459); //Start timer1 with Capture capability and interrupts.

	return Status;
}

/*****************************************************************************/
/**
 * This function is the handler which performs processing for the timer counter.
 * It is called from an interrupt context.
 *
 * @param	CallBackRef is a pointer to the callback function
 * @param	TmrCtrNumber is the number of the timer to which this
 *		handler is associated with.
 *
 * @return	None.
 *
 * @note		None.
 *
 *****************************PWM handler*********************************/
//static void TimerCounterHandler_1(void *CallBackRef, u8 TmrCtrNumber) {
//	/* Mark if period timer expired */
//	if (TmrCtrNumber == TMRCTR_1_0) {
////		xil_printf("PWM handler - TMRCTR_1_0\r\n");
//		PeriodTimerHit = TRUE;
//	}
//
//	/* Mark if high time timer expired */
//	if (TmrCtrNumber == TMRCTR_1_1) {
////		xil_printf("PWM handler - TMRCTR_1_1\r\n");
//		HighTimerHit = TRUE;
//	}
//}
/*****************************Capture Handler******************************/
void TimerCounterHandler_0(void *CallBackRef, u8 TmrCtrNumber) {

//	XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

	/* Mark if period timer expired */
	if (TmrCtrNumber == TMRCTR_0_0) {
//		capture0 = XTmrCtr_GetCaptureValue(InstancePtr, TMRCTR_0_0);
		capture0 = Xil_In32(0x42800004);
//			xil_printf("In Tmrctr interrupt handler - TMRCTR_0 %u\r\n",
//					capture0);
		PeriodTimerHit = TRUE;
	}

	/* Mark if high time timer expired */
	if (TmrCtrNumber == TMRCTR_0_1) {
//		capture1 = XTmrCtr_GetCaptureValue(InstancePtr, TMRCTR_0_1);
		capture1 = Xil_In32(0x42800014);
//			xil_printf("In Tmrctr interrupt handler - TMRCTR_1 %u\r\n",
//					capture1);
		HighTimerHit = TRUE;

		if (capture1 > capture0) {
			pulsewidth = capture1 - capture0;
		} else {
			pulsewidth = capture1 - capture0 + 0xFFFFFFFF + 1;
		}

	}

}

/*****************************************************************************/
/**
 * This function setups the interrupt system such that interrupts can occur
 * for the timer counter. This function is application specific since the actual
 * system may or may not have an interrupt controller.  The timer counter could
 * be directly connected to a processor without an interrupt controller.  The
 * user should modify this function to fit the application.
 *
 * @param	IntcInstancePtr is a pointer to the Interrupt Controller
 *		driver Instance.
 * @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance.
 * @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
 *		xparameters.h.
 * @param	IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_VEC_ID
 *		value from xparameters.h.
 *
 * @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE.
 *
 * @note		none.
 *
 ******************************************************************************/
int TmrCtrSetupIntrSystem(XScuGic *IntcInstancePtr, XTmrCtr *TmrCtrInstancePtr,
		u16 DeviceId, u16 IntrId) {
	int Status;
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
			IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId, 0xA8, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, IntrId,
			(Xil_ExceptionHandler) XTmrCtr_InterruptHandler, TmrCtrInstancePtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/* Enable the interrupt for the Timer device */
	XScuGic_Enable(IntcInstancePtr, IntrId);

	/* Initialize the exception table */
	Xil_ExceptionInit();

	/* Register the interrupt controller handler with the exception table */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)
	INTC_HANDLER, IntcInstancePtr);

	/* Enable non-critical exceptions */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

