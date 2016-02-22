/**************************************************************************//**
 * @file
 * @brief Relative humidity and temperature sensor demo for SLSTK3400A_EFM32HG
 * @version 4.2.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_device.h"
#include <stdint.h>
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "i2cspm.h"
#include "si7013.h"
#include "rtcdriver.h"
#include "graphics.h"
#include "em_adc.h"
#include "em_usart.h"
#include "bspconfig.h"
#include "bsp.h"
#include <string.h>     // required for strlen() function
#include <stdio.h>

/**************************************************************************//**
 * Local defines
 *****************************************************************************/

/** Time (in ms) between periodic updates of the measurements. */
#define PERIODIC_UPDATE_MS      2000
/** Voltage defined to indicate dead battery. */
#define LOW_BATTERY_THRESHOLD   2800
#define COM_PORT gpioPortC // USART location #1: PD0 and PD1
#define UART_TX_pin 0      // PE10
#define UART_RX_pin 1      // PE11


/**************************************************************************//**
 * Local variables
 *****************************************************************************/
/* RTC callback parameters. */
static void (*rtcCallback)(void*) = NULL;
static void * rtcCallbackArg = 0;

/** This flag tracks if we need to update the display
 *  (animations or measurements). */
static volatile bool updateDisplay = true;
/** This flag tracks if we need to perform a new
 *  measurement. */
static volatile bool updateMeasurement = true;
/** Flag used to indicate ADC is finished */
static volatile bool adcConversionComplete = false;

/** Timer used for periodic update of the measurements. */
RTCDRV_TimerID_t periodicUpdateTimerId;


/**************************************************************************//**
 * Local prototypes
 *****************************************************************************/
static void gpioSetup(void);
static void periodicUpdateCallback(RTCDRV_TimerID_t id, void *user);
static void memLcdCallback(RTCDRV_TimerID_t id, void *user);
static uint32_t checkTemp(void);
static void adcInit(void);
static void createString(char *string, int32_t value);
static void uartInit(void);





static volatile uint32_t msTicks; /* counts 1ms timeTicks */

static void Delay(uint32_t dlyTicks);

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
static void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}
/**************************************************************************//**
 * @brief  Helper function to perform data measurements.
 *****************************************************************************/
static int performMeasurements(I2C_TypeDef *i2c, uint32_t *rhData, int32_t *tData, uint32_t *tempDataSensor)
{
  *tempDataSensor = checkTemp();
  Si7013_MeasureRHAndTemp(i2c, SI7021_ADDR, rhData, tData);
  return 0;
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
  uint32_t         rhData;
  bool             si7013_status;
  int32_t          tempData;
  int32_t		   tempDataSensor;
  int32_t          oldTemp;
  uint8_t 		   i;

  /* Chip errata */
  CHIP_Init();

  /* Initalize hardware */
  gpioSetup();
  adcInit();
  GRAPHICS_Init();
  RTCDRV_Init();
  I2CSPM_Init(&i2cInit);
  BSP_LedsInit();
  uartInit();


  /* Get initial sensor status */
  si7013_status = Si7013_Detect(i2cInit.port, SI7021_ADDR, NULL);

  /* Set up periodic update of the display. */
  RTCDRV_AllocateTimer(&periodicUpdateTimerId);
  RTCDRV_StartTimer(periodicUpdateTimerId, rtcdrvTimerTypePeriodic,
                    PERIODIC_UPDATE_MS, periodicUpdateCallback, NULL);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

  GRAPHICS_ShowStatus(si7013_status, false);
  EMU_EnterEM2(false);

  updateDisplay = true;

  while (true)
  {
    if (updateMeasurement)
    {
      performMeasurements(i2cInit.port, &rhData, &tempData, &tempDataSensor);
      char string[6];
      createString(string, tempData);
      for (i = 0; i < sizeof(string) - 1; i++)
      {
    	  while (!(USART1 ->STATUS & (1 << 6))); // wait for TX buffer to empty
          USART1 ->TXDATA = string[i]; // print each character of the test string
      }
      updateMeasurement = false;
      if(oldTemp != tempData + 2000)
      {
    	  oldTemp = tempData;
      }
      else
      {
    	  updateDisplay = false;
      }

    }

    if (updateDisplay)
    {
      updateDisplay = false;
      GRAPHICS_Draw(tempData, rhData, tempDataSensor*10);
      BSP_LedToggle(1);
      Delay(25);
      BSP_LedToggle(1);
    }
    EMU_EnterEM2(false);
  }
}


/**************************************************************************//**
 * @brief This function is called whenever we want to measure the supply v.
 *        It is reponsible for starting the ADC and reading the result.
 *****************************************************************************/
static uint32_t checkTemp(void)
{
  uint32_t tempDataSensor;
  /* Sample ADC */
  adcConversionComplete = false;
  ADC_Start(ADC0, adcStartSingle);
  while (!adcConversionComplete) EMU_EnterEM2(false);
  tempDataSensor = ADC_DataSingleGet( ADC0 );
  return tempDataSensor;
}

static void uartInit(void)
{
	USART_InitAsync_TypeDef uartInit = {
			.enable = usartDisable,   // Wait to enable the transmitter and receiver
			.refFreq = 0, // Setting refFreq to 0 will invoke the CMU_ClockFreqGet() function and measure the HFPER clock
			.baudrate = 115200,          // Desired baud rate
			.oversampling = usartOVS16,     // Set oversampling value to x16
			.databits = usartDatabits8, // 8 data bits
			.parity = usartNoParity,  // No parity bits
			.stopbits = usartStopbits1, // 1 stop bit
			.mvdis = false,          // Use majority voting
			.prsRxEnable = false,          // Not using PRS input
			.prsRxCh = usartPrsRxCh0, // Doesn't matter which channel we select
	};

	USART_InitAsync(USART1, &uartInit);  // Apply configuration struct to USART1
	USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | _USART_ROUTE_LOCATION_LOC0; // Clear RX/TX buffers and shift regs, enable transmitter and receiver pins

	USART_IntClear(USART1, _USART_IF_MASK); // Clear any USART interrupt flags

	USART_Enable(USART1, usartEnable);     // Enable transmitter and receiver
}


/**************************************************************************//**
 * @brief ADC Interrupt handler (ADC0)
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
   uint32_t flags;

   /* Clear interrupt flags */
   flags = ADC_IntGet( ADC0 );
   ADC_IntClear( ADC0, flags );

   adcConversionComplete = true;
}


/**************************************************************************//**
 * @brief ADC Initialization
 *****************************************************************************/
static void adcInit(void)
{
   ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
   ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;
   ADC_InitSingle_TypeDef initSingle2 = ADC_INITSINGLE_DEFAULT;

   /* Enable ADC clock */
   CMU_ClockEnable( cmuClock_ADC0, true );

   /* Initiate ADC peripheral */
   ADC_Init(ADC0, &init);

   /* Setup single conversions for internal VDD/3 */
   initSingle.acqTime = adcAcqTime16;
   initSingle.input   = adcSingleInpVDDDiv3;
   ADC_InitSingle( ADC0, &initSingle );

   initSingle2.reference = adcRef1V25;
   initSingle2.input   = adcSingleInpCh4;
   ADC_InitSingle( ADC0, &initSingle2 );

   /* Manually set some calibration values */
   ADC0->CAL = (0x7C << _ADC_CAL_SINGLEOFFSET_SHIFT) | (0x1F << _ADC_CAL_SINGLEGAIN_SHIFT);

   /* Enable interrupt on completed conversion */
   ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
   NVIC_ClearPendingIRQ( ADC0_IRQn );
   NVIC_EnableIRQ( ADC0_IRQn );
}

static void createString(char *string, int32_t value)
{
	if (value < 0)
	  {
	    value = -value;
	    string[0] = '-';
	  }
	  else
	  {
	    string[0] = ' ';
	  }
	  string[5] = 0;
	  string[4] = '0' + (value % 1000) / 100;
	  string[3] = '.';
	  string[2] = '0' + (value % 10000) / 1000;
	  string[1] = '0' + (value % 100000) / 10000;

	  if (string[1] == '0')
	  {
	    string[1] = ' ';
	  }
}



/**************************************************************************//**
* @brief Setup GPIO interrupt for pushbuttons.
*****************************************************************************/
static void gpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART1, true);    // Enable USART0 peripheral clock

  /* Enable si7021 sensor isolation switch */
  GPIO_PinModeSet(gpioPortC, 8, gpioModePushPull, 1);
  GPIO_PinModeSet(COM_PORT, UART_TX_pin, gpioModePushPull, 1); // Configure UART TX pin as digital output, initialize high since UART TX idles high (otherwise glitches can occur)
  GPIO_PinModeSet(COM_PORT, UART_RX_pin, gpioModeInput, 0); // Configure UART RX pin as input (no filter)
}


/**************************************************************************//**
 * @brief   The actual callback for Memory LCD toggling
 * @param[in] id
 *   The id of the RTC timer (not used)
 *****************************************************************************/
static void memLcdCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  rtcCallback(rtcCallbackArg);
}


/**************************************************************************//**
 * @brief   Register a callback function at the given frequency.
 *
 * @param[in] pFunction  Pointer to function that should be called at the
 *                       given frequency.
 * @param[in] argument   Argument to be given to the function.
 * @param[in] frequency  Frequency at which to call function at.
 *
 * @return  0 for successful or
 *         -1 if the requested frequency does not match the RTC frequency.
 *****************************************************************************/
int rtcIntCallbackRegister(void (*pFunction)(void*),
                           void* argument,
                           unsigned int frequency)
{
  RTCDRV_TimerID_t timerId;
  rtcCallback    = pFunction;
  rtcCallbackArg = argument;

  RTCDRV_AllocateTimer(&timerId);

  RTCDRV_StartTimer(timerId, rtcdrvTimerTypePeriodic, 1000 / frequency,
                    memLcdCallback, NULL);

  return 0;
}


/**************************************************************************//**
 * @brief Callback used to count between measurement updates
 *****************************************************************************/
static void periodicUpdateCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  updateDisplay = true;
  updateMeasurement = true;
}
