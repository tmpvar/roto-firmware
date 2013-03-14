
#include "roto.h"
#include <stdlib.h>

uint8_t bootloader_force;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t roto_CDC_Interface =
  {
    .Config =
      {
        .ControlInterfaceNumber   = 0,
        .DataINEndpoint           =
          {
            .Address          = CDC_TX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
          },
        .DataOUTEndpoint =
          {
            .Address          = CDC_RX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
          },
        .NotificationEndpoint =
          {
            .Address          = CDC_NOTIFICATION_EPADDR,
            .Size             = CDC_NOTIFICATION_EPSIZE,
            .Banks            = 1,
          },
      },
  };

int ParserState[4];

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs
 */
static FILE USBSerialStream;
static bool connected = false, wasConnected = false;

int channel = 0;
int channels[2] = { ADC_CHANNEL0, ADC_CHANNEL1 };
uint16_t results[4];
bool reading = false;

void parseChunk(uint16_t byte) {
  char current = (char)byte;

  if (current == '!') {
    bootloader_force = 0xbb;
    wdt_enable(WDTO_250MS);
    for (;;);
  }
}

/*

  Allow a reset into the (adafruit CDC) bootloader

  + CDC bootloader source https://github.com/adafruit/lufa-lib/blob/master/trunk/Bootloaders/CDC/BootloaderCDC.c#L139-142
  + source of this code https://groups.google.com/d/topic/lufa-support/-w1pP0K3Elk/discussion

*/
void __bootloader_test(void)
        __attribute__ ((naked))
        __attribute__ ((section (".init0")));
void __bootloader_test(void)
{
        __asm volatile ("    lds r24, bootloader_force\n"
                        "    cpi r24, 0xbb\n"
                        "    brne 1f\n"
                        "    ldi r25, 0x00\n"
                        "    sts bootloader_force, r25\n"
                        "    ret\n"
                        "1:\n"
                        : : : "memory");
}


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{

  SetupHardware();
  uint16_t bytesAvailable;

  CDC_Device_CreateStream(&roto_CDC_Interface, &USBSerialStream);
  sei();

  for (;;)
  {

    //
    // Read pressures from 16 buttons
    //
    SensorStates();

    if (connected && !wasConnected) {
      wasConnected = true;
      fputs("roto\n", &USBSerialStream);
      fputs("{\"name\": \"roto\", \"version\" : \"0.0.1\" }\n", &USBSerialStream);
    } else if (!connected && wasConnected) {
      wasConnected = false;
    } else if ((bytesAvailable = CDC_Device_BytesReceived(&roto_CDC_Interface))) {
      while (bytesAvailable--) {
        parseChunk(CDC_Device_ReceiveByte(&roto_CDC_Interface));
      }
    }

    CDC_Device_USBTask(&roto_CDC_Interface);
    USB_USBTask();
  }
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Disable clock division */
  clock_prescale_set(clock_div_1);
  USB_Init();

  //
  // Setup sensor power pins
  //
  DDRB = 0;

  // Configure ADC
  ADC_Init(ADC_PRESCALE_32 | ADC_FREE_RUNNING);

  ADC_SetupChannel(0);
  ADC_SetupChannel(1);


  PORTB = 0xFF;
}

void SensorStates(void) {
  char buffer[64];

  uint16_t result;
  if (!reading) {
    ADC_StartReading(ADC_REFERENCE_AVCC | ADC_LEFT_ADJUSTED | channels[channel]);
    reading = true;
  } else if (ADC_IsReadingComplete()) {
    result = ADC_GetResult();

    if (results[channel] != result) {
      if (connected) {
        sprintf(buffer, "%d,%u\n", channel, result);
        fputs(buffer, &USBSerialStream);
      }

      results[channel] = result;
    }
    reading = false;
  }
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void) {
  bool ConfigSuccess = true;
  ConfigSuccess &= CDC_Device_ConfigureEndpoints(&roto_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void) {
  CDC_Device_ProcessControlRequest(&roto_CDC_Interface);
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo) {
  connected = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
}

