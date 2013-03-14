
#ifndef _TPAD_H_
#define _TPAD_H_

  /* Includes: */
    #include <avr/io.h>
    #include <avr/wdt.h>
    #include <avr/power.h>
    #include <avr/interrupt.h>
    #include <string.h>
    #include <stdio.h>

    #include "Descriptors.h"

    #include <LUFA/Drivers/Peripheral/ADC.h>
    #include <LUFA/Drivers/USB/USB.h>

    /* Function Prototypes: */
    void SetupHardware(void);
    void SensorStates(void);

    void EVENT_USB_Device_Connect(void);
    void EVENT_USB_Device_Disconnect(void);
    void EVENT_USB_Device_ConfigurationChanged(void);
    void EVENT_USB_Device_ControlRequest(void);

#endif

