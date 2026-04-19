# KMSh_ModbusRTU_Leds
ModbusRTU realization and application on KMSh V3.0 (STM32H745IIT6). Controls LEDs activation, brightness and blinking.

### Addresses:
#### LED_x enable:         coils[0x0000 - 0x0007]
#### LED_x brightness:     regs [0x0000 - 0x0007]
#### LED_x blink interval: regs [0x0010 - 0x0017]
#### LED_x blinking mode:  regs [0x0020 - 0x0027]
