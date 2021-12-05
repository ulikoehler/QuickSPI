# QuickSPI

Practical Arduino SPI wrapper, allowing direct register access with proper error handling.

This is intended to be similar to [AdaFruit BusIO](https://github.com/adafruit/Adafruit_BusIO) but provide a more convenient API and less memory footprint. For example, every [Adafruit_BusIO_Register](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_BusIO_Register.h) is an actual object occupying a bunch of bytes of RAM and does not provide verified read functionality.

## How?

QuickSPI provides classes & macro tricks to automatically define appropriate functions for any register, with zero memory footprint for additional registers.