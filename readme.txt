power_meter

This is a simple example of how to receive samples using uhd and do something with the samples.
The power measurements in this application has to be calibrated to your radio.  The values
output will most likely not be correct.

"windows_count" is a #define at the top of the source code
This controlls how many sample reads we integrate signals before reporting a received power level.

In the power measurment math, the input impedance to the ADC is assumed to be 100 ohms (200 diff).
You will probably have to play with the value and calibrate your unit to get the correct response.

