# Temp_Control
Temperature Conotrol with PID and autotune

The code uses a look up table to convert ADC readings into degrees C.
It uses the PID and autotune library to control the dutycycle of a relay.
The relay is opened/closed with a timer interval using the timer library,

The matlab program uses the manufacturer provided resistance versus temperature data and converts it to voltage/ADC counts based on serial resitors and supply voltage.
