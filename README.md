This is an STM32CubeIDE demo program showing how to use the new Music Project Development board.
It assumes the use of a Nucleo-G431RB board, the Adafruit 4682 microSD card, and a TLV320DAC3100
audio DAC.  It's not very intelligent; it just goes "ping" out from one stereo channel and plays
a sine wave out from the other one, changing the frequency whenever it receives a wireless update
message from the ESP32.
