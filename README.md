# Zed F9P Firmware using STM32 HAL
Tested on STM32 Nucleo-F446RE, but should work on stm32 device. The example in `main.c` connects UART1 on the microcontroller to UART1 on the GNSS module. `DO NOT CONNECT TO UART2 OF F9P` since the module does not allow coniguration changes using UART2. 

# Referenced Documentation
- [Interface Description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.13_InterfaceDescription_UBX-21023318.pdf)
- [Journal Dump](https://docs.google.com/document/d/1aJ1K6xspJr7Fzseq05Ad5D2fW64Y2SJVbcG4NPxoBFQ/edit?usp=sharing)
    - [pdf download in repo](GPS_Firmware_DMA\ZED-F9P-GNSS-Firmware-Journal.pdf)

# Video References
- override printf to use SWV Trace debugging [link](https://www.youtube.com/watch?v=sPzQ5CniWtw)
- GPS DMA Development References [link](https://www.youtube.com/watch?v=Bo6MC5A8uTE)
- Fix Floats not displaying in SWV console [link](https://www.youtube.com/watch?v=kAhRdpLDc84)
