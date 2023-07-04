# PWMGeneracionMedicion
Sistema que permite generar una señal PWM controlada por comandos vía UART visualizando en tiempo real frecuencia y duty de la señal en un display 16x2 y almacenando dicha información en una tarjeta SD. Se implementa en el uC STM32F103C8 utilizando el IDE STM32Cube V.1.6.1, con FreeRTOS y lenguaje C.

Hardware utilizado:

 + Blue Pill STM32F103C8
 + Modulo SD (SPI)
 + Conversor USB- TTL (UART)
 + Adaptador serie- paralelo (I2C) 
 + Display 16x2

