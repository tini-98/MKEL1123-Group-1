# Milestone 5: **Source code and Instructions**

## Overview

The compiled code of the overall system is presented in this milestone. The overall system consists of 3 input devices which are the ultrasonic sensor, DHT22 sensor and water flow sensor. The outputs peripherals are LEDs, buzzer and LCD. TheFREERTOS and nodeMCU features should be used in this system to achieve a real-time operation. However, FREERTOS is not not used since there is only gradual changes in the input measured which can be handled using conditional looping. Meanwhile, nodeMCU is is not functioning well with the board. 

## Source code

_The source is generated in C language:_ [Source code](workspace_flood_detection_system.zip)

## Procedures

1. Create new workspace for the system
2. Choose STM32F411RET6 and Nucleo board
3. Once the workspace is created, change the settings accordingly.
##### DHT22
- Click RCC → High Speed Clock (HSE) to Crystal/Ceramic Resonator
- Click Clock Configuration tab → HCLK (MHz) to 72
- Click Pinout and Configuration tab
- Click Timer → Click TIM1 →
- Clock Source set to Internal Clock
- Configuration → Parameter Settings →
- Prescaler set to 71
- Set PB7 GPIO_Output -> Change label to DHT22_Out
##### Ultrasonic sensor
- Set PA9 to GPIO_Output -> Change label to TRIG
- Set PA8 to GPIO_Input -> Change label to ECHO
##### Water flow sensor
- Set PB0 to GPIO_EXTI0 
##### LCD
- Set PB9 to I2C1_SDA
- Set PB8 to I2C1_SCL
##### Buzzer
- Set PA1 to GPIO_Output -> Change label to Buzzer
##### LEDs
- Set PA4 to GPIO_Output -> Change label to LED_Green
- Set PA5 to GPIO_Output -> Change label to LED_Yellow
- Set PA6 to GPIO_Output -> Change label to LED_Red
4. The newly set configurations is saved to generate C code
5. Cable was used to link the board with the STM32Cube IDE
6. Before building the project, two additional library and source code are copied for the LCD called: 
- i2c-lcd.h (Inc)
- i2c-lcd.c (Src)
7. Project is built and debugged until it is free of error
8. Project is launched

