K Lawson
Exploration of STM32 Cube code generator.  Can TX.  It's good for setting up the pins but it's easier porting a demo
at the moment so far as code generation is concerned.




Configuration	CANTx
STM32CubeMX 	4.11.0
Date	04/05/2016
MCU	STM32F103C8Tx



PERIPHERALS	MODES	FUNCTIONS	PINS
ADC1	IN0	ADC1_IN0	PA0-WKUP
ADC1	IN1	ADC1_IN1	PA1
ADC1	IN2	ADC1_IN2	PA2
ADC1	IN3	ADC1_IN3	PA3
ADC1	IN4	ADC1_IN4	PA4
CAN	Master	CAN_RX	PA11
CAN	Master	CAN_TX	PA12
I2C1	I2C	I2C1_SCL	PB6
I2C1	I2C	I2C1_SDA	PB7
USART1	Asynchronous	USART1_RX	PA10
USART1	Asynchronous	USART1_TX	PA9
USART3	Asynchronous	USART3_RX	PB11
USART3	Asynchronous	USART3_TX	PB10



Pin Nb	PINs	FUNCTIONs	LABELs
2	PC13-TAMPER-RTC	GPIO_Output	
3	PC14-OSC32_IN	GPIO_Output	
4	PC15-OSC32_OUT	GPIO_Output	
5	PD0-OSC_IN	GPIO_Output	
6	PD1-OSC_OUT	GPIO_Output	
10	PA0-WKUP	ADC1_IN0	
11	PA1	ADC1_IN1	
12	PA2	ADC1_IN2	
13	PA3	ADC1_IN3	
14	PA4	ADC1_IN4	
21	PB10	USART3_TX	
22	PB11	USART3_RX	
25	PB12	GPIO_Input	
26	PB13	GPIO_Input	
27	PB14	GPIO_Input	
28	PB15	GPIO_Input	
29	PA8	GPIO_Input	
30	PA9	USART1_TX	
31	PA10	USART1_RX	
32	PA11	CAN_RX	
33	PA12	CAN_TX	
42	PB6	I2C1_SCL	
43	PB7	I2C1_SDA	



SOFTWARE PROJECT

Project Settings : 
Project Name : CANTx
Project Folder : C:\Users\keithl\Desktop\STM32Demo\STM32F103Cube\CANTx
Toolchain / IDE : TrueSTUDIO
Firmware Package Name and Version : STM32Cube FW_F1 V1.2.0


Code Generation Settings : 
STM32Cube Firmware Library Package : Copy all used libraries into the project folder
Generate peripheral initialization as a pair of '.c/.h' files per IP : No
Backup previously generated files when re-generating : No
Delete previously generated files when not re-generated : Yes
Set all free pins as analog (to optimize the power consumption) : No


Toolchains Settings : 
Compiler Optimizations : Balanced Size/Speed






