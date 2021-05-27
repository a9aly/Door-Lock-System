# Smart Lock System
Smart Lock System with password using STM32 Nucleo board

# How to run:

1 - Download the repository files and extract them.

2 - Open "SmartLockEmbedded.ioc" using cubeMX software.

3 - You should see screen like the photo below apply the same connections to your STM32L432KCUx board.

- D4-D7: connected to D4-D7 of LCD 1602 DISPLAY.

- EN - RW - RS: connected to LCD DISPLAY EN and RW.

- Trig and ECHO: connected to ultrasonic sensor.

- ServoMotor: connected to PWM pin of servo motor.

- BuzzerVVD: Connected to active buzzer.

- USART RX/TX: connected to usb HCO06 module for bluetooth connection.
 
![image](https://user-images.githubusercontent.com/74613419/119780433-1a5e7000-beca-11eb-9660-c6d07422e3d2.png)

4 - Generate Code.

5 - Keil uvision5 window will appear, build and load your code on your board. 

6 - You should see an output like that on your screen. Congratulations!!

![image](https://user-images.githubusercontent.com/74613419/119781559-72e23d00-becb-11eb-9460-cf2db0308e5c.png)
