## UrPersonalTrainer (ECSE444 Final Project)

UrPersonalTrainer(UPT) is a wearable IoT device that tracks
trainee’s squat exercises. During the training, the trainee shall place
the UPT on his/her back, and the UPT is capable of
counting the number of push-ups the trainee does. Once the
trainee achieves his/her daily push-ups objectives, the UPT
notifies its user visually and verbally via a speaker and a LED. In
addition, the UPT would generate a daily santé report indicating how many
calories were burnt based on today's training record.

#### This project is accomplished by designing an integrated embedded system that consists of: 
* STM32L475(ARM CortexM4) MCU with the built-in 3-D accelerometer sensor
* Speaker, LED, and a pre-insertion resistor
* USB 2.0 A male to Micro B cable

#### The following technologies were implemented to achieve the project objective:
* DAC
* I2C
* GPIO
* UART
* QSPI Flash
