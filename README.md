# BL40A1812 Introduction to Embedded Systems

| Master (UNO)  | Slave (MEGA) |
| ------------  | ------------ |
| Motion sensor | Display      |
| Buzzer        | Keypad       |

Master can operate alone and trigger the alarm, if motion is detected. Slave is the user interface and can communicate to the master that a valid password has been used. If the slave is not connected to the master, it should inform the user about it with a error message.