# What Universal-Motor-Driver is

Universal-Motor-Driver (UMD) is an Arduino based software and hardware intended to drive universal brushed motor, like many circuit based on TDA1085.

# Main components

UMD consist of two parts:
- software: a C/C++ code running on Atmel Atmega-328p (Arduino Uno and compatibles boards)
- hardware: divided in three parts
    - zero crossing detector circuit (zcd), used for synchronize Arduino on main power
    - power circuit, used for feed motor
    - feedback circuit, used for monitoring the speed of the motor and adjust feed power consequently

# Working principle

Image below shows UMD working diagram

![alt text](https://github.com/teo666/Universal-Motor-Controller/blob/master/doc/img/working_block.png)

- The main power is connected to zero crossing detector circuit that transform sine wave signal into a sqare wave signal and send it to IC, main power sorce is conneted to power switch circuit
- IC is an Atmega-328p
- the power switch circuit is formade up of a triac, it controls the power of the motor and it is drived by IC
- the tacho is the feedback circuit of the system, it generate a signal to IC in relation to motor speed
