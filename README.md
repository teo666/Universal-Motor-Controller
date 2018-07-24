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

Image below shows UMD's working diagram

![alt text](https://github.com/teo666/Universal-Motor-Controller/blob/master/doc/img/working_block.png)

- The main power is connected to zero crossing detector circuit that transform sine wave signal to a square wave signal and send it to IC, main power source is conneted to motor power switching circuit
- IC is an Atmega-328p
- the power switching circuit is for made up of a triac, it controls the power of the motor and it is drived by IC
- the tacho is the feedback circuit of the system, it generate a signal to IC in relation to motor speed.

The main target of the driver is to changes power feeding the motor to prevent slowdowns when the load changes, to achieve this goal IC analyzes signal coming from tachogenerator and consequntly it swiches triac.

For a correct switching, IC has to know the exact phase of main power source, this is guarantee by ZCD circuit.

ZCD generates a square wave with a doubled frequency compared to main power source frequency, in my case, considering a 50Hz main frequency it generate a square wave of 100Hz.
This signal is connected to a special pin of the Atmega that is able to recognize the rising edge of ZCD output signal and rise a hardware interrupt that freeze the current program execution flow and execute a special routine. This mechanism allow to the Atmega to know when the main power source has reached the value of 0 volt so that it can syncronyze the motor power circuit.

(For a more comprensive and detailed guide about triac working principle read https://en.wikipedia.org/wiki/Thyristor and https://en.wikipedia.org/wiki/TRIAC)

The above simple mechanism is enough to drive motors and it allows to change their speed with a potentiometer, for example we could read an analog value and map it along the time interval  punctuated by two rising edge coming from ZCD circuit, and switch triac in relation to that value. The figure below illustrate that mechanism

