# What Universal-Motor-Controller is

Universal-Motor-Controller (UMC) is an Arduino based software and hardware intended to drive universal brushed motor, like many circuit based on TDA1085.

# Main components

UMC consist of two parts:
- software: a C/C++ code running on Atmel Atmega-328p (Arduino Uno and compatibles boards)
- hardware: divided in three parts
    - zero crossing detector circuit (zcd), used for synchronize Arduino on main power
    - power circuit, used for feed motor
    - feedback circuit, used for monitoring the speed of the motor and adjust feed power consequently

# Working principle

Image below shows UMC's working diagram

![alt text](./doc/img/working_block.png)

- The main power is connected to zero crossing detector circuit that transform sine wave signal to a square wave signal and send it to IC, main power source is conneted to motor power switching circuit
- IC is an Atmega-328p
- the power switching circuit is for made up of a triac, it controls the power of the motor and it is drived by IC
- the tacho is the feedback circuit of the system, it generate a signal to IC in relation to motor speed.

The main target of the driver is to changes power feeding the motor to prevent slowdowns when the load changes, to achieve this goal IC analyzes signal coming from tachogenerator and consequntly it swiches triac.

For a correct switching, IC has to know the exact phase of main power source, this is guarantee by ZCD circuit.

ZCD generates a square wave with a doubled frequency compared to main power source frequency, in my case, considering a 50Hz main frequency it generate a square wave of 100Hz.
This signal is connected to a special pin of the Atmega that is able to recognize the rising edge of ZCD output signal and rise a hardware interrupt that freeze the current program execution flow and execute a special routine. This mechanism allow to the Atmega to know when the main power source has reached the value of 0 volt so that it can syncronyze the motor power circuit.

(For a more comprensive and detailed guide about triac working principle read https://en.wikipedia.org/wiki/Thyristor and https://en.wikipedia.org/wiki/TRIAC)

The above simple mechanism is enough to drive motors and it allows to change their speed with a potentiometer, for example we could read an analog value and map it along the time interval punctuated by two rising edge coming from ZCD circuit, and switch triac in relation to that value. The figure below illustrate that mechanism

![alt text](./doc/img/pot.png)

Suppose to have a potentiometer to adjust the motor power: if potentiometer is turned to left the motor power decrease otherwise it increase. To achive that we have to change the switching point of triac inside main power source half period. In our case the potentiometer is connected to one of the avr analog input pin and readed value from potentiometer is mapped into a value between 0 and 999, and assume that we divide the semiperiord in 999 slice with the same width going from 0 to 999.

Now, if the potentiometer read value is 0 we want the mimimun motor power so we have to leave triac turned off.
If potentiometer get a value of 999 the desire power is maximum, we have to excite trigger immidiatly after the 0 volt point, or equivalentemente at 0th slice.
For any other value of potentiometer we have to trigger triac at slice numbered as (999 - pot_value) where pot_value is the value readed from potentiometer.

At this point the main question is: how divide half period into equal slices to know when turn on triac?
The response is: with timers.

Basically timers are hardware device enclosed into many ICs (avrs too) that can be used to count time. Think about timer like an unsigned variable going from 0 to a max value, and its content is increased by one (or decreased by one, or set to zero) every time that something happen, the something that happens is the clock signal, eventually scaled by a constant. Hardware can be configured to execute some special code (exactly like interrupt mechanism previously described) each time timer value reaches a special value, for example its maximum value.

The Atmega-328p (like arduino) is usually clocked with a 16MHz crystal oscillator, suppose to scale the clock with a 128 factor and use that signal to feed a 8bit timer. With that configuration timer increase its value by one every 
1 / (16e6 / 128) = 0.000008 seconds or 8 microseconds, and suppose to configure hardware to execute the special code (called Interrupt Service Routine or ISR) every time timer reach maximum value that in a 8bit configuration is 255, and the ISR increase the `tick_count` variable value by one starting from 0.

After 256 * 8 = 2048 microseconds `tick_count` value is incresed to 1  
After 4096 microsecond `tick_count` value is 2  
After 6144 microsecond `tick_count` value is 3  
After 8192 microsecond `tick_count` value is 4  
...  
After 999424 microseconds `tick_count` value is 488

or equivalently, when `tick_count` is 488 approximately a seconds is passed, that is how Arduino libraries implements millis, micros and delay functions.

The idea that we use is the same: increase a variable by one periodically inside the timer's overflow ISR (executed when timer reaches its maximum value) and reset it when ZCD find a zero volt point... then reapeat.

Go back to potentiometer example. Suppose to be immediatly after a 0 volt point and ZCD has reset our variable, and also use following condition to turn on triac.

``` c++
if( tick_count >= (999 - pot_value)){
    turn_on_triac();
}
```
- if potentiometer is set to maximum value (999) then (999 - pot_value) = 0, our variable is greater or equal to 0 and the triac is turned on
- if potentiometer is set to minimum value (0) then (999 - pot_value) = 999, our variable is less than 999 and the triac remains off
- likewise intermediate values

when next 0 volt poin occur, ZCD interrupt is executed, the `tick_count` variable is reset, and the process is repeated.

At this point we have a mixture of software and hardware that is commoly called *dimmer*, a device that can change power given to a load turning a potentiometer, those kind of devices are usually used to change halogen lamps brightness, the next step is feedback circuit integration.

The feedback circuit allow system to react to load changes and increase motor power to avoid slowdowns. The feedback circuit receives a signal coming from the tachogenerator of the motor, make some process on it and send it to IC.
The tachogenerator signal is commonly an AC sine wave signal that increase its frequency and voltage as the speed increases, so it is possible use frequency or voltage or event both together to analyze the speed of the motor, in UMC frequency is used.
The tacho AC signal is converted to an AC square wave signal and then only the positive component is taken, giving to IC a DC square wave signal. The figure below show the tacho processing signal

![alt text](./doc/img/tacho.png)

Once IC receives the feedback signal it has to get its frequency to know the actual speed motor and change its power consequently. To do that UMC use the same tecnique used for ZCD: interrupt mechanism.

Each time feedback signal goes from low to high (rising edge) IC's hardware generates an interrupt, the normal code execution is paused and associated ISR is executed.  
Previously we have talked about measuring signal frequency, in truth we do not need know the exact frequency, instead just a value related to frequency on wich make a comparison.  
At this point is relatively simple to get an evaluation of feedback signal frequency. Do you remember the use of timer for periodically check if triac has to be turned on? Ok, we use the same routine for increase a counter associated to feedback signal and we will reset that value in ISR associated to feedback rising edge event. In that manner we have a value related to frequency: the higher the frequency (motor speed) the lower is the couter.

At this point IC has all needed parameter for controlling power motor, and for do that we use a PID controller, you can read more about PID controller on [Wikipedia](https://en.wikipedia.org/wiki/PID_controller), about 
[used library](https://playground.arduino.cc/Code/PIDLibrary) and 
[detailed library explanation](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/).


# Hardware
DISCLAIMER: I am not a electrical engineer and all these circuits were taken on web and put together with a limited knowledge of them. If you have suggest to improve them and UMC you are welcome.

Below are reported the schematic circuits used in UMC
### Zero crossing detector circuit

![alt text](./doc/img/zcd_eagle.png)

note: VCC is 5 volt, all resistor are 1/4 watt,T1 and OK1 can be changed with equivalent components.  
PORTD,PIN2 is equivalent to pin 2 on Arduino-UNO board

### Motor power circuit

![alt text](./doc/img/power_eagle.png)

note: all resisor are 1/4 watt except R8 that is 1 watt (instead one 100 ohm resistor i recilced four 470 ohm resistor in parallel) , C3 must have appropriate galvanic isolation, for T2 I used a BTB16 couse BT138 gave me some problem.  

PORTD,PIN7 is equivalent to pin 7 on Arduino-UNO board

![alt text](./doc/img/zcd_power_board.jpg)

note: zcd and motor power circuits on same pcb

![alt text](./doc/img/zcd_power_board_bottom.jpg)

### Feedback circuit

![alt text](./doc/img/tacho_eagle.png)

note: note: all resisor are 1/4 watt, Vcc is 9 volts, IC1 can be replaced with another opamp  

PORTD,PIN3 is equivalent to pin 3 on Arduino-UNO board

![alt text](./doc/img/tacho_board.jpg)  

this board contains the potentiometer connector too and a push button connected to PORTD,PIN6 that is equivalent to pin 6 on Arduino-UNO board. We will talk about the push button later.

All together

![alt text](./doc/img/assembled_board.jpg)


### Demo video

This [video](https://youtu.be/NKULTa6kquY) is a demonstration about UMC capabilities.  
This [video](https://youtu.be/l1mFbbu4X3Q) show capabilities of UMC on low speed.  
These two [video1](https://youtu.be/YVdLVX_Wkz4) and [video2](https://youtu.be/LLw0HUfIIPY) show a motor driven by UMC connected to a bench grindstone disconnected from main power.
# Speed limiter

UMC comes with a spftearec based speed limiter. Speed limiter use the feedback signal to to achive this goal. Speed limiter is usefull because it allow to map all potentiometer turning range inside the desired motor speed limits.  
In some application we could need maximum motor speed around 2000 rmp and in other 4000 rmp, but we want reach those speed when potentiometer is completely at the most, and leave hardware and software as they are, without needs of modifications.
At the moment it is possible limit the speed only during avr's boot process using the *programming button*.
Programming Button is a push button that if pressed connect PORTD,PIN6 (pin 6 of Arduino Uno) to ground, and allow us to set limit speed values (low and high), these values are stored in avr's EEPROM so they are accessible at the next restart, whitout needs to set up them every time.

**CAUTION: programming button is used to switch from automatic to manual mode too. Manual mode allow to disable the feedback circuit and use UMC as a simple dimmer. Suppose to have upper speed limit set to 1000 rpm and runnign motor with this speed, so with potentiometer at maximum. If you press programming button in this condition the motor will receice all possible power and its speed will increase with possible motor or equipment damage.**

### Usage

- Connect Arduino to PC, on Arduino IDE open serial monitor (ensure all parameter are correct: serial port, board etc)
- press and hold the *programming button*
- reset the board keeping *programming button* pressed
- read serial monitor keeping *programming button* pressed and follow printed instructions

At the end of process if you turn potentiometer the speed motor is limeted inside selected range.

# Changelog

v1.0.1
- UMC comes with its own version of PID library, it is heavily based on Brett Beauregard's PID library  
(library link:  https://playground.arduino.cc/Code/PIDLibrary  
email: <br3ttb@gmail.com>  
library explanation: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/): the new version is basically identical to Brett Beauregard's version except for the facts that it is now asynchronous and some unused methods are deleted.
- UMC can now change its PID parameters dynamically, related to motor speed.
- Improved motor stability on low speed
- Code revisiting.
- UMC comes now with a test mode usefull for calibration.

# Known problems and Improvements

This is a nightly UMC version, there are many things to improve

- Lower speed limit can't be set to 0.
- Slow start integration
- Program lives as it is, there isn't an Object Oriented Programming model and integrations with other libraries are difficult due to time dependent application nature.
- UMC doesn't allow connect multiple motor due low number of Atmega-328p interrupt pins. One is necessary for ZCD circuit the other for feedback circuit. I'm trying to remove this lack using a frequncy to analog converter that technically will allow drive up to six motor. Should be possible monitor feedback signal without the need of interrupt pin by checking it periodically inside timer's ISR, but I have not tested this method yet.
- OOP usage is releted to the above two problems.

