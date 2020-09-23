# Bicycle-Sensor-Prototype

A project designed to increase the safety of bicycle riders when near vehicles or pedestrians. The closer a vehicle gets to the bicycle, LEDS will begin to light up at the back to alert the driver of the presence of the cyclist. The closer a cyclist gets to a pedestrian a buzzer will sound at the front to let pedestrains know that a cyclist is behind them. The distance to the nearest object will be displayed on a LCD.

## Hardware
-Texas Instruments MSP430FR4133 Microprocessor (https://www.ti.com/tool/MSP-EXP430FR4133)

-2 Ultrasonic sensors. For the back and front (https://www.sparkfun.com/products/15569)

-Mosfets to convert from 3.3V to 5V and vice-versa. This is neccessary because the ultrasonic sensors need 5 volt inputs however the microprocessor can only handle a 3.3V signal. Providing 5V to the microprocessor can potentially damage the pins on the launchpad

-4 LEDS. Red, Green, Yellow, Orange.

-Buzzer (https://www.sparkfun.com/products/10293)

## Software

-The program begins by supplying a trigger signal to the ultrasonic sensors. The trigger tells the ultrasonic sensor to activate. The sensors will then return an echo wavelength which must be decoded to get an actual distance (currently in centimeters). 

-However because of noise from the sensors we must send multiple triggers to pickup multiple echos. Currently the system reads in 10 echos and sorts these values using the quicksort algorithm. Then the median of these values is taken and used.

-For the LEDS (back sensor)
  - 0-10cm --> Red
  - 10-30cm --> Yellow
  - 30 - 60 cm --> Orange
  - 60 - 150cm --> Green

-For the buzzer (front sensor)
  - 0-20cm --> quadruple beep
  - 20-60cm --> double beep
  - greater than 60cm --> no beep
  
  ## Prototype Stages
### Stage 1
  ![Stage 1](https://github.com/AikamBoparai/Bicycle-Sensor-Prototype/blob/master/images/Stage1.png)

### Stage 2 (Physical Component Board)  
  ![Stage 2](https://github.com/AikamBoparai/Bicycle-Sensor-Prototype/blob/master/images/Stage2.png)

### Stage 3 (Prototype)  
  ![Final Stage](https://github.com/AikamBoparai/Bicycle-Sensor-Prototype/blob/master/images/FinalStage.jpeg)
  

