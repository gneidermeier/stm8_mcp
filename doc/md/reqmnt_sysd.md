# System Requirements {#reqmnt_sysd}

## Product Requirements

Based on product specification for a common 30A electronic speed controller
for electric powered model RC aircraft (ESC)

https://www.hobbypartz.com/60p-dye-1003-30a-esc.html
https://www.hobbypartz.com/60p-dye-1003-30a-esc.html
https://www.castlecreations.com/en/thunderbird-36-esc-010-0051-00
https://www.castlecreations.com/en/thunderbird-36-esc-010-0051-00


Original Product Specification

https://docs.google.com/document/d/1lf2JGDnzeH23M5-8bcwml33lvyoHymr7PNT7GcYSZf0/edit?usp=sharing
https://docs.google.com/document/d/1lf2JGDnzeH23M5-8bcwml33lvyoHymr7PNT7GcYSZf0/edit?usp=sharing


Brushless Motor Control Test Fixture System Requirements Specification ST STM8 Microprocessor

https://drive.google.com/file/d/1_XUsJsZAACf-17b9JeOuLfON8XRH4xup/view?usp=sharing
https://drive.google.com/file/d/1_XUsJsZAACf-17b9JeOuLfON8XRH4xup/view?usp=sharing



## R&D Requirements

External controls: timing, speed, STOP

Serial TTY interface

SPI/CAN/Ethernet/WIFI/BLE

Calibration/tuning interface (e.g. CAN)


## Hardware Requirements

Requires an open circuit board but somewhat ruggedized - thru hole with available test points.

DC power supply connector.

MCU is STM8 Discovery eval board (transition to own STM8 layout)

3 phase bullet connectors.

13.8v DC operation
13.8v DC operation

Resistor divider measurement 3 phases

Available comparator trigger (LM339 or something likethat)

Power LED. 

Activity LED on input to each phase bridge driver.

Must meet all 30A ESC category requirements. 

Current sense IC


## Software Requirements

Implementation on STM8s MCU
STVD/Cosmic (potentially to sdcc)

