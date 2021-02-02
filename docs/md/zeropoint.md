## Zero Crossing Point Estimation {#zeropoint}

There are a number of ways that the zero-crossing point can be estimated from 
the back-EMF voltage measurement. Safe to say there is no 1 silver bullet!
the back-EMF voltage measurement. Safe to say there is no 1 silver bullet!

### Hardware comparator

The most straightforward way to get the exact time of the zero-crossing point
with no estimation required is to integrate a hardware comparator into the 
circuit (feature of the A/D peripheral on some MCUs). The comparator takes
the neutral point voltage and the phase voltage and output an external trigger
to the MCU when the condition switches at the ZCP. 

### Midpoint estimation method

The challenge of trying to use the back-EMF signal directly lies in part
with the noise present in the sensor (unfiltered resistor divider) and also in 
timing the measurement, since the system (motor) is inherently of sync to begin 
with. To detect the time of the zero-crossing point, the addition of a comparator 
to the circuit can provide an external trigger at exact time of the ZCP.
In this way there is a ready error signal with the desired setpoint for the ZCP
time to be exactly midway (30-degrees).

Estimation of the ZCP from sampling the back-EMF voltage is based on timing the 
sample acquisition to acquire 1 sample at 15 degrees and 1 sample at 45 degrees, where 
sample acquisition to acquire 1 sample at 15 degrees and 1 sample at 45 degrees, where 
the two should add to 0 once the motor is in time and the ZCP is occuring at 30 degrees.
It seems to be problematic with this approach as the availability of number 
of samples is linked to the motor speed as well as PWM frequency.

### Integration approach

Estimation of the ZCP from sampling the back-EMF voltage is based on averaging 
as many samples as occur over the 60-degrees duration of the floating sector of 
the measured phase. The premise being that a timed motor will have equal distribution of 
the area under the back-EMF curve around the zero-crossing point as the ZCP should 
occur at exactly halfway point during the floating-phase time period (30-degrees). 

Instead of comparing the average to the ideal neutral voltage, the error comparison
is done between the btdc samples and the atdc samples. The ratio between the average of 
the atdc and  btdc sides of the trapezoid ranges from 1/1 when the motor is in time and 
the atdc and  btdc sides of the trapezoid ranges from 1/1 when the motor is in time and 
proportionately above or below one to the amount of timing error ( error = atdc/btdc - 1)
proportionately above or below one to the amount of timing error ( error = atdc/btdc - 1)

TIM3 is configured to interrupt at 4-times the intended commutation switching 
rate, such that TIM3 ISR shall occur 4 times during each commutation period (i.e. 
at 15 degree intervals within a given 60-degree commutation sector). 
at 15 degree intervals within a given 60-degree commutation sector). 

ADC start conversion is triggered by TIM2 ISR (PWM rising edge). Number of PWM 
pulses (i.e. samples) within the floating period is inverse to motor speed so the  
number of samples collected over the 60-degrees time would need to be counted. 
So a counter in either the TIM2 (PWM rising edge) ISR, or the ADC conversion complete
ISR. Counter need to be 0'd start of each commtation period. 

Alternatively, always intiialze buffer to 1/2 DC point (neutral point voltge) and 
Alternatively, always intiialze buffer to 1/2 DC point (neutral point voltge) and 
always average across X samples (number of samples needed at lowest motor speed at which 
the closed loop is effective). 

The battery voltage measurement is obtained by taking the A/D measurement during 
the active PWM driving section of the commutation period.

<b>TODO: This  shall NOT be at 15-degree intervals to use approach 3</b> 
<b>TODO: This  shall NOT be at 15-degree intervals to use approach 3</b> 

\startuml

== Commutation Time + 15 ==
== Commutation Time + 15 ==

Driver -> stm8s_ADC1: acquire back-EMF sample [0]
Driver -> stm8s_ADC1: acquire back-EMF sample [0]

== Commutation Time + 30 ==

Driver -> stm8s_ADC1: acquire back-EMF sample [1]
Driver -> stm8s_ADC1: acquire back-EMF sample [1]

== Commutation Time + 45 ==

Driver -> stm8s_ADC1: acquire back-EMF sample [2]
Driver -> stm8s_ADC1: acquire back-EMF sample [2]

== Commutation Time + 60 ==

Driver -> stm8s_ADC1: acquire back-EMF sample [3]
Driver -> stm8s_ADC1: acquire back-EMF sample [3]
Driver -> Driver: average 4 samples

\enduml 
