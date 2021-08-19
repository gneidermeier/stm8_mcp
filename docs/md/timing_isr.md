# Timing Concurrency Multitaking {#timing_isr}

Timing domains in motor control must be highly synchronized.

## System Update and Background Task timing

The event handler for System Timer ISR multiplexes the BL Control Task and the 
Periodic Task, which occur at different rates. 
 
Periodic Task provides an extern function to flag it for invocation from the 
Background Task.

\startuml

System Timer ISR -> Driver: Update()
Driver ->  BL: Update()
Driver -> Periodic Task: Task_set_ready()

\enduml


## Commutation Sequencer timing

The Commutation Timer period varies inversely with the motor output so there must be 
a Commutation Timer event to occur at least at each 60-degree sector. In order to 
synchronize PWM capture for zero-crossing detection, the Commutation Timer is configured 
to interrupt at at 15 degree intervals within a given 60-degree commutation sector.

\startuml

Commutation Timer ISR -> Driver: Step()
Driver -> BL: Step()
BL ->  Sequencer: Step()
Sequencer-> PWM: Phase_control()

\enduml


## PWM Sample sequence timing

You could try ....

\startuml

Sequencer -> stm8s_TIM2: TIM2_CCxCmd( CHANNEL_x, ENABLE )
stm8s_TIM2 -> stm8_isr: TIM2_UPD_OVF_BRK_IRQHandler()
stm8_isr -> stm8s_ADC1: ADC1_StartConversion();
stm8s_ADC1 -> stm8_isr: ADC1_IRQHandler()
stm8_isr -> Driver: On_ADC_Conversion_Rdy()

\enduml

sometimes it works.
