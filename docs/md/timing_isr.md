# Timing Concurrency Multitaking {#timing_isr}

Timing domains in motor control are highly synchronized.

## Update and Background Task Timing

TIM4 ISR is utilized as a scheduling trigger (period of 0.512 ms ) for 
BLDC_Update() which embodies the software state machine and control-loop logic/timing.

Periodic Background Task timing synchronous to the ISR - TaskRdy flag

\startuml

stm8_isr -> Driver: Driver_update()
Driver ->  BLDC_sm: BLDC_update()
Driver -> BG_task: Task_set_ready() 

\enduml

blah blah


## Sequencer Timing

TIM3 is configured to interrupt at 4-times the intended commutation switching
-rate, such that TIM3 ISR shall occur 4 times during each commutation period 
(i.e. at 15 degree intervals within a given 60-degree commutation sector). 
(i.e. at 15 degree intervals within a given 60-degree commutation sector). 

\startuml

stm8_isr -> Driver: step()
Driver -> Stepper: Step()
Stepper -->  Driver: Driver_get_ADC()
Stepper ->  MCU_stm8: Phase_control()

\enduml

foo bar period the end.


## PWM Sample Sequence Timing

YOu could try ....

\startuml

Sequencer -> stm8s_TIM2: TIM2_CCxCmd( CHANNEL_x, ENABLE )
stm8s_TIM2 -> stm8_isr: TIM2_UPD_OVF_BRK_IRQHandler()
stm8_isr -> stm8s_ADC1: ADC1_StartConversion();
stm8_isr -> stm8s_ADC1: ADC1_StartConversion();
stm8s_ADC1 -> stm8_isr: ADC1_IRQHandler()
stm8s_ADC1 -> stm8_isr: ADC1_IRQHandler()
stm8_isr -> Driver: On_ADC_Conversion_Rdy()

\enduml 

sometimes it works.
