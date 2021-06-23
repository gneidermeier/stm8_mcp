# Foreground Background Context {#mainc}

Next @subpage background

## Main Background Task Timing

The MCU startup sequence is initiated from the ISR power-on/reset vector, and
the ISR handler jumps to the entry point of the C runtime, which by convention calls main().

main() embodies an infinite loop which is referred to as the background task.
So long as interrupts are not masked, any ISRs can interrupt main(). The
execution context of main() is restored from the stack upon return from the ISR.

The stm8s has the capability to assign relative priorities to interrupt sources,
but this adds complexity so nested interrupts are avoided in this implementation.

There are multiple peripherals that must be serviced (ISRs) so the system must
be paritioned such that the least amount of cpu time is spent in any one ISR.
Most of the time that can be spent in ISRs must be reserved to perform the motor
commutation sequence, PWM and ADC sampling sequence.
Also the time spent in critical sections implemented (DI/EI) must be minimized as
this also adds to the interrupt latency.

The Background task updates the state machine actions and transitions, so they are
maintained at a periodic rate. Within the state machine actions there may be expected
to accommodate any non-deterministic behavior of the software related to e.g. the
servicing the communication stack, the UI, fault management. These sub-tasks are pushed
into the background task but in the worst case, should not exceed the Timer period (hence a frame-overrun).

\startuml

stm8_ISR -> Main: main()

Main -> stm8_ISR: Enable TIM4 ISR

group while

  opt 0.5 ms timer expires
    stm8_ISR ->> BG_task: On_Timer_tick()  
    BG_task ->> BG_task: Set_ready()
  end

  Main -> BG_task: Check_ready()
  BG_task --> Main: Task Ready

  opt Task Ready
    Main -> BG_task: BG_update()
    BG_task -> BG_task: BG_sm_actions()
    BG_task -> BG_task: BG_sm_transitions()
  end

end

\enduml
