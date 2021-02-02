# Commutation Timing Control {#ramp_ctrl}

## Ramp Timing

At each time-step of the state-machine, BLDC_OL_comm_tm decrements by 
BLDC_ONE_RAMP_UNIT. The slope would be expressed by the ratio RAMP_UNIT / EXEC_RATE 
The ramp function likely should be executed in the ISR context (see TIM4 setup)
and not the Background task to ensure ramp timing not affected by interrupt latency.

When the BLDC_OL_comm_tm  has been decremented to the end ramp point 
(BLDC_OL_TM_HI_SPEED), the state-machine logic shall transition to state RUNNING, and
the motor is expected to have attained stable operating condition and discernable back-EMF voltage is present. 

BLDC_Update() is called from TIM4_UPD_OVF_IRQHandler ISR and encapsulates any control functions that 
have deterministic timing requirements i.e. those related directly to motor control and timing.

TIM4 set to 0.000512 S  (512 uS) - see Platform STM8S MCU_Init().
TIM4 set to 0.000512 S  (512 uS) - see Platform STM8S MCU_Init().

The code is easier to read than the diagram. The difference between the present value and
target value is taken for the sign/direction of ramp. The ramp unit is added/subtracted 
on a temp copy of the present timing value so that it can be tested for overshoot/undershoot of the target value. 

\startuml

  start
    :u16 = BLDC_OL_comm_tm;
    :u16 = BLDC_OL_comm_tm;
    note right: grab current timing value

    if (u16 > tgt_commutation_per) then (yes)
    if (u16 > tgt_commutation_per) then (yes)
        :u16 -= ONE_RAMP_UNIT;
        :u16 -= ONE_RAMP_UNIT;
        if (u16 < tgt_commutation_per) then (yes)
        if (u16 < tgt_commutation_per) then (yes)
            :u16 = tgt_commutation_per;
            :u16 = tgt_commutation_per;
            note left: undershoot, clamp to target
        endif
        :BLDC_OL_comm_tm  = u16;
        :BLDC_OL_comm_tm  = u16;
    elseif (u16 < tgt_commutation_per) then (else)
    elseif (u16 < tgt_commutation_per) then (else)
        :u16 += ONE_RAMP_UNIT;
        :u16 += ONE_RAMP_UNIT;
        if (u16 > tgt_commutation_per) then (yes)
        if (u16 > tgt_commutation_per) then (yes)
            :u16 = tgt_commutation_per;
            :u16 = tgt_commutation_per;
            note left: overshoot clamp to target
        endif
        :BLDC_OL_comm_tm  = u16;
        :BLDC_OL_comm_tm  = u16;
    endif
  stop

\enduml



## Open Loop Timing 

In open loop operation the commutation timing lookup table is a statically
generated mapping of commutation period across the operating range of the duty-cycle
at speed/dc greater than ramp.

\startuml

  start

  if (state == RAMP) then (ramp)
    :Commanded_Dutycycle = RAMP_DUTYCYCLE;
    :tgt_timing = Get_OL_Timing( Commanded_Dutycycle );
    :timing_ramp_control( tgt_timing );

  elseif (state == RUNNING) then
    :Commanded_Dutycycle = get_DC_from_UI_speed();
    note right: convert/scale UI speed to PWM DC

    if (closed_loop_operation) then
      :tgt_timing = TBD();
    else
      :tgt_timing = Get_OL_Timing( Commanded_Dutycycle );
      note right: fall back to timing table
    endif
    :timing_ramp_control( tgt_timing );
  endif


  stop

\enduml


