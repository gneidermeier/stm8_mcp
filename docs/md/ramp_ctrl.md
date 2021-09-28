# Commutation Timing Control {#ramp_ctrl}

## Ramp Timing Control

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

The code is easier to read than the diagram. The difference between the present value and
target value is taken for the sign/direction of ramp. The ramp unit is added/subtracted
on a temp copy of the present timing value so that it can be tested for overshoot/undershoot of the target value.

\startuml

  start
    :u16 = BLDC_OL_comm_tm;
    note right: grab current timing value

    if (u16 > tgt_commutation_per) then (yes)
        :u16 -= ONE_RAMP_UNIT;
        if (u16 < tgt_commutation_per) then (yes)
            :u16 = tgt_commutation_per;
            note left: undershoot, clamp to target
        endif
        :BLDC_OL_comm_tm  = u16;
    elseif (u16 < tgt_commutation_per) then (else)
        :u16 += ONE_RAMP_UNIT;
        if (u16 > tgt_commutation_per) then (yes)
            :u16 = tgt_commutation_per;
            note left: overshoot clamp to target
        endif
        :BLDC_OL_comm_tm  = u16;
    endif
  stop

\enduml

## Open Loop Timing Control

In OPEN_LOOP_CONTROL state, the motor speed is held at the speed it reached at the
end of the ramp as it begins to sample the back-EMF signal and compute the 
control error term. If the back-EMF value is plausible and the control error 
remains within the pre-determined controllability range, the software state 
transitions to CLOSED LOOP CONTROL.

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

## Closed Loop Timing Control

In CLOSED LOOP CONTROL, the proportional control algorithm is engaged: This is 
implemented by multiplying the control error term by a constant of proportionality
(kP) and summing the resultant control output to the present motor speed setpoint.

\startuml
  start

  :error_term = (1 * scale_64) - (back_EMF_falling * scale_64 / back_EMF_rising);

  if ((error_term > ERROR_MIN) && (timing_error < ERROR_MAX)) then
      :correction = timing_error * PROP_GAIN;
      :BL_set_timing(current_setpoint + correction);
      :Controllable = TRUE;
  else
        :Controllable = FALSE;
  endif
  
  stop

\enduml
