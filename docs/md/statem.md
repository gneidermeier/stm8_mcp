# State Machine Representation {#statem}

The motor operation cycle is represented in the software design as a
state-machine.

From powerup, the software state transitions to ARMING state.
In ARMING state, the system checks for valid operating environment including
 validity of system voltage and control signal.

Applying speed signal greater than a minimum start speed of around 10%, ALIGNMENT
is initiated at which time the software attempts to synchronise the motor position 
to a known step in the commutation sequence. This is accomplished by driving the
 active motor phase with a PWM duty-cycle that allows sufficient current to energize 
 the coil enough to assert the rotor fully into a known slot in the commutation 
 cycle. A half second is allowed for rotor alignemnt.

Upon reaching the end of the alignment period, the software transitions to RAMP 
state, where motor is sped up to stable, minimal operating speed by progressively 
shortening the length of the commutation period causing the motor to spin faster. 
It is speculated that some motors may respond better to a ramp derived from an 
exponential function, as opposed to the linear one presently being used. The 
ramp-to speed must be determined by the readability of the back-EMF signal, which 
will vary according to the kv rating of the BL motor. Upon attaining the  ramp-to 
speed, the software state transitions from RAMP to OPEN LOOP CONTROL.

In OPEN_LOOP_CONTROL state, the motor speed is held at the speed it reached at the
end of the ramp as it begins to sample the back-EMF signal and compute the 
control error term. If the back-EMF value is plausible and the control error 
remains within the pre-determined controllability range, the software state 
transitions to CLOSED LOOP CONTROL. 

In CLOSED LOOP CONTROL, the proportional control algorithm is engaged: This is 
implemented by multiplying the control error term by a constant of proportionality
(kP) and summing the resultant control output to the present motor speed setpoint.

Transition to STOPPED occurs when 1) user speed-control input falls below minimum 
motor operating speed  2) error condition such as stalled motor detected.

\startuml

[*] -> ARMING:
ARMING -> STOPPED: [servo pulse == VALID]
STOPPED -down-> ALIGNMENT: [servo input > ARMED]
ALIGNMENT -down-> RAMPUP: [alignment time == ELAPSED]
RAMPUP -down-> OL_CONTROL: [motor speed >= ramp to speed]
OL_CONTROL -> CL_CONTROL: [control error < threshold && backEMF == readable]
CL_CONTROL -> STOPPED:
RAMPUP -down-> STOPPED:
OL_CONTROL -down-> STOPPED:
CL_CONTROL -down-> STOPPED:

\enduml
