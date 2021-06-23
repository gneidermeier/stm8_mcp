# State Machine Representation {#statem}

The motor operation cycle is represented in the software design as a
state-machine.

From powerup, the software initializes the state-machine to RESET.
In RESET, the system attempts to reinitialize and clear faults.

Applying speed signal greater than 0 initiates RAMP - the motor is induced
to start turning by initiating the commutation sequence at the maximum timing
period at the fixed RampupDC PWM value - progressively shortening the length of the
commutation period causes the motor to spin faster.

Transition to RUNNING occurs when the commutation time period length converges
on the period length value from the open-loop timing table (indexed by the Duty
Cycle value.

There is no provision for reversing the sequence (airplane ESCs don't have reverse).

Presently there is no specific requirement for alignment state (stepping
the motor to a known position from which either the forward
or reverse commutation switch sequence can be initiated).

\startuml

[*] -> RESET: powerup
RESET -down-> READY: [UI_speed > 0]
READY -down-> RAMP: [UI_speed > _RampupDC_]
RAMP -down-> RUNNING: [ BLDC_OL_comm_tm <= Get_OL_Timing( _RampupDC_ )]
RUNNING -> RESET: BLDC_Stop()
RUNNING -> FAULT
FAULT -> RESET : BLDC_Stop()

\enduml
