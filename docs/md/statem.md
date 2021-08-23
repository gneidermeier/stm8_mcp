# State Machine Representation {#statem}

The motor operation cycle is represented in the software design as a
state-machine.

From powerup, the software initializes the state-machine to RESET.
In RESET, the system attempts to reinitialize and clear faults.

Applying speed signal greater than 0 initiates ALIGNMENT, where the controller 
applies one commutation step with the active motor phase driven with a PWM 
duty-cycle that induces current to energize it enough to pull the rotor 
into place. 

The state-machine transitioned to RAMP, where motor output is ramped 
to stable, minimal operating speed by progressively shortening the length of the
commutation period causing the motor to spin faster. Some motors may respond
better to an exponential ramp as opposed to a linear one.

Transition to RUNNING occurs once the motor attains a stable, minimal operating 
speed and the software is able to detect the zero-crossing point and control the
motor timing by closed-loop control.

Transition to STOP occurs when 1) user speed-control input falls below minimum 
motor operating speed  2) error condition such as stalled motor detected.

\startuml

[*] -> RESET: powerup
RESET -down-> STOPPED:
STOPPED -down-> ALIGN: [servo input > ARMED]
ALIGN -down-> RAMP:
RAMP -down-> RUNNING:
RUNNING -> STOPPED: BLDC_Stop()
RUNNING -> FAULT
FAULT -> RESET

\enduml
