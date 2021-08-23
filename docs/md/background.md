## Background Task Operations {#background}

The Background Task encompasses all code executed in the context of main() i.e.
outside of an ISR context. Code executed in the ISR context(s) must be minimal in 
order for the system to maintain real-time operation which is a must for motor 
control). 

### Servo input handler

The analog slider and digital trim switch are for developement use only. The normal
throttle control use case is the standard RC radio servo proportional signal.


### Data acquisition and logging

The controller communicates through the serial uart which suppports printf().
print() is blocking so should be constrained to the Background context. Most of 
the code runs in an ISR context and due to this consideration, printf() is not 
available in most places where there is information of interest. Therefore a 
Data Dictionary (DD) class is envisioned which would aggregate system data and 
integrate with the commucation statck, UI and data acquisition functionalities.

@subpage data_dict
