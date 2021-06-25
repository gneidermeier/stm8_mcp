# Fault Monitor  {#faultm}

## Fault Matrix

fault_matrix is an array of faultm_mat_t that implements the fault tracking table.

\startuml

start
    :mask = faultm_ID;
    note right: mask = *alternatively* (1 << faultm_ID)
    :pfaultm->state =  (FALSE != pfaultm->enabled);
    :pfaultm->bucket = -1;
    note right
      set bucket to max signifies latched - in lieu of explicit state variable
      -1 is always max unsigned int and thus > *faultm_mat_t:bucket*
    end note

    :fault_status_reg |= mask;
    note: 8-bit status-word
stop

\enduml

The system word is available externally by Faultm_get_status().

## Fault Persistence

A fault diagnostic can be managed with leaky bucket algorithm to provide immunity
to noisy signal causing false triggering. If the error-condition persists, the
bucket increments until the threshold at which the fault will be latched, otherwise
the bucket counter would eventually decerement to 0 so no history of the fault remains.
Note this allows/assumes the tolerance of a condition for some amount of
of time (a high-current load might be tolerated but a short-circuit probably not).  

*Faultm_upd()* latches the fault activation upon number of occurences of the
test-condition reaching a threshold. Ideally, the threshold could be configured to
each fault individually (by writing code to initialize explicitly, or alternatively some const array).

The *Faultm_upd()* function call is intended to be invoked with the fault-diagnostic test-condition
on a periodic scheduled update - with this being performed in the periodic (background) task, it is
updated at a rate of around twice a millisecond.

     Faultm_upd(VOLTAGE_NG, Vsystem < V_SHUTDOWN_THR);

Leaky bucket does not necessarily apply to all diagnostic conditions. For example, the THROTTLE_HIGH
fault is set/cleared explicitly calling *Faultm_set( fault_ID )* with the ID of the fault code.

\startuml

start

:pfault = fault_matrix[fault_ID];
note right: pointer to element at index
:Threshold = pfault->threshold;
note right
 Threshld should be cfg'able per
 fault (in code, or const-array)
end note

if (tcondition) then (yes)
    if ( (pfault->bucket) < Threshold ) then (yes)
        :pfault->bucket += 1;
    else (no)
        :Faultm_set( fault_ID );
        note right
          - sets status word
          - once the fault is set,
            it is not cleared
        end note
    endif
else (no)
    if ( pfault->bucket > 0 ) then (yes)
        :pfault->bucket -= 1;
    endif
endif

stop

\enduml
