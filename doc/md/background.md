## Background Task Operations {#background}

The background task deals with the UI and the communication stack so its exact
timing can't be known but worst case must not exceed some portion of the system 
frame time (TIM4 period).


### Throttle Input Handler

The analog slider and digital trim switch are for developement use only. The normal 
throttle control use case is the standard RC radio servo proportional signal. 

Spektrum and other newer receivers have a friendly voice warning if throttle high. 
However, the ESC should still do its own detection.


\startuml

  start

    :adc_tmp16 = ADC1_GetBufferValue( ADC1_CHANNEL_3 );
    :adc_tmp16 = ADC1_GetBufferValue( ADC1_CHANNEL_3 );
    :Analog_slider = adc_tmp16 / 4;
    :Analog_slider = adc_tmp16 / 4;
    floating note
      ADC range [ 0: 1023 ] 
      PWM range [ 0: 250 ]
    end note
    :UI_Speed = 0;

    :tmp_sint16 = Digital_trim_switch;
    :tmp_sint16 = Digital_trim_switch;
    note right: dig. trim is signed
    :tmp_sint16 += Analog_slider;
    :tmp_sint16 += Analog_slider;
    floating note: clip negative to 0
    if (tmp_sint16 > 0) then
    if (tmp_sint16 > 0) then
        if (tmp_sint16 > THR_MAX) then
        if (tmp_sint16 > THR_MAX) then
            :tmp_sint16 = THR_MAX;
            :tmp_sint16 = THR_MAX;
        endif
        :UI_Speed = tmp_sint16;
        :UI_Speed = tmp_sint16;
    endif
    
    if (RESET == sm_state) then
        floating note
          require stick to be 
          down before arm/ready
        end note
        if (Analog_slider > 0) then
            :Faultm_set(THROTTLE_HI);
            :UI_Speed = 0;
        endif
    elseif (RUNNING == sm_state)
        if (Analog_slider > 0) then
            floating note: prevent going to slow
            if (UI_Speed < LOW_SPEED_THR) then
                :UI_Speed = U8_MAX;
                note
                  U8_MAX signifies to 
                  call BLDC_Stop()
                end note  
            endif
        endif
    endif

    :BLDC_PWMDC_Set(UI_Speed);
  stop

\enduml
