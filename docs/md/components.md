# Software Component Modules {#components}

See modules section (software components are identified by defgroup directive in Doxygen comments).

The class diagram illustrates dependency and various interactions between modules.

\startuml

class           BLDC_sm {
  BLDC_STATE_T BLDC_state
  uint16_t BLDC_commut_pd
  uint16_t Set_speed
  uint16_t UI_speed
  void BLDC_stop()
  uint16_t BLDC_get_commut_pd()
  BLDC_STATE_T BLDC_get_state()
  void BLDC_update()
}
class           BG_task{
  uint16_t Analog_slider
  uint8_t UI_speed
  int8_t Digital_trim
  uint8_t Task_rdy_flag
  uint16_t Vsystem
  void Task_set_rdy()
  void Task_Chk_rdy()
}
class           Stepper{
  uint8_t Sequence_step
  uint16_t BEMF_fall
  uint16_t BEMF_rise
  uint16_t Get_sys_voltage()
  void Stepper()
}
class           Driver{
  uint16_t ADC_instant
  uint16_t BEMF_fbuf[4]
  void Driver_update()
  void Driver_step()
  void Driver_post_ADC()
  uint16_t Driver_get_ADC()
  uint16_t Driver_get_BEMF()
}
class           Faultm{
  fault_status_t Status_word
  faultm_mat_t fault_matrix[ NR_DEFINED_FAULTS ]
  void Faultm_set(faultm_ID_t)
  fault_status_t Faultm_get_status()
  void Faultm_upd(faultm_ID_t, faultm_assert_t)
  void Faultm_init()
}
class           MCU_stm8{
  void Set_PWM_DC(uint16_t)
  void Stop_PWM()
  void Phase_control()
}
class           stm8_isr{
  void TIM2_UPD_OVF_BRK_IRQHandler()
  void TIM3_UPD_OVF_BRK_IRQHandler()
  void TIM4_UPD_OVF_IRQHandler()
  void ADC2_IRQHandler()
}

stm8_isr ..> Driver
note on link
Driver_update()
Driver_step()
Driver_post_ADC_con()
end note

Driver .down.> Stepper
note on link: Stepper()

Driver ..> BLDC_sm
note on link: BLDC_update

Driver .> BG_task
note on link: Task_set_ready()

BG_task ..> Faultm
note on link
    Faultm_upd()
    Faultm_set()
end note

BG_task ..> BLDC_sm
note on link: BLDC_get_state()
BG_task ..> Stepper
note on link: Get_sys_voltage()
BG_task ..> Driver
note on link
    Driver_get_pulse_perd()
    Driver_get_pulse_dur()
end note

BLDC_sm ..> Faultm
note on link
    Faultm_get_status()
    Faultm_init()
end note

BLDC_sm ..> MCU_stm8
note on link
    Set_PWM_DC()
    Stop_PWM()
end note

Stepper .up.> Driver
note on link
    Driver_get_ADC()
    Driver_get_BEMF()
end note

Stepper ..> MCU_stm8
note on link: Phase_control()

\enduml

