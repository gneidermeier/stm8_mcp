# Software Component Modules {#components}

See modules section (software components are identified by defgroup directive in Doxygen comments).

The class diagram illustrates dependency and various interactions between modules.


\startuml

class           BLDC_sm {
  BLDC_STATE_T BLDC_state
  uint16_t BLDC_commut_pd
  uint16_t BLDC_commut_pd
  uint16_t Set_speed
  uint16_t Set_speed
  uint16_t UI_speed
  uint16_t UI_speed
  void BLDC_stop()
  uint16_t BLDC_get_commut_pd()
  uint16_t BLDC_get_commut_pd()
  BLDC_STATE_T BLDC_get_state()
  void BLDC_update()
}
class           BG_task{
  uint16_t Analog_slider
  uint16_t Analog_slider
  uint8_t UI_speed
  int8_t Digital_trim
  uint8_t Task_rdy_flag
  uint16_t Vsystem
  uint16_t Vsystem
  void Task_set_rdy()
  void Task_Chk_rdy()
}
class           Stepper{
  uint8_t Sequence_step
  uint16_t BEMF_fall
  uint16_t BEMF_fall
  uint16_t BEMF_rise
  uint16_t BEMF_rise
  uint16_t Get_sys_voltage()
  uint16_t Get_sys_voltage()
  void Stepper()
}
class           Driver{
  uint16_t ADC_instant
  uint16_t ADC_instant
  uint16_t BEMF_fbuf[4]
  uint16_t BEMF_fbuf[4]
  void Driver_update()
  void Driver_step()
  void Driver_post_ADC()
  uint16_t Driver_get_ADC()
  uint16_t Driver_get_ADC()
  uint16_t Driver_get_BEMF()
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

stm8_isr ..> "Driver_update()" Driver
stm8_isr ..> "Driver_step()" Driver
stm8_isr ..> "Driver_post_ADC_con()" Driver

Driver ..> "Stepper()" Stepper
Driver ..> "BLDC_update()" BLDC_sm
Driver ..> "Task_set_ready()" BG_task

BG_task ..> "Faultm_upd()" Faultm
BG_task ..> "Faultm_set()" Faultm
BG_task ..> "BLDC_get_state()" BLDC_sm
BG_task ..> "Get_sys_voltage()" Stepper

BLDC_sm ..> "Faultm_get_status()" Faultm
BLDC_sm ..> "Faultm_init()" Faultm
BLDC_sm ..> "Set_PWM_DC()" MCU_stm8
BLDC_sm ..> "Stop_PWM()" MCU_stm8

Stepper ..> "Driver_get_ADC()" Driver
Stepper ..> "Driver_get_BEMF()" Driver
Stepper ..> "Phase_control()" MCU_stm8

\enduml

