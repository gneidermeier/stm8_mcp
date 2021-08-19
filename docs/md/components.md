# Software Component Modules {#components}

See modules section (software components are identified by defgroup directive in Doxygen comments).

The class diagram illustrates dependency and various interactions between modules.

\startuml

class           CPU_abstraction {
  void ISR_System_timer(void)
  void ISR_Commutation_timer(void)
  void ISR_PWM_edge(void)
}
class           Sys_Driver {
  void On_PWM_edge(void)
  void On_Time_Capture(void)
  void On_system_timer(void)
  void On_commutation_timer(void)
}
class           BL_machine {
  void BL_reset(void)
  void BL_On_systick(void)
}
class           Phase_Switcher {
  Sequence_Step(void)
  void Sequence_Step_0(void)
}
class           MCU_Interface {
  void MCU_init(void)
  int putchar(char c)
  int getchar(void)
}
class           PWM_Phase {
  void PWM_PhX_Disable(void)
  void PWM_PhX_Enable(void)
  void PWM_set_dutycycle(uint16_t)
  void PWM_setup(void)
}
class           Fault_Mon {
  void Faultm_init(void)
  void Faultm_upd(faultm_ID_t faultm_ID, faultm_assert_t tcondition)
  void Faultm_set(faultm_ID_t faultm_ID)
  void Faultm_clr(faultm_ID_t faultm_ID)
  fault_status_reg_t Faultm_get_status(void)
}
class           BG_Task {
  void BG_task_wake()
  void BG_task_exec()  
}
class           Term_IO {
}
class           Data_Dictionary {
    void * Sys_vars_LUT[16]
}
class           PDU_Manager {
}
class           User_Interface {
  void Log_print()
}

CPU_abstraction ..> Sys_Driver
note on link
  On_PWM_edge()
  On_timer_Capture()
  On_system_timer()
  On_commutation_timer()
end note

Sys_Driver ..> BL_machine
note on link: BL_On_systick()

Sys_Driver ..> BG_task
note on link: BG_task_wake()

BL_machine ..> PWM_Phase
note on link: PWM_set_dutycycle()

BL_machine ..> Phase_Switcher
note on link: Sequence_Step()

BL_machine ..> Fault_Mon
note on link
    Faultm_init()
    Faultm_get_status()	
end note

Phase_Switcher ..> PWM_Phase

PWM_Phase ..> MCU_Interface

Term_IO ..> MCU_Interface
note on link
  putchar()
  getchar()
end note 

User_Interface ..> Term_IO
note on link: printf()

BG_task ..> User_Interface
note on link: Log_print()

User_Interface ..> Data_Dictionary
User_Interface ..> PDU_Manager

Data_Dictionary ..> BL_machine
Data_Dictionary ..> Fault_Mon
Data_Dictionary ..> Phase_Switcher

\enduml

