# Introduction

This is an implementation of a brushless motor driver on STM8 microcontrollers.
It's function is to perform the commutation of a typical trapezoidally wound
brushless PMSM (permanent magnet synchronous motor). The drive function is achieved
in oftware by applying to the motor what is known as a 6-step drive sequence.
Timing of the motor is obtained by measurement of the motor back-EMF using ADC
input to the STM8. This eliminates the requirement for a hall-effect sensor to
obtain the necessary rotational angle of the motor, from which the motor speed
and timing of the commutation switching is determined.

# Obtaining and installing toolchain

Presently the defacto toolchain is ST Visual Develop with COSMIC C compiler -
these tools have free licenses, but are available for Windows only.

Alternatively, sdcc (Windows or Linux) has been experimented with - unfortunately
it seems that the sdcc linker does not optimize out unused code resulting in
unecessarily large code size, depending on the spedific SPL files compiled into
the project.

Download IDE and compiler toolchain:

* Download STVD [STVD](https://www.st.com/en/development-tools/stvd-stm8.html)
* Install STVD - run the downloaded STVD installer 'sttoolset_pack42.exe'
* Download COSMIC STM8 C Compiler(https://www.cosmicsoftware.com/download_stm8_free.php)
* Install COSMIC STM8 C Compiler and restart machine (to update PATH etc.)
* Follow instructions to register for the free license for COSMIC STM8 C Compiler
* Save COSMIC license file to e.g :
    C:\Program Files (x86)\COSMIC\FSE_Compilers\CXSTM8\License_

# Working with stm8_mcp project source code

Install STSW-STM8069 STM8S/A Standard peripheral library
Information
of latest version 2.3.0:
  Contents
  STM8S_StdPeriph_Driver V2.3.0 (release notes)
  STM8S_StdPeriph_Examples V2.0.4 (release notes)
  STM8S_StdPeriph_Template V2.3.0 (release notes)
  STM8S_EVAL V1.0.1 (release notes)
  ST Visual Develop (STVD) software toolchain
  STVD Version 4.3.12
  Supported compilers:
  Cosmic STM8 32K compiler Version 4.4.7
  Raisonance STM8/ST7 C compiler Version 2.60.16.0273
  Raisonance IDE RIDE7 (RIDE) software toolchain
  IAR Embedded Workbench for STM8 IDE (EWSTM8) software toolchain
  
* Download en.stsw-stm8069.zip (requires registration with ST)
    https://www.st.com/en/embedded-software/stsw-stm8069.html
* unzip en.stsw-stm8069.zip into your project directory

Cloning source code from github:

* cd Your_project_directory/STM8S_StdPeriph_Lib/Project/
* git clone https://github.com/gneidermeier/stm8_mcp.git (master branch)
* Open project in STVD, 'File | Open Workspace' .. browse to
   STM8S_StdPeriph_Lib/Project/stm8_mcp/STVD/Cosmic/STVD_workspace.stw'

Select the active project:

* In the STVD_workspace window, right click e.g. stm8s105_devblk

Selecting the active project sets the platform by inserting the compiler
define '-dS105_DEV' into the compile command.

* Compile and link project (F7)
* Run code in debugger (Alt-D D)

Note on installation/configuration of SPL:

The stm8_mcp project was copied from the SPL template V2.3.0.
(STM8S_StdPeriph_Lib/Project/STM8S_StdPeriph_Template).

As in the template project, the location of the SPL source files is set in the
stm_mcp project as a relative path. The SPL library files are kept external to
the stm_mcp project and not distributed with it, thus the requirement to download
it separately.

If creating a new project and not a copy of the template, be sure to setup the
toolchain in STVD:

* Select the toolchain e.g. 'STM8 Cosmic' and set Toolchain root e.g.:
  C:\Program Files (x86)\COSMIC\FSE_Compilers\CXSTM8
* In project settings, set the MCU Selection e.g. 'STM8S105K6' (issues compiler
  directive -dSTM8S105)

# Running a motor with hardware

Do this at your ownn risk to your dev board and equipment!

The hardware consists of:

* stm8s105 development board (stm8s105c Discovery or stm8s105k "Black" board)
* 1100kv brushless motor
* Benchtop power supply set to 14.2v (see resistor divider ratio on schematic)
  (TODO: eventual support for 11.1v 3s pack)
* FTDI cable (for remote user interface)
* A 3-phase driver circuit - please see reference here:
  https://simple-circuit.com/wp-content/uploads/2018/02/arduino-sensorless-bldc-motor-controller-ir2101.png

 The current version of the hardware schematic (in Kikad) for the stm8s105k
 configuration can be found in this repositiory:
  https://github.com/gneidermeier/stm8_mcp/blob/master/docs/schem/bl_hw_stm8s105k_blue.tar.gz

 Connect the FTDI cable (orange to STM8 Rx, yellow to STM8 Tx)
 Power the STM8 dev board (USB micro connector or 5 volt power source connected
 to STM8 5v pin).

 Power the bench supply (keep eye on current meter)!

 In putty, tera term, minicom etc.:
   Pressing comma or period keys (< and > signs) starts the motor if
   not running, and increases/decreases the PWM duty-cycle applied to the motor phases.
   Space bar stops the motor.

 Motor operation is limited to about 50% duty-cycle.

# Build Documentation

Prerequisites:

* Doxygen installed
* Download `plantuml.jar`
* Set `PLANTUML_JAR_PATH` environment variable to location of `plantuml.jar`

To build the documentation:

1. Open a terminal to the project root directory
2. Execute `doxygen docs/config/doxyfile.cfg`

This will generate the documentation under the `<project_root>/build` folder. Open
`<project_root>/build/html/index.html` to view the HTML documentation.

## Update documentation on GitHub pages

Checkout the `gh-pages` branch. Replace all of the contents in the `/docs` folder of the `gh-pages` branch with the contents
of `<project_root>/build/html` in the build branch.
