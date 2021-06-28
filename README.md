# Introduction

This is a minimalistic implementation of a brushless motor driver on STM8
microcontrollers. It is presently limited to the small (1100kv) brushless
outrunners such as seen on cheap drones and RC planes.

It's function is to perform the commutation of a typical trapezoidally wound
brushless PMSM (permanent magnet synchronous motor). 
The software drives the motor by applying the 6-step drive sequence.
Timing of the motor is obtained by measurement of the motor back-EMF using ADC
input to the STM8. This eliminates the dependency on a Hall sensor for determining motor commutation timing.

# Obtaining and installing toolchain

Presently the de-facto toolchain is ST Visual Develop with COSMIC C compiler -
these tools have free licenses, but are available for Windows OS only.

`sdcc` (available for multiple OS) may be supported at some point and eventually 
transition to [patched for use with SDCC](https://github.com/bschwand/STM8-SPL-SDCC) 
as that one has all C files split into one function per file (SDCC linker 
unable to perform dead code elimination?) 

Download IDE and compiler toolchain:

* Download STVD [STVD](https://www.st.com/en/development-tools/stvd-stm8.html)
* Install STVD - run the downloaded STVD installer 'sttoolset_pack42.exe'
* Download [COSMIC STM8 C Compiler](https://www.cosmicsoftware.com/download_stm8_free.php)
* Install COSMIC STM8 C Compiler and restart machine (to update PATH etc.)
* Follow instructions to register for the free license for COSMIC STM8 C Compiler
* Save COSMIC license file to e.g :
    `C:\Program Files (x86)\COSMIC\FSE_Compilers\CXSTM8\License_`

# Working with stm8_mcp project source code

## Installing STSW-STM8069 STM8S/A Standard peripheral library

* Navigate browser to [Standard peripheral library homepage - application note STSW-STM8069](https://www.st.com/en/embedded-software/stsw-stm8069.html) 
* Register with ST if required to do so. 
* Get an email from ST with link to download en.stsw-stm8069.zip (SPL Version 2.3.1)
* unzip `en.stsw-stm8069.zip` into your project directory.

Cloning source code from github:

* `cd Your_project_directory/STM8S_StdPeriph_Lib/Project/`
* `git clone https://github.com/gneidermeier/stm8_mcp.git` (master branch)
* Open project in STVD, 'File | Open Workspace' .. browse to
   `STM8S_StdPeriph_Lib\Project\stm8_mcp\STVD\Cosmic\STVD_workspace.stw`

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

# Running the controller with a motor
The critical component of the hardware is the [3-phase driver](https://simple-circuit.com/wp-content/uploads/2017/12/brushless-dc-motor-3-phase-bridge-circuit.png) 
circuit. The 3-phase driver circuit is implemented in large part by the [IR2104 
half-bridge IC](https://www.infineon.com/dgdl/Infineon-IR2104-DS-v01_00-EN.pdf?fileId=5546d462533600a4015355c7c1c31671). Most variations of the this kind of circuit use an [N channel MOSFET](https://www.infineon.com/dgdl/irfz44npbf.pdf?fileId=5546d462533600a40153563b3a9f220d)
for both the high-side as well as low-side of the half-bridge. The IR2104 incorporates a 
built-in charge pump for driving the high-side (Vgs of HS going to be significantly 
higher than Vbat).
 
[Reference project](https://simple-circuit.com/arduino-sensorless-bldc-motor-controller-esc/) 
implements the controller with an Arduino Uno, which has the 
additional capability of the input comparator. This provides a means for detecting zero-crossing point and thus direct measurement of motor timing.

It is possible to run the software without a motor connected.
Connecting a motor can cause the potential for more dramatic failure scenario so 
do it at your own risk to your dev board and equipment!

The following equipment is suggested:
* stm8s105 development board (stm8s105c Discovery or stm8s105k "Black" board)
* 1100kv brushless motor
* Benchtop power supply (alternatively, 11.1v 3s pack)
* FTDI cable (for remote user interface)
* A 3-phase driver circuit e.g. the hardware schematic (in Kikad) for the stm8s105k
 configuration can be [found in this repositiory](https://github.com/gneidermeier/stm8_mcp/blob/master/docs/schem/bl_hw_stm8s105k_blue.tar.gz)

 Connect the FTDI cable (orange to STM8 Rx, yellow to STM8 Tx)
 Power the STM8 dev board (USB micro connector or 5 volt power source connected
 to STM8 5v pin).

 Power the bench supply (keep eye on current meter)!

 Use remote serial terminal (via e.g. PuTTY, Tera Term, Minicom) to interact 
 with motor software.

  * Pressing comma or period keys (< and > signs) starts the motor if
   not running.
  * Once motor is running, pressing comma or period keys (< and > signs) increases/decreases the PWM duty-cycle applied to the motor phase   
  * Space bar stops the motor.

 Motor operation is limited to about 50% duty-cycle.

# Build Documentation

The project documentation is generated from Doxygenized comments in the code 
and supplemented by diagrams generated from markup by PlantUML. GitHub pages 
is set to serve from a `gh-pages` branch of the project repo - [Documentation link](https://gneidermeier.github.io/stm8_mcp/docs/index.html).

GitHub now provides option for [Simpler GitHub Pages publishing](https://github.blog/2016-08-17-simpler-github-pages-publishing/) which could be considered.

## Prerequisites:

* [Doxygen] {https://www.doxygen.nl/download.html} - available for Windows as 
self-installing archive and included GUI, or as 32-bit or 64-bit standalone binary zip if preferred.
* [PlantUML](https://plantuml.com/download) is distributed as a single jar 
* (java archive) and requires a functional java (installation of which not 
covered here, sorry!)
* Set `PLANTUML_JAR_PATH` environment variable to location of `plantuml.jar`
** Windows 10 and Windows 8
** In Search, search for and then select: System (Control Panel)
** Click the Advanced system settings link.
** Click Environment Variables. In the section System Variables, add the new system variable by selecting New

## To build the documentation:

1. Open the Command Prompt (or alternatively, Power Shell) in Windows and 
 execute the command `cd <project_root>` to navigate into the project root 
 directory.
2. In the project root directory type the command 
 `doxygen docs/config/doxyfile.cfg`

This will generate the documentation under the `<project_root>/build` folder. Open
`<project_root>/build/html/index.html` to view the HTML documentation.

## Update documentation on GitHub pages

In a separate project folder, create a clone of the project repo in which to 
checkout the `gh-pages` 
[https://github.com/gneidermeier/stm8_mcp/tree/gh-pages](branch). 

In the gh-pages clone-repo, create an orphan branch 'gh-pages'. Replace all of 
the contents in the `/docs` folder of the `gh-pages` branch with the contents
of `<project_root>/build/html` in the build branch. This is shown below in 
Unix shell syntax for clarity (we have Git Bash and/or Cygwin on our Windows 
workstations). 

    cd /path/to/my/clone_clone_of_gh-pages
    git checkout --orphan gh-pages
    git rm -rf .
    rm '.gitignore'
    echo "#Title of Readme" > README.md
    git add README.md
    cp -r `<project_root>/build/html/* /docs
    git commit -a -m "Initial Commit"
    git push -u origin gh-pages


