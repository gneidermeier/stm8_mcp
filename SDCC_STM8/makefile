# STM8 device (STM8S105 Discovery)
DEVICE       =STM8S105
MCU          =stm8s105?6

STLINK = stlink # stlinkv2

SDCC         =sdcc

# Add process specific arguments here
CFLAGS       = -mstm8
#CFLAGS      +=--opt-code-size
#CFLAGS      +=--std-sdcc99
#CFLAGS      +=--nogcse
#CFLAGS      +=--vc --debug

CFLAGS       = -mstm8 --all-callee-saves --debug --verbose --stack-auto --fverbose-asm  --float-reent --no-peep
#CFLAGS      +=--all-callee-saves
#CFLAGS      +=--debug
#CFLAGS      +=--verbose
#CFLAGS      +=--stack-auto
#CFLAGS      +=--fverbose-asm
#CFLAGS      +=--float-reent
#CFLAGS      +=--no-peep
LDFLAGS       = -mstm8 

SOURCE       = main
SOURCE_DIR   = .
OUTPUT_DIR   = $(SOURCE_DIR)/build
#OUTPUT_DIR   = .
StdPeriph    = $(SOURCE_DIR)/STM8S_StdPeriph_Driver

# Add include paths here
INCLUDEPATH  = -I$(SOURCE_DIR)/
INCLUDEPATH += -I$(StdPeriph)/inc
# GN: the stm8_mcp sources are relative to makefile working directory
INCLUDEPATH += -I../inc

def: compile flash

all: clean compile_obj compile

compile:
	$(SDCC)    $(LDFLAGS) --out-fmt-ihx  -o $(OUTPUT_DIR)/ $(OUTPUT_DIR)/$(SOURCE).rel  $(OUTPUT_DIR)/stm8s_gpio.rel
#	$(OUTPUT_DIR)/stm8s_clk.rel \
#	$(OUTPUT_DIR)/stm8s_tim2.rel \
#	--out-fmt-elf --all-callee-saves --debug --verbose --stack-auto --fverbose-asm  --float-reent --no-peep

compile_obj:
	mkdir -p $(OUTPUT_DIR)
	$(SDCC) $(CFLAGS) $(INCLUDEPATH) -D $(DEVICE) -o $(OUTPUT_DIR)/ -c $(StdPeriph)/src/stm8s_gpio.c
	$(SDCC) $(CFLAGS) $(INCLUDEPATH) -D $(DEVICE) -o $(OUTPUT_DIR)/ -c $(SOURCE_DIR)/$(SOURCE).c	
#	$(SDCC) $(CFLAGS) $(INCLUDEPATH) -D $(DEVICE) -o $(OUTPUT_DIR)/ -c $(StdPeriph)/src/stm8s_clk.c
#	$(SDCC) $(CFLAGS) $(INCLUDEPATH) -D $(DEVICE) -o $(OUTPUT_DIR)/ -c $(StdPeriph)/src/stm8s_tim2.c

clean:
	rm -f $(OUTPUT_DIR)/*.rel  $(OUTPUT_DIR)/*.lst $(OUTPUT_DIR)/*.sym $(OUTPUT_DIR)/*.rst $(OUTPUT_DIR)/*.asm
	rm -f $(OUTPUT_DIR)/*.map  $(OUTPUT_DIR)/*.elf $(OUTPUT_DIR)/*.ihx $(OUTPUT_DIR)/*.lk $(OUTPUT_DIR)/*.adb
	
flash:
	stm8flash -c $(STLINK) -p $(MCU) -w $(OUTPUT_DIR)/$(SOURCE).ihx

# make stlink work ... see https://github.com/hbendalibraham/stm8_started/issues/1
openocd:
	openocd -f interface/stlink-dap.cfg -f target/stm8s105.cfg -c "init" -c "reset halt"

gdb:
	$(SDCC) $(LDFLAGS) --out-fmt-elf  -o $(OUTPUT_DIR)/stm8debug.elf $(OUTPUT_DIR)/$(SOURCE).rel  $(OUTPUT_DIR)/stm8s_gpio.rel
	stm8-gdb $(OUTPUT_DIR)/stm8debug.elf --tui
	
