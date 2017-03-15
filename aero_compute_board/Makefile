# Source: https://github.com/mfischer/Altera-Makefile
###################################################################
# Project Configuration:
#
# Specify the name of the design (project) and the Quartus II
# Settings File (.qsf)
###################################################################

PROJECT = aero_compute_board_only_fpga
TOP_LEVEL_ENTITY = main
ASSIGNMENT_FILES = $(PROJECT).qpf $(PROJECT).qsf

###################################################################
# Part, Family, Boardfile DE1 or DE2
FAMILY = "MAX 10"

# Aero
PART = 10M08SAM153I7G
BOARDFILE = config/MAX10M08.cfg

###################################################################

###################################################################
# Select your cable type here.
# Make sure it has been installed via the programmer GUI
# in Quartus.
#
# Examples:
#   USB-Blaster
#   ByteBlasterMV
CABLE=USB-Blaster
QUARTUS_PGM=quartus_pgm
###################################################################

###################################################################
# Setup your sources here

SRCS = src/main.v

###################################################################
# Main Targets
#
# all: build everything
# clean: remove output files and database
# program: program your device with the compiled design
###################################################################

all: smart.log $(PROJECT).asm.rpt $(PROJECT).sta.rpt $(PROJECT).jam

clean:
	rm -rf *.rpt *.chg smart.log *.htm *.eqn *.pin *.sof *.pof *.smsg *.summary *.jdi *.sld db incremental_db output_files *.qpf *.qsf *.jam *.done *.qws

map: smart.log $(PROJECT).map.rpt
fit: smart.log $(PROJECT).fit.rpt
asm: smart.log $(PROJECT).asm.rpt
sta: smart.log $(PROJECT).sta.rpt
smart: smart.log

###################################################################
# Executable Configuration
###################################################################

MAP_ARGS = --read_settings_files=on --family=$(FAMILY) --part=$(PART) $(addprefix --source=,$(SRCS))

FIT_ARGS = --part=$(PART) --read_settings_files=on
ASM_ARGS =
STA_ARGS =

###################################################################
# Target implementations
###################################################################

STAMP = echo done >

$(PROJECT).map.rpt: map.chg $(SOURCE_FILES)
	quartus_map $(MAP_ARGS) $(PROJECT)
	$(STAMP) fit.chg

$(PROJECT).fit.rpt: fit.chg $(PROJECT).map.rpt
	quartus_fit $(FIT_ARGS) $(PROJECT)
	$(STAMP) asm.chg
	$(STAMP) sta.chg

$(PROJECT).asm.rpt: asm.chg $(PROJECT).fit.rpt
	quartus_asm $(ASM_ARGS) $(PROJECT)

$(PROJECT).sta.rpt: sta.chg $(PROJECT).fit.rpt
	quartus_sta $(STA_ARGS) $(PROJECT)

smart.log: $(ASSIGNMENT_FILES)
	quartus_sh --determine_smart_action $(PROJECT) > smart.log

$(PROJECT).jam: $(PROJECT).pof
	quartus_cpf -c $(PROJECT).pof $(PROJECT).jam

###################################################################
# Project initialization
###################################################################

$(ASSIGNMENT_FILES):
	quartus_sh --prepare -f $(FAMILY) -t $(TOP_LEVEL_ENTITY) $(PROJECT)
	-cat config/common.cfg >> $(PROJECT).qsf
	-cat $(BOARDFILE) >> $(PROJECT).qsf
map.chg:
	$(STAMP) map.chg
fit.chg:
	$(STAMP) fit.chg
sta.chg:
	$(STAMP) sta.chg
asm.chg:
	$(STAMP) asm.chg

###################################################################
# Programming the device
###################################################################

program-volatile: $(PROJECT).sof
	quartus_pgm --no_banner -c $(CABLE) --mode=jtag -o "P;$(PROJECT).sof"
program: $(PROJECT).pof
	quartus_pgm --no_banner -c $(CABLE) --mode=jtag -o "BPV;$(PROJECT).pof"
