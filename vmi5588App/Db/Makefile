TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#
#  # Search for .db files in $SYS_DB_PATHS, the list 
# of paths output by "convertGemRelease.pl dbPaths"
vpath %.db $(SYS_DB_PATHS)
#
#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#TDCT_CFG = ../tdct.cfg
DEPSCHS = $(wildcard ../*.sch)
DB += vmi5588Test.db

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
$(COMMON_DIR)/%.db: %.db
	cp $< $@

