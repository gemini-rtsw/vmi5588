TOP=../..
include $(TOP)/configure/CONFIG

#CROSS_OPT = NO
LIBRARY_IOC += vmi5588

INC += vmi5588.h

vmi5588_SRCS += drvVmi5588.c
vmi5588_SRCS += devBiVmi5588.c

DBD = vmi5588.dbd
vmi5588_DBD += base.dbd
vmi5588_DBD += vmi5588driver.dbd

# optional pingpong test facility
LIBRARY_IOC += pingpong
pingpong_SRCS += pingpong.c
DBD += pingpong.dbd

SCRIPTS_HOST += ../vmi5588.py
SCRIPTS_HOST += ../vmi5588.adl

include $(TOP)/configure/RULES


