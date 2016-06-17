TOP=../..
include $(TOP)/configure/CONFIG

LIBRARY_IOC += vmi5588

INC += vmi5588.h
DBD += vmi5588.dbd

vmi5588_SRCS += drvVmi5588.c
vmi5588_SRCS += devBiVmi5588.c


# optional pingpong test facility
LIBRARY_IOC += pingpong
pingpong_SRCS += pingpong.c
DBD += pingpong.dbd



include $(TOP)/configure/RULES


