#!/usr/bin/python

import os
import sys
import subprocess

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)
target_ioc = str(sys.argv[1])
print target_ioc

medm = '/gem_sw/epics/R3.14.12.4/extensions/bin/linux-x86_64/medm'
vmi5588 = '/gem_sw/work/R3.14.12.4/support/vmi5588/adl/vmi5588.adl'
p1 = subprocess.Popen(['host', target_ioc], stdout=subprocess.PIPE)
p2 = subprocess.Popen(['awk', '/has address/ { print $4 ; exit}'], stdin=p1.stdout, stdout=subprocess.PIPE)
ipaddress = p2.communicate()[0]

print ipaddress

os.environ["EPICS_CA_ADDR_LIST"] = ipaddress

print os.environ["EPICS_CA_ADDR_LIST"]
macro = '"T=vmi5588:"'
print macro
subprocess.Popen([medm, '-x', '-macro', macro, vmi5588])

print ("starting vmi5588 for %s!") % ipaddress
sys.exit()

