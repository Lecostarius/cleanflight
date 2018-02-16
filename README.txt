Overall structure of Cleanflight / Betaflight / iNav
----------------------------------------------------

Top level directory:

|- docs
|           Almost completely empty, contains docs for two boards
|- downloads
|           Downloaded files (e.g. the arm crosscompiler zipfile) go here; initially empty
|- lib
|           external libraries: STM32.. from STMicro, MAVLink, dyad [a communication library], DSP_Lib
|- make
|           sub-Makefiles (included by toplevel Makefile)
|- obj
|           all object files, elf files, and hex files are created here
|- src 
|           this is the source code
|- support
|           ST Microelectronics STM Flashloader stuff
|- tools
|           the arm_none_eabi crosscompiler


Makefile
build_docs.sh
fake_travis_build.sh 
travis.sh 
.travis.yml
Notes.md 
README.md
Vagrantfile


The Make process and the make/ subdirectory
--------------------------------------------
Basic usage: 
make clean ; make TARGET=NAZE

Some explicit main Makefile targets that are useful:
 version 
 targets
 help
 clean 

 Some more Makefile targets that are a bit less useful but still:
 test, test unittest, unbrick, cppcheck, binary, hex, flash, openocd-gdb, all, official

Content of the make/ directory:
make 
  |-local.mk 
  |     developer preferences, this is in .gitignore, the developers playground
  |-linux|windows|macosx].mk
  |     OS dependent setting of environment variable PYTHON
  |-tools.mk
  |     the makefile that defines targets 'arm_sdk_install' and 'openocd_install', 'stm32flash_install',
  |     'dfuutil_install', 'uncrustify_install', 'breakpad_install', plus a few others
  |-targets.mk 
  |     sets only a lot of makefile variables dependent on the board (TARGET) you specify
  |     this contains information about F1/F3/F4/F7 etc.
 
