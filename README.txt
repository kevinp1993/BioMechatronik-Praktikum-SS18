AMiRo-OS is the operating system for the base version of the Autonomous Mini
Robot (AMiRo) [1]. It utilizes ChibiOS (a real-time operating system for
embedded devices developed by Giovanni di Sirio; see <http://chibios.org>) as
system kernel and extends it with platform specific functionalities.

Copyright (C) 2016..2017  Thomas Schöpping et al.
(a complete list of all authors is given below)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

This research/work was supported by the Cluster of Excellence
Cognitive Interaction Technology 'CITEC' (EXC 277) at Bielefeld
University, which is funded by the German Research Foundation (DFG).

Authors:
 - Thomas Schöpping        <tschoepp[at]cit-ec.uni-bielefeld.de>
 - Timo Korthals           <tkorthals[at]cit-ec.uni-bielefeld.de>
 - Stefan Herbrechtsmeier  <sherbrec[at]cit-ec.uni-bielefeld.de>
 - Teerapat Chinapirom     <tchinapirom[at]cit-ec.uni-bielefeld.de>
 - Robert Abel
 - Marvin Barther
 - Claas Braun
 - Tristan Kenneweg

References:
 [1] S. Herbrechtsmeier, T. Korthals, T. Schopping and U. Rückert, "AMiRo: A
     modular & customizable open-source mini robot platform," 2016 20th
     International Conference on System Theory, Control and Computing (ICSTCC),
     Sinaia, 2016, pp. 687-692.



################################################################################
#                                                                              #
#        RRRRRRRR   EEEEEEEE     AAA     DDDDDDDD   MM     MM  EEEEEEEE        #
#        RR     RR  EE          AA AA    DD     DD  MMM   MMM  EE              #
#        RR     RR  EE         AA   AA   DD     DD  MMMM MMMM  EE              #
#        RRRRRRRR   EEEEEE    AA     AA  DD     DD  MM MMM MM  EEEEEE          #
#        RR   RR    EE        AAAAAAAAA  DD     DD  MM     MM  EE              #
#        RR    RR   EE        AA     AA  DD     DD  MM     MM  EE              #
#        RR     RR  EEEEEEEE  AA     AA  DDDDDDDD   MM     MM  EEEEEEEE        #
#                                                                              #
################################################################################

This file will help you to setup all required software on your system, compile
the source code, and flash it to the AMiRo modules.

================================================================================

CONTENTS:
 1  Required software
   1.1  Git
   1.2  GCC ARM Embedded Toolchain
   1.3  ChibiOS
   1.4  AMiRo-BLT
 2  Recommended software
   2.1  gtkterm and hterm
   2.2  QtCreator
 3  Building and flashing

================================================================================



1 - REQUIRED SOFTWARE
---------------------

In order to compile the source code, you need to install the GCC for ARM
embedded devices. Since AMiRo-OS requires ChibiOS as system kernel, you need a
copy of that project as well. Furthermore, AMiRo-OS requires a compatible
bootloader, such as provided by the AMiRo-BLT project.


1.1 - Git
---------

Since all main- and subprojects are available as Git repositories, installing a
recent version of the tool is mandatory.


1.2 GCC ARM Embedded Toolchain
------------------------------

Various versions of the GCC for ARM embedded devices can be found at
<https://launchpad.net/gcc-arm-embedded>. It is highly recommended to use the
version 4.8 with update 2014-q1 since some others will cause issues. For
installation of the compiler toolchain, please follow the instructions that can
be found on the web page.

If you are running a 64-bit operating system, you will have to install several
32-bit libraries in order to make the compiler work. The required packages are
libc6, libstdc++6, and libncurses5. You can run the following shell commands to
install the according 32-bit versions of the packages:
  >$ sudo dpkg --add-architecture i386 && sudo apt-get update
  >$ sudo apt-get install libc6:i386 libstdc++6:i386 libncurses5:i386


1.3 ChibiOS
-----------

Since AMiRo-OS uses ChibiOS as underlying system kernel, you need to acquire a
copy of it as well. First, go to the directory which contains the AMiRo-OS
folder (but do not go into the AMiRo-OS directory itself!). Now clone the GIT
repository of ChibiOS and checkout version 2.6.x:
  >$ git clone https://github.com/ChibiOS/ChibiOS.git ChibiOS
  >$ cd ChibiOS
  >$ git checkout 2e6dfc7364e7551483922ea6afbaea8f3501ab0e
It is highly recommended to use exactly this commit. Although newer commits in
the 2.6.x branch might work fine, AMiRo-OS is not compatible with ChibiOS
version 3 or newer.

AMiRo-OS comes with some patches to ChibiOS, which must be applied as well
before compiling the project. Therefore you need to copy all files from the
./patches directory of AMiRo-OS to the root directory of ChibiOS. You can then
apply the patches via the following command:
  >$ for i in `ls | grep patch`; do git am --ignore-space-change --ignore-whitespace < ${i}; done
If the files could not be patched successfully, you are probably using an
incompatible version of ChibiOS (try to checkout the correct commit as denoted
above).


1.4 AMiRo-BLT
-------------

AMiRo-BLT is an additional software project, which is developed in parallel with
AMiRo-OS. If you did not receive a copy of AMiRo-BLT with AMiRo-OS, you can find
all code and documentation at <https://opensource.cit-ec.de/projects/amiro-os>.
Instructions for installation and how to use the software provided by AMiRo-BLT
can be found on the web page or in the project's readme file. It is highly
recommended to install AMiRo-BLT in the same directory as AMiRo-OS and ChibiOS
and name its root directory 'amiro-blt'.



2 - RECOMMENDED SOFTWARE
------------------------

In order to fully use all features of AMiRo-OS it is recommended to install the
'hterm' or 'gtkterm' application for accessing the robot. To ease further
development, this project offers support for the QtCreator IDE.


2.1 - gtkterm and hterm
-----------------------

Depending on your operating system, it is recommended to install 'gtkterm' for
Linux (available in the Ubuntu repositories), or 'hterm' for Windows. For
gtkterm you need to modify the configuration file ~/.gtktermrc (it is generated
automatically when you start the application for the first time) as follows:

  port	= /dev/ttyUSB0
  speed	= 115200
  bits	= 8
  stopbits	= 1
  parity	= none
  flow	= none
  wait_delay	= 0
  wait_char	= -1
  rs485_rts_time_before_tx	= 30
  rs485_rts_time_after_tx	= 30
  echo	= False
  crlfauto	= True

For hterm you must need to configure the tool analogously.


2.2 - QtCreator
---------------

In order to setup QtCreator projects for the three AMiRo base modules, a script
is provided in the directory ./ide/qtcreator/. It is accompanied by an
additional README.txt file, which contains further information.



3 - BUILDING AND FLASHING
-------------------------

Each time you modify any part of AMiRo-OS, you need to recompile the whole
project for the according AMiRo module. Therefore you have to use the makefiles
provided in ./devices/<DeviceToRecompile>/ by simply executing 'make' in the
according directory. If you want to compile all modules at once, you can also
use the makefile in the ./devices/ folder.

After compilation, you always have to flash the generated program to the robot.
Therefore you need to install the SerialBoot tool provided by the AMiRo-BLT
project. By default AMiRo-OS assumes AMiRo-BLT to be installed in the same
folder and its root directory to be named 'amiro-blt'. If this is the case, it
will automatically detect the SerialBoot tool. Otherwise the tool must be
accessible globally under the environment variable 'SERIALBOOT'. You can make
it so by appending the following line to your ~/.bashrc file:

  export SERIALBOOT=</absolute/path/to/the/SerialBoot/binary>

You can test the tool by calling it via the variable:
  >$ ${SERIALBOOT}
This should print some information about the tool.

Similar to the compilation procedure as described above, you can flash either
each module separately, or all modules at once by executing 'make flash' from
the according directory. Note that you must connect the programming cable either
to the DiWheelDrive or the PowerManagement module for flashing the operating
system. All other modules are powered off after reset so that only these two
offer a bootloader that is required for flashing.

================================================================================
