SETTING UP AN AMIRO-OS PROJECT FOR QT-CREATOR
=============================================

1 - Creating the Project Files
------------------------------
Run the "setup.sh" script in this directory and follow the instructions.
  > ./setup.sh



!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
! THAT'S IT for the basic setup.                                                     !
! The next steps are only necessary, if you want to integrate 'make' into QtCreator, !
! or if QtCreator crashes when opening the projects.                                 !
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



2 - Setup the AMiRo build environment for QtCreator
---------------------------------------------------
Launch the QtCreator application, click on
  Tools -> Options
and select "Build & Run".

Under the tab "Compilers" add the arm-none-eabi-g++ compiler as another GCC.

Under the tab "Debuggers" add the arm-none-eabi-gdb debugger.

Under the tab "Kits" manually create a new kit:
  Name: AMiRo
  Device type: Desktop
  Sysroot: [gcc-arm-none-eabi include path]
  Compiler: [the compiler added above]
  Environment: SERIALBOOT=[absolute path to the SerialBoot binary]
  Debugger: [the debugger added above]
  QT version: none
With everything else empty or default.
If there is a warning sign shown right beside the new kit, check the setup for errors.



3 - Open the Project(s) with QtCreator
--------------------------------------
For convenience, the absolute path to your device (e.g. /home/juser/amiro-os/devices/PowerManagement)
will be referenced in the following by "MOD_PATH".

In the QtCreator application, click on
  File -> Open File or Project
and open the according *.creator file.

Select the project in the "Projects" view and click on the "Projects" icon on the very left.
Switch to the "Build & Run" tab.

Change the kit to "AMiRo" (right above the "Build" and "Run" buttons).

Add a new build configuration and call it "make".
  Build directory: MOD_PATH
  Build Steps: make --directory=MOD_PATH all
  Clean Steps: make --directory=MOD_PATH clean
Optionally you can add "-j" to the build setp in order to parallelize the compilation process.

Add another new build configuration by cloning the new "make" configuration and call t "make flash".
Replace "all" at the end of the Build Step with "flash"

Remove the "Default" build configuration.

Repeat this step for reach further project.
