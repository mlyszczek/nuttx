README
======

  This README file provides information about the port of NuttX to the NXP
  i.MXRT evaluation kit, MIMXRT1050-EVKB.  This board features the
  MIMXRT1052DVL6A MCU.  Some of the features of this board include:

    o Processor

      - MIMXRT1052DVL6A processor

    o Memory

      - 256 Mb SDRAM memory
      - 512 Mb Hyper Flash
      - Footprint for QSPI Flash
      - TF socket for SD card

    o Display and Audio

      - Parallel LCD connector
      - Camera connector
      - Audio CODEC
      - 4-pole audio headphone jack
      - External speaker connection
      - Microphone
      - SPDIF connector

    o Connectivity

      - Micro USB host and OTG connectors
      - Ethernet (10/100T) connector
      - CAN transceivers
      - Arduino® interface

Configurations
==============

Information Common to All Configurations
----------------------------------------
Each i.MX RT 10050 configuration is maintained in a sub-directory and
can be selected as follow:

   tools/configure.sh [OPTIONS] imxrt1050-evk/<subdir>

Where typical options are -l to configure to build on Linux or -c to
configure for Cygwin under Linux.  'tools/configure.sh -h' will show
you all of the options.

Before building, make sure the PATH environment variable include the
correct path to the directory than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

  make

The <subdir> that is provided above as an argument to the tools/configure.sh
must be is one of the following.

NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART3 (i.e., for the Arduino serial shield).

  3. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.  This NSH
    configuration is focused on low level, command-line driver testing.
    It has no network.
