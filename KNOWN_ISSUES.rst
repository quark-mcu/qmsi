Known Issues and Workarounds
****************************

Affected version: QMSI 1.4.0 RC3.

=========== ====================================================================
Issue       UART 0 is reserved on SE C1000 development platform
----------- --------------------------------------------------------------------
Implication On SE C1000 development platform, UART 0 is reserved by the BLE
            module.
----------- --------------------------------------------------------------------
Workaround  Use UART 1 instead.
=========== ====================================================================

=========== ====================================================================
Issue       I2C transfer speeds are sub-optimal.
----------- --------------------------------------------------------------------
Implication I2C transfer speeds are not as fast as they could be.
----------- --------------------------------------------------------------------
=========== ====================================================================

=========== ====================================================================
Issue       SPI Slave controller doesn't generate an interrupt if the /CS line
            is de-asserted.
----------- --------------------------------------------------------------------
Implication The SPI Slave driver cannot detect an incomplete interrupt based
            SPI transaction (SPI Master terminates the transfer before
            sending/receiving all data frames). At the moment if the SPI Master
	    prematurely ends the ongoing interrupt based transaction, there is
	    no way for the SPI Slave user application to recover; it will spin
	    and wait for ever for the user callback to be called.
----------- --------------------------------------------------------------------
Workaround  Short the /CS line with a GPIO pin. The GPIO can be setup in the
            application with an interrupt to alert the application that the /CS
	    line has been de-asserted
=========== ====================================================================

=========== ====================================================================
Issue       Bootloader fails to update over USB.
----------- --------------------------------------------------------------------
Implication There is a very small chance that when using dfu-utils to update an
            image on SE C1000 SoC that the transfer will hang.
----------- --------------------------------------------------------------------
Workaround  Re-initiate file transfer.
=========== ====================================================================

=========== ====================================================================
Issue       On SE C1000, if mass erase is performed on the ARC flash partition,
            garbled output can be observed on the UART.
----------- --------------------------------------------------------------------
Implication When using save/restore functionality on SE C1000, if the ARC flash
            is mass erased, garbled text can be seen on the output of the UART
	    instead of expected text.
----------- --------------------------------------------------------------------
Workaround  Perform a cold reset after performing a mass erase of ARC flash.
=========== ====================================================================

=========== ====================================================================
Issue       Cannot use comparator 0 after SE C1000 wakes up.
----------- --------------------------------------------------------------------
Implication On SE C1000, after performing a sleep restore cycle, comparator 0
            behaves erratically regarding interrupts.
----------- --------------------------------------------------------------------
Workaround  Use another comparator instead.
=========== ====================================================================

=========== ====================================================================
Issue       Reset button issue with SE C1000 development platform fab E.
----------- --------------------------------------------------------------------
Implication On the FabE revision of SE C1000, if the SoC has been halted and the
            reset button is pressed, the SoC does not resume properly.
----------- --------------------------------------------------------------------
Workaround  Power down the board and repower it to restore correct operations.
            Or use the reset button while the SoC is in an active state.
=========== ====================================================================

=========== ====================================================================
Issue       OpenOCD loses connection with ARC when it is in sleep mode.
------------ --------------------------------------------------------------------
Implication OpenOCD resume command fails to affect ARC after it has been
            restored from sleep mode.
------------ --------------------------------------------------------------------
Workaround  After restoring from sleep mode, re-establish OpenOCD connection
            ARC and then issue resume command.
=========== ====================================================================

=========== ====================================================================
Issue       Setting watchpoint on SoC Watch variable in bss causes system to
            hang.
----------- --------------------------------------------------------------------
Implication For SE C1000, attempting to set a watchpoint on a SoC Watch variable
            in bss causes the system to hang when the system wakes after sleep.
----------- --------------------------------------------------------------------
Workaround  In this situation do not set a watchpoint on SoC watch variables
            in bss.
=========== ====================================================================

=========== ====================================================================
Issue       Bootloader fails to enable USB.
----------- --------------------------------------------------------------------
Implication On very rare occasions the USB may fail to initialise when the
            device is powered on.
----------- --------------------------------------------------------------------
Workaround  Reconnecting the device resolves the issue.
=========== ====================================================================

=========== ====================================================================
Issue       SPI slave max speed as stated in the datasheet is not achievable in
            TX mode or TX/RX mode.
----------- --------------------------------------------------------------------
Implication Random data is inserted into the transfer for fast transfer speeds.
----------- --------------------------------------------------------------------
Workaround  SPI transfer speeds up to the following will ensure that this issue
            does not occur:
	              - 1.6 MHz for release build
		      - 0.8 MHz for debug build
=========== ====================================================================

=========== ====================================================================
Issue       jflash does not work properly for sensor application debugging.
----------- --------------------------------------------------------------------
Implication Debugging of sensor applications using jflash fails
----------- --------------------------------------------------------------------
Workaround  Use gdb-arc directly when debugging sensor applications.
=========== ====================================================================

=========== ====================================================================
Issue       Last frame of a receive SPI slave transfer appears to be stuck in
            the RX shift logic if /CS line is never de-asserted.
----------- --------------------------------------------------------------------
Implication If the SPI master does not de-assert the /CS line after completing
            the transfer, the SPI slave user application will wait forever for
	    the last frame to come in.
----------- --------------------------------------------------------------------
Workaround  Use two qm_spi_irq_transfer() calls to handle two separate SPI
            transactions initiated by the SPI master.
=========== ====================================================================

=========== ====================================================================
Issue       Bootloader FPR may be triggered before control is handed to the
            application.
----------- --------------------------------------------------------------------
Implication If an application unmasks FPR interrupts a violation may have
            already occurred resulting in a system reset.
----------- --------------------------------------------------------------------
Workaround  Clear any FPR interrupts before unmasking at the application level.
=========== ====================================================================

=========== ====================================================================
Issue       Grove electricity sensor readings for release builds are not
            consistent.
----------- --------------------------------------------------------------------
Implication Electricity sensor shows unconsistent readings although they do
            average out to the expected value.
----------- --------------------------------------------------------------------
Workaround  Build the application in debug mode.
=========== ====================================================================
