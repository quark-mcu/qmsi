Known Issues and Workarounds
****************************

Affected version: QMSI 1.4.0.

=========== ====================================================================
Issue       Bootloader occasionally fails to update over USB.
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
Issue       Bootloader ocasionally fails to enable USB.
----------- --------------------------------------------------------------------
Implication On very rare occasions the USB may fail to initialise when the
            device is powered on.
----------- --------------------------------------------------------------------
Workaround  Reconnecting the device resolves the issue.
=========== ====================================================================

=========== ====================================================================
Issue       jflash does not work properly for sensor application debugging.
----------- --------------------------------------------------------------------
Implication Debugging of sensor applications using jflash fails
----------- --------------------------------------------------------------------
Workaround  Use gdb-arc directly when debugging sensor applications.
=========== ====================================================================

=========== ====================================================================
Issue       Comparators fires a spurious interrupt during power up if being
            configured for negative polarity at the same time.
----------- --------------------------------------------------------------------
Implication If a callback is registered the user will observe the spurious
            interrupt when setting the pin to be triggered at negative
            polarity while powering up the pin.
----------- --------------------------------------------------------------------
Workaround  Don't set the polarity to negative immediately when powering
            up a comparator pin. The following will avoid the issue:
                 1. Power up the comparator pin and set cmp_en = 0.
                 2. Wait for 10us.
                 3. Change the comparator to negative polarity and set cmp_en=1.
=========== ====================================================================

=========== ====================================================================
Ìssue       The ‘accel’ example application fails to compile with the
            Intel(R) Integrated Performance Primitives (IPP) library enabled.
----------- --------------------------------------------------------------------
Implication When the IPP_LIBRARY_PATH environment variable is set the ‘accel’
            application compilation fails as it cannot find the ‘lippsq’
            library.
----------- --------------------------------------------------------------------
Workaround  In the Makefile change this line :
            LDLIBS += -L$(IPP_LIBRARY_PATH)/lib –lippsq
            To this :
            LDLIBS += -L$(IPP_LIBRARY_PATH)/lib/$(BUILD)/$(SOC)/$(TARGET) -lippsq
=========== ====================================================================
