Fixed Issues and Workarounds
****************************

Issues fixed since version: QMSI 1.3.1.

=========== ====================================================================
Issue       SE C1000 DMA freezes if performed after a save/restore cycle.
----------- --------------------------------------------------------------------
Implication When operating in debug mode, after performing a sleep/restore cycle
            if DMA is then configured, the system is flooded with block
            interrupts and stalls, rendering it unusable.
----------- --------------------------------------------------------------------
Workaround  Disable save/restore context or run in release mode.
=========== ====================================================================

=========== ====================================================================
Issue       Repeated runs of AON counters for SE C1000 may result in failures.
----------- --------------------------------------------------------------------
Implication AON periodic timer may fail to have ready bit set if example is
            re-run multiple times on SE C1000.
----------- --------------------------------------------------------------------
Workaround  Perform cold reset in-between runs.
=========== ====================================================================

=========== ====================================================================
Issue       I2C master terminate TX IRQ fails in release builds.
----------- --------------------------------------------------------------------
Implication For D2000, attempting to terminate an IRQ based TX transfer when
            operating at FAST_PLUS mode will result in failure.
            For SE C1000 sensor, attempting to abort a multi master IRQ based TX
            transfer operating at STANDARD mode will result in failure.
----------- --------------------------------------------------------------------
Workaround  Operate I2C at different speeds or run in debug builds.
=========== ====================================================================

=========== ====================================================================
Issue       SPI master transfer speeds are sub-optimal when using save/restore
            builds on SE C1000.
----------- --------------------------------------------------------------------
Implication SPI master performance is degraded when using save/restore
            functionality on SE C1000.
----------- --------------------------------------------------------------------
Workaround  Either reduce transfer speeds on SPI master or disable save/restore
            context.
=========== ====================================================================

=========== ====================================================================
Issue       OpenOCD loses connection with ARC when it is in sleep mode.
----------- --------------------------------------------------------------------
Implication OpenOCD resume command fails to affect ARC after it has been
            restored from sleep mode.
----------- --------------------------------------------------------------------
Workaround  After restoring from sleep mode, re-establish OpenOCD connection
            ARC and then issue resume command.
=========== ====================================================================

=========== ====================================================================
Issue       Mixing debug and release binaries for Mailbox example causes issues.
----------- --------------------------------------------------------------------
Implication When executing Mailbox on SE C1000, if there is a mixture between
            release ande debug binaries used, unexpected results can occur.
----------- --------------------------------------------------------------------
Workaround  Use either only release binaries or debug binaries.
=========== ====================================================================

=========== ====================================================================
Issue       After restoring context on I2C slave, the I2C master reports
            tx_abort events, but slave receives data.
----------- --------------------------------------------------------------------
Implication I2C master reports no ACK bit from slave after restoring context of
            I2C on slave, when running on SE C1000.
----------- --------------------------------------------------------------------
Workaround  Do not enter low power modes on SE C1000 when performing I2C slave
            transfers and using save/restore context functionality.
=========== ====================================================================

=========== ====================================================================
Issue       SE C1000 SPI / Sensor SPI example may return incorrect CHIP ID.
----------- --------------------------------------------------------------------
Implication There is a chance after flashing the example application that the
            returned value of CHIP ID is incorrect causing the application to
            fail.
----------- --------------------------------------------------------------------
Workaround  After flashing the application onto the board, perform a cold reset.
=========== ====================================================================

=========== ====================================================================
Issue       The wIndex and wLength fields of the USB setup packet are not
            properly checked.
----------- --------------------------------------------------------------------
Implication This has security implications when an attacker sends a length that
            is bigger than the internal USB buffer or when an attacker sends an
	    index which is bigger than the endpoint buffer.
----------- --------------------------------------------------------------------
Workaround  No known workaround.
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
