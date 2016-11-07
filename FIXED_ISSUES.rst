Fixed Issues and Workarounds
****************************

Issues fixed since version: QMSI 1.3.0.

=========== ====================================================================
Issue       I2C driver has hard dependency on DMA code.
----------- --------------------------------------------------------------------
Implication I2C driver includes DMA code even if functionality is unused.
----------- --------------------------------------------------------------------
Workaround
=========== ====================================================================

=========== ====================================================================
Issue       SE C1000 ARC performance is reduced after sleep.
----------- --------------------------------------------------------------------
Implication After performing a sleep/restore cycle, the ARC performance drops.
----------- --------------------------------------------------------------------
Workaround  Enable the instruction cache on the ARC core after sleep.
            Refer to sys/sensor/init.c for the exact sequence.
----------- --------------------------------------------------------------------
Resolution  On SE C1000, if using SAVE / RESTORE functionality, the system cache
            is re-enabled by the power states driver and the user should not
            need to do anything.
=========== ====================================================================

=========== ====================================================================
Issue       SE C1000 SS SPI does not restore clk polarity after save/restore
            cycle.
----------- --------------------------------------------------------------------
Implication On SE C1000, after performing a sleep restore cycle, the SS SPI clk
            polarity is not restored to what it was prior to entering sleep
            modes.
----------- --------------------------------------------------------------------
Workaround  Re-set clk polarity after restoring context on SE C1000.
=========== ====================================================================

=========== ====================================================================
Issue       After restoring context on SS ADC on SE C1000, not all settings are
            restored.
----------- --------------------------------------------------------------------
Implication On SE C1000, after a restore cycle, the SS ADC settings are not
            restored correctly, this includes any calibration and mode settings.
            There is also a spurious interrupt if interrupts were enabled.
----------- --------------------------------------------------------------------
Workaround  After resuming, re-calibrate and set mode of SS ADC. Disregard
            spurious interrupt.
=========== ====================================================================

=========== ====================================================================
Issue       D2000 hangs if the UART prints during soc_deep_sleep before the
            system has fully restored to the active state.
----------- --------------------------------------------------------------------
Implication If the user callback attempts to send data over the UART during a
            soc_deep_sleep callback when the system is still transitioning to
            the active state, the system will hang on wake.
----------- --------------------------------------------------------------------
Workaround  Avoid printing over the UART during user callbacks until after the
            SoC has fully resumed operations in the active state.
----------- --------------------------------------------------------------------
Resolution  System state is now fully restored before user callback is executed.
=========== ====================================================================