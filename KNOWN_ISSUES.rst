Known Issues and Workarounds
****************************

Affected version: QMSI 1.2.0.

=========== ====================================================================
Issue       UART 0 is reserved on SE C1000 development platform
----------- --------------------------------------------------------------------
Implication On SE C1000 development platform, UART 0 is reserved by the BLE
            module.
----------- --------------------------------------------------------------------
Workaround  Use UART 1 instead.
=========== ====================================================================

=========== ====================================================================
Issue       SPI 16 MHz transfer failing on SE C1000 development platform
----------- --------------------------------------------------------------------
Implication On SE C1000, comparison of RX and TX is not correct when using the
            16 MHz speed.
----------- --------------------------------------------------------------------
Workaround  Use a transfer speed slower than 16 MHz.
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
=========== ====================================================================

=========== ====================================================================
Issue       I2C transfer speeds are sub-optimal.
----------- --------------------------------------------------------------------
Implication I2C transfer speeds are not as fast as they could be.
----------- --------------------------------------------------------------------
=========== ====================================================================

=========== ====================================================================
Issue       I2C driver has hard dependency on DMA code.
----------- --------------------------------------------------------------------
Implication I2C driver includes DMA code even if functionality is unused.
----------- --------------------------------------------------------------------
Workaround
=========== ====================================================================

=========== ====================================================================
Issue       Spurious character is printed on UART example for SE C1000.
----------- --------------------------------------------------------------------
Implication SE C1000 prints an additional ü character when printing.
----------- --------------------------------------------------------------------
Workaround  Ignore first character that is printed.
=========== ====================================================================
