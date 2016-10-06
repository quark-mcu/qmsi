Fixed Issues and Workarounds
****************************

Issues fixed since version: QMSI 1.1.0.

=========== ====================================================================
Issue       DMA errors are not generated for peripherals with invalid settings
----------- --------------------------------------------------------------------
Implication If an invalid address is provided for a peripheral in a DMA
            transfer, an error callback is not triggered.
----------- --------------------------------------------------------------------
Workaround  Use correct addresses for peripherals in DMA transfers.
=========== ====================================================================

=========== ====================================================================
Issue       I2C high speed mode fails on SE C1000 Development Platform
----------- --------------------------------------------------------------------
Implication On the SE C1000 development platform, Fab A/B, 330Ω resistor causes
            I2C transfers to fail in high-speed scenarios.
----------- --------------------------------------------------------------------
Workaround  Use the SE C1000 development platform Fab C, which has a 33Ω
            resistor.
=========== ====================================================================

=========== ====================================================================
Issue       UART - DMA transfers do not immediately report errors.
----------- --------------------------------------------------------------------
Implication Break interrupts or FIFO overruns may not be caught in a DMA UART
            transfer.
----------- --------------------------------------------------------------------
Workaround  If interrupts are required, use IRQ-based transfers instead.
=========== ====================================================================

=========== ====================================================================
Issue       If an application wakes up from power_soc_sleep() using the RTC on
            D2000, and completes, the system becomes bricked.
----------- --------------------------------------------------------------------
Implication The system is not fully restored from the soc_sleep function when
            using RTC as wake up source.
----------- --------------------------------------------------------------------
Workaround  The function power_soc_sleep() needs to be updated with the
            following:
            Place the following line at the start of the function:
            uint32_t lp_clk_save = QM_SCSS_CCU->ccu_lp_clk_ctl;
            Place the following line at the end of the function(last line).
            QM_SCSS_CCU->ccu_lp_clk_ctl = lp_clk_save;
=========== ====================================================================

=========== ====================================================================
Issue       Grove shield electricity sensor does not compile for x86 on SE
            C1000.
----------- --------------------------------------------------------------------
Implication Building the example application for x86 on the SE C1000 will result
            in a compilation error
----------- --------------------------------------------------------------------
Workaround  Compile the example for the SE C1000 ARC.
=========== ====================================================================

=========== ====================================================================
Issue       Power_soc sample application comment: "On the SE C1000 development
            platform this pin is found on header J13 PIN 20".
----------- --------------------------------------------------------------------
Implication Incorrect header number in comment
----------- --------------------------------------------------------------------
Workaround  Should be J14 not J13
=========== ====================================================================

=========== ====================================================================
Issue       GPIO sample app comments say: "On the SE C1000 development board,
            PIN_OUT and PIN_INTR are located on header P4 PIN 42 and 40"
----------- --------------------------------------------------------------------
Implication Incorrect header number in comment
----------- --------------------------------------------------------------------
Workaround  Should be J15 not P4
=========== ====================================================================

=========== ====================================================================
Issue       sensor/gpio sample app comments say: "On the SE C1000 development
            platform, PIN_OUT (J15 header, PIN 36) and PIN_INTR (J15 header,
            PIN 42)."
----------- --------------------------------------------------------------------
Implication Incorrect pin number in comment
----------- --------------------------------------------------------------------
Workaround  Should be pin 40 not 42
=========== ====================================================================

=========== ====================================================================
Issue       sensor/interrupt sample App	comments say: "On the SE C1000
            development platform, PIN_OUT and PIN_INTR are located on header
            J15, PIN 36 and 42 respectively"
----------- --------------------------------------------------------------------
Implication Incorrect pin number in comment
----------- --------------------------------------------------------------------
Workaround  Should be pin 40 not 42
=========== ====================================================================
