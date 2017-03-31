Hardware Issues with Software Workarounds.
******************************************

=========== ====================================================================
Tag         FIX_1
----------- --------------------------------------------------------------------
Implication I2C slave: General call interrupt not cleared.
            When the slave controller is addressed by a general call,
            the interrupt may not be cleared when ic_clr_gen_call is read.
            When the SoC core is running at 32MHz, the problem appears when the
            I2C clock is less than ~125kHz.
----------- --------------------------------------------------------------------
Affect      Intel Quark D2000, Intel Quark SE C1000
----------- --------------------------------------------------------------------
Workaround  Read repeatedly ic_clr_gen_call register until interrupt is
            cleared.
=========== ====================================================================

=========== ====================================================================
Tag         FIX_2
----------- --------------------------------------------------------------------
Implication After power-on reset RSTN / cold reset de-assertion, if AONPT_RST is
            set to 1 before the first RTC Clock edge after reset deassertion,
            then AON Periodic Timer will not get started asexpected. This is
            because the synchronizer flip-flop stages used for synchronizing
            AONPT_RST (sysclk domain register) to RTC Clock domain has reset
            value of ‘1’ wrongly. Hence if AONPT_RST is set to 1 before first
            rtc clock edge, then synchronizer stages would get ‘1’ always and
            hence post synchronizer detection of ‘0’ to ‘1’ transition of
            AONPT_RST to start AONPT does not appen leading to this failure (not
            starting AONPT). This failure happens only at POR/cold reset
            deassertion time and not at other times.
----------- --------------------------------------------------------------------
Workaround  On first reset of Always on Timer, make sure one RTC clock cycle has
            passed before issuing the reset command.
=========== ====================================================================

=========== ====================================================================
Tag         FIX_3
----------- --------------------------------------------------------------------
Implication When issuing a clear or reset command to the Always On Periodic
            Timer twice within one RTC clock cycle, plus the time for the bit to
            de-assert, the clear will never de-assert and no further interrupts
            will occur.
----------- --------------------------------------------------------------------
Workaround  After writing to the control register, poll on clear or reset bit
            and wait until the next cycle before continuing.
=========== ====================================================================

=========== ====================================================================
Tag         FIX_4
----------- --------------------------------------------------------------------
Implication On SE C1000 development platform, UART 0 is reserved by the BLE
            module
----------- --------------------------------------------------------------------
Affect      Intel Quark SE C1000
----------- --------------------------------------------------------------------
Workaround  use UART_1
=========== ====================================================================

=========== ====================================================================
Tag         FIX_5
----------- --------------------------------------------------------------------
Implication maximum I2C transfer speeds (as stated in datasheet) have not
            been validated
----------- --------------------------------------------------------------------
Affect      Intel Quark D2000, Intel Quark SE C1000
----------- --------------------------------------------------------------------
Workaround  None
=========== ====================================================================

=========== ====================================================================
Tag         FIX_6
----------- --------------------------------------------------------------------
Implication The SPI Slave driver cannot detect an incomplete interrupt based
            SPI transaction (SPI Master terminates the transfer before
            sending/receiving all data frames). At the moment if the SPI Master
            prematurely ends the ongoing interrupt based transaction, there is
            no way for the SPI Slave user application to recover; it will spin
            and wait for ever for the user callback to be called.

----------- --------------------------------------------------------------------
Affect      Intel Quark D2000, Intel Quark SE C1000
----------- --------------------------------------------------------------------
Workaround  Short the /CS line with a GPIO pin. The GPIO can be setup in the
            application with an interrupt to alert the application that the /CS
            line has been de-asserted
=========== ====================================================================

=========== ====================================================================
Tag         FIX_7
----------- --------------------------------------------------------------------
Implication SPI slave max speed as stated in the datasheet is not achievable in
            TX mode or TX/RX mode (Random data is inserted into the transfer for
            fast transfer speeds).


----------- --------------------------------------------------------------------
Affect      Intel Quark D2000, Intel Quark SE C1000
----------- --------------------------------------------------------------------
Workaround  SPI transfer speeds up to the following will ensure that this issue
            does not occur:
                      - 1.6 MHz for release build
                      - 0.8 MHz for debug build

=========== ====================================================================

=========== ====================================================================
Tag         FIX_8
----------- --------------------------------------------------------------------
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