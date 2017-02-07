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
