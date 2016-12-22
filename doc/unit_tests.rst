QMSI Unit Test Guidelines
#########################

.. contents::

Overview
********
This document is intended to outline the rules for QMSI unit tests.

Unity test framework
********************
The unit test framework which is being used for Mint River is Unity. Some
unnecessary files have been cut out, resulting is a slightly leaner version of
Unity (mint_river-bsp/tests/unity). It is licensed under standard MIT License.
You can find more information about it `here
<http://www.throwtheswitch.org/unity/>`_

Multi-threaded tests
********************
A number of functions are provided for manipulating register values in a delayed
thread. These can be found in mint_river-bsp/test/unit_test/test_common.c It is
recommended that only one thread is used per unit test. See <example> to see the
correct usage of threads.

Unit test guidelines
********************
* Coverage metrics must meet the following requirements:
  - Function coverage must be 100%.
  - Decision coverage should be as close to 100% as possible (80% minimum).

* Each unit test should have a header comment describing... [TBD].

* The set_up() function is called before every unit test and must reset the
  registers to a known state.

* All registers and fields that are being manipulated by the API should have a
  corresponding ASSERT(). For example if an API function writes values to two
  registers, both should be checked with an assert.

* All register fields, masks and offsets must be redefined in the unit test. In
  other words, the values defined in the driver must not be reused in the unit
  test.

* API functions should be tested in isolation where possible. In other words,
  when testing the read_config() function, the write_config() should not be used
  at all to populate the registers for the test.

* When writing application level unit tests API calls should be stubbed out in
  order to isolate the code being tested.

* Each assert in the unit test should use the correct assert function for the
  data type being checked. Asserts that take two values to compare must be in
  the following format ASSERT(expected result, test value). For example:
  - TEST_ASSERT_EQUAL_TRUE(rc);
  - TEST_ASSERT_EQUAL_INT(0, rc);
  - TEST_ASSERT_EQUAL_HEX(0x8000, reg);
  A full list of asserts are available in the `Unity assertions reference
  <https://github.com/ThrowTheSwitch/Unity/blob/master/docs/UnityAssertionsReference.pdf>`_

* Where possible there should be ASSERTS to check for all failure cases.
  - Every QM_CHECK should have a corresponding assert.
  - Other failure scenarios (non-zero returns) in the driver should be tested.

* Function return codes must be checked with an ASSERT.

* In the interest of readability, ASSERTS should not contain function calls.
  Instead, the return code should be assigned to a variable and that should be
  passed into the ASSERT.

* Unit tests for a particular SoC feature should be guarded with a "#if
  (HAS_<FEATURE_X>)". TEST_IGNORE() should be used to prevent the test from
  running against unsupported SoCs or cores. For example:

  .. code-block:: c

          void unit_test(void)
          {
          #if (HAS_FEATURE_X)
                  /* do unit test stuff */
          #else
                  TEST_IGNORE();
          #endif /* HAS_FEATURE_X */
          }

  If the unit test file grows to be too complex or messy it may be necessary to
  split the file up into several files which test different features.

* When testing register manipulation, the register being tested  must not be in
  the expected state before the checks are performed. For example, if the
  result of an operation would leave a bit equal to 0, the bit should be 1
  before the operation is performed.
