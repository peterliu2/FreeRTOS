/*******************************************************************************
 * (c) Copyright 2008-2013 Microsemi SoC Products Group.  All rights reserved.
 *
 *  SmartFusion2 Microcontroller Subsystem GPIO bare metal software driver public
 *  API.
 *
 * SVN $Revision: 5487 $
 * SVN $Date: 2013-03-29 15:33:22 +0000 (Fri, 29 Mar 2013) $
 */

/*=========================================================================*//**
  @mainpage SmartFusion2 MSS GPIO Bare Metal Driver.

  @section intro_sec Introduction
  The SmartFusion2 Microcontroller Subsystem (MSS) includes a block of 32 general
  purpose input/outputs (GPIO).
  This software driver provides a set of functions for controlling the MSS GPIO
  block as part of a bare metal system where no operating system is available.
  This driver can be adapted for use as part of an operating system but the
  implementation of the adaptation layer between this driver and the operating
  system's driver model is outside the scope of this driver.

  @section hw_dependencies Hardware Flow Dependencies
  The configuration of all features of the MSS GPIOs is covered by this driver
  with the exception of the SmartFusion2 IOMUX configuration. SmartFusion2
  allows multiple non-concurrent uses of some external pins through IOMUX
  configuration. This feature allows optimization of external pin usage by
  assigning external pins for use by either the microcontroller subsystem or the
  FPGA fabric. The MSS GPIOs share SmartFusion2 device external pins with the
  FPGA fabric and with other MSS peripherals via an IOMUX. The MSS GPIO ports
  can alternatively be routed to the FPGA fabric through an IOMUX.
  The IOMUXs are configured using the SmartFusion2 MSS configurator tool. You
  must ensure that the MSS GPIOs are enabled and configured in the SmartFusion2
  MSS configurator if you wish to use them. For more information on IOMUXs,
  refer to the IOMUX section of the SmartFusion2 Microcontroller Subsystem (MSS)
  User’s Guide.
  The base address, register addresses and interrupt number assignment for the
  MSS GPIO block are defined as constants in the SmartFusion2 CMSIS HAL. You
  must ensure that the latest SmartFusion2 CMSIS HAL is included in the project
  settings of the software tool chain used to build your project and that it is
  generated into your project.

  @section theory_op Theory of Operation
  The MSS GPIO driver functions are grouped into the following categories:
    - Initialization
    - Configuration
    - Reading and setting GPIO state
    - Interrupt control

  Initialization
  The MSS GPIO driver is initialized through a call to the MSS_GPIO_init()
  function. The MSS_GPIO_init() function must be called before any other MSS
  GPIO driver functions can be called.

  Configuration
  Each GPIO port is individually configured through a call to the
  MSS_GPIO_config() function. Configuration includes deciding if a GPIO port
  will be used as an input, an output or both. GPIO ports configured as inputs
  can be further configured to generate interrupts based on the input's state.
  Interrupts can be level or edge sensitive.

  Reading and Setting GPIO State
  The state of the GPIO ports can be read and set using the following functions:
    - MSS_GPIO_get_inputs()
    - MSS_GPIO_get_outputs()
    - MSS_GPIO_set_outputs()
    - MSS_GPIO_set_output()
    - MSS_GPIO_drive_inout()

  Interrupt Control
  Interrupts generated by GPIO ports configured as inputs are controlled using
  the following functions:
    - MSS_GPIO_enable_irq()
    - MSS_GPIO_disable_irq()
    - MSS_GPIO_clear_irq()

 *//*=========================================================================*/
#ifndef MSS_GPIO_H_
#define MSS_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../../CMSIS/m2sxxx.h"

/*-------------------------------------------------------------------------*//**
  The mss_gpio_id_t enumeration is used to identify individual GPIO ports as an
  argument to functions:
    - MSS_GPIO_config()
    - MSS_GPIO_set_output() and MSS_GPIO_drive_inout()
    - MSS_GPIO_enable_irq(), MSS_GPIO_disable_irq() and MSS_GPIO_clear_irq()
 */
typedef enum __mss_gpio_id_t {
    MSS_GPIO_0 = 0,
    MSS_GPIO_1 = 1,
    MSS_GPIO_2 = 2,
    MSS_GPIO_3 = 3,
    MSS_GPIO_4 = 4,
    MSS_GPIO_5 = 5,
    MSS_GPIO_6 = 6,
    MSS_GPIO_7 = 7,
    MSS_GPIO_8 = 8,
    MSS_GPIO_9 = 9,
    MSS_GPIO_10 = 10,
    MSS_GPIO_11 = 11,
    MSS_GPIO_12 = 12,
    MSS_GPIO_13 = 13,
    MSS_GPIO_14 = 14,
    MSS_GPIO_15 = 15,
    MSS_GPIO_16 = 16,
    MSS_GPIO_17 = 17,
    MSS_GPIO_18 = 18,
    MSS_GPIO_19 = 19,
    MSS_GPIO_20 = 20,
    MSS_GPIO_21 = 21,
    MSS_GPIO_22 = 22,
    MSS_GPIO_23 = 23,
    MSS_GPIO_24 = 24,
    MSS_GPIO_25 = 25,
    MSS_GPIO_26 = 26,
    MSS_GPIO_27 = 27,
    MSS_GPIO_28 = 28,
    MSS_GPIO_29 = 29,
    MSS_GPIO_30 = 30,
    MSS_GPIO_31 = 31
} mss_gpio_id_t;

/*-------------------------------------------------------------------------*//**
  These constant definitions are used as an argument to the
  MSS_GPIO_set_outputs() function to identify GPIO ports. A logical OR of these
  constants can be used to specify multiple GPIO ports.
  These definitions can also be used to identify GPIO ports through logical
  operations on the return value of the MSS_GPIO_get_inputs() function.
 */
#define MSS_GPIO_0_MASK         0x00000001uL
#define MSS_GPIO_1_MASK         0x00000002uL
#define MSS_GPIO_2_MASK         0x00000004uL
#define MSS_GPIO_3_MASK         0x00000008uL
#define MSS_GPIO_4_MASK         0x00000010uL
#define MSS_GPIO_5_MASK         0x00000020uL
#define MSS_GPIO_6_MASK         0x00000040uL
#define MSS_GPIO_7_MASK         0x00000080uL
#define MSS_GPIO_8_MASK         0x00000100uL
#define MSS_GPIO_9_MASK         0x00000200uL
#define MSS_GPIO_10_MASK        0x00000400uL
#define MSS_GPIO_11_MASK        0x00000800uL
#define MSS_GPIO_12_MASK        0x00001000uL
#define MSS_GPIO_13_MASK        0x00002000uL
#define MSS_GPIO_14_MASK        0x00004000uL
#define MSS_GPIO_15_MASK        0x00008000uL
#define MSS_GPIO_16_MASK        0x00010000uL
#define MSS_GPIO_17_MASK        0x00020000uL
#define MSS_GPIO_18_MASK        0x00040000uL
#define MSS_GPIO_19_MASK        0x00080000uL
#define MSS_GPIO_20_MASK        0x00100000uL
#define MSS_GPIO_21_MASK        0x00200000uL
#define MSS_GPIO_22_MASK        0x00400000uL
#define MSS_GPIO_23_MASK        0x00800000uL
#define MSS_GPIO_24_MASK        0x01000000uL
#define MSS_GPIO_25_MASK        0x02000000uL
#define MSS_GPIO_26_MASK        0x04000000uL
#define MSS_GPIO_27_MASK        0x08000000uL
#define MSS_GPIO_28_MASK        0x10000000uL
#define MSS_GPIO_29_MASK        0x20000000uL
#define MSS_GPIO_30_MASK        0x40000000uL
#define MSS_GPIO_31_MASK        0x80000000uL

/*-------------------------------------------------------------------------*//**
  These constant definitions are used as an argument to the MSS_GPIO_config()
  function to specify the I/O mode of each GPIO port.
 */
#define MSS_GPIO_INPUT_MODE              0x0000000002uL
#define MSS_GPIO_OUTPUT_MODE             0x0000000005uL
#define MSS_GPIO_INOUT_MODE              0x0000000003uL

/*-------------------------------------------------------------------------*//**
  These constant definitions are used as an argument to the MSS_GPIO_config()
  function to specify the interrupt mode of each GPIO port.
 */
#define MSS_GPIO_IRQ_LEVEL_HIGH           0x0000000000uL
#define MSS_GPIO_IRQ_LEVEL_LOW            0x0000000020uL
#define MSS_GPIO_IRQ_EDGE_POSITIVE        0x0000000040uL
#define MSS_GPIO_IRQ_EDGE_NEGATIVE        0x0000000060uL
#define MSS_GPIO_IRQ_EDGE_BOTH            0x0000000080uL

/*-------------------------------------------------------------------------*//**
  The mss_gpio_inout_state_t enumeration is used to specify the output state of
  an INOUT GPIO port as an argument to the MSS_GPIO_drive_inout() function.
 */
typedef enum mss_gpio_inout_state {
    MSS_GPIO_DRIVE_LOW = 0,
    MSS_GPIO_DRIVE_HIGH,
    MSS_GPIO_HIGH_Z
} mss_gpio_inout_state_t;

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_init() function initializes the SmartFusion2 MSS GPIO block. It
  resets the MSS GPIO hardware block and it also clears any pending MSS GPIO
  interrupts in the ARM Cortex-M3 interrupt controller. When the function exits,
  it takes the MSS GPIO block out of reset.

   @param
    This function has no parameters.

   @return
    This function does not return a value.
 */
void MSS_GPIO_init( void );

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_config() function is used to configure an individual GPIO port.

  @param port_id
    The port_id parameter identifies the GPIO port to be configured. An
    enumeration item of the form MSS_GPIO_n, where n is the number of the GPIO
    port, is used to identify the GPIO port. For example, MSS_GPIO_0 identifies
    the first GPIO port and MSS_GPIO_31 is the last one.

  @param config
    The config parameter specifies the configuration to be applied to the GPIO
    port identified by the port_id parameter. It is a logical OR of the required
    I/O mode and the required interrupt mode. The interrupt mode is not relevant
    if the GPIO is configured as an output only.
       These I/O mode constants are allowed:
           - MSS_GPIO_INPUT_MODE
           - MSS_GPIO_OUTPUT_MODE
           - MSS_GPIO_INOUT_MODE
       These interrupt mode constants are allowed:
           - MSS_GPIO_IRQ_LEVEL_HIGH
           - MSS_GPIO_IRQ_LEVEL_LOW
           - MSS_GPIO_IRQ_EDGE_POSITIVE
           - MSS_GPIO_IRQ_EDGE_NEGATIVE
           - MSS_GPIO_IRQ_EDGE_BOTH

   @return
    none.

  Example:
  The following call will configure GPIO 4 as an input generating interrupts on
  a Low to High transition of the input:
  @code
  MSS_GPIO_config( MSS_GPIO_4, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
  @endcode
 */
void MSS_GPIO_config
(
    mss_gpio_id_t port_id,
    uint32_t config
);

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_set_outputs() function is used to set the state of all GPIO ports
  configured as outputs.

  @param value
    The value parameter specifies the state of the GPIO ports configured as
    outputs. It is a bit mask of the form (MSS_GPIO_n_MASK | MSS_GPIO_m_MASK)
    where n and m are numbers identifying GPIOs. For example, (MSS_GPIO_0_MASK |
    MSS_GPIO_1_MASK | MSS_GPIO_2_MASK ) specifies that the first, second and
    third GPIO outputs must be set High and all other GPIO outputs set Low. The
    driver provides 32 mask constants, MSS_GPIO_0_MASK to MSS_GPIO_31_MASK
    inclusive, for this purpose.

  @return
    none.

  Example 1:
    Set GPIOs outputs 0 and 8 high and all other GPIO outputs low.
    @code
        MSS_GPIO_set_outputs( MSS_GPIO_0_MASK | MSS_GPIO_8_MASK );
    @endcode

  Example 2:
    Set GPIOs outputs 2 and 4 low without affecting other GPIO outputs.
    @code
        uint32_t gpio_outputs;
        gpio_outputs = MSS_GPIO_get_outputs();
        gpio_outputs &= ~( MSS_GPIO_2_MASK | MSS_GPIO_4_MASK );
        MSS_GPIO_set_outputs(  gpio_outputs );
    @endcode

  @see MSS_GPIO_get_outputs()
 */
static __INLINE void
MSS_GPIO_set_outputs
(
    uint32_t value
)
{
    GPIO->GPIO_OUT = value;
}

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_set_output() function is used to set the state of a single GPIO
  port configured as an output.
  Note: Using bit-band writes might be a better option than this function for
        performance critical applications where the application code is not
        intended to be ported to a processor other than the ARM Cortex-M3 in
        SmartFusion2. The bit-band write equivalent to this function would be:
            GPIO_BITBAND->GPIO_OUT[port_id] = (uint32_t)value;

  @param port_id
    The port_id parameter identifies the GPIO port that is to have its output
    set. An enumeration item of the form MSS_GPIO_n, where n is the number of
    the GPIO port, is used to identify the GPIO port. For example, MSS_GPIO_0
    identifies the first GPIO port and MSS_GPIO_31 is the last one.

  @param value
    The value parameter specifies the desired state for the GPIO output. A value
    of 0 will set the output Low and a value of 1 will set the output High.

  @return
    This function does not return a value.

  Example:
  The following call will set GPIO output 12 High, leaving all other GPIO
  outputs unaffected:
  @code
    _GPIO_set_output(MSS_GPIO_12, 1);
  @endcode
 */
void MSS_GPIO_set_output
(
    mss_gpio_id_t       port_id,
    uint8_t             value
);

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_get_inputs() function is used to read the current state all GPIO
  ports configured as inputs.

  @return
    This function returns a 32-bit unsigned integer where each bit represents
    the state of a GPIO input. The least significant bit represents the state of
    GPIO input 0 and the most significant bit the state of GPIO input 31.

  Example:
    Read and assign the current state of the GPIO outputs to a variable.
    @code
        uint32_t gpio_inputs;
        gpio_inputs = MSS_GPIO_get_inputs();
    @endcode
 */
static __INLINE uint32_t
MSS_GPIO_get_inputs( void )
{
    return GPIO->GPIO_IN;
}

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_get_outputs() function is used to read the current state all GPIO
  ports configured as outputs.

  @return
     This function returns a 32-bit unsigned integer where each bit represents
     the state of a GPIO output. The least significant bit represents the state
     of GPIO output 0 and the most significant bit the state of GPIO output 31.

  Example:
    Read and assign the current state of the GPIO outputs to a variable.
    @code
        uint32_t gpio_outputs;
        gpio_outputs = MSS_GPIO_get_outputs();
    @endcode
 */
static __INLINE uint32_t
MSS_GPIO_get_outputs( void )
{
    return GPIO->GPIO_OUT;
}

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_drive_inout() function is used to set the output state of a
  single GPIO port configured as an INOUT. An INOUT GPIO can be in one of three
  states:
    - High
    - Low
    - High impedance
  An INOUT output would typically be used where several devices can drive the
  state of a shared signal line. The High and Low states are equivalent to the
  High and Low states of a GPIO configured as an output. The High impedance
  state is used to prevent the GPIO from driving its output state onto the
  signal line, while at the same time allowing the input state of the GPIO to
  be read.

  @param port_id
    The port_id parameter identifies the GPIO port for which you want to change
    the output state. An enumeration item of the form MSS_GPIO_n, where n is the
    number of the GPIO port, is used to identify the GPIO port. For example,
    MSS_GPIO_0 identifies the first GPIO port and MSS_GPIO_31 is the last one.

  @param inout_state
    The inout_state parameter specifies the state of the GPIO port identified by
    the port_id parameter. Allowed values of type mss_gpio_inout_state_t are as
    follows:
        - MSS_GPIO_DRIVE_HIGH
        - MSS_GPIO_DRIVE_LOW
        - MSS_GPIO_HIGH_Z  (High impedance)

  @return
    This function does not return a value.

  Example:
    The call to MSS_GPIO_drive_inout() below will set the GPIO 7 output to the
    high impedance state.
    @code
    MSS_GPIO_drive_inout( MSS_GPIO_7, MSS_GPIO_HIGH_Z );
    @endcode
 */
void MSS_GPIO_drive_inout
(
    mss_gpio_id_t           port_id,
    mss_gpio_inout_state_t  inout_state
);

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_enable_irq() function is used to enable interrupt generation for
  the specified GPIO input. Interrupts are generated based on the state of the
  GPIO input and the interrupt mode configured for it by MSS_GPIO_config().

  @param port_id
    The port_id parameter identifies the GPIO port for which you want to enable
    interrupt generation. An enumeration item of the form MSS_GPIO_n, where n is
    the number of the GPIO port, is used to identify the GPIO port. For example,
    MSS_GPIO_0 identifies the first GPIO port and MSS_GPIO_31 is the last one.

  @return
    This function does not return a value.

  Example:
    The call to MSS_GPIO_enable_irq() below will allow GPIO 8 to generate
    interrupts.
    @code
    MSS_GPIO_enable_irq( MSS_GPIO_8 );
    @endcode
 */
void MSS_GPIO_enable_irq
(
    mss_gpio_id_t port_id
);

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_disable_irq() function is used to disable interrupt generation
  for the specified GPIO input.

  @param port_id
    The port_id parameter identifies the GPIO port for which you want to disable
    interrupt generation. An enumeration item of the form MSS_GPIO_n, where n is
    the number of the GPIO port, is used to identify the GPIO port. For example,
    MSS_GPIO_0 identifies the first GPIO port and MSS_GPIO_31 is the last one.

  @return
    This function does not return a value.

  Example:
    The call to MSS_GPIO_disable_irq() below will prevent GPIO 8 from generating
    interrupts.
    @code
    MSS_GPIO_disable_irq( MSS_GPIO_8 );
    @endcode
 */
void MSS_GPIO_disable_irq
(
    mss_gpio_id_t port_id
);

/*-------------------------------------------------------------------------*//**
  The MSS_GPIO_clear_irq() function is used to clear a pending interrupt from
  the specified GPIO input.
  Note: The MSS_GPIO_clear_irq() function must be called as part of any GPIO
        interrupt service routine (ISR) in order to prevent the same interrupt
        event retriggering a call to the GPIO ISR.

  @param port_id
    The port_id parameter identifies the GPIO port for which you want to clear
    the interrupt. An enumeration item of the form MSS_GPIO_n, where n is the
    number of the GPIO port, is used to identify the GPIO port. For example,
    MSS_GPIO_0 identifies the first GPIO port and MSS_GPIO_31 is the last one.

  @return
    none.

  Example:
    The example below demonstrates the use of the MSS_GPIO_clear_irq() function
    as part of the GPIO 9 interrupt service routine.
    @code
    void GPIO9_IRQHandler( void )
    {
        do_interrupt_processing();

        MSS_GPIO_clear_irq( MSS_GPIO_9 );
    }
    @endcode
 */
void MSS_GPIO_clear_irq
(
    mss_gpio_id_t port_id
);

#ifdef __cplusplus
}
#endif

#endif /* MSS_GPIO_H_ */
