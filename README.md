![GPIO Expander for BL602 / BL604 on Apache NuttX RTOS](https://lupyuen.github.io/images/expander-title.jpg)

# GPIO Expander for BL602 / BL604 on Apache NuttX RTOS

[(Tested on PineDio Stack BL604)](https://lupyuen.github.io/articles/pinedio2)

See https://lupyuen.github.io/articles/pinedio2#gpio-expander

PineDio Stack BL604 has an interesting problem on Apache NuttX RTOS... Too many GPIOs! Let's make it work.

GPIO Expander exposes GPIOs 0 to 22 as `/dev/gpio0` to `/dev/gpio22`, for easier development of NuttX Apps for PineDio Stack BL604.

GPIO Expander calls [`bl602_configgpio`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L58-L140), [`bl602_gpioread`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L218-L230) and [`bl602_gpiowrite`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L197-L216) to configure / read / write GPIOs

Warning: [BL602 EVB GPIO Driver](https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c) will be disabled when we enable GPIO Expander.

(Because GPIO Expander needs GPIO Lower Half which conflicts with BL602 EVB GPIO Driver)

Robert Lipe has an excellent article that explains the current limitations of the BL602 EVB GPIO Driver (and why we need the GPIO Expander)...

-   ["Buttons on BL602 NuttX"](https://www.robertlipe.com/buttons-on-bl602-nuttx/)

# Status

-   Tested OK with GPIO Interrupts from Touch Panel and LVGL Test App `lvgltest`

    (With `IOEP_ATTACH` in `cst816s_register`)

-   Tested OK with Push Button

    (With `IOEP_ATTACH` in `bl602_bringup`)

-   Tested OK with Push Button GPIO Command: `gpio -t 8 -w 1 /dev/gpio12`

    (Comment out `IOEP_ATTACH` in `bl602_bringup`)

-   Tested OK with LoRaWAN Test App `lorawan_test`

    (With "GPIO Informational Output" logging disabled)

-   SX1262 Library is now configured by Kconfig / menuconfig to access `/dev/gpio10`, `/dev/gpio15`, `/dev/gpio19` (instead of `dev/gpio0`, `/dev/gpio1`, `/dev/gpio2`). 

    In menuconfig: Library Routines → Semtech SX1262 Library

    - SPI Test device path  
    - Chip Select device path 
    - Busy device path
    - DIO1 device path           

-   Logging for SX1262 Library is now disabled by default and can be configured by Kconfig / menuconfig.

    In menuconfig: Library Routines → Semtech SX1262 Library → Logging -> Debugging

-   Logging for SPI Test Driver has been moved from "Enable Informational Debug Output" to "SPI Informational Output"

__TODO__: GPIO Expander will enforce checks at runtime to be sure that NuttX Apps don't tamper with the GPIOs used by SPI, I2C and UART

__TODO__: GPIO Expander will check that the SPI / I2C / UART Pins are correctly defined (e.g. MISO vs MOSI) and are not reused

# Install Driver

To add this repo to your NuttX project...

```bash
pushd nuttx/nuttx/drivers/ioexpander
git submodule add https://github.com/lupyuen/bl602_expander
ln -s bl602_expander/bl602_expander.c .
popd

pushd nuttx/nuttx/include/nuttx/ioexpander
ln -s ../../../drivers/ioexpander/bl602_expander/bl602_expander.h .
popd
```
Next update the Makefile and Kconfig...

-   [See the modified Makefile and Kconfig](https://github.com/lupyuen/incubator-nuttx/commit/f72f7d546aa9b458b5cca66d090ac46ea178ca63)

Then update the NuttX Build Config...

```bash
## TODO: Change this to the path of our "incubator-nuttx" folder
cd nuttx/nuttx

## Preserve the Build Config
cp .config ../config

## Erase the Build Config and Kconfig files
make distclean

## For PineDio Stack BL604: Configure the build for BL604
./tools/configure.sh bl602evb:pinedio

## For BL602: Configure the build for BL602
./tools/configure.sh bl602evb:nsh

## For ESP32: Configure the build for ESP32.
## TODO: Change "esp32-devkitc" to our ESP32 board.
./tools/configure.sh esp32-devkitc:nsh

## Restore the Build Config
cp ../config .config

## Edit the Build Config
make menuconfig 
```

In menuconfig, enable the BL602 GPIO Expander under "Device Drivers → IO Expander/GPIO Support → Enable IO Expander Support".

Set "Number of pins" to 23.

Enable "GPIO Lower Half".

Edit the function `bl602_bringup` in this file...

```text
nuttx/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c
```

And call `bl602_expander_initialize` to initialise our driver, just after `bl602_gpio_initialize`:

https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L646-L754

```c
#ifdef CONFIG_IOEXPANDER_BL602_EXPANDER
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/bl602_expander.h>
FAR struct ioexpander_dev_s *bl602_expander = NULL;
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */
...
int bl602_bringup(void) {
  ...
  /* Existing Code */

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
  ret = bl602_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

  /* New Code */

#ifdef CONFIG_IOEXPANDER_BL602_EXPANDER
  /* Must load BL602 GPIO Expander before other drivers */

  bl602_expander = bl602_expander_initialize();
  if (bl602_expander == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Expander: %d\n", ret);
      return -ENOMEM;
    }

  /* Register pin drivers */

  /* Touch Panel (GPIO 9): a non-inverted, falling-edge interrupting pin */
  {
    gpio_pinset_t pinset = BOARD_TOUCH_INT;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);

    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_FALLING);
  }

  /* Push Button (GPIO 12): a non-inverted, falling-edge interrupting pin */
  {
    #define BOARD_BUTTON_INT (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN12)
    gpio_pinset_t pinset = BOARD_BUTTON_INT;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);

    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_FALLING);

    #warning TODO: Move IOEP_ATTACH to Button Handler
    // void *handle = IOEP_ATTACH(bl602_expander,
    //                            (ioe_pinset_t)1 << gpio_pin,
    //                            button_isr_handler,
    //                            NULL);  ////  TODO
    // DEBUGASSERT(handle != NULL);
  }

  /* SX1262 Busy (GPIO 10): a non-inverted, input pin */
  {
    #define BOARD_SX1262_BUSY (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN10)
    gpio_pinset_t pinset = BOARD_SX1262_BUSY;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INPUT_PIN, gpio_pin);
  }

  /* SX1262 Chip Select (GPIO 15): a non-inverted, output pin */
  {
    gpio_pinset_t pinset = BOARD_SX1262_CS;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_OUTPUT_PIN, gpio_pin);
  }

  /* SX1262 Interupt (GPIO 19): a non-inverted, falling-edge interrupt */
  {
    gpio_pinset_t pinset = BOARD_GPIO_INT1;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);

    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_FALLING);
  }
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */
```

We must load the GPIO Expander before other drivers (e.g. CST816S Touch Panel), because GPIO Expander provides GPIO functions for the drivers.

We need to disable BL602 GPIO Driver when we enable GPIO Expander, because GPIO Expander needs GPIO Lower Half which can't coexist with BL602 GPIO Driver:

```c
#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
  ret = bl602_gpio_initialize();
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L643)

`button_isr_handler` is defined as...

```c
static int button_isr_handler(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, FAR void *arg)
{
  #warning TODO: Move button_isr_handler to Button Handler
  gpioinfo("Button Pressed\n");
  return 0;
}
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L1038-L1044)

Here's how we created the BL602 GPIO Expander...

# BL602 EVB Limitations

The NuttX GPIO Driver for BL602 EVB supports one GPIO Input, one GPIO Output and one GPIO Interrupt ... And names them sequentially: "/dev/gpio0", "/dev/gpio1", "/dev/gpio2"

-   [BL602 EVB GPIO Driver](https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L537-L607)

Which can be super confusing because "/dev/gpio0" doesn't actually map to BL602 GPIO Pin 0.

[("/dev/gpio0" maps to BL602 GPIO Pin 10)](https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/include/board.h#L49-L52)

[("/dev/gpio1" maps to BL602 GPIO Pin 15)](https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/include/board.h#L54-L58)

[("/dev/gpio2" maps to BL602 GPIO Pin 19)](https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/include/board.h#L59-L63)

What happens when we try to support 23 GPIOs on PineDio Stack BL604? Yep the GPIO Names will look really messy on NuttX.

All 23 GPIOs on PineDio Stack #BL604 are wired up! Let's simplify NuttX and name the GPIOs as "/dev/gpio0" to "/dev/gpio22".

(So that "/dev/gpioN" will map to BL602 GPIO Pin N)

TODO

# Test Touch Panel

BL602 GPIO Expander tested OK with Touch Panel and LVGL Test App...

```text
bl602_expander_irq_enable: Enable interrupt
bl602_expander_option: pin=9, option=2, value=0xe
bl602_expander_option: Unsupported interrupt both edge: pin=9
gplh_enable: pin9: Disabling callback=0 handle=0
gplh_enable: WARNING: pin9: Already detached
gpio_pin_register: Registering /dev/gpio9
bl602_expander_option: pin=9, option=2, value=0xa
bl602_expander_option: Falling edge: pin=9
bl602_expander_set_intmod: gpio_pin=9, int_ctlmod=1, int_trgmod=0
bl602_expander_option: pin=12, option=2, value=0xe
bl602_expander_option: Unsupported interrupt both edge: pin=12
gplh_enable: pin12: Disabling callback=0 handle=0
gplh_enable: WARNING: pin12: Already detached
gpio_pin_register: Registering /dev/gpio12
bl602_expander_option: pin=12, option=2, value=0xa
bl602_expander_option: Falling edge: pin=12
bl602_expander_set_intmo: gpio_pin=12, int_ctlmod=1, int_trgmod=0
bl602_expander_direction: Unsupported direction: pin=10, direction=IN
bl602_expander_option: pin=10, option=2, value=0
bl602_expander_option: ERROR: Unsupported interrupt: 0, pin=10
gpio_pin_register: Registering /dev/gpio10
bl602_expander_direction: Unsupported direction: pin=15, direction=OUT
gpio_pin_register: Registering /dev/gpio15
bl602_expander_option: pin=19, option=2, value=0xe
bl602_expander_option: Unsupported interrupt both edge: pin=19
gplh_enable: pin19: Disabling callback=0 handle=0
gplh_enable: WARNING: pin19: Already detached
gpio_pin_register: Registering /dev/gpio19
bl602_expander_option: pin=19, option=2, value=0xa
bl602_expander_option: Falling edge: pin=19
bl602_expander_set_intmod: gpio_pin=19, int_ctlmod=1, int_trgmod=0
cst816s_register: path=/dev/input0, addr=21
bl602_expander_attach: pinset=200, callback=0x2305e542, arg=0x42020d40
bl602_expander_attach: Attach callback for gpio=9, callback=0x2305e542, arg=0x42020d40
cst816s_register: Driver registered

NuttShell (NSH) NuttX-10.3.0-RC0

nsh> ls /dev
/dev:
 console
 gpio10
 gpio12
 gpio15
 gpio19
 gpio9
 i2c0
 input0
 lcd0
 null
 spi0
 spitest0
 timer0
 urandom
 zero

nsh> lvgltest
tp_init: Opening /dev/input0
cst816s_open:
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e542, arg=0x42020d40
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_trasfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=9, touch=2, x=639, y=1688
cst816s_get_touch_data: UP: id=0, touch=2, x=201, y=20
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       20
bl602_expander_interrupt: Interrupt! context=0x42024b08, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e542, arg=0x42020d40
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=204, y=209
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       204
cst816s_get_touch_data:   y:       209
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=204, y=209
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       204
cst816s_get_touch_data:   y:       209
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=5, touch=2, x=588, y=2
cst816s_get_touch_data: UP: id=0, touch=2, x=204, y=209
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       204
cst816s_get_touch_data:   y:       209
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e542, arg=0x42020d40
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=7, y=207
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       7
cst816s_get_touch_data:   y:       207
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=7, y=207
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       7
cst816s_get_touch_data:   y:       207
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=5, touch=2, x=588, y=2
cst816s_get_touch_data: UP: id=0, touch=2, x=7, y=207
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       7
cst816s_get_touch_data:   y:       207
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e542, arg=0x42020d40
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=1, y=22
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       22
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst86s_get_touch_data: DOWN: id=0, touch=0, x=1, y=22
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       22
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=5, touch=2, x=588, y=2
cst816s_get_touch_data: UP: id=0, touch=2, x=1, y=22
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       22
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e542, arg=0x42020d40
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=119, y=120
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       119
cst816s_get_touch_data:   y:       120
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=119, y=120
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       119
cst816s_get_touch_data:   y:       120
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=5, touch=2, x=588, y=2
cst816s_get_touch_data: UP: id=0, touch=2, x=119, y=120
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       119
cst816s_get_touch_data:   y:       120
tp_cal result
offset x:20, y:1
range x:189, y:200
invert x/y:1, x:0, y:1
```

# Test Push Button

BL602 GPIO Expander tested OK with Push Button and GPIO Command...

(Comment out `IOEP_ATTACH` in `bl602_bringup`)

```text
nsh> gpio -t 8 -w 1 /dev/gpio12
Driver: /dev/gpio12
gplh_enable: pin12: Disabling callback=0 handle=0
gplh_enable: WARNING: pin12: Already detached
bl602_expander_option: pin=12, option=2, value=0x6
bl602_expander_option: Rising edge: pin=12
bl602_expander_set_intmod: gpio_pin=12, int_ctlmod=1, int_trgmod=1
gplh_read: pin12: value=0x420218af
bl602_expander_readpin: pin=12, value=1
  Interrupt pin: Value=1
gplh_attach: pin12: callback=0x23060752
gplh_enable: pin12: Enabling callback=0x23060752 handle=0
gplh_enable: pin12: Attaching 0x23060752
bl602_expander_attach:pinset=1000, callback=0x2305f42c, arg=0x420209e0
bl602_expander_attach: Attach callback for gpio=12, callback=0x2305f42c, arg=0x420209e0
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=12, callback=0x2305f42c, arg=0x420209e0
gplh_handler: pin12: pinset: c callback=0x23060752
gplh_enable: pin12: Disabling callback=0x23060752 handle=0x4201d1a0
gplh_enable: pin12: Detaching handle=0x4201d1a0
bl602_expander_detach: Detach callback for gpio=12, callback=0x2305f42c, arg=0x420209e0
gplh_attach: pin12: callback=0
gplh_read: pin12: value=0x420218af
bl602_expander_readpin: pin=12, value=1
  Verify:        Value=1
```

# Test LoRaWAN

BL602 GPIO Expander tested OK with LoRaWAN Test App...

(With "GPIO Informational Output" logging disabled)

```text
bl602_expander_option: Unsupported interrupt both edge: pin=9
gplh_enable: WARNING: pin9: Already detached
bl602_expander_option: Unsupported interrupt both edge: pin=12
gplh_enable: WARNING: pin12: Already detached
bl602_expander_direction: Unsupported direction: pin=10, direction=IN
bl602_expander_option: ERROR: Unsupported interrupt: 0, pin=10
bl602_expander_direction: Unsupported direction: pin=15, direction=OUT
bl602_expander_option: Unsupported interrupt both edge: pin=19
gplh_enable: WARNING: pin19: Already detached
cst816s_register: path=/dev/input0, addr=21
cst816s_register: Driver registered

NuttShell (NSH) NuttX-10.3.0-RC0

nsh> ls /dev
/dev:
 console
 gpio10
 gpio12
 gpio15
 gpio19
 gpio9
 i2c0
 input0
 lcd0
 null
 spi0
 spitest0
 timer0
 urandom
 zero

nsh> lorawan_test
init_entropy_pool
offset = 2228
temperature = 34.825230 Celsius
offset = 2228
temperature = 37.017929 Celsius
offset = 2228
temperature = 33.535404 Celsius
offset = 2228
temperature = 35.986069 Celsius

###### ===================================== ######

Application name   : lorawan_test
Application version: 1.2.0
GitHub base version: 5.0.0

###### ===================================== ######

init_event_queue
TimerInit:     0x4201c764
TimerInit:     0x4201c780
TimerInit:     0x4201c79c
TimerInit:     0x4201c818
TimerInit:     0x4201c8cc
TimerInit:     0x4201c8e8
TimerInit:     0x4201c904
TimerInit:     0x4201c920
TODO: RtcGetCalendarTime
TODO: SX126xReset
init_gpio
DIO1 pintype before=5
init_gpio: change DIO1 to Trigger GPIO Interrupt on Rising Edge
gplh_enable: WARNING: pin19: Already detached
DIO1 pintype after=8
Starting process_dio1
init_spi
SX126xSetTxParams: power=22, rampTime=7
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1
TimerInit:     0x4201b864
TimerInit:     0x4201b7d0
RadioSetModem
RadioSetModem
RadioSetPublicNetwork: public syncword=3444
RadioSleep
callout_handler: lock
process_dio1 started
process_dio1: event=0x4201b88c
TODO: EepromMcuReadBuffer
TODO: EepromMcuReadBuffer
TODO: EepromMcuReadBuffer
TODO: EepromMcuReadBuffer
TODO: EepromMcuReadBuffer
TODO: EepromMcuReadBuffer
TODO: EepromMcuReadBuffer
TODO: EepromMcuReadBuffer
RadioSetModem
RadioSetPublicNetwork: public syncword=3444
DevEui      : 4B-C1-5E-E7-37-7B-B1-5B
JoinEui     : 00-00-00-00-00-00-00-00
Pin         : 00-00-00-00

TimerInit:     0x4201c3bc
TimerInit:     0x4201c3d8
TimerInit:     0x4201c29c
TODO: RtcGetCalendarTme
TODO: RtcBkupRead
TODO: RtcBkupRead
RadioSetChannel: freq=923200000
RadioSetTxConfig: modem=1, power=13, fdev=0, bandwidth=0, datarate=10, coderate=            1, preambleLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, tim            eout=4000
RadioSetTxConfig: SpreadingFactor=10, Bandwidth=4, CodingRate=1, LowDatarateOpti            mize=0, PreambleLength=8, HeaderType=0, PayloadLength=255, CrcMode=1, InvertIQ=0
RadioStandby
RadioSetModem
SX126xSetTxParams: power=13, rampTime=7
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1
SecureElementRandomNumber: 0xce9d8482
RadioSend: size=23
00 00 00 00 00 00 00 00 00 5b b1 7b 37 e7 5e c1 4b 82 84 46 8a 70 d4
RadioSend: PreambleLength=8, HeaderType=0, PayloadLength=23, CrcMode=1, InvertIQ            =0
TimerStop:     0x4201b864
TimerStart2:   0x4201b864, 4000 ms
callout_reset: evq=0x420131a8, ev=0x4201b864

###### =========== MLME-Request ============ ######
######               MLME_JOIN               ######
###### ===================================== ######
STATUS      : OK
StartTxProcess
TimerInit:     0x42015b08
TimerSetValue: 0x42015b08, 42249 ms
OnTxTimerEvent: timeout in 42249 ms, event=0
TimerStop:     0x42015b08
TimerSetValue: 0x42015b08, 42249 ms
TimerStart:    0x42015b08
TimerStop:     0x42015b08
TimerStart2:   0x42015b08, 42249 ms
callout_reset: evq=0x420131a8, ev=0x42015b08
handle_event_queue
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_TX_DONE
TimerStop:     0x4201b864
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerSetValue: 0x4201c780, 4988 ms
TimerStart:    0x4201c780
TimerStop:     0x4201c780
TimerStart2:   0x4201c780, 4988 ms
callout_reset: evq=0x420131a8, ev=0x4201c780
TimerSetValue: 0x4201c79c, 5988 ms
TimerStart:    0x4201c79c
TimerStop:     0x4201c79c
TimerStart2:   0x4201c79c, 5988 ms
callout_reset: evq=0x420131a8, ev=0x4201c79c
TODO: RtcGetCalendarTime
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x4201c780
callout_handler: lock
handle_event_queue: ev=0x4201c780
TimerStop:     0x4201c780
RadioStandby
RadioSetChannel: freq=923200000
RadioSetRxConfig
RadioStandby
RadioSetModem
RadioSetRxConfig done
RadioRx
TimerStop:     0x4201b7d0
TimerStart2:   0x4201b7d0, 3000 ms
callout_reset: evq=0x420131a8, ev=0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_PREAMBLE_DETECTED
RadioOnDioIrq
RadioIrqProcess
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_HEADER_VALID
RadioOnDioIrq
RadioIrqProcess
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_RX_DONE
TimerStop:     0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerStop:     0x4201c79c
OnTxData

###### =========== MLME-Confirm ============ ######
STATUS      : OK
OnJoinRequest
###### ===========   JOINED     ============ ######

OTAA

DevAddr     :  018178EC


DATA RATE   : DR_2

TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
UplinkProcess
PrepareTxFrame: Transmit to LoRaWAN: Hi NuttX (9 bytes)
PrepareTxFrame: status=0, maxSize=11, currentSize=11
LmHandlerSend: Data frame
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioSetChannel: freq=923400000
RadioSetTxConfig: modem=1, power=13, fdev=0, bandwidth=0, datarate=9, coderate=1            , preambleLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, time            out=4000
RadioSetTxConfig: SpreadingFactor=9, Bandwidth=4, CodingRate=1, LowDatarateOptim            ize=0, PreambleLength=8, HeaderType=0, PayloadLength=128, CrcMode=1, InvertIQ=0
RadioStandby
RadioSetModem
SX126xSetTxParams: power=13, rampTime=7
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1
RadioSend: size=22
40 ec 78 81 01 00 01 00 01 f4 ca 04 16 1b 85 35 fe 43 7d 93 7a 3c
RadioSend: PreambleLength=8, HeaderType=0, PayloadLength=22, CrcMode=1, InvertIQ            =0
TimerStop:     0x4201b864
TimerStart2:   0x4201b864, 4000 ms
callout_reset: evq=0x420131a8, ev=0x4201b864

###### =========== MCPS-Request ============ ######
######           MCPS_UNCONFIRMED            ######
###### ===================================== ######
STATUS      : OK
PrepareTxFrame: Transmit OK
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_TX_DONE
TimerStop:     0x4201b864
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerSetValue: 0x4201c780, 980 ms
TimerStart:    0x4201c780
TimerStop:     0x4201c780
TimerStart2:   0x4201c780, 980 ms
callout_reset: evq=0x420131a8, ev=0x4201c780
TimerSetValue: 0x4201c79c, 1988 ms
TimerStart:    0x4201c79c
TimerStop:     0x4201c79c
TimerStart2:   0x4201c79c, 1988 ms
callout_reset: evq=0x420131a8, ev=0x4201c79c
TODO: RtcGetCalendarTime
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x4201c780
callout_handler: lock
handle_event_queue: ev=0x4201c780
TimerStop:     0x4201c780
RadioStandby
RadioSetChannel: freq=923400000
RadioSetRxConfig
RadioStandby
RadioSetModem
RadioSetRxConfig done
RadioRx
TimerStop:     0x4201b7d0
TimerStart2:   0x4201b7d0, 3000 ms
callout_reset: evq=0x420131a8, ev=0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_RX_TX_TIMEOUT
TimerStop:     0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerStop:     0x4201c79c
TimerStop:     0x4201c764
OnTxData

###### =========== MCPS-Confirm ============ ######
STATUS      : OK

###### =====   UPLINK FRAME        1   ===== ######

CLASS       : A

TX PORT     : 1
TX DATA     : UNCONFIRMED
48 69 20 4E 75 74 74 58 00

DATA RATE   : DR_3
U/L FREQ    : 923400000
TX POWER    : 0
CHANNEL MASK: 0003

TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
UplinkProcess
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x42015b08
callout_handler: lock
handle_event_queue: ev=0x42015b08
OnTxTimerEvent: timeout in 42249 ms, event=0x42015b08
TimerStop:     0x42015b08
TimerSetValue: 0x42015b08, 42249 ms
TimerStart:    0x42015b08
TimerStop:     0x42015b08
TimerStart2:   0x42015b08, 42249 ms
callout_reset: evq=0x420131a8, ev=0x42015b08
RadioOnDioIrq
RadioIrqProcess
UplinkProcess
PrepareTxFrame: Transmit to LoRaWAN: Hi NuttX (9 bytes)
PrepareTxFrame: status=0, maxSize=53, currentSize=53
LmHandlerSend: Data frame
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioSetChannel: freq=923400000
RadioSetTxConfig: modem=1, power=13, fdev=0, bandwidth=0, datarate=9, coderate=1, preambleLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, timeout=4000
RadioSetTxConfig: SpreadingFactor=9, Bandwidth=4, CodingRate=1, LowDatarateOptimize=0, PreambleLength=8, HeaderType=0, PayloadLength=128, CrcMode=1, InvertIQ=0
RadioStandby
RadioSetModem
SX126xSetTxParams: power=13, rampTime=7
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1
RadioSend: size=22
40 ec 78 81 01 00 02 00 01 16 29 a6 e7 02 47 ba b6 fb cd e2 74 14
RadioSend: PreambleLength=8, HeaderType=0, PayloadLength=22, CrcMode=1, InvertIQ=0
TimerStop:     0x4201b864
TimerStart2:   0x4201b864, 4000 ms
callout_reset: evq=0x420131a8, ev=0x4201b864

###### =========== MCPS-Request ============ ######
######           MCPS_UNCONFIRMED            ######
###### ===================================== ######
STATUS      : OK
PrepareTxFrame: Transmit OK
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_TX_DONE
TimerStop:     0x4201b864
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerSetValue: 0x4201c780, 980 ms
TimerStart:    0x4201c780
TimerStop:     0x4201c780
TimerStart2:   0x4201c780, 980 ms
callout_reset: evq=0x420131a8, ev=0x4201c780
TimerSetValue: 0x4201c79c, 1988 ms
TimerStart:    0x4201c79c
TimerStop:     0x4201c79c
TimerStart2:   0x4201c79c, 1988 ms
callout_reset: evq=0x420131a8, ev=0x4201c79c
TODO: RtcGetCalendarTime
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x4201c780
callout_handler: lock
handle_event_queue: ev=0x4201c780
TimerStop:     0x4201c780
RadioStandby
RadioSetChannel: freq=923400000
RadioSetRxConfig
RadioStandby
RadioSetModem
RadioSetRxConfig done
RadioRx
TimerStop:     0x4201b7d0
TimerStart2:   0x4201b7d0, 3000 ms
callout_reset: evq=0x420131a8, ev=0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_RX_TX_TIMEOUT
TimerStop:     0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerStop:     0x4201c79c
TimerStop:     0x4201c764
OnTxData

###### =========== MCPS-Confirm ============ ######
STATUS      : OK

###### =====   UPLINK FRAME        2   ===== ######

CLASS       : A

TX PORT     : 1
TX DATA     : UNCONFIRMED
48 69 20 4E 75 74 74 58 00

DATA RATE   : DR_3
U/L FREQ    : 923400000
TX POWER    : 0
CHANNEL MASK: 0003

TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
UplinkProcess
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x42015b08
callout_handler: lock
handle_event_queue: ev=0x42015b08
OnTxTimerEvent: timeout in 42249 ms, event=0x42015b08
TimerStop:     0x42015b08
TimerSetValue: 0x42015b08, 42249 ms
TimerStart:    0x42015b08
TimerStop:     0x42015b08
TimerStart2:   0x42015b08, 42249 ms
callout_reset: evq=0x420131a8, ev=0x42015b08
RadioOnDioIrq
RadioIrqProcess
UplinkProcess
PrepareTxFrame: Transmit to LoRaWAN: Hi NuttX (9 bytes)
PrepareTxFrame: status=0, maxSize=53, currentSize=53
LmHandlerSend: Data frame
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioSetChannel: freq=923200000
RadioSetTxConfig: modem=1, power=13, fdev=0, bandwidth=0, datarate=9, coderate=1, preambleLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, timeout=4000
RadioSetTxConfig: SpreadingFactor=9, Bandwidth=4, CodingRate=1, LowDatarateOptimize=0, PreambleLength=8, HeaderType=0, PayloadLength=128, CrcMode=1, InvertIQ=0
RadioStandby
RadioSetModem
SX126xSetTxParams: power=13, rampTime=7
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1
RadioSend: size=22
40 ec 78 81 01 00 03 00 01 ba a5 45 1e 60 15 2f 89 fc 99 28 6e 4f
RadioSend: PreambleLength=8, HeaderType=0, PayloadLength=22, CrcMode=1, InvertIQ=0
TimerStop:     0x4201b864
TimerStart2:   0x4201b864, 4000 ms
callout_reset: evq=0x420131a8, ev=0x4201b864

###### =========== MCPS-Request ============ ######
######           MCPS_UNCONFIRMED            ######
###### ===================================== ######
STATUS      : OK
PrepareTxFrame: Transmit OK
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_TX_DONE
TimerStop:     0x4201b864
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerSetValue: 0x4201c780, 980 ms
TimerStart:    0x4201c780
TimerStop:     0x4201c780
TimerStart2:   0x4201c780, 980 ms
callout_reset: evq=0x420131a8, ev=0x4201c780
TimerSetValue: 0x4201c79c, 1988 ms
TimerStart:    0x4201c79c
TimerStop:     0x4201c79c
TimerStart2:   0x4201c79c, 1988 ms
callout_reset: evq=0x420131a8, ev=0x4201c79c
TODO: RtcGetCalendarTime
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x4201c780
callout_handler: lock
handle_event_queue: ev=0x4201c780
TimerStop:     0x4201c780
RadioStandby
RadioSetChannel: freq=923200000
RadioSetRxConfig
RadioStandby
RadioSetModem
RadioSetRxConfig done
RadioRx
TimerStop:     0x4201b7d0
TimerStart2:   0x4201b7d0, 3000 ms
callout_reset: evq=0x420131a8, ev=0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_RX_TX_TIMEOUT
TimerStop:     0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerStop:     0x4201c79c
TimerStop:     0x4201c764
OnTxData

###### =========== MCPS-Confirm ============ ######
STATUS      : OK

###### =====   UPLINK FRAME        3   ===== ######

CLASS       : A

TX PORT     : 1
TX DATA     : UNCONFIRMED
48 69 20 4E 75 74 74 58 00

DATA RATE   : DR_3
U/L FREQ    : 923200000
TX POWER    : 0
CHANNEL MASK: 0003

TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
UplinkProcess
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x42015b08
callout_handler: lock
handle_event_queue: ev=0x42015b08
OnTxTimerEvent: timeout in 42249 ms, event=0x42015b08
TimerStop:     0x42015b08
TimerSetValue: 0x42015b08, 42249 ms
TimerStart:    0x42015b08
TimerStop:     0x42015b08
TimerStart2:   0x42015b08, 42249 ms
callout_reset: evq=0x420131a8, ev=0x42015b08
RadioOnDioIrq
RadioIrqProcess
UplinkProcess
PrepareTxFrame: Transmit to LoRaWAN: Hi NuttX (9 bytes)
PrepareTxFrame: status=0, maxSize=53, currentSize=53
LmHandlerSend: Data frame
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioSetChannel: freq=923400000
RadioSetTxConfig: modem=1, power=13, fdev=0, bandwidth=0, datarate=9, coderate=1, preableLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, timeout=4000
RadioSetTxConfig: SpreadingFactor=9, Bandwidth=4, CodingRate=1, LowDatarateOptimize=0, PreambleLength=8, HeaderType=0, PayloadLength=128, CrcMode=1, InvertIQ=0
RadioStandby
RadioSetModem
SX126xSetTxParams: power=13, rampTime=7
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1
RadioSend: size=22
40 ec 78 81 01 00 04 00 01 0f 50 b4 c9 58 d0 31 1b 4d 44 37 51 f7
RadioSend: PreambleLength=8, HeaderType=0, PayloadLength=22, CrcMode=1, InvertIQ=0
TimerStop:     0x4201b864
TimerStart2:   0x4201b864, 4000 ms
callout_reset: evq=0x420131a8, ev=0x4201b864

###### =========== MCPS-Request ============ ######
######           MCPS_UNCONFIRMED            ######
###### ===================================== ######
STATUS      : OK
PrepareTxFrame: Transmit OK
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_TX_DONE
TimerStop:     0x4201b864
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerSetValue: 0x4201c780, 980 ms
TimerStart:    0x4201c780
TimerStop:     0x4201c780
TimerStart2:   0x4201c780, 980 ms
callout_reset: evq=0x420131a8, ev=0x4201c780
TimerSetValue: 0x4201c79c, 1988 ms
TimerStart:    0x4201c79c
TimerStop:     0x4201c79c
TimerStart2:   0x4201c79c, 1988 ms
callout_reset: evq=0x420131a8, ev=0x4201c79c
TODO: RtcGetCalendarTime
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x4201c780
callout_handler: lock
handle_event_queue: ev=0x4201c780
TimerStop:     0x4201c780
RadioStandby
RadioSetChannel: freq=923400000
RadioSetRxConfig
RadioStandby
RadioSetModem
RadioSetRxConfig done
RadioRx
TimerStop:     0x4201b7d0
TimerStart2:   0x4201b7d0, 3000 ms
callout_reset: evq=0x420131a8, ev=0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_RX_TX_TIMEOUT
TimerStop:     0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerStop:     0x4201c79c
TimerStop:     0x4201c764
OnTxData

###### =========== MCPS-Confirm ============ ######
STATUS      : OK

###### =====   UPLINK FRAME        4   ===== ######

CLASS       : A

TX PORT     : 1
TX DATA     : UNCONFIRMED
48 69 20 4E 75 74 74 58 00

DATA RATE   : DR_3
U/L FREQ    : 923400000
TX POWER    : 0
CHANNEL MASK: 0003

TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
UplinkProcess
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x42015b08
callout_handler: lock
handle_event_queue: ev=0x42015b08
OnTxTimerEvent: timeout in 42249 ms, event=0x42015b08
TimerStop:     0x42015b08
TimerSetValue: 0x42015b08, 42249 ms
TimerStart:    0x42015b08
TimerStop:     0x42015b08
TimerStart2:   0x42015b08, 42249 ms
callout_reset: evq=0x420131a8, ev=0x42015b08
RadioOnDioIrq
RadioIrqProcess
UplinkProcess
PrepareTxFrame: Transmit to LoRaWAN: Hi NuttX (9 bytes)
PrepareTxFrame: status=0, maxSize=53, currentSize=53
LmHandlerSend: Data frame
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioSetChannel: freq=923200000
RadioSetTxConfig: modem=1, power=13, fdev=0, bandwidth=0, datarate=9, coderate=1, preambleLen=8, fixLen=0, crcOn=1, freqHopOn=0, hopPeriod=0, iqInverted=0, timeout=4000
RadioSetTxConfig: SpreadingFactor=9, Bandwidth=4, CodingRate=1, LowDatarateOptimize=0, PreambleLength=8, HeaderType=0, PayloadLength=128, CrcMode=1, InvertIQ=0
RadioStandby
RadioSetModem
SX126xSetTxParams: power=13, rampTime=7
SX126xSetPaConfig: paDutyCycle=4, hpMax=7, deviceSel=0, paLut=1
RadioSend: size=22
40 ec 78 81 01 00 05 00 01 00 c2 cb 38 37 48 ae 23 9e 35 9e c7 ed
RadioSend: PreambleLength=8, HeaderType=0, PayloadLength=22, CrcMode=1, InvertIQ=0
TimerStop:     0x4201b864
TimerStart2:   0x4201b864, 4000 ms
callout_reset: evq=0x420131a8, ev=0x4201b864

###### =========== MCPS-Request ============ ######
######           MCPS_UNCONFIRMED            ######
###### ===================================== ######
STATUS      : OK
PrepareTxFrame: Transmit OK
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_TX_DONE
TimerStop:     0x4201b864
TODO: RtcGetCalendarTime
TODO: RtcBkupRead
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerSetValue: 0x4201c780, 980 ms
TimerStart:    0x4201c780
TimerStop:     0x4201c780
TimerStart2:   0x4201c780, 980 ms
callout_reset: evq=0x420131a8, ev=0x4201c780
TimerSetValue: 0x4201c79c, 1988 ms
TimerStart:    0x4201c79c
TimerStop:     0x4201c79c
TimerStart2:   0x4201c79c, 1988 ms
callout_reset: evq=0x420131a8, ev=0x4201c79c
TODO: RtcGetCalendarTime
callout_handler: unlock
callout_handler: evq=0x420131a8, ev=0x4201c780
callout_handler: lock
handle_event_queue: ev=0x4201c780
TimerStop:     0x4201c780
RadioStandby
RadioSetChannel: freq=923200000
RadioSetRxConfig
RadioStandby
RadioSetModem
RadioSetRxConfig done
RadioRx
TimerStop:     0x4201b7d0
TimerStart2:   0x4201b7d0, 3000 ms
callout_reset: evq=0x420131a8, ev=0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
DIO1 add event
handle_event_queue: ev=0x4201b88c
RadioOnDioIrq
RadioIrqProcess
IRQ_RX_TX_TIMEOUT
TimerStop:     0x4201b7d0
RadioOnDioIrq
RadioIrqProcess
RadioSleep
TimerStop:     0x4201c79c
TimerStop:     0x4201c764
OnTxData

###### =========== MCPS-Confirm ============ ######
STATUS      : OK

###### =====   UPLINK FRAME        5   ===== ######

CLASS       : A

TX PORT     : 1
TX DATA     : UNCONFIRMED
48 69 20 4E 75 74 74 58 00

DATA RATE   : DR_3
U/L FREQ    : 923200000
TX POWER    : 0
CHANNEL MASK: 0003

TODO: EepromMcuWriteBuffer
TODO: EepromMcuWriteBuffer
UplinkProcess
```
