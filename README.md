# GPIO Expander for BL602 / BL604 on Apache NuttX RTOS

See https://lupyuen.github.io/articles/pinedio2#gpio-expander

GPIO Expander handles multiple GPIOs on BL602 / BL604 by calling [`bl602_configgpio`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L58-L140), [`bl602_gpioread`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L218-L230) and [`bl602_gpiowrite`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L197-L216).

GPIO Expander exposes GPIOs 0 to 22 as `/dev/gpio0` to `/dev/gpio22`, for easier development of NuttX Apps for PineDio Stack BL604.

We'll skip `/dev/gpio0` to `/dev/gpio2` because they are already used by the SX1262 Driver. [(See this)](https://lupyuen.github.io/articles/sx1262#gpio-interface)

(On PineDio Stack: GPIO 0 is MISO, GPIO 1 is SDA, GPIO 2 is SCL. So we shouldn't touch GPIOs 0, 1 and 2 anyway. [See this](https://lupyuen.github.io/articles/pinedio2#appendix-gpio-assignment))

Warning: BL602 GPIO Driver will be disabled when we enable GPIO Expander, because GPIO Expander needs GPIO Lower Half which can't coexist with BL602 GPIO Driver.

# Status

- Works OK with GPIO Interrupts from Touch Panel and LVGL Test App

  (With `IOEP_ATTACH` in `cst816s_register`)

- Works OK with Push Button

  (With `IOEP_ATTACH` in `bl602_bringup`)

- Works OK with Push Button GPIO Command: `gpio -t 8 -w 1 /dev/gpio12`

  (Comment out `IOEP_ATTACH` in `bl602_bringup`)

__TODO__: Call [`bl602_configgpio`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L58-L140), [`bl602_gpioread`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L218-L230) and [`bl602_gpiowrite`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L197-L216) to configure / read / write GPIOs

__TODO__: Test with LoRaWAN Test App

__TODO__: GPIO Expander will enforce checks at runtime to be sure that NuttX Apps don't tamper with the GPIOs used by SPI, I2C and UART

__TODO__: GPIO Expander will check that the SPI / I2C / UART Pins are correctly defined (e.g. MISO vs MOSI) and are not reused

__TODO__: Eventually SX1262 Library will be configured by Kconfig to access `/dev/gpio10`, `/dev/gpio15`, `/dev/gpio19` (instead of `dev/gpio0`, `/dev/gpio1`, `/dev/gpio2`)

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

Edit the function `bl602_bringup` or `esp32_bringup` in this file...

```text
## For BL602:
nuttx/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c

## For ESP32: Change "esp32-devkitc" to our ESP32 board 
nuttx/boards/xtensa/esp32/esp32-devkitc/src/esp32_bringup.c
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

  /* Touch Panel GPIO 9: a non-inverted, falling-edge interrupting pin */
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

  /* Push Button GPIO 12: a non-inverted, falling-edge interrupting pin */
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
    void *handle = IOEP_ATTACH(bl602_expander,
                               (ioe_pinset_t)1 << gpio_pin,
                               button_isr_handler,
                               NULL);  ////  TODO
    DEBUGASSERT(handle != NULL);
  }

  /* Testing GPIO 3: a non-inverted, input pin */
  {
    uint8_t gpio_pin = 3;

    IOEXP_SETDIRECTION(bl602_expander, gpio_pin, IOEXPANDER_DIRECTION_IN);
    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INVERT,
                    (FAR void *)IOEXPANDER_VAL_NORMAL);
    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_DISABLE);
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INPUT_PIN, gpio_pin);
  }

  /* Testing GPIO 4: a non-inverted, output pin */
  {
    uint8_t gpio_pin = 4;

    IOEXP_SETDIRECTION(bl602_expander, gpio_pin, IOEXPANDER_DIRECTION_OUT);
    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INVERT,
                    (FAR void *)IOEXPANDER_VAL_NORMAL);
    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_DISABLE);
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_OUTPUT_PIN, gpio_pin);
  }

  /* Testing GPIO 5: a non-inverted, edge interrupting pin */
  {
    uint8_t gpio_pin = 5;

    IOEXP_SETDIRECTION(bl602_expander, gpio_pin, IOEXPANDER_DIRECTION_IN);
    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INVERT,
                    (FAR void *)IOEXPANDER_VAL_NORMAL);
    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_BOTH);
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);
  }

  /* Testing GPIO 6: a non-inverted, level interrupting pin */
  {
    uint8_t gpio_pin = 6;

    IOEXP_SETDIRECTION(bl602_expander, gpio_pin, IOEXPANDER_DIRECTION_IN);
    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INVERT,
                    (FAR void *)IOEXPANDER_VAL_NORMAL);
    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_HIGH);
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);
  }
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */
```

We must load the GPIO Expander before other drivers (e.g. CST816S Touch Panel), because GPIO Expander provides GPIO functions for the drivers.

We need to disable BL602 GPIO Driver when we enable GPIO Expander, because GPIO Expander needs GPIO Lower Half which can't coexist with BL602 GPIO Driver:

https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L643

```c
#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
```

`button_isr_handler` is defined as...

https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L1038-L1044

```c
static int button_isr_handler(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, FAR void *arg)
{
  #warning TODO: Move button_isr_handler to Button Handler
  gpioinfo("Button Pressed\n");
  return 0;
}
```

# Output Log

BL602 GPIO Expander works OK with LVGL Test App...

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
bl602_expander_set_intmod: gpio_pin=12, int_ctlmod=1, int_trgmod=0
spi_test_driver_register: devpath=/dev/spitest0, spidev=0
cst816s_register: path=/dev/input0, addr=21
bl602_expander_attach: pinset=200, callback=0x2305e700, arg=0x42020b90
bl602_expander_attach: Attach callback for gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_register: Driver registered

NuttShell (NSH) NuttX-10.3.0-RC0
nsh> bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_poll_notify:

bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_poll_notify:

nsh> lvgltest
tp_init: Opening /dev/input0
cst816s_open:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=4, touch=2, x=558, y=3330
cst816s_get_touch_data: Can't return touch data: id=4, touch=2, x=558, y=3330
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: UP: id=0, touch=0, x=216, y=25
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       216
cst816s_get_touch_data:   y:       25
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_ge_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=9, touch=2, x=639, y=1688
cst816s_get_touch_data: UP: id=0, touch=2, x=215, y=19
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       215
cst816s_get_touch_data:   y:       19
bl602_expander_interrupt: Interrupt! context=0x42024b08, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=216, y=218
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       216
cst816s_get_touch_data:   y:       218
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=216, y=218
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       216
cst816s_get_touch_data:   y:       218
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=216, y=218
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       216
cst816s_get_touch_data:   y:       218
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=9, touch=2, x=639, y=1688
cst816s_get_touch_data: UP: id=0, touch=2, x=216, y=218
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       216
cst816s_get_touch_data:   y:       218
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=32, y=200
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       32
cst816s_get_touch_data:   y:       200
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=32, y=200
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       32
cst816s_get_touch_data:   y:       200
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=4, touch=2, x=668, y=3586
cst816s_get_touch_data: UP: id=0, touch=2, x=32, y=200
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       32
cst816s_get_touch_data:   y:       200
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=31, y=15
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       31
cst816s_get_touch_data:   y:       15
cst816s_get_touch_data:
cst816s_i2c_rad:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=31, y=15
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       31
cst816s_get_touch_data:   y:       15
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=4, touch=2, x=668, y=3586
cst816s_get_touch_data: UP: id=0, touch=2, x=31, y=15
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       31
cst816s_get_touch_data:   y:       15
bl602_expander_interrupt: Interrupt! context=0x420248c8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=9, callback=0x2305e700, arg=0x42020b90
cst816s_poll_notify:
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=119, y=116
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       119
cst816s_get_touch_data:   y:       116
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=119, y=116
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       119
cst816s_get_touch_data:   y:       116
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=4, touch=2, x=668, y=3586
cst816s_get_touch_data: UP: id=0, touch=2, x=119, y=116
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       119
cst816s_get_touch_data:   y:       116
tp_cal result
offset x:19, y:31
range x:199, y:184
invert x/y:1, x:0, y:1
```

Testing Push Button with GPIO Command (comment out `IOEP_ATTACH` in `bl602_bringup`)...

```text
nsh> gpio -t 8 -w 1 /dev/gpio12
Driver: /dev/gpio12
gplh_enable: pin12: Disabling callback=0 handle=0
gplh_enable: WARNING: pin12: Already detached
bl602_expander_option: pin=12, option=2, value=0x6
bl602_expander_option: Rising edge: pin=12
bl602_expander_set_intmod: gpio_pin=12, int_ctlmod=1, int_trgmod=1
gplh_read: pin12: value=0x420216ff
bl602_expander_readpin: TODO: pin=12
                                      Interrupt pin: Value=0
gplh_attach: pin12: callback=0x230609d4
gplh_enable: pin12: Enabling callback=0x230609d4 handle=0
gplh_enable: pin12: Attaching 0x230609d4
bl602_expander_attach: pinset=1000, callback=0x2305f5c0, arg=0x420209e0
bl602_expander_attach: Attach callback for gpio=12, callback=0x2305f5c0, arg=0x420209e0
bl602_expander_interrupt: Interrupt! context=0x42012db8, priv=0x4201d0f0
bl602_expander_interrupt: Call gpio=12, callback=0x2305f5c0, arg=0x420209e0
gplh_handler: pin12: pinset: c callback=0x230609d4
gplh_enable: pin12: Disabling callback=0x230609d4 handle=0x4201d1a0
gplh_enable: pin12: Detaching handle=0x4201d1a0
bl602_expander_detach: Detach callback for gpio=12, callback=0x2305f5c0, arg=0x420209e0
gplh_attach: pin12: callback=0
gplh_read: pin12: value=0x420216ff
bl602_expander_readpin: TODO: pin=12
  Verify:        Value=0

nsh> ls /dev
/dev:
 console
 gpio12
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
```
