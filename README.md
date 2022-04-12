# GPIO Expander for BL602 / BL604 on Apache NuttX RTOS

See https://lupyuen.github.io/articles/pinedio2#gpio-expander

GPIO Expander handles multiple GPIOs on BL602 / BL604 by calling [`bl602_configgpio`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L58-L140), [`bl602_gpioread`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L218-L230) and [`bl602_gpiowrite`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L197-L216).

GPIO Expander exposes GPIOs 0 to 22 as `/dev/gpio0` to `/dev/gpio22`, for easier development of NuttX Apps for PineDio Stack BL604.

We'll skip `/dev/gpio0` to `/dev/gpio2` because they are already used by the SX1262 Driver. [(See this)](https://lupyuen.github.io/articles/sx1262#gpio-interface)

(On PineDio Stack: GPIO 0 is MISO, GPIO 1 is SDA, GPIO 2 is SCL. So we shouldn't touch GPIOs 0, 1 and 2 anyway. [See this](https://lupyuen.github.io/articles/pinedio2#appendix-gpio-assignment))

Warning: BL602 GPIO Driver will be disabled when we enable GPIO Expander, because GPIO Expander needs GPIO Lower Half which can't coexist with BL602 GPIO Driver.

TODO: Call [`bl602_configgpio`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L58-L140), [`bl602_gpioread`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L218-L230) and [`bl602_gpiowrite`](https://github.com/lupyuen/incubator-nuttx/blob/pinedio/arch/risc-v/src/bl602/bl602_gpio.c#L197-L216) to configure / read / write GPIOs

TODO: Handle GPIO Interrupts

TODO: GPIO Expander will enforce checks at runtime to be sure that NuttX Apps don't tamper with the GPIOs used by SPI, I2C and UART

TODO: GPIO Expander will check that the SPI / I2C / UART Pins are correctly defined (e.g. MISO vs MOSI) and are not reused

TODO: Eventually SX1262 Library will access `/dev/gpio10`, `/dev/gpio15`, `/dev/gpio19` instead of `dev/gpio0`, `/dev/gpio1`, `/dev/gpio2`

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

-   [See the modified Makefile and Kconfig](https://github.com/lupyuen/incubator-nuttx/commit/96f26693070eb9d6bbbf016ac6375ef041e3b24a?diff=unified)

Then update the NuttX Build Config...

```bash
## TODO: Change this to the path of our "incubator-nuttx" folder
cd nuttx/nuttx

## Preserve the Build Config
cp .config ../config

## Erase the Build Config and Kconfig files
make distclean

## For BL602: Configure the build for BL602
./tools/configure.sh bl602evb:nsh

## For PineDio Stack BL604: Configure the build for BL604
./tools/configure.sh bl602evb:pinedio

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

And call `bl602_expander_initialize` to initialise our driver:

https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L827-L874

```c
#ifdef CONFIG_IOEXPANDER_BL602_EXPANDER
#include <nuttx/ioexpander/bl602_expander.h>
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */
...
int bl602_bringup(void) {
  ...
#ifdef CONFIG_IOEXPANDER_BL602_EXPANDER
  /* Get an instance of the BL602 GPIO Expander */

  FAR struct ioexpander_dev_s *ioe = bl602_expander_initialize();
  if (ioe == NULL)
    {
      gpioerr("ERROR: bl602_expander_initialize failed\n");
      return -ENOMEM;
    }

  /* Register four pin drivers */

  /* GPIO 3: an non-inverted, input pin */

  IOEXP_SETDIRECTION(ioe, 3, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INVERT,
                  (FAR void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  gpio_lower_half(ioe, 3, GPIO_INPUT_PIN, 3);

  /* GPIO 4: an non-inverted, output pin */

  IOEXP_SETDIRECTION(ioe, 4, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 4, IOEXPANDER_OPTION_INVERT,
                  (FAR void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 4, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  gpio_lower_half(ioe, 4, GPIO_OUTPUT_PIN, 4);

  /* GPIO 5: an non-inverted, edge interrupting pin */

  IOEXP_SETDIRECTION(ioe, 5, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 5, IOEXPANDER_OPTION_INVERT,
                  (FAR void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 5, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_BOTH);
  gpio_lower_half(ioe, 5, GPIO_INTERRUPT_PIN, 5);

  /* GPIO 6: a non-inverted, level interrupting pin */

  IOEXP_SETDIRECTION(ioe, 6, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 6, IOEXPANDER_OPTION_INVERT,
                  (FAR void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 6, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_HIGH);
  gpio_lower_half(ioe, 6, GPIO_INTERRUPT_PIN, 6);
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */
```

We need to disable BL602 GPIO Driver when we enable GPIO Expander, because GPIO Expander needs GPIO Lower Half which can't coexist with BL602 GPIO Driver:

https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L635

```c
#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
```
