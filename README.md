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

https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L643-L699

```c
#ifdef CONFIG_IOEXPANDER_BL602_EXPANDER
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/bl602_expander.h>
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */
...
int bl602_bringup(void) {
  ...
  /* This is existing code */

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
  ret = bl602_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

  /* This is new code */

#ifdef CONFIG_IOEXPANDER_BL602_EXPANDER
  /* Must load BL602 GPIO Expander before other drivers */

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

We must load the GPIO Expander before other drivers (e.g. CST816S Touch Panel), because GPIO Expander provides GPIO functions for the drivers.

We need to disable BL602 GPIO Driver when we enable GPIO Expander, because GPIO Expander needs GPIO Lower Half which can't coexist with BL602 GPIO Driver:

https://github.com/lupyuen/incubator-nuttx/blob/expander/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L643

```c
#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
```

# Output Log

```text
spi_test_driver_register: devpath=/dev/spitest0, spidev=0
bl602_expander_set_intmod: gpio_pin=9, int_ctlmod=1, int_trgmod=0
bl602_irq_attach: Attach 0x2305e692
bl602_irq_enable: Disable interrupt
bl602_irq_enable: Enable interrupt
bl602_expander_direction: pin=3 direction=IN
bl602_expander_direction: pin=3 direction=IN
gpio_pin_register: Registering /dev/gpio3
bl602_expander_direction: pin=4 direction=OUT
bl602_expander_direction: pin=4 direction=OUT
gpio_pin_register: Registering /dev/gpio4
bl602_expander_direction: pin=5 direction=IN
gplh_enable: pin5: Disabling callback=0 handle=0
gplh_enable: WARNING: pin5: Already detached
gpio_pin_register: Registering /dev/gpio5
bl602_expander_direction: pin=6 direction=IN
gplh_enable: pin6: Disabling callback=0 handle=0
gplh_enable: WARNING: pin6: Already detached
gpio_pin_register: Registering /dev/gpio6

NuttShell (NSH) NuttX-10.3.0-RC0
nsh> ls /dev
/dev:
 console
 gpio3
 gpio4
 gpio5
 gpio6
 i2c0
 input0
 lcd0
 null
 spi0
 spitest0
 timer0
 urandom
 zero
nsh>
```
