# GPIO Expander for BL602 / BL604 on Apache NuttX RTOS

See https://lupyuen.github.io/articles/pinedio2#gpio-expander

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

-   TODO: [See the modified Makefile and Kconfig]()

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

  /* Pin 3: an non-inverted, input pin */

  IOEXP_SETDIRECTION(ioe, 3, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INVERT,
                  (FAR void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  gpio_lower_half(ioe, 3, GPIO_INPUT_PIN, 3);

  /* Pin 4: an non-inverted, output pin */

  IOEXP_SETDIRECTION(ioe, 4, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 4, IOEXPANDER_OPTION_INVERT,
                  (FAR void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 4, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  gpio_lower_half(ioe, 4, GPIO_OUTPUT_PIN, 4);

  /* Pin 5: an non-inverted, edge interrupting pin */

  IOEXP_SETDIRECTION(ioe, 5, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 5, IOEXPANDER_OPTION_INVERT,
                  (FAR void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 5, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_BOTH);
  gpio_lower_half(ioe, 5, GPIO_INTERRUPT_PIN, 5);

  /* Pin 6: a non-inverted, level interrupting pin */

  IOEXP_SETDIRECTION(ioe, 6, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 6, IOEXPANDER_OPTION_INVERT,
                  (FAR void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 6, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_HIGH);
  gpio_lower_half(ioe, 6, GPIO_INTERRUPT_PIN, 3);
#endif  /* CONFIG_IOEXPANDER_BL602_EXPANDER */
```
