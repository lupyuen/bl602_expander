/****************************************************************************
 * drivers/ioexpander/bl602_expander.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/bl602_expander.h>
#include <nuttx/ioexpander/gpio.h>
#include <arch/board/board.h>
#include "../arch/risc-v/src/common/riscv_internal.h"
#include "../arch/risc-v/src/bl602/bl602_gpio.h"

#if defined(CONFIG_IOEXPANDER_BL602_EXPANDER)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* Interrupt Handler */

typedef int isr_handler(int irq, FAR void *context, FAR void *arg);

/* This type represents on registered pin interrupt callback */

struct bl602_expander_callback_s
{
  ioe_pinset_t pinset;          /* Set of pin interrupts that will generate the callback */
  ioe_callback_t cbfunc;        /* The saved callback function pointer */
  FAR void* cbarg;              /* The saved callback argument */
};
#endif

/* This structure represents the state of the I/O Expander driver */

struct bl602_expander_dev_s
{
  struct ioexpander_dev_s dev;  /* Nested structure to allow casting as public gpio expander */
#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
  FAR struct bl602_expander_dev_s *flink; /* Supports a singly linked list of drivers */
#endif
  sem_t exclsem;                /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  struct work_s work;           /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct bl602_expander_callback_s cb[CONFIG_IOEXPANDER_NPINS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bl602_expander_lock(FAR struct bl602_expander_dev_s *priv);

static int bl602_expander_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int dir);
static int bl602_expander_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                       int opt, void *val);
static int bl602_expander_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value);
static int bl602_expander_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value);
static int bl602_expander_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int bl602_expander_multiwritepin(FAR struct ioexpander_dev_s *dev,
                              FAR uint8_t *pins, FAR bool *values,
                              int count);
static int bl602_expander_multireadpin(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count);
static int bl602_expander_multireadbuf(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *bl602_expander_attach(FAR struct ioexpander_dev_s *dev,
                       ioe_pinset_t pinset,
                       ioe_callback_t callback, FAR void *arg);
static int bl602_expander_detach(FAR struct ioexpander_dev_s *dev,
                       FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_BL602_EXPANDER_MULTIPLE
/* If only a single device is supported, then the driver state structure may
 * as well be pre-allocated.
 */

static struct bl602_expander_dev_s g_skel;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_bl602_expander_ops =
{
  bl602_expander_direction,
  bl602_expander_option,
  bl602_expander_writepin,
  bl602_expander_readpin,
  bl602_expander_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , bl602_expander_multiwritepin
  , bl602_expander_multireadpin
  , bl602_expander_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , bl602_expander_attach
  , bl602_expander_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_expander_intmask
 *
 * Description:
 *   Set Interrupt Mask for a GPIO Pin. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L143-L169
 *
 ****************************************************************************/

static void bl602_expander_intmask(uint8_t gpio_pin, int intmask)
{
  uint32_t tmp_val;

  if (gpio_pin < GPIO_PIN28)
    {
      tmp_val = getreg32(BL602_GPIO_INT_MASK1);
      if (intmask == 1)
        {
          tmp_val |= (1 << gpio_pin);
        }
      else
        {
          tmp_val &= ~(1 << gpio_pin);
        }

      putreg32(tmp_val, BL602_GPIO_INT_MASK1);
    }
  else
    {
      gpioerr("Invalid pin %d\n", gpio_pin);
      DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: bl602_expander_set_intmod
 *
 * Description:
 *   Set GPIO Interrupt Mode. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L171-L212
 *
 ****************************************************************************/

static void bl602_expander_set_intmod(uint8_t gpio_pin,
              uint8_t int_ctlmod, uint8_t int_trgmod)
{
  gpioinfo("gpio_pin=%d, int_ctlmod=%d, int_trgmod=%d\n", gpio_pin, int_ctlmod, int_trgmod); ////
  uint32_t tmp_val;

  if (gpio_pin < GPIO_PIN10)
    {
      /* GPIO0 ~ GPIO9 */

      tmp_val = gpio_pin;
      modifyreg32(BL602_GPIO_INT_MODE_SET1,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
  else if (gpio_pin < GPIO_PIN20)
    {
      /* GPIO10 ~ GPIO19 */

      tmp_val = gpio_pin - GPIO_PIN10;
      modifyreg32(BL602_GPIO_INT_MODE_SET2,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
  else if (gpio_pin < GPIO_PIN28)
    {
      /* GPIO20 ~ GPIO27 */

      tmp_val = gpio_pin - GPIO_PIN20;
      modifyreg32(BL602_GPIO_INT_MODE_SET3,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
  else
    {
      gpioerr("Invalid pin %d\n", gpio_pin);
      DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: bl602_expander_get_intstatus
 *
 * Description:
 *   Get GPIO Interrupt Status. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L214-L234
 *
 ****************************************************************************/

static int bl602_expander_get_intstatus(uint8_t gpio_pin)
{
  uint32_t tmp_val = 0;

  if (gpio_pin < GPIO_PIN28)
    {
      /* GPIO0 ~ GPIO27 */

      tmp_val = getreg32(BL602_GPIO_INT_STAT1);
    }
  else
    {
      gpioerr("Invalid pin %d\n", gpio_pin);
      DEBUGPANIC();
    }

  return (tmp_val & (1 << gpio_pin)) ? 1 : 0;
}

/****************************************************************************
 * Name: bl602_expander_intclear
 *
 * Description:
 *   Clear GPIO Interrupt. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L236-L254
 *
 ****************************************************************************/

static void bl602_expander_intclear(uint8_t gpio_pin, uint8_t int_clear)
{
  if (gpio_pin < GPIO_PIN28)
    {
      /* GPIO0 ~ GPIO27 */

      modifyreg32(BL602_GPIO_INT_CLR1,
                  int_clear ? 0 : (1 << gpio_pin),
                  int_clear ? (1 << gpio_pin) : 0);
    }
  else
    {
      gpioerr("Invalid pin %d\n", gpio_pin);
      DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: bl602_expander_irq_attach
 *
 * Description:
 *   Attach Interrupt Handler to GPIO Interrupt.
 *   Based on https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L477-L505 
 *
 ****************************************************************************/

static int NOTUSED_bl602_expander_irq_attach(gpio_pinset_t pinset, FAR isr_handler *callback, FAR void *arg)
{
  int ret = 0;
  uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  ////FAR struct bl602_gpint_dev_s *dev = NULL;

  DEBUGASSERT(callback != NULL);

  /* Configure the pin that will be used as interrupt input */

  #warning TODO: bl602_expander_set_intmod
  #warning TODO: Check GLB_GPIO_INT_TRIG_NEG_PULSE  ////  TODO
  bl602_expander_set_intmod(gpio_pin, 1, GLB_GPIO_INT_TRIG_NEG_PULSE);

  #warning TODO: bl602_configgpio
  ret = bl602_configgpio(pinset);
  if (ret < 0)
    {
      gpioerr("Failed to configure GPIO pin %d\n", gpio_pin);
      return ret;
    }

  /* Make sure the interrupt is disabled */

  ////bl602_expander_intmask(gpio_pin, 1);

  ////irq_attach(BL602_IRQ_GPIO_INT0, bl602_expander_interrupt, dev);
  ////bl602_expander_intmask(gpio_pin, 0);

  gpioinfo("Attach %p\n", callback);

  return 0;
}

/****************************************************************************
 * Name: bl602_expander_irq_enable
 *
 * Description:
 *   Enable or disable GPIO Interrupt.
 *   Based on https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L507-L535
 *
 ****************************************************************************/

static int bl602_expander_irq_enable(bool enable)
{
  if (enable)
    {
      gpioinfo("Enable interrupt\n");
      up_enable_irq(BL602_IRQ_GPIO_INT0);
    }
  else
    {
      gpioinfo("Disable interrupt\n");
      up_disable_irq(BL602_IRQ_GPIO_INT0);
    }

  return 0;
}

/****************************************************************************
 * Name: bl602_expander_interrupt
 *
 * Description:
 *   Handle GPIO Interrupt. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L256-L304
 *
 ****************************************************************************/

static int bl602_expander_interrupt(int irq, void *context, void *arg)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)arg;
  uint32_t time_out = 0;
  uint8_t gpio_pin;

  gpioinfo("Interrupt! context=%p, priv=%p\n", context, priv);
  DEBUGASSERT(priv != NULL);

  /* TODO: Check only the GPIO Pins that have registered for interrupts */

  for (gpio_pin = 0; gpio_pin < CONFIG_IOEXPANDER_NPINS; gpio_pin++)
    {
      /* Found the GPIO for the interrupt */

      if (1 == bl602_expander_get_intstatus(gpio_pin))
        {
          FAR struct bl602_expander_callback_s *cb = &priv->cb[gpio_pin];
          ioe_callback_t cbfunc = cb->cbfunc;
          FAR void* cbarg = cb->cbarg;

          /* Attempt to clear the Interrupt Status */

          bl602_expander_intclear(gpio_pin, 1);

          /* Check Interrupt Status with timeout */

          time_out = 32;
          do
            {
              time_out--;
            }
          while ((1 == bl602_expander_get_intstatus(gpio_pin)) && time_out);
          if (!time_out)
            {
              gpiowarn("WARNING: Clear GPIO interrupt status fail.\n");
            }

          /* If time_out==0, Interrupt Status not cleared */

          bl602_expander_intclear(gpio_pin, 0);

          /* NOTE: Callback will run in the context of Interrupt Handler */

          if (cbfunc == NULL)
            {
              gpioinfo("Missing callback for GPIO %d\n", gpio_pin);
            }
          else
            {
              gpioinfo("Call gpio=%d, callback=%p, arg=%p\n", gpio_pin, cbfunc, cbarg);
              cbfunc(&priv->dev, gpio_pin, cbarg);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bl602_expander_lock
 *
 * Description:
 *   Get exclusive access to the I/O Expander
 *
 ****************************************************************************/

static int bl602_expander_lock(FAR struct bl602_expander_dev_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->exclsem);
}

#define bl602_expander_unlock(p) nxsem_post(&(p)->exclsem)

/****************************************************************************
 * Name: bl602_expander_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int bl602_expander_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int direction)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  gpioinfo("pin=%u direction=%s\n",
           pin, (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the pin direction in the I/O Expander */
#warning TODO: bl602_expander_direction

  bl602_expander_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: bl602_expander_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int bl602_expander_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                       int opt, FAR void *value)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret = -ENOSYS;

  ////gpioinfo("addr=%02x pin=%u option=%u\n",  priv->addr, pin, opt);

  DEBUGASSERT(priv != NULL);

  /* Check for pin polarity inversion. */

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      /* Get exclusive access to the I/O Expander */

      ret = bl602_expander_lock(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* Set the pin option */
#warning TODO: bl602_expander_option

      bl602_expander_unlock(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: bl602_expander_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually TRUE will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int bl602_expander_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  gpioinfo("pin=%u value=%u\n", pin, value);

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Write the pin value */
#warning TODO: bl602_expander_writepin

  bl602_expander_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: bl602_expander_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high, except if OPTION_INVERT has been set on
 *            this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int bl602_expander_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  ////gpioinfo("pin=%u\n", priv->addr);

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS &&
              value != NULL);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the pin value */
#warning TODO: bl602_expander_readpin

  /* Return the pin value via the value pointer */
#warning TODO: bl602_expander_readpin

  bl602_expander_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: bl602_expander_readbuf
 *
 * Description:
 *   Read the buffered pin level.
 *   This can be different from the actual pin state. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the level is stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int bl602_expander_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the buffered pin level */
#warning TODO: bl602_expander_readbuf

  bl602_expander_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: bl602_expander_getmultibits
 *
 * Description:
 *  Read multiple bits from I/O Expander registers.
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int bl602_expander_getmultibits(FAR struct bl602_expander_dev_s *priv, FAR uint8_t *pins,
                             FAR bool *values, int count)
{
  ioe_pinset_t pinset;
  int pin;
  int ret = OK;
  int i;

  /* Read the pinset from the IO-Expander hardware */
#warning Missing logic

  /* Read the requested bits */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin >= CONFIG_IOEXPANDER_NPINS)
        {
          return -ENXIO;
        }

      values[i] = (((pinset >> pin) & 1) != 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: bl602_expander_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pins - The list of pin indexes to alter in this call
 *   val - The list of pin levels.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int bl602_expander_multiwritepin(FAR struct ioexpander_dev_s *dev,
                              FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  ioe_pinset_t pinset;
  int pin;
  int ret;
  int i;

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the pinset from the IO-Expander hardware */
#warning Missing logic

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin >= CONFIG_IOEXPANDER_NPINS)
        {
          bl602_expander_unlock(priv);
          return -ENXIO;
        }

      if (values[i])
        {
          pinset |= ((ioe_pinset_t)1 << pin);
        }
      else
        {
          pinset &= ~((ioe_pinset_t)1 << pin);
        }
    }

  /* Now write back the new pins states */
#warning Missing logic

  bl602_expander_unlock(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: bl602_expander_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   valptr - Pointer to a buffer where the pin levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int bl602_expander_multireadpin(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  gpioinfo("count=%u\n", count);

  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = bl602_expander_getmultibits(priv, pins, values, count);
  bl602_expander_unlock(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: bl602_expander_multireadbuf
 *
 * Description:
 *   Read the buffered level of multiple pins. This routine may be faster
 *   than individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the buffered levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int bl602_expander_multireadbuf(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  gpioinfo("count=%u\n", count);

  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = bl602_expander_getmultibits(priv, pins, values, count);
  bl602_expander_unlock(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: bl602_expander_attach
 *
 * Description:
 *   Attach a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   gpio_pin - GPIO Pin for the callback (Only 1 pin supported, not a pinset)
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback. NOTE: Callback will run in the context of
 *              Interrupt Handler.
 *   arg      - Argument that will be provided to the callback function
 *
 * Returned Value:
 *   Callback Handle on success, else NULL
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *bl602_expander_attach(FAR struct ioexpander_dev_s *dev,
                       ioe_pinset_t gpio_pin,
                       ioe_callback_t callback, FAR void *arg)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  FAR struct bl602_expander_callback_s *cb;
  int ret = 0;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(gpio_pin < CONFIG_IOEXPANDER_NPINS);
  cb = &priv->cb[gpio_pin];

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Lock failed\n");
      return NULL;
    }

  if (callback == NULL) /* Detach Callback */
    {
      /* Disable GPIO Interrupt and clear Interrupt Callback */

      gpioinfo("Detach callback for gpio=%d, callback=%p, arg=%p\n",
              cb->pinset, cb->cbfunc, cb->cbarg);
      bl602_expander_intmask(gpio_pin, 1);
      cb->pinset = 0;
      cb->cbfunc = NULL;
      cb->cbarg  = NULL;
      ret = 0;
    }
  else if (cb->cbfunc == NULL) /* Attach Callback */
    {
      /* Set Interrupt Callback and enable GPIO Interrupt */

      gpioinfo("Attach callback for gpio=%d, callback=%p, arg=%p\n", 
               gpio_pin, callback, arg);
      cb->pinset = gpio_pin;
      cb->cbfunc = callback;
      cb->cbarg  = arg;
      bl602_expander_intmask(gpio_pin, 0);
      ret = 0;
    }
  else /* Callback already attached */
    {
      gpioerr("ERROR: GPIO %d already attached\n", gpio_pin);
      ret = -EBUSY;
    }

  /* Unlock the I/O Expander */

  bl602_expander_unlock(priv);
  if (ret == 0)
    {
      return cb;
    }
  else
    {
      return NULL;
    }
}
#endif

/****************************************************************************
 * Name: bl602_expander_detach_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by bl602_expander_attach_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int bl602_expander_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  FAR struct bl602_expander_callback_s *cb =
    (FAR struct bl602_expander_callback_s *)handle;

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&priv->cb[CONFIG_IOEXPANDER_NPINS - 1]);
  UNUSED(priv);
  gpioinfo("Detach callback for gpio=%d, callback=%p, arg=%p\n",
           cb->pinset, cb->cbfunc, cb->cbarg);

  /* Disable the GPIO Interrupt */

  DEBUGASSERT(cb->pinset < CONFIG_IOEXPANDER_NPINS);
  bl602_expander_intmask(cb->pinset, 1);

  /* Clear the Interrupt Callback */

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}
#endif

#ifdef NOTUSED
/****************************************************************************
 * Name: bl602_expander_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void bl602_expander_irqworker(void *arg)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)arg;
  ioe_pinset_t pinset;
  int ret;
  int i;

  /* Read the pinset from the IO-Expander hardware */
#warning Missing logic

  /* Perform pin interrupt callbacks */

  for (i = 0; i < CONFIG_IOEXPANDER_NPINS; i++)
    {
      /* Is this entry valid (i.e., callback attached)?  If so, did andy of
       * the requested pin interrupts occur?
       */

      if (priv->cb[i].cbfunc != NULL)
        {
          /* Did any of the requested pin interrupts occur? */

          ioe_pinset_t match = pinset & priv->cb[i].pinset;
          if (match != 0)
            {
              /* Yes.. perform the callback */

              priv->cb[i].cbfunc(&priv->dev, match, priv->cb[i].cbarg);
            }
        }
    }

  /* Re-enable interrupts */
#warning Missing logic
}
#endif /* NOTUSED */

#ifdef NOTUSED
/****************************************************************************
 * Name: bl602_expander_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 *   NOTE: A more typical prototype for an interrupt handler would be:
 *
 *     int bl602_expander_interrupt(int irq, FAR void *context, FAR void *arg)
 *
 *   However, it is assume that the lower half, board specific interface
 *   can provide intercept the actual interrupt, and call this function with
 *   the arg that can be mapped to the provide driver structure instance.
 *
 *   Presumably the lower level interface provides an attach() method that
 *   provides both the address of bl602_expander_interrupt() as well as the arg value.
 *
 ****************************************************************************/

static void bl602_expander_interrupt(FAR void *arg)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Defer interrupt processing to the worker thread.  This is not only
   * much kinder in the use of system resources but is probably necessary
   * to access the I/O expander device.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in bl602_expander_irqworker() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
      /* Disable interrupts */
#warning Missing logic

      /* Schedule interrupt related work on the high priority worker
       * thread.
       */

      work_queue(HPWORK, &priv->work, bl602_expander_irqworker,
                 (FAR void *)priv, 0);
    }

  return OK;
}
#endif /* NOTUSED */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_expander_initialize
 *
 * Description:
 *   Initialize a I/O Expander device.
 *
 * NOTE: There are no arguments to the initialization function this
 * skeleton example.  Typical implementations take two arguments:
 *
 * 1) A reference to an I2C or SPI interface used to interactive with the
 *    device, and
 * 2) A read-only configuration structure that provides things like:  I2C
 *    or SPI characteristics and callbacks to attach, enable, and disable
 *    interrupts.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *bl602_expander_initialize(void)
{
  int ret;
  uint8_t gpio_pin;
  FAR struct bl602_expander_dev_s *priv;

#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
  /* Allocate the device state structure */

  priv = (FAR struct bl602_expander_dev_s *)kmm_zalloc(sizeof(struct bl602_expander_dev_s));
  if (!priv)
    {
      gpioerr("ERROR: Failed to allocate driver instance\n");
      return NULL;
    }
#else
  /* Use the one-and-only I/O Expander driver instance */

  priv = &g_skel;
#endif

  /* Initialize the device state structure
   * NOTE: Normally you would also save the I2C/SPI device interface and
   * any configuration information here as well.
   */

  priv->dev.ops = &g_bl602_expander_ops;

#ifdef CONFIG_IOEXPANDER_INT_ENABLE

  /* Disable interrupts for all GPIO Pins */

  for (gpio_pin = 0; gpio_pin < CONFIG_IOEXPANDER_NPINS; gpio_pin++)
    {
      bl602_expander_intmask(gpio_pin, 1);
    }

  /* Attach the I/O expander interrupt handler and enable interrupts */

  irq_attach(BL602_IRQ_GPIO_INT0, bl602_expander_interrupt, priv);

  ret = bl602_expander_irq_enable(true);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to enable interrupts\n");
      kmm_free(priv);
      return NULL;
    }

#endif

  nxsem_init(&priv->exclsem, 0, 1);
  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */
