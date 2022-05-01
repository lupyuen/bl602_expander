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

#ifndef CONFIG_IOEXPANDER_INT_ENABLE
#error GPIO Interrupts must be handled by BL602 GPIO Expander
#endif /* !CONFIG_IOEXPANDER_INT_ENABLE */

#ifndef CONFIG_GPIO_LOWER_HALF
#error GPIO Lower Half is required by BL602 GPIO Expander
#endif /* !CONFIG_GPIO_LOWER_HALF */

#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
#error Multiple BL602 GPIO Expanders are not supported
#endif /* CONFIG_BL602_EXPANDER_MULTIPLE */

#ifdef CONFIG_IOEXPANDER_MULTIPIN
#error Multipin BL602 GPIO Expander is not supported
#endif /* CONFIG_IOEXPANDER_MULTIPIN */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* Callback for a registered pin interrupt */

struct bl602_expander_callback_s
{
  ioe_pinset_t pinset;   /* Set of pins that will trigger the interrupt callback */
  ioe_callback_t cbfunc; /* Callback function */
  FAR void* cbarg;       /* Callback argument */
};
#endif

/* I/O Expander Driver State */

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

/* I/O Expander Operations */

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
 *   https://github.com/apache/incubator-nuttx/blob/master/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L143-L169
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
 *   https://github.com/apache/incubator-nuttx/blob/master/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L171-L212
 *
 ****************************************************************************/

static void bl602_expander_set_intmod(uint8_t gpio_pin,
              uint8_t int_ctlmod, uint8_t int_trgmod)
{
  gpioinfo("gpio_pin=%d, int_ctlmod=%d, int_trgmod=%d\n", gpio_pin, int_ctlmod, int_trgmod);
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
 *   https://github.com/apache/incubator-nuttx/blob/master/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L214-L234
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
 *   https://github.com/apache/incubator-nuttx/blob/master/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L236-L254
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
 * Name: bl602_expander_irq_enable
 *
 * Description:
 *   Enable or disable GPIO Interrupt.
 *   Based on https://github.com/apache/incubator-nuttx/blob/master/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L507-L535
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
 *   https://github.com/apache/incubator-nuttx/blob/master/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L256-L304
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

  gpioinfo("WARNING: Unimplemented direction: pin=%u, direction=%s\n",
           pin, (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");
  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Unlock the I/O Expander */

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

  gpioinfo("pin=%u, option=%u, value=%p\n", pin, opt, value);

  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle each option */

  switch(opt)
    {
      case IOEXPANDER_OPTION_INTCFG: /* Interrupt Trigger */
        {
          switch((uint32_t)value)
            {
              case IOEXPANDER_VAL_RISING: /* Rising Edge */
                {
                  gpioinfo("Rising edge: pin=%u\n", pin);
                  bl602_expander_set_intmod(pin, 1, GLB_GPIO_INT_TRIG_POS_PULSE);
                  break;
                }

              case IOEXPANDER_VAL_FALLING: /* Falling Edge */
                {
                  gpioinfo("Falling edge: pin=%u\n", pin);
                  bl602_expander_set_intmod(pin, 1, GLB_GPIO_INT_TRIG_NEG_PULSE);
                  break;
                }

              case IOEXPANDER_VAL_BOTH: /* Both Edge (Unimplemented) */
                {
                  gpioinfo("WARNING: Unimplemented interrupt both edge: pin=%u\n", pin);
                  break;
                }

              case IOEXPANDER_VAL_DISABLE: /* Disable (Unimplemented) */
                {
                  gpioinfo("WARNING: Unimplemented disable interrupt, use detach instead: pin=%u\n", pin);
                  break;
                }

              default: /* Unsupported Interrupt */
                {
                  gpioerr("ERROR: Unsupported interrupt: %d, pin=%u\n", value, pin);
                  ret = -EINVAL;
                  break;
                }
            }
          break;
        }

      default: /* Unsupported Option */
        {
          gpioerr("ERROR: Unsupported option: %d, pin=%u\n", opt, pin);
          ret = -ENOSYS;
        }
    }

  /* Unlock the I/O Expander */

  bl602_expander_unlock(priv);
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

static int bl602_expander_writepin(FAR struct ioexpander_dev_s *dev,
                                   uint8_t pin,
                                   bool value)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  gpioinfo("pin=%u, value=%u\n", pin, value);

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Write the pin value. Warning: Pin Number passed as BL602 Pinset */

  bl602_gpiowrite(pin << GPIO_PIN_SHIFT, value);

  /* Unlock the I/O Expander */

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

static int bl602_expander_readpin(FAR struct ioexpander_dev_s *dev, 
                                  uint8_t pin,
                                  FAR bool *value)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL && pin < CONFIG_IOEXPANDER_NPINS &&
              value != NULL);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the pin value. Warning: Pin Number passed as BL602 Pinset */

  *value = bl602_gpioread(pin << GPIO_PIN_SHIFT);

  /* Unlock the I/O Expander */

  bl602_expander_unlock(priv);
  gpioinfo("pin=%u, value=%u\n", pin, *value);
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

static int bl602_expander_readbuf(FAR struct ioexpander_dev_s *dev, 
                                  uint8_t pin,
                                  FAR bool *value)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  int ret;

  gpioerr("ERROR: Not supported\n");
  DEBUGPANIC();

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* TODO: Read the buffered pin level */

  /* Unlock the I/O Expander */

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
  #error Not implemented
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
  #error Not implemented
  return 0;
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
 *   pinset   - The set of pin events that will generate the callback
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
                       ioe_pinset_t pinset,
                       ioe_callback_t callback, FAR void *arg)
{
  FAR struct bl602_expander_dev_s *priv = (FAR struct bl602_expander_dev_s *)dev;
  FAR struct bl602_expander_callback_s *cb = NULL;
  int ret = 0;

  gpioinfo("pinset=0x%lx, callback=%p, arg=%p\n", pinset, callback, arg);
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the I/O Expander */

  ret = bl602_expander_lock(priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Lock failed\n");
      return NULL;
    }

  /* Handle each GPIO Pin in the pinset */

  for (uint8_t gpio_pin = 0; gpio_pin < CONFIG_IOEXPANDER_NPINS; gpio_pin++)
    {
      /* If GPIO Pin is set in the pinset... */

      if (pinset & ((ioe_pinset_t)1 << gpio_pin))
        {
          cb = &priv->cb[gpio_pin];

          if (callback == NULL) /* Detach Callback */
            {
              /* Disable GPIO Interrupt and clear Interrupt Callback */

              gpioinfo("Detach callback for gpio=%lu, callback=%p, arg=%p\n",
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

          /* Only 1 GPIO Pin allowed */

          DEBUGASSERT(pinset == ((ioe_pinset_t)1 << gpio_pin));
          break;
        }
    }

  /* Unlock the I/O Expander and return the handle */

  bl602_expander_unlock(priv);
  return (ret == 0) ? cb : NULL;
}
#endif

/****************************************************************************
 * Name: bl602_expander_detach
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
  gpioinfo("Detach callback for gpio=%lu, callback=%p, arg=%p\n",
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_expander_initialize
 *
 * Description:
 *   Initialize a I/O Expander device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *bl602_expander_initialize(
  const gpio_pinset_t *gpio_inputs,
  uint8_t gpio_input_count,
  const gpio_pinset_t *gpio_outputs,
  uint8_t gpio_output_count,
  const gpio_pinset_t *gpio_interrupts,
  uint8_t gpio_interrupt_count,
  const gpio_pinset_t *other_pins,
  uint8_t other_pin_count)
{
  int i;
  int ret;
  uint8_t pin;
  bool gpio_is_used[CONFIG_IOEXPANDER_NPINS];
  FAR struct bl602_expander_dev_s *priv;

  DEBUGASSERT(gpio_input_count + gpio_output_count + gpio_interrupt_count +
    other_pin_count <= CONFIG_IOEXPANDER_NPINS);

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

  /* Initialize the device state structure */

  priv->dev.ops = &g_bl602_expander_ops;
  nxsem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* Disable GPIO interrupts */

  ret = bl602_expander_irq_enable(false);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to disable GPIO interrupts\n");
#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
      kmm_free(priv);
#endif /* CONFIG_BL602_EXPANDER_MULTIPLE */
      return NULL;
    }

  /* Disable interrupts for all GPIO Pins */

  for (pin = 0; pin < CONFIG_IOEXPANDER_NPINS; pin++)
    {
      bl602_expander_intmask(pin, 1);
    }

  /* Attach the I/O expander interrupt handler and enable interrupts */

  irq_attach(BL602_IRQ_GPIO_INT0, bl602_expander_interrupt, priv);

  ret = bl602_expander_irq_enable(true);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to enable GPIO interrupts\n");
#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
      kmm_free(priv);
#endif /* CONFIG_BL602_EXPANDER_MULTIPLE */
      return NULL;
    }
#endif

  /* Mark the GPIOs in use */

  memset(gpio_is_used, 0, sizeof(gpio_is_used));

  /* Configure and register the GPIO Inputs */

  for (i = 0; i < gpio_input_count; i++)
    {
      gpio_pinset_t pinset = gpio_inputs[i];
      uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      DEBUGASSERT(gpio_pin < CONFIG_IOEXPANDER_NPINS);
      if (gpio_is_used[gpio_pin])
        {
          gpioerr("ERROR: GPIO pin %d is already in use\n", gpio_pin);
#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
          kmm_free(priv);
#endif /* CONFIG_BL602_EXPANDER_MULTIPLE */
          return NULL;
        }
      gpio_is_used[gpio_pin] = true;

      ret = bl602_configgpio(pinset);
      DEBUGASSERT(ret == OK);
      gpio_lower_half(&priv->dev, gpio_pin, GPIO_INPUT_PIN, gpio_pin);
    }

  /* Configure and register the GPIO Outputs */

  for (i = 0; i < gpio_output_count; i++)
    {
      gpio_pinset_t pinset = gpio_outputs[i];
      uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      DEBUGASSERT(gpio_pin < CONFIG_IOEXPANDER_NPINS);
      if (gpio_is_used[gpio_pin])
        {
          gpioerr("ERROR: GPIO pin %d is already in use\n", gpio_pin);
#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
          kmm_free(priv);
#endif /* CONFIG_BL602_EXPANDER_MULTIPLE */
          return NULL;
        }
      gpio_is_used[gpio_pin] = true;

      ret = bl602_configgpio(pinset);
      DEBUGASSERT(ret == OK);
      gpio_lower_half(&priv->dev, gpio_pin, GPIO_OUTPUT_PIN, gpio_pin);
    }

  /* Configure and register the GPIO Interrupts */

  for (i = 0; i < gpio_interrupt_count; i++)
    {
      gpio_pinset_t pinset = gpio_interrupts[i];
      uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      DEBUGASSERT(gpio_pin < CONFIG_IOEXPANDER_NPINS);
      if (gpio_is_used[gpio_pin])
        {
          gpioerr("ERROR: GPIO pin %d is already in use\n", gpio_pin);
#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
          kmm_free(priv);
#endif /* CONFIG_BL602_EXPANDER_MULTIPLE */
          return NULL;
        }
      gpio_is_used[gpio_pin] = true;

      ret = bl602_configgpio(pinset);
      DEBUGASSERT(ret == OK);
      gpio_lower_half(&priv->dev, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);
    }

  /* Validate the other pins (I2C, SPI, etc) */

  for (i = 0; i < other_pin_count; i++)
    {
      gpio_pinset_t pinset = other_pins[i];
      uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      DEBUGASSERT(gpio_pin < CONFIG_IOEXPANDER_NPINS);
      if (gpio_is_used[gpio_pin])
        {
          gpioerr("ERROR: GPIO pin %d is already in use\n", gpio_pin);
#ifdef CONFIG_BL602_EXPANDER_MULTIPLE
          kmm_free(priv);
#endif /* CONFIG_BL602_EXPANDER_MULTIPLE */
          return NULL;
        }
      gpio_is_used[gpio_pin] = true;
    }

  /* TODO: Validate the Pin Functions (e.g. MISO vs MOSI) */

  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */
