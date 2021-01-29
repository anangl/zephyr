/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_pcal6408a

#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include "gpio_utils.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(pcal6408a, CONFIG_GPIO_LOG_LEVEL);

enum {
	PCAL6408A_REG_INPUT_PORT                = 0x00,
	PCAL6408A_REG_OUTPUT_PORT               = 0x01,
	PCAL6408A_REG_POLARITY_INVERSION        = 0x02,
	PCAL6408A_REG_CONFIGURATION             = 0x03,
	PCAL6408A_REG_OUTPUT_DRIVE_STRENGTH_0   = 0x40,
	PCAL6408A_REG_OUTPUT_DRIVE_STRENGTH_1   = 0x41,
	PCAL6408A_REG_INPUT_LATCH               = 0x42,
	PCAL6408A_REG_PULL_UP_DOWN_ENABLE       = 0x43,
	PCAL6408A_REG_PULL_UP_DOWN_SELECT       = 0x44,
	PCAL6408A_REG_INTERRUPT_MASK            = 0x45,
	PCAL6408A_REG_INTERRUPT_STATUS          = 0x46,
	PCAL6408A_REG_OUTPUT_PORT_CONFIGURATION = 0x4F,
};

struct pcal6408a_pins_cfg {
	uint8_t configured_as_inputs;
	uint8_t outputs_high;
	uint8_t pull_ups_selected;
	uint8_t pulls_enabled;
};

struct pcal6408a_triggers {
	uint8_t masked;
	uint8_t dual_edge;
	uint8_t on_low;
};

struct pcal6408a_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	sys_slist_t callbacks;
	struct k_sem lock;
	struct k_work work;
	const struct device *dev;
	struct gpio_callback nint_gpio_cb;
	struct pcal6408a_pins_cfg pins_cfg;
	struct pcal6408a_triggers triggers;
};

struct pcal6408a_drv_cfg {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;

	const struct device *i2c;
	uint16_t i2c_addr;
	uint8_t init_out_low;
	uint8_t init_out_high;
	const struct device *nint_gpio_dev;
	gpio_pin_t nint_gpio_pin;
	gpio_dt_flags_t nint_gpio_flags;
};

static int pcal6408a_pins_cfg_apply(const struct device *dev,
				    struct pcal6408a_pins_cfg pins_cfg)
{
	const struct pcal6408a_drv_cfg *drv_cfg = dev->config;
	int rc;

	rc = i2c_reg_write_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
				PCAL6408A_REG_PULL_UP_DOWN_SELECT,
				pins_cfg.pull_ups_selected);
	if (rc != 0) {
		LOG_ERR("%s: failed to select pull-up/pull-down resistors: %d",
			dev->name, rc);
		return -EIO;
	}

	rc = i2c_reg_write_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
				PCAL6408A_REG_PULL_UP_DOWN_ENABLE,
				pins_cfg.pulls_enabled);
	if (rc != 0) {
		LOG_ERR("%s: failed to enable pull-up/pull-down resistors: %d",
			dev->name, rc);
		return -EIO;
	}

	rc = i2c_reg_write_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
				PCAL6408A_REG_OUTPUT_PORT,
				pins_cfg.outputs_high);
	if (rc != 0) {
		LOG_ERR("%s: failed to set outputs: %d",
			dev->name, rc);
		return -EIO;
	}

	rc = i2c_reg_write_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
				PCAL6408A_REG_CONFIGURATION,
				pins_cfg.configured_as_inputs);
	if (rc != 0) {
		LOG_ERR("%s: failed to configure pins: %d",
			dev->name, rc);
		return -EIO;
	}

	return 0;
}

static int pcal6408a_pin_configure(const struct device *dev,
				   gpio_pin_t pin,
				   gpio_flags_t flags)
{
	struct pcal6408a_drv_data *drv_data = dev->data;
	struct pcal6408a_pins_cfg pins_cfg;
	int rc;

	/*
	 * This device does not support open-source outputs, and open-drain
	 * outputs can be only configured port-wise. It also does not support
	 * debouncing.
	 */
	if ((flags & GPIO_SINGLE_ENDED) != 0 ||
	    (flags & GPIO_INT_DEBOUNCE) != 0) {
		return -ENOTSUP;
	}

	/*
	 * Drive strenght configuration in this device is incompatible with
	 * the currently available GPIO API flags, hence it is not supported.
	 */
	if ((flags & (GPIO_DS_ALT_LOW | GPIO_DS_ALT_HIGH)) != 0) {
		return -ENOTSUP;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	pins_cfg = drv_data->pins_cfg;

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
		if ((flags & GPIO_PULL_UP) != 0) {
			pins_cfg.pull_ups_selected |=  BIT(pin);
		} else {
			pins_cfg.pull_ups_selected &= ~BIT(pin);
		}

		pins_cfg.pulls_enabled |=  BIT(pin);
	} else {
		pins_cfg.pulls_enabled &= ~BIT(pin);
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			pins_cfg.outputs_high &= ~BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			pins_cfg.outputs_high |=  BIT(pin);
		}

		pins_cfg.configured_as_inputs &= ~BIT(pin);
	} else {
		pins_cfg.configured_as_inputs |=  BIT(pin);
	}

	rc = pcal6408a_pins_cfg_apply(dev, pins_cfg);
	if (rc == 0) {
		drv_data->pins_cfg = pins_cfg;
	}

	k_sem_give(&drv_data->lock);

	return rc;
}

static int pcal6408a_process_input(const struct device *dev,
				   gpio_port_value_t *value)
{
	const struct pcal6408a_drv_cfg *drv_cfg = dev->config;
	struct pcal6408a_drv_data *drv_data = dev->data;
	int rc;
	uint8_t interrupt_sources;
	uint8_t inputs;

	rc = i2c_reg_read_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
			       PCAL6408A_REG_INTERRUPT_STATUS,
			       &interrupt_sources);
	if (rc != 0) {
		LOG_ERR("%s: failed to read interrupt sources: %d",
			dev->name, rc);
		return -EIO;
	}

	/* This read also clears the generated interrupt if any. */
	rc = i2c_reg_read_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
			       PCAL6408A_REG_INPUT_PORT,
			       &inputs);
	if (rc != 0) {
		LOG_ERR("%s: failed to read input port: %d",
			dev->name, rc);
		return -EIO;
	}

	if (value) {
		*value = inputs;
	}

	if (interrupt_sources) {
		uint8_t dual_edge_triggers = drv_data->triggers.dual_edge;
		uint8_t falling_edge_triggers =
			(~dual_edge_triggers & drv_data->triggers.on_low);
		uint8_t fired_triggers = 0;

		/* For dual edge triggers, react to all state changes. */
		fired_triggers |= (interrupt_sources & dual_edge_triggers);
		/* For single edge triggers, fire callbacks only for the pins
		 * that transitioned to their configured target state (0 for
		 * falling edges, 1 otherwise, hence the XOR operation below).
		 */
		fired_triggers |= ((inputs & interrupt_sources)
				   ^ falling_edge_triggers);

		gpio_fire_callbacks(&drv_data->callbacks, dev, fired_triggers);
	}

	return 0;
}

static void pcal6408a_work_handler(struct k_work *work)
{
	struct pcal6408a_drv_data *drv_data = CONTAINER_OF(work,
		struct pcal6408a_drv_data, work);

	k_sem_take(&drv_data->lock, K_FOREVER);

	(void)pcal6408a_process_input(drv_data->dev, NULL);

	k_sem_give(&drv_data->lock);
}

static void pcal6408a_nint_gpio_handler(const struct device *dev,
					struct gpio_callback *gpio_cb,
					uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct pcal6408a_drv_data *drv_data = CONTAINER_OF(gpio_cb,
		struct pcal6408a_drv_data, nint_gpio_cb);

	k_work_submit(&drv_data->work);
}

static int pcal6408a_port_get_raw(const struct device *dev,
				  gpio_port_value_t *value)
{
	struct pcal6408a_drv_data *drv_data = dev->data;
	int rc;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	/* Reading of the input port also clears the generated interrupt,
	 * thus the configured callbacks must be fired also here if needed.
	 */
	rc = pcal6408a_process_input(dev, value);

	k_sem_give(&drv_data->lock);

	return rc;
}

static int pcal6408a_port_set_raw(const struct device *dev,
				  uint8_t mask,
				  uint8_t value,
				  uint8_t toggle)
{
	const struct pcal6408a_drv_cfg *drv_cfg = dev->config;
	struct pcal6408a_drv_data *drv_data = dev->data;
	int rc;
	uint8_t output;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	output  = (drv_data->pins_cfg.outputs_high & ~mask);
	output |= (value & mask);
	output ^= toggle;
	/*
	 * No need to limit `out` to only pins configured as outputs,
	 * as the chip anyway ignores all other bits in the register.
	 */
	rc = i2c_reg_write_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
				PCAL6408A_REG_OUTPUT_PORT, output);
	if (rc == 0) {
		drv_data->pins_cfg.outputs_high = output;
	}

	k_sem_give(&drv_data->lock);

	if (rc != 0) {
		LOG_ERR("%s: failed to write output port: %d", dev->name, rc);
		return -EIO;
	}

	return 0;
}

static int pcal6408a_port_set_masked_raw(const struct device *dev,
					 gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	return pcal6408a_port_set_raw(dev, (uint8_t)mask, (uint8_t)value, 0);
}

static int pcal6408a_port_set_bits_raw(const struct device *dev,
				       gpio_port_pins_t pins)
{
	return pcal6408a_port_set_raw(dev, (uint8_t)pins, (uint8_t)pins, 0);
}

static int pcal6408a_port_clear_bits_raw(const struct device *dev,
					 gpio_port_pins_t pins)
{
	return pcal6408a_port_set_raw(dev, (uint8_t)pins, 0, 0);
}

static int pcal6408a_port_toggle_bits(const struct device *dev,
				      gpio_port_pins_t pins)
{
	return pcal6408a_port_set_raw(dev, 0, 0, (uint8_t)pins);
}

static int pcal6408a_triggers_apply(const struct device *dev,
				    struct pcal6408a_triggers triggers)
{
	const struct pcal6408a_drv_cfg *drv_cfg = dev->config;
	int rc;

	rc = i2c_reg_write_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
				PCAL6408A_REG_INPUT_LATCH,
				~(triggers.masked));
	if (rc != 0) {
		LOG_ERR("%s: failed to configure input latch: %d",
			dev->name, rc);
		return -EIO;
	}

	rc = i2c_reg_write_byte(drv_cfg->i2c, drv_cfg->i2c_addr,
				PCAL6408A_REG_INTERRUPT_MASK,
				triggers.masked);
	if (rc != 0) {
		LOG_ERR("%s: failed to configure interrupt mask: %d",
			dev->name, rc);
		return -EIO;
	}

	return 0;
}

static int pcal6408a_pin_interrupt_configure(const struct device *dev,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	struct pcal6408a_drv_data *drv_data = dev->data;
	struct pcal6408a_triggers triggers;
	int rc;

	/* This device supports only edge-triggered interrupts. */
	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	triggers = drv_data->triggers;

	if (mode == GPIO_INT_MODE_DISABLED) {
		triggers.masked |=  BIT(pin);
	} else {
		triggers.masked &= ~BIT(pin);
	}

	if (trig == GPIO_INT_TRIG_BOTH) {
		triggers.dual_edge |=  BIT(pin);
	} else {
		triggers.dual_edge &= ~BIT(pin);

		if (trig == GPIO_INT_TRIG_LOW) {
			triggers.on_low |=  BIT(pin);
		} else {
			triggers.on_low &= ~BIT(pin);
		}
	}

	rc = pcal6408a_triggers_apply(dev, triggers);
	if (rc == 0) {
		drv_data->triggers = triggers;
	}

	k_sem_give(&drv_data->lock);

	return rc;
}

static int pcal6408a_manage_callback(const struct device *dev,
				     struct gpio_callback *callback,
				     bool set)
{
	struct pcal6408a_drv_data *drv_data = dev->data;

	return gpio_manage_callback(&drv_data->callbacks, callback, set);
}

static int pcal6408a_init(const struct device *dev)
{
	const struct pcal6408a_drv_cfg *drv_cfg = dev->config;
	struct pcal6408a_drv_data *drv_data = dev->data;
	const struct pcal6408a_pins_cfg initial_pins_cfg = {
		.configured_as_inputs = ~(drv_cfg->init_out_low |
					  drv_cfg->init_out_high),
		.outputs_high = drv_cfg->init_out_high,
		.pull_ups_selected = 0,
		.pulls_enabled = 0,
	};
	const struct pcal6408a_triggers initial_triggers = {
		.masked = 0xff,
	};
	int rc;

	if (!device_is_ready(drv_cfg->i2c)) {
		LOG_ERR("%s is not ready", drv_cfg->i2c->name);
		return -ENODEV;
	}

	rc = pcal6408a_pins_cfg_apply(dev, initial_pins_cfg);
	if (rc != 0) {
		return rc;
	}

	drv_data->pins_cfg = initial_pins_cfg;

	rc = pcal6408a_triggers_apply(dev, initial_triggers);
	if (rc != 0) {
		return rc;
	}

	drv_data->triggers = initial_triggers;

	if (drv_cfg->nint_gpio_dev) {
		if (!device_is_ready(drv_cfg->nint_gpio_dev)) {
			LOG_ERR("%s is not ready",
				drv_cfg->nint_gpio_dev->name);
			return -ENODEV;
		}

		rc = gpio_pin_configure(drv_cfg->nint_gpio_dev,
					drv_cfg->nint_gpio_pin,
					GPIO_INPUT | drv_cfg->nint_gpio_flags);
		if (rc != 0) {
			LOG_ERR("%s: failed to configure nINT pin: %d",
				dev->name, rc);
			return -EIO;
		}

		rc = gpio_pin_interrupt_configure(drv_cfg->nint_gpio_dev,
						  drv_cfg->nint_gpio_pin,
						  GPIO_INT_EDGE_TO_ACTIVE);
		if (rc != 0) {
			LOG_ERR("%s: failed to configure nINT interrupt: %d",
				dev->name, rc);
			return -EIO;
		}

		gpio_init_callback(&drv_data->nint_gpio_cb,
				   pcal6408a_nint_gpio_handler,
				   BIT(drv_cfg->nint_gpio_pin));
		rc = gpio_add_callback(drv_cfg->nint_gpio_dev,
				       &drv_data->nint_gpio_cb);
		if (rc != 0) {
			LOG_ERR("%s: failed to add nINT callback: %d",
				dev->name, rc);
			return -EIO;
		}
	}

	/* Device configured, unlock it so that it can be used. */
	k_sem_give(&drv_data->lock);

	return 0;
}

static const struct gpio_driver_api pcal6408a_drv_api = {
	.pin_configure = pcal6408a_pin_configure,
	.port_get_raw = pcal6408a_port_get_raw,
	.port_set_masked_raw = pcal6408a_port_set_masked_raw,
	.port_set_bits_raw = pcal6408a_port_set_bits_raw,
	.port_clear_bits_raw = pcal6408a_port_clear_bits_raw,
	.port_toggle_bits = pcal6408a_port_toggle_bits,
	.pin_interrupt_configure = pcal6408a_pin_interrupt_configure,
	.manage_callback = pcal6408a_manage_callback,
};


#define INIT_NINT_GPIO_FIELDS(idx)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, nint_gpios),		\
	(								\
		.nint_gpio_dev = DEVICE_DT_GET(				\
			DT_GPIO_CTLR(DT_DRV_INST(idx), nint_gpios)),	\
		.nint_gpio_pin = DT_INST_GPIO_PIN(idx, nint_gpios),	\
		.nint_gpio_flags = DT_INST_GPIO_FLAGS(idx, nint_gpios),	\
	), ())

#define GPIO_PCAL6408A_INST(idx)					   \
	static const struct pcal6408a_drv_cfg pcal6408a_cfg##idx = {	   \
		.common = {						   \
			.port_pin_mask =				   \
				GPIO_PORT_PIN_MASK_FROM_DT_INST(idx),	   \
		},							   \
		.i2c = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(idx))),		   \
		.i2c_addr = DT_INST_REG_ADDR(idx),			   \
		.init_out_low = DT_INST_PROP(idx, init_out_low),	   \
		.init_out_high = DT_INST_PROP(idx, init_out_high),	   \
		INIT_NINT_GPIO_FIELDS(idx)				   \
	};								   \
	static struct pcal6408a_drv_data pcal6408a_data##idx = {	   \
		.lock = Z_SEM_INITIALIZER(pcal6408a_data##idx.lock, 1, 1), \
		.work = Z_WORK_INITIALIZER(pcal6408a_work_handler),	   \
		.dev = DEVICE_DT_INST_GET(idx),				   \
	};								   \
	DEVICE_DT_INST_DEFINE(idx, pcal6408a_init,			   \
			      device_pm_control_nop,			   \
			      &pcal6408a_data##idx, &pcal6408a_cfg##idx,   \
			      POST_KERNEL,				   \
			      CONFIG_GPIO_PCAL6408A_INIT_PRIORITY,	   \
			      &pcal6408a_drv_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_PCAL6408A_INST)
