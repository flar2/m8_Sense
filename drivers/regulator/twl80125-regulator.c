/* drivers/regulator/twl80125-regulator.c
 *
 * Copyright (C) 2014 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/i2c.h>
#include <linux/delay.h>

enum {
	VREG_TYPE_BUCK,
	VREG_TYPE_LDO,
	VREG_TYPE_MAX
};

struct vreg_type_lookup_table {
	uint32_t type;
	const char *type_name;
};

struct twl80125_vreg {
	struct device *dev;
	struct list_head reg_list;
	struct regulator_desc rdesc;
	struct regulator_dev *rdev;
	struct regulator_init_data *init_data;
	const char *regulator_name;
	u32 resource_id;
	int regulator_type;
	u32 enable_addr;
	u32 enable_bit;
	u32 base_addr;
	u32 use_count;
	struct mutex mlock;
	u32 inited;
	bool always_on;
	u32 init_microvolt;
};

struct twl80125_regulator {
	struct device *dev;
	struct twl80125_vreg *twl80125_vregs;
	int en_gpio;
	int total_vregs;
	bool is_enable;
};

static struct twl80125_regulator *regulator = NULL;

#define LDO_UV_FORMULA_VMIN	500000
#define LDO_UV_VMIN             600000
#define LDO_UV_STEP              25000
#define LDO_UV_VMAX            3500000
#define BUCK_UV_VMIN            600000
#define BUCK_UV_STEP             12500
#define BUCK_UV_VMAX           2187500

#define TWL80125_ENABLE		0x8

static int use_ioexpander = 0;

static int twl80125_enable(struct twl80125_regulator *reg, bool enable)
{
	int ret = 0;
	if(use_ioexpander == 0){
		gpio_direction_output(reg->en_gpio, 1);
		if (enable) {
			gpio_set_value(reg->en_gpio, 1);
			reg->is_enable = true;
			pr_info("[TWL80125] %s: gpio[%d]=%d\n", __func__, reg->en_gpio, gpio_get_value(reg->en_gpio));
		} else {
			gpio_set_value(reg->en_gpio, 0);
			reg->is_enable = false;
			pr_info("[TWL80125] %s: gpio[%d]=%d\n", __func__, reg->en_gpio, gpio_get_value(reg->en_gpio));
		}
	}
	return ret;
}

static bool twl80125_is_enable(struct twl80125_regulator *reg)
{
	return reg->is_enable;
}

static int twl80125_i2c_write(struct device *dev, u8 reg_addr, u8 data)
{
	int res;
	int orig_state;
	struct i2c_client *client = to_i2c_client(dev);
	struct twl80125_regulator *reg = i2c_get_clientdata(client);
	u8 values[] = {reg_addr, data};

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= values,
		},
	};

	
	orig_state = twl80125_is_enable(reg);
	if (orig_state == false)
		twl80125_enable(reg, true);
	res = i2c_transfer(client->adapter, msg, 1);

	pr_info("[TWL80125] %s, i2c: addr: %x, val: %x, %d\n", __func__, reg_addr, data, res);
	
	
	
	if ((orig_state == false && !(reg_addr == TWL80125_ENABLE && data != 0x00)) ||
	    (reg_addr == TWL80125_ENABLE && data == 0x00)) {
		twl80125_enable(reg, false);
	}
	
	if (res > 0)
		res = 0;

	return res;
}

int twl80125_i2c_ex_write(u8 reg_addr, u8 data)
{
	int res = -EINVAL;
	if (regulator) {
		twl80125_i2c_write(regulator->dev, reg_addr, data);
		res = 0;
	} else {
		pr_err("%s: regulator not init\n", __func__);
	}

	return res;
}
EXPORT_SYMBOL(twl80125_i2c_ex_write);

static int twl80125_i2c_read(struct device *dev, u8 reg_addr, u8 *data)
{
	int res;
	int curr_state;
	struct i2c_client *client = to_i2c_client(dev);
	struct twl80125_regulator *reg = i2c_get_clientdata(client);

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg_addr,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= data,
		},
	};

	
	curr_state = twl80125_is_enable(reg);
	if (curr_state == 0)
		twl80125_enable(reg, true);
	res = i2c_transfer(client->adapter, msg, 2);
	pr_info("[TWL80125] %s, i2c: base: %x, data: %x, %d\n", __func__, reg_addr, *data, res);
	
	if (curr_state == 0)
		twl80125_enable(reg, false);
	
	if (res >= 0)
		res = 0;

	return res;
}
int twl80125_i2c_ex_read(u8 reg_addr, u8 *data)
{
	int res = -EINVAL;
	if (regulator) {
		twl80125_i2c_read(regulator->dev, reg_addr, data);
		res = 0;
	} else {
		pr_err("%s: regulator not init\n", __func__);
	}

	return res;
}
EXPORT_SYMBOL(twl80125_i2c_ex_read);

static int twl80125_check_error(void)
{
	int res = -EINVAL;
	u8 error_code = 0;

	if (regulator) {
		res = twl80125_i2c_read(regulator->dev, 0xA, &error_code);
		pr_info("%s: error code=%d\n", __func__, error_code);
	}
	else
		pr_err("%s: regulator not init\n", __func__);

	return res;
}

static int twl80125_clear_error(void)
{
        int res = -EINVAL;

        if (regulator) {
                res = twl80125_i2c_write(regulator->dev, 0xA, 0);
        }
        else
                pr_err("%s: regulator not init\n", __func__);

        return res;
}

static int twl80125_vreg_is_enabled(struct regulator_dev *rdev)
{
	struct twl80125_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t val = 0;
	int rc = 0;

	if (vreg->inited != 1) {
		pr_info("%s: vreg not inited ready\n", __func__);
		return -1;
	}
	mutex_lock(&vreg->mlock);
	rc = twl80125_i2c_read(dev, vreg->enable_addr, &val);
	mutex_unlock(&vreg->mlock);

	return ((val & (1 << vreg->enable_bit))? 1: 0);
}

static int twl80125_vreg_enable(struct regulator_dev *rdev)
{
	struct twl80125_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t val = 0;
	int rc = 0;

	
	twl80125_check_error();
	twl80125_clear_error();
	mutex_lock(&vreg->mlock);
	rc = twl80125_i2c_read(dev, vreg->enable_addr, &val);
	val |= (1 << vreg->enable_bit);
	rc = twl80125_i2c_write(dev, vreg->enable_addr, val);
	mutex_unlock(&vreg->mlock);

	return rc;
}

static int twl80125_vreg_disable(struct regulator_dev *rdev)
{

	struct twl80125_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t val = 0;
	int rc = 0;

	mutex_lock(&vreg->mlock);
	rc = twl80125_i2c_read(dev, vreg->enable_addr, &val);
	val &= ~(1 << vreg->enable_bit);
	rc = twl80125_i2c_write(dev, vreg->enable_addr, val);
	mutex_unlock(&vreg->mlock);

	return rc;
}

static int twl80125_vreg_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV,
			    unsigned *selector)
{
	struct twl80125_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t uv_step = 0;
	int rc = 0;

	mutex_lock(&vreg->mlock);

	if (vreg->regulator_type == VREG_TYPE_LDO) {
		if (min_uV < LDO_UV_VMIN)
			min_uV = LDO_UV_VMIN;
		if (max_uV < min_uV)
			max_uV = min_uV;
		if (max_uV > LDO_UV_VMAX)
			max_uV = LDO_UV_VMAX;
		uv_step = (max_uV - LDO_UV_FORMULA_VMIN) / LDO_UV_STEP;

		pr_info("%s: type: VREG_TYPE_LDO, uv_step: %d (max:%d)\n", __func__, uv_step, max_uV);
		twl80125_i2c_write(dev, vreg->base_addr, uv_step);
	} else if (vreg->regulator_type == VREG_TYPE_BUCK) {
		if (min_uV < BUCK_UV_VMIN)
                        min_uV = BUCK_UV_VMIN;
                if (max_uV < min_uV)
                        max_uV = min_uV;
                if (max_uV > BUCK_UV_VMAX)
                        max_uV = BUCK_UV_VMAX;

	        uv_step = (max_uV - BUCK_UV_VMIN) / BUCK_UV_STEP;
		pr_info("%s: type: 0, uv_step: %d (max:%d)\n", __func__, uv_step, max_uV);
	        twl80125_i2c_write(dev, vreg->base_addr, uv_step);
	} else
		pr_err("%s: non support vreg type %d\n", __func__, vreg->regulator_type);

	mutex_unlock(&vreg->mlock);
	return rc;
}

static int twl80125_vreg_get_voltage(struct regulator_dev *rdev)
{
	struct twl80125_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t val = 0;
	uint32_t vol = 0;

	if (vreg->inited != 1) {
		pr_info("%s: vreg not inited ready: %s\n", __func__, vreg->regulator_name);
		return -1;
	}

	mutex_lock(&vreg->mlock);
	twl80125_i2c_read(dev, vreg->base_addr, &val);
	if (vreg->regulator_type == VREG_TYPE_LDO)
		vol = val * LDO_UV_STEP + LDO_UV_VMIN;
	else if (vreg->regulator_type == VREG_TYPE_BUCK)
		vol = val * BUCK_UV_STEP + BUCK_UV_VMIN;
	else
		pr_err("%s: non support vreg type %d\n", __func__, vreg->regulator_type);
	mutex_unlock(&vreg->mlock);

	return vol;
}

static struct regulator_ops twl80125_vreg_ops = {
	.enable		= twl80125_vreg_enable,
	.disable	= twl80125_vreg_disable,
	.is_enabled	= twl80125_vreg_is_enabled,
	.set_voltage	= twl80125_vreg_set_voltage,
	.get_voltage	= twl80125_vreg_get_voltage,
};

static struct regulator_ops *vreg_ops[] = {
	[VREG_TYPE_BUCK]		= &twl80125_vreg_ops,
	[VREG_TYPE_LDO]			= &twl80125_vreg_ops,
};

static int twl80125_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct device_node *child = NULL;
	struct twl80125_vreg *twl80125_vreg = NULL;
	struct twl80125_regulator *reg;
	struct regulator_init_data *init_data;
	int en_gpio = 0;
	int num_vregs = 0;
	int vreg_idx = 0;
	int ret = 0;

	if (!dev->of_node) {
		dev_err(dev, "%s: device tree information missing\n", __func__);
		return -ENODEV;
	}

	reg = kzalloc(sizeof(struct twl80125_regulator), GFP_KERNEL);
	if (reg == NULL) {
		dev_err(dev, "%s: could not allocate memory for reg.\n", __func__);
		return -ENOMEM;
	}

	reg->dev = dev;

	ret = of_property_read_u32(node, "twl,use-ioexpander", &use_ioexpander);
	pr_info("use-ioexpander = %d \n", use_ioexpander);
	if(use_ioexpander == 1) {
		ret = of_property_read_u32(node, "twl,enable-ioexp",
		&en_gpio);
		pr_info("enable-ioexp = %d \n", en_gpio);
		reg->en_gpio = en_gpio;
	}
	else {
		en_gpio = of_get_named_gpio(node, "twl,enable-gpio", 0);
		if (gpio_is_valid(en_gpio)) {
			pr_info("%s: Read TWL80125_enable gpio: %d\n", __func__, en_gpio);
			reg->en_gpio = en_gpio;
			gpio_request(reg->en_gpio, "TWL80125_EN_GPIO");
			gpio_direction_output(reg->en_gpio, 1);
			mdelay(5); 
		} else {
			pr_err("%s: Fail to read TWL80125_enable gpio: %d\n", __func__, en_gpio);
			goto fail_free_regulator;
		}
	}
	
	for_each_child_of_node(node, child)
		num_vregs++;
	reg->total_vregs = num_vregs;
	

	reg->twl80125_vregs = kzalloc(sizeof(struct twl80125_vreg) * num_vregs, GFP_KERNEL);
	if (reg->twl80125_vregs == NULL) {
		dev_err(dev, "%s: could not allocate memory for twl80125_vreg\n", __func__);
		return -ENOMEM;
	}

	
	for_each_child_of_node(node, child) {
		twl80125_vreg = &reg->twl80125_vregs[vreg_idx++];
		ret = of_property_read_string(child, "regulator-name",
				&twl80125_vreg->regulator_name);
		if (ret) {
			dev_err(dev, "%s: regulator-name missing in DT node\n", __func__);
			goto fail_free_vreg;
		}

		ret = of_property_read_u32(child, "twl,resource-id",
				&twl80125_vreg->resource_id);
		if (ret) {
			dev_err(dev, "%s: twl,resource-id missing in DT node\n", __func__);
			goto fail_free_vreg;
		}

		ret = of_property_read_u32(child, "twl,regulator-type",
				&twl80125_vreg->regulator_type);
		if (ret) {
			dev_err(dev, "%s: twl,regulator-type missing in DT node\n", __func__);
			goto fail_free_vreg;
		}

		if ((twl80125_vreg->regulator_type < 0)
		    || (twl80125_vreg->regulator_type >= VREG_TYPE_MAX)) {
			dev_err(dev, "%s: invalid regulator type: %d\n", __func__, twl80125_vreg->regulator_type);
			ret = -EINVAL;
			goto fail_free_vreg;
		}
		twl80125_vreg->rdesc.ops = vreg_ops[twl80125_vreg->regulator_type];

		ret = of_property_read_u32(child, "twl,enable-addr",
				&twl80125_vreg->enable_addr);
		if (ret) {
			dev_err(dev, "%s: Fail to get vreg enable address.\n", __func__);
			goto fail_free_vreg;
		}

		ret = of_property_read_u32(child, "twl,enable-bit",
				&twl80125_vreg->enable_bit);
		if (ret) {
			dev_err(dev, "%s: Fail to get vreg enable bit.\n", __func__);
			goto fail_free_vreg;
		}

		ret = of_property_read_u32(child, "twl,base-addr",
				&twl80125_vreg->base_addr);
		if (ret) {
			dev_err(dev, "%s: Fail to get vreg base address.\n", __func__);
			goto fail_free_vreg;
		}

		if (of_property_read_bool(child, "ldo-always-on"))
			twl80125_vreg->always_on = true;
		else
			twl80125_vreg->always_on = false;

		init_data = of_get_regulator_init_data(dev, child);
		if (init_data == NULL) {
			dev_err(dev, "%s: unable to allocate memory\n", __func__);
			ret = -ENOMEM;
			goto fail_free_vreg;
		}

		if (init_data->constraints.name == NULL) {
			dev_err(dev, "%s: regulator name not specified\n", __func__);
			ret = -EINVAL;
			goto fail_free_vreg;
		}


		ret = of_property_read_u32(child, "twl,init-microvolt",
				&twl80125_vreg->init_microvolt);

		if (ret)
			twl80125_vreg->init_microvolt = -1;

		if (twl80125_vreg->regulator_type == VREG_TYPE_LDO)
			twl80125_vreg->rdesc.n_voltages 	= 117;
		else if (twl80125_vreg->regulator_type == VREG_TYPE_BUCK)
			twl80125_vreg->rdesc.n_voltages 	= 128;

		twl80125_vreg->rdesc.name 	= init_data->constraints.name;
		twl80125_vreg->dev		= dev;
		init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE;

		INIT_LIST_HEAD(&twl80125_vreg->reg_list);
		mutex_init(&twl80125_vreg->mlock);
		twl80125_vreg->rdev = regulator_register(&twl80125_vreg->rdesc, dev, init_data, twl80125_vreg, child);
		pr_info("%s: register regulator, name=%s, type=%d.\n", __func__, twl80125_vreg->regulator_name, twl80125_vreg->regulator_type);
		if (IS_ERR(twl80125_vreg->rdev)) {
			ret = PTR_ERR(twl80125_vreg->rdev);
			twl80125_vreg->rdev = NULL;
			pr_err("%s: regulator register failed: %s, ret = %d\n", __func__, twl80125_vreg->rdesc.name, ret);
			goto fail_free_vreg;
		}
		twl80125_vreg->inited = 1;
	}
	i2c_set_clientdata(client, reg);
	regulator = reg;
	ret = twl80125_enable(reg, true);

	vreg_idx = 0;
	for(vreg_idx = 0; vreg_idx < num_vregs; vreg_idx++) {
		twl80125_vreg = &reg->twl80125_vregs[vreg_idx];
		if(twl80125_vreg->init_microvolt != -1) {
			twl80125_vreg_set_voltage(twl80125_vreg->rdev, twl80125_vreg->init_microvolt,
				twl80125_vreg->init_microvolt, NULL);
			pr_info("%s: init vreg voltage: %d.\n", twl80125_vreg->regulator_name, twl80125_vreg->init_microvolt);
		}
	}
	return ret;

fail_free_vreg:
	kfree(reg->twl80125_vregs);

fail_free_regulator:
	kfree(reg);
	return ret;
}

static int twl80125_suspend(struct i2c_client *client, pm_message_t state)
{
	struct twl80125_regulator *reg;
	int total_vreg_num = 0, on_vreg_num = 0;
	int idx = 0;

	reg = i2c_get_clientdata(client);
	total_vreg_num = reg->total_vregs;

	pr_info("%s\n", __func__);
	for (idx = 0; idx < total_vreg_num; idx++) {
		if (reg->twl80125_vregs[idx].always_on)
			on_vreg_num++;
		else
			twl80125_vreg_disable(reg->twl80125_vregs[idx].rdev);
	}

	if (on_vreg_num == 0)
		twl80125_enable(reg, false);

	return 0;
}

static int twl80125_resume(struct i2c_client *client)
{
	struct twl80125_regulator *reg;

	pr_info("%s\n", __func__);
	reg = i2c_get_clientdata(client);
	if (twl80125_is_enable(reg) == false)
		twl80125_enable(reg, true);

	return 0;
}

static int twl80125_remove(struct i2c_client *client)
{
	struct twl80125_regulator *reg;

	reg = i2c_get_clientdata(client);
	kfree(reg->twl80125_vregs);
	kfree(reg);

	return 0;
}

static struct of_device_id twl80125_match_table[] = {
	{.compatible = "htc,twl80125-regulator"},
	{},
};

static const struct i2c_device_id twl80125_id[] = {
	{"twl80125-regulator", 0},
	{},
};

static struct i2c_driver twl80125_driver = {
	.driver = {
		.name		= "twl80125-regulator",
		.owner		= THIS_MODULE,
		.of_match_table	= twl80125_match_table,
	},
	.probe		= twl80125_probe,
	.remove		= twl80125_remove,
	.suspend	= twl80125_suspend,
	.resume		= twl80125_resume,
	.id_table	= twl80125_id,
};

int __init twl80125_regulator_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&twl80125_driver);
	if (ret)
		pr_err("%s: Driver registration failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(twl80125_regulator_init);

static void __exit twl80125_regulator_exit(void)
{
	i2c_del_driver(&twl80125_driver);
}

MODULE_AUTHOR("Jim Hsia <jim_hsia@htc.com>");
MODULE_DESCRIPTION("TWL80125 regulator driver");
MODULE_LICENSE("GPL v2");

module_init(twl80125_regulator_init);
module_exit(twl80125_regulator_exit);


