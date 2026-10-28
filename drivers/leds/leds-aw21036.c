// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW21036 36-channel LED controller
 *
 * The AW21036 has a flat register map, unlike the AW200xx family. Each
 * channel has an 8-bit brightness register starting at 0x01; writes are
 * latched by writing 0x00 to the update register (0x49).
 */

#include <linux/delay.h>
#include <linux/bits.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>

#define AW21036_CHANNELS	36
#define AW21036_REG_GCR		0x00
#define AW21036_REG_BRIGHTNESS	0x01
#define AW21036_REG_UPDATE	0x49
#define AW21036_REG_GCCR	0x6e
#define AW21036_REG_ID		0x7f
#define AW21036_CHIP_ID		0x18
#define AW21036_GCR_ENABLE	BIT(0)
#define AW21036_GCR_PWM_8MHZ	BIT(4)

struct aw21036;

struct aw21036_led {
	struct led_classdev cdev;
	struct aw21036 *chip;
	u8 channel;
};

struct aw21036 {
	struct i2c_client *client;
	struct gpio_desc *enable_gpio;
	struct mutex lock;
	struct aw21036_led leds[AW21036_CHANNELS];
	bool channels_used[AW21036_CHANNELS];
	u8 global_current;
};

static int aw21036_brightness_set(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	struct aw21036_led *led = container_of(cdev, struct aw21036_led, cdev);
	struct aw21036 *chip = led->chip;
	int ret;

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(chip->client,
					AW21036_REG_BRIGHTNESS + led->channel,
					brightness);
	if (ret >= 0)
		ret = i2c_smbus_write_byte_data(chip->client,
						AW21036_REG_UPDATE, 0x00);
	mutex_unlock(&chip->lock);

	return ret;
}

static void aw21036_disable(void *data)
{
	struct gpio_desc *enable_gpio = data;

	if (enable_gpio)
		gpiod_set_value_cansleep(enable_gpio, 0);
}

static int aw21036_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw21036 *chip;
	struct device_node *child;
	u32 global_current = 128;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "SMBus byte-data transactions are required\n");

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	mutex_init(&chip->lock);

	ret = device_property_read_u32(dev, "awinic,global-current",
				       &global_current);
	if (ret && ret != -EINVAL && ret != -ENODATA)
		return dev_err_probe(dev, ret, "failed to read global current\n");
	if (global_current > U8_MAX)
		return dev_err_probe(dev, -EINVAL, "global current must be 0..255\n");
	chip->global_current = global_current;

	chip->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->enable_gpio),
				     "failed to get enable GPIO\n");
	ret = devm_add_action_or_reset(dev, aw21036_disable, chip->enable_gpio);
	if (ret)
		return ret;
	if (chip->enable_gpio)
		usleep_range(2000, 2500);

	ret = i2c_smbus_read_byte_data(client, AW21036_REG_ID);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");
	if (ret != AW21036_CHIP_ID)
		return dev_err_probe(dev, -ENODEV, "unexpected chip ID 0x%02x\n", ret);

	ret = i2c_smbus_write_byte_data(client, AW21036_REG_GCR,
					AW21036_GCR_ENABLE | AW21036_GCR_PWM_8MHZ);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable controller\n");
	ret = i2c_smbus_write_byte_data(client, AW21036_REG_GCCR,
					chip->global_current);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to set global current\n");

	for_each_available_child_of_node(dev->of_node, child) {
		struct aw21036_led *led;
		struct led_init_data init_data = {};
		u32 channel;

		ret = of_property_read_u32(child, "reg", &channel);
		if (ret || channel < 1 || channel > AW21036_CHANNELS) {
			dev_err(dev, "%pOF: invalid channel number\n", child);
			ret = -EINVAL;
			goto err_child;
		}
		if (chip->channels_used[channel - 1]) {
			dev_err(dev, "%pOF: channel %u is duplicated\n", child,
				channel);
			ret = -EINVAL;
			goto err_child;
		}

		led = &chip->leds[channel - 1];
		led->chip = chip;
		led->channel = channel - 1;
		led->cdev.max_brightness = U8_MAX;
		led->cdev.brightness_set_blocking = aw21036_brightness_set;
		init_data.fwnode = of_fwnode_handle(child);
		ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
		if (ret) {
			dev_err(dev, "failed to register LED for %pOF: %d\n",
				child, ret);
			goto err_child;
		}
		chip->channels_used[channel - 1] = true;
	}

	i2c_set_clientdata(client, chip);
	return 0;

err_child:
	of_node_put(child);
	return ret;
}

static const struct of_device_id aw21036_of_match[] = {
	{ .compatible = "awinic,aw21036" },
	{}
};
MODULE_DEVICE_TABLE(of, aw21036_of_match);

static struct i2c_driver aw21036_driver = {
	.driver = {
		.name = "aw21036",
		.of_match_table = aw21036_of_match,
	},
	.probe = aw21036_probe,
};
module_i2c_driver(aw21036_driver);

MODULE_DESCRIPTION("Awinic AW21036 LED controller driver");
MODULE_LICENSE("GPL");
