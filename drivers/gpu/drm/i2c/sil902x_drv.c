/*
 * Copyright (C) 2010 Francisco Jerez.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/module.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
//#include <drm/i2c/sil902x.h>

#define SIL9022_I2C_ADDR_SLAVE    0x63

struct sil902x_encoder_params {
int rev;
};

struct sil902x_priv {
	struct sil902x_encoder_params config;
	struct i2c_client *client;

	uint8_t saved_state[0x10];
	//uint8_t saved_slave_state[0x10];
};

#define to_sil902x_priv(x) \
	((struct sil902x_priv *)to_encoder_slave(x)->slave_priv)

#define sil902x_dbg(client, format, ...) do {				\
		if (drm_debug & DRM_UT_KMS)				\
			dev_printk(KERN_DEBUG, &client->dev,		\
				   "%s: " format, __func__, ## __VA_ARGS__); \
	} while (0)
#define sil902x_info(client, format, ...)		\
	dev_info(&client->dev, format, __VA_ARGS__)

#define sil902x_err(client, format, ...)			\
	dev_err(&client->dev, format, __VA_ARGS__)

#define SIL902X_I2C_ADDR_MASTER			0x38
#define SIL902X_I2C_ADDR_SLAVE			0x39

/* HW register definitions */

#define SIL902X_DETECT				0x3D

#  define SIL902X_DETECT_HOTPLUG_STAT		0x05
/*
#define SIL902X_VENDOR_LO			0x0
#define SIL902X_VENDOR_HI			0x1
#define SIL902X_DEVICE_LO			0x2
#define SIL902X_DEVICE_HI			0x3
#define SIL902X_REVISION				0x4
#define SIL902X_FREQ_MIN				0x6
#define SIL902X_FREQ_MAX				0x7
#define SIL902X_CONTROL0				0x8
#  define SIL902X_CONTROL0_POWER_ON		0x01
#  define SIL902X_CONTROL0_EDGE_RISING		0x02
#  define SIL902X_CONTROL0_INPUT_24BIT		0x04
#  define SIL902X_CONTROL0_DUAL_EDGE		0x08
#  define SIL902X_CONTROL0_HSYNC_ON		0x10
#  define SIL902X_CONTROL0_VSYNC_ON		0x20

#  define SIL902X_DETECT_INTR_STAT		0x01

#  define SIL902X_DETECT_RECEIVER_STAT		0x04
#  define SIL902X_DETECT_INTR_MODE_RECEIVER	0x00
#  define SIL902X_DETECT_INTR_MODE_HOTPLUG	0x08
#  define SIL902X_DETECT_OUT_MODE_HIGH		0x00
#  define SIL902X_DETECT_OUT_MODE_INTR		0x10
#  define SIL902X_DETECT_OUT_MODE_RECEIVER	0x20
#  define SIL902X_DETECT_OUT_MODE_HOTPLUG	0x30
#  define SIL902X_DETECT_VSWING_STAT		0x80
#define SIL902X_CONTROL1				0xa
#  define SIL902X_CONTROL1_DESKEW_ENABLE		0x10
#  define SIL902X_CONTROL1_DESKEW_INCR_SHIFT	5
#define SIL902X_GPIO				0xb
#define SIL902X_CONTROL2				0xc
#  define SIL902X_CONTROL2_FILTER_ENABLE		0x01
#  define SIL902X_CONTROL2_FILTER_SETTING_SHIFT	1
#  define SIL902X_CONTROL2_DUALLINK_MASTER	0x40
#  define SIL902X_CONTROL2_SYNC_CONT		0x80
#define SIL902X_DUALLINK				0xd
#  define SIL902X_DUALLINK_ENABLE		0x10
#  define SIL902X_DUALLINK_SKEW_SHIFT		5
#define SIL902X_PLLZONE				0xe
#  define SIL902X_PLLZONE_STAT			0x08
#  define SIL902X_PLLZONE_FORCE_ON		0x10
#  define SIL902X_PLLZONE_FORCE_HIGH		0x20
*/
/* HW access functions */

static uint8_t
sil902x_write(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	uint8_t buf[] = {addr, val};
	int ret;

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0)
		sil902x_err(client, "Error %d writing to subaddress 0x%x\n",
			   ret, addr);
    return ret;
}

static uint8_t
sil902x_read(struct i2c_client *client, uint8_t addr)
{
	uint8_t val;
	int ret;

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, &val, sizeof(val));
	if (ret < 0)
		goto fail;

	return val;

fail:
	sil902x_err(client, "Error %d reading from subaddress 0x%x\n",
		   ret, addr);
	return 0;
}

static void
sil902x_save_state(struct i2c_client *client, uint8_t *state)
{
	int i;

	for (i = 0x8; i <= 0xe; i++)
		state[i] = sil902x_read(client, i);
}

static void
sil902x_restore_state(struct i2c_client *client, uint8_t *state)
{
	int i;

	for (i = 0x8; i <= 0xe; i++)
		sil902x_write(client, i, state[i]);
}

static int hdmi_cap = 0; /* FIXME */

static void
sil902x_set_power_state(struct i2c_client *client, bool on)
{
	if(on){
	/* Turn on DVI or HDMI */
		if (hdmi_cap)
			sil902x_write(client, 0x1A, 0x01 | 4);
		else
			sil902x_write(client, 0x1A, 0x00);
	}
	else {
		/* disable tmds before changing resolution */
		if (hdmi_cap)
			sil902x_write(client, 0x1A, 0x11);
		else
			sil902x_write(client, 0x1A, 0x10);
	}
/*	uint8_t control0 = sil902x_read(client, SIL902X_CONTROL0);

	if (on)
		control0 |= SIL902X_CONTROL0_POWER_ON;
	else
		control0 &= ~SIL902X_CONTROL0_POWER_ON;

	sil902x_write(client, SIL902X_CONTROL0, control0);*/
}
/*
static void
sil902x_init_state(struct i2c_client *client,
		  struct sil902x_encoder_params *config,
		  bool duallink)
{
	sil902x_write(client, SIL902X_CONTROL0,
		     SIL902X_CONTROL0_HSYNC_ON |
		     SIL902X_CONTROL0_VSYNC_ON |
		     (config->input_edge ? SIL902X_CONTROL0_EDGE_RISING : 0) |
		     (config->input_width ? SIL902X_CONTROL0_INPUT_24BIT : 0) |
		     (config->input_dual ? SIL902X_CONTROL0_DUAL_EDGE : 0));

	sil902x_write(client, SIL902X_DETECT,
		     SIL902X_DETECT_INTR_STAT |
		     SIL902X_DETECT_OUT_MODE_RECEIVER);

	sil902x_write(client, SIL902X_CONTROL1,
		     (config->input_skew ? SIL902X_CONTROL1_DESKEW_ENABLE : 0) |
		     (((config->input_skew + 4) & 0x7)
		      << SIL902X_CONTROL1_DESKEW_INCR_SHIFT));

	sil902x_write(client, SIL902X_CONTROL2,
		     SIL902X_CONTROL2_SYNC_CONT |
		     (config->pll_filter ? 0 : SIL902X_CONTROL2_FILTER_ENABLE) |
		     (4 << SIL902X_CONTROL2_FILTER_SETTING_SHIFT));

	sil902x_write(client, SIL902X_PLLZONE, 0);

	if (duallink)
		sil902x_write(client, SIL902X_DUALLINK,
			     SIL902X_DUALLINK_ENABLE |
			     (((config->duallink_skew + 4) & 0x7)
			      << SIL902X_DUALLINK_SKEW_SHIFT));
	else
		sil902x_write(client, SIL902X_DUALLINK, 0);
}*/

/* DRM encoder functions */

static void
sil902x_encoder_set_config(struct drm_encoder *encoder, void *params)
{
	struct sil902x_priv *priv = to_sil902x_priv(encoder);

	priv->config = *(struct sil902x_encoder_params *)params;
}

static void
sil902x_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct sil902x_priv *priv = to_sil902x_priv(encoder);
	bool on = (mode == DRM_MODE_DPMS_ON);
//	bool duallink = (on && encoder->crtc->mode.clock > 165000);

	sil902x_set_power_state(drm_i2c_encoder_get_client(encoder), on);

//	if (priv->duallink_slave)
//		sil902x_set_power_state(priv->duallink_slave, duallink);
}

static void
sil902x_encoder_save(struct drm_encoder *encoder)
{
	struct sil902x_priv *priv = to_sil902x_priv(encoder);

	sil902x_save_state(drm_i2c_encoder_get_client(encoder),
			  priv->saved_state);

//	if (priv->duallink_slave)
//		sil902x_save_state(priv->duallink_slave,
//				  priv->saved_slave_state);
}

static void
sil902x_encoder_restore(struct drm_encoder *encoder)
{
	struct sil902x_priv *priv = to_sil902x_priv(encoder);

	sil902x_restore_state(drm_i2c_encoder_get_client(encoder),
			     priv->saved_state);

//	if (priv->duallink_slave)
//		sil902x_restore_state(priv->duallink_slave,
//				     priv->saved_slave_state);
}

static bool
sil902x_encoder_mode_fixup(struct drm_encoder *encoder,
			  const struct drm_display_mode *mode,
			  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int
sil902x_encoder_mode_valid(struct drm_encoder *encoder,
			  struct drm_display_mode *mode)
{
	struct sil902x_priv *priv = to_sil902x_priv(encoder);

	if (mode->clock < 32000)
		return MODE_CLOCK_LOW;

	if (mode->clock > 330000)
		return MODE_CLOCK_HIGH;
//	if (mode->clock > 330000 ||
//	    (mode->clock > 165000 && !priv->duallink_slave))
//		return MODE_CLOCK_HIGH;

	return MODE_OK;
}
/*
static void
sil902x_encoder_mode_set(struct drm_encoder *encoder,
			struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode)
{
	struct sil902x_priv *priv = to_sil902x_priv(encoder);
	bool duallink = adjusted_mode->clock > 165000;

	sil902x_init_state(drm_i2c_encoder_get_client(encoder),
			  &priv->config, duallink);

//	if (priv->duallink_slave)
//		sil902x_init_state(priv->duallink_slave,
//				  &priv->config, duallink);

	sil902x_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}*/

static void sil902x_encoder_mode_set(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct sil902x_priv *priv = to_sil902x_priv(encoder);
	struct i2c_client *client = drm_i2c_encoder_get_client(encoder);
	u16 data[4];
	u32 refresh;
	u8 *tmp;
	int i;

	/* Power up */
	sil902x_write(client, 0x1E, 0x00);

	dev_dbg(&client->dev, "%s: %dx%d, pixclk %d\n", __func__,
			mode->hdisplay, mode->vdisplay,
			mode->clock * 1000);

	/* set TPI video mode */
	data[0] = mode->clock / 10;
	data[2] = mode->htotal;
	data[3] = mode->vtotal;
	refresh = data[2] * data[3];
	refresh = (mode->clock * 1000) / refresh;
	data[1] = refresh * 100;
	tmp = (u8 *)data;
	for (i = 0; i < 8; i++)
		sil902x_write(client, i, tmp[i]);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	sil902x_write(client, 0x08, 0x70);
	/* Set input format to RGB */
	sil902x_write(client, 0x09, 0x00);
	/* set output format to RGB */
	sil902x_write(client, 0x0A, 0x00);
	/* audio setup */
	sil902x_write(client, 0x25, 0x00);
	sil902x_write(client, 0x26, 0x40);
	sil902x_write(client, 0x27, 0x00);
}

static irqreturn_t sil902x_detect_handler(int irq, void *data)
{
	struct sil902x_priv *priv = data;
	struct i2c_client *client = priv->client;
	int dat;

	dat = sil902x_read(client, 0x3D);
	if (dat & 0x1) {
		/* cable connection changes */
		if (dat & 0x4) {
			printk("plugin\n");
		} else {
			printk("plugout\n");
		}
	}
	sil902x_write(client, 0x3D, dat);

	return IRQ_HANDLED;
}

static enum drm_connector_status
sil902x_encoder_detect(struct drm_encoder *encoder,
		      struct drm_connector *connector)
{
	struct i2c_client *client = drm_i2c_encoder_get_client(encoder);

	if (sil902x_read(client, SIL902X_DETECT) & SIL902X_DETECT_HOTPLUG_STAT)
		return connector_status_connected;
	else
		return connector_status_disconnected;
}

static int
sil902x_encoder_get_modes(struct drm_encoder *encoder,
			 struct drm_connector *connector)
{
	return 0;
}

static int
sil902x_encoder_create_resources(struct drm_encoder *encoder,
				struct drm_connector *connector)
{
	return 0;
}

static int
sil902x_encoder_set_property(struct drm_encoder *encoder,
			    struct drm_connector *connector,
			    struct drm_property *property,
			    uint64_t val)
{
	return 0;
}

static void
sil902x_encoder_destroy(struct drm_encoder *encoder)
{
	struct sil902x_priv *priv = to_sil902x_priv(encoder);

//	if (priv->duallink_slave)
//		i2c_unregister_device(priv->duallink_slave);

	kfree(priv);
	drm_i2c_encoder_destroy(encoder);
}

static struct drm_encoder_slave_funcs sil902x_encoder_funcs = {
	.set_config = sil902x_encoder_set_config,
	.destroy = sil902x_encoder_destroy,
	.dpms = sil902x_encoder_dpms,
	.save = sil902x_encoder_save,
	.restore = sil902x_encoder_restore,
	.mode_fixup = sil902x_encoder_mode_fixup,
	.mode_valid = sil902x_encoder_mode_valid,
	.mode_set = sil902x_encoder_mode_set,
	.detect = sil902x_encoder_detect,
	.get_modes = sil902x_encoder_get_modes,
	.create_resources = sil902x_encoder_create_resources,
	.set_property = sil902x_encoder_set_property,
};

/* I2C driver functions */

static int
sil902x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
		/* Set 902x in hardware TPI mode on and jump out of D3 state */
	int device_id,rev1,rev2;

    if(sil902x_write(client, 0xc7, 0x00) < 0) {
		dev_err(&client->dev, "SIL902x: cound not find device\n");
		return -ENODEV;
	}

		/* read device ID */
	device_id = sil902x_read(client, 0x1b);
	if (device_id != 0xb0) {
		dev_err(&client->dev, "not found. id is 0x%02x instead of 0xb0\n",
				device_id);
		return -ENODEV;
	}

	rev1 = sil902x_read(client, 0x1c);
	rev2 = sil902x_read(client, 0x1d);
/*	if (client->irq) {
		request_threaded_irq(client->irq, NULL, sil902x_detect_handler,
				IRQF_TRIGGER_FALLING,
				"SIL902x_det", priv);
		sil902x_write(client, 0x3c, 0x01);
	}
*/

	sil902x_info(client, "Detected device %s:%x.%x\n",
		     (device_id == 0xb0) ? "sil9022": NULL, rev1, rev2);

	return 0;
}

static int
sil902x_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_client *
sil902x_detect_slave(struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg = {
		.addr = SIL902X_I2C_ADDR_SLAVE,
		.len = 0,
	};
	const struct i2c_board_info info = {
		I2C_BOARD_INFO("sil902x", SIL902X_I2C_ADDR_SLAVE)
	};

	if (i2c_transfer(adap, &msg, 1) != 1) {
		sil902x_dbg(adap, "No dual-link slave found.");
		return NULL;
	}

	return i2c_new_device(adap, &info);
}

static int
sil902x_encoder_init(struct i2c_client *client,
		    struct drm_device *dev,
		    struct drm_encoder_slave *encoder)
{
	struct sil902x_priv *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	encoder->slave_priv = priv;
	encoder->slave_funcs = &sil902x_encoder_funcs;
    priv->client =client;

	if (client->irq) {
		request_threaded_irq(client->irq, NULL, sil902x_detect_handler,
				IRQF_TRIGGER_FALLING,
				"SIL902x_det", priv);
		sil902x_write(client, 0x3c, 0x01);
    }

	return 0;
}

static struct i2c_device_id sil902x_ids[] = {
	{ "sil902x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sil902x_ids);

static struct drm_i2c_encoder_driver sil902x_driver = {
	.i2c_driver = {
		.probe = sil902x_probe,
		.remove = sil902x_remove,
		.driver = {
			.name = "sil902x",
		},
		.id_table = sil902x_ids,
	},
	.encoder_init = sil902x_encoder_init,
};

/* Module initialization */

static int __init
sil902x_init(void)
{
	return drm_i2c_encoder_register(THIS_MODULE, &sil902x_driver);
}

static void __exit
sil902x_exit(void)
{
	drm_i2c_encoder_unregister(&sil902x_driver);
}

MODULE_AUTHOR("Francisco Jerez <currojerez@riseup.net>");
MODULE_DESCRIPTION("Silicon Image sil902x TMDS transmitter driver");
MODULE_LICENSE("GPL and additional rights");

module_init(sil902x_init);
module_exit(sil902x_exit);
