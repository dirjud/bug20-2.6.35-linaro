/*
 * 	bmi_audio.c
 *
 * 	BMI audio device driver for audio PIMs
 *
 * The BMI Audio driver and this API were developed to support
 * audio playback and record on the BUG audio PIMs.
 *
 * The following operating modes are supported:
 *
 *      Operating Mode                PIM
 *      ---------------------------- -------
 *      Stereo DAC Playback           Yes
 *      Stereo ADC Record             Yes
 *      Output Amplifier Control      Yes
 *      Input Mixer Control           Yes
 */

/*
 * 	This code was derived from the following sources:
 *
 * @file	pmic_audio.c
 * @file	mxc-alsa-mixer.c
 * @file	mxc-alsa-pmic.c
 *
 * Copyright 2011 Bug Labs
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * TODO:
 *   - I believe I need to hookup some widget/controls to let admp know when
 *     a jack has been inserted/removed
 */

#define DEBUG
#define AIC3X_CONTROLS	// use custom controls for aic3x codec
#define INPUT		// else disables input-dev

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

// the paths here are a bit odd as typ omap soc drivers are in sound/soc/omap
#include "../../../../sound/soc/omap/omap-mcbsp.h"

// BMI/BUG interfaces
#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_audio.h>

// control interface - LED/RESET/MODULE ACTIVATION
#include <asm/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/major.h>

// I2C interface - IOX
#include <linux/i2c.h>

// Input interface - BUTTONS/JACKS
#include <linux/input.h>
#include <linux/interrupt.h>

/*
 *	Global variables
 */
static ushort fcc_test = 0;

// BMI defines
#define BMIAUDIO_VERSION	"1.0"
#define GPIO_SPARE		0
#define GPIO_RESET		1
#define BMI_NUM_SLOTS		4       // Number of PIMs

// I2C Slave Addresses
#define BMI_IOX_I2C_ADDRESS     0x71    // 7-bit address
#define BMI_CODEC_I2C_ADDRESS   0x18    // 7-bit address

// I2C IOX register definitions
#define IOX_INPUT_REG           0x0     // IOX input data register
#define IOX_OUTPUT_REG          0x1     // IOX output data register
#define IOX_POLARITY_REG        0x2     // IOX polarity data register
#define IOX_CONTROL             0x3     // IOX direction control register
#define IOX_INPUT_MASK          0xfc    // bitmask of all inputs (vs outputs)

// BMI private device structure
struct bmi_audio
{
	struct bmi_device   *bdev;		// BMI device
	struct cdev 	     cdev;		// control character device
	struct device       *class_dev;		// control device class
	struct i2c_client   *iox;		// io-expander
	struct i2c_client   *codec;		// audio codec
	unsigned int         active;		// PIM active
	char                 int_name[32];	// interrupt name for /proc
#ifdef INPUT
	char                 inp_name[32];	// input device name
	struct input_dev    *input_dev;		// button/insertion input device
#endif
	struct work_struct   work;		// workqueue for isr
	unsigned char        iox_data;		// previous state of iox
	struct platform_device *snd_device;	// soc snd device
};

static struct bmi_audio bmi_audio[BMI_NUM_SLOTS];	// PIM structures
static int major;					// control device major

static int activate_slot(struct bmi_device *bdev);
static void deactivate_slot(struct bmi_device *bdev);
static int set_speaker_amp(struct bmi_device *bdev, bool enable);
static struct snd_soc_card snd_soc_bugaudio[BMI_NUM_SLOTS];

//
// I2C devices
//
static struct i2c_board_info iox_info = {
	I2C_BOARD_INFO("AUDIO_IOX", BMI_IOX_I2C_ADDRESS),
};
static struct i2c_board_info codec_info = {
	I2C_BOARD_INFO("tlv320aic3x", BMI_CODEC_I2C_ADDRESS),
};

/* Codec regulators:
 *
 *  The aic3x codec requires the following regulators:
 *    IOVDD
 *    DVDD
 *    AVDD
 *    DRVDD
 *  On the BugAudio PIM these regulators are always on and tied to:
 *    IOVDD - 1.8V
 *    DVDD  - 3.3V
 *    AVDD  - 3.3V
 *    DRVDD - 3.3V
 *
 * The consumers in this case are the aic3x codec which have dev_name's
 * of:
 *  4-0018 - i2c bus 4 (slot0), device 0x18
 *  6-0018 - i2c bus 6 (slot2), device 0x18
 *  7-0018 - i2c bus 7 (slot3), device 0x18
 */
struct regulator_consumer_supply aic3x_consumer_1_8[] = {
	REGULATOR_SUPPLY("IOVDD", "4-0018"),
	REGULATOR_SUPPLY("IOVDD", "6-0018"),
	REGULATOR_SUPPLY("IOVDD", "7-0018"),
};
struct regulator_consumer_supply aic3x_consumer_3_3[] = {
	REGULATOR_SUPPLY("AVDD", "4-0018"),
	REGULATOR_SUPPLY("AVDD", "6-0018"),
	REGULATOR_SUPPLY("AVDD", "7-0018"),
	REGULATOR_SUPPLY("DRVDD", "4-0018"),
	REGULATOR_SUPPLY("DRVDD", "6-0018"),
	REGULATOR_SUPPLY("DRVDD", "7-0018"),
	REGULATOR_SUPPLY("DVDD", "4-0018"),
	REGULATOR_SUPPLY("DVDD", "6-0018"),
	REGULATOR_SUPPLY("DVDD", "7-0018"),
};
struct regulator_init_data fixed_1_8_init_data = {
	.constraints = {
		.name			= "1V8",
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies= aic3x_consumer_1_8,
	.num_consumer_supplies= ARRAY_SIZE(aic3x_consumer_1_8),
};
struct regulator_init_data fixed_3_3_init_data = {
	.constraints = {
		.name			= "3V3",
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies= aic3x_consumer_3_3,
	.num_consumer_supplies= ARRAY_SIZE(aic3x_consumer_3_3),
};
static struct fixed_voltage_config fixed_1_8_pdata = {
	.supply_name   = "board-3V3",
	.microvolts    = 1800000,
	.gpio          = -EINVAL,
	.enabled_at_boot = 1,
	.init_data     = &fixed_1_8_init_data,
};
static struct fixed_voltage_config fixed_3_3_pdata = {
	.supply_name   = "board-1V8",
	.microvolts    = 3300000,
	.gpio          = -EINVAL,
	.enabled_at_boot = 1,
	.init_data     = &fixed_3_3_init_data,
};
/* to keep device-unregister happy */
static void fixed_regulator_release(struct device *dev) {
}
/* expirementally found there were 6 regulators being defined so give these
 * 7, and 8
 * TODO: don't like choosing 7,8 for id's - is it 'ok' to overlap?
 */
static struct platform_device fixed_regulator_devices[] = {
	{
		.name          = "reg-fixed-voltage",
		.id            = 7, 
		.dev = {
			.platform_data = &fixed_1_8_pdata,
			.release = &fixed_regulator_release,
		},
	},
	{
		.name          = "reg-fixed-voltage",
		.id            = 8,
		.dev = {
			.platform_data = &fixed_3_3_pdata,
			.release = &fixed_regulator_release,
		},
	}
};

//
// I2C routines
//

// read byte from I2C IO expander
static int ReadByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char *data)
{
	int     ret = 0;

	ret = i2c_master_send(client, &offset, 1);
	if (ret == 1)
		ret = i2c_master_recv(client, data, 1);
	if (ret < 0)
		printk(KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);
	return ret;
}

// write byte to I2C IO expander
static int WriteByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char data)
{
	int     ret = 0;
	unsigned char msg[2];

	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));

	if (ret < 0)
		printk(KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

	return ret;
}


/** set_speaker_amp - enable/disable speaker amplifier
 * @param bdev - BMI device
 * @param bEnable - 0=disable,1=enable 
 */
static int set_speaker_amp(struct bmi_device *bdev, bool bEnable)
{
	int slot = bdev->slot->slotnum;
	struct bmi_audio *audio = &bmi_audio[slot];
	unsigned char iox_data;

	//printk("%s %s\n", __func__, bEnable?"on":"off");
	if (ReadByte_IOX (audio->iox, IOX_OUTPUT_REG, &iox_data) < 0)
		return -ENODEV;
	if (bEnable)
		iox_data |= GETSTAT_AMP;
	else
		iox_data &= ~GETSTAT_AMP;
	if (WriteByte_IOX(audio->iox, IOX_OUTPUT_REG, iox_data) < 0)
		return -ENODEV;

	return 0;
}


//
// control cdev routines
//

// open
int cntl_open (struct inode *inode, struct file *file)
{	
	struct bmi_audio *audio;

	audio = container_of (inode->i_cdev, struct bmi_audio, cdev);
	file->private_data = audio;
	return 0;

}

// release
int cntl_release (struct inode *inode, struct file *file)
{	
	file->private_data = 0;
	return 0;
}

// ioctl
int cntl_ioctl (struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct bmi_audio *audio = (struct bmi_audio *) (file->private_data);
	unsigned char iox_data;
	int slot;

	// error if audio/bdev not present
	if (audio == 0)
		return -ENODEV;

	if (audio->bdev == 0)
		return -ENODEV;
	
	slot = audio->bdev->slot->slotnum;
	//printk("%s: cmd=%d arg=%p slot%d\n", __func__, cmd, (void*)arg, slot);

#if 0 // dont think we need this anymore - codec is not local
	// get codec transfer structure
	if ((cmd == BMI_AUDIO_WCODEC) || (cmd == BMI_AUDIO_RCODEC)) {
		if (copy_from_user (&codec_xfer, (struct codec_xfer *) arg, sizeof(struct codec_xfer))) {
			printk (KERN_INFO "bmi_audio: ioctl(%d): copy_from_user error\n", slot);
			return -EFAULT;
		}
	}
#endif

	// ioctl's
	switch (cmd) {

		case BMI_AUDIO_RLEDOFF:
			bmi_slot_gpio_set_value (slot, RED_LED, 1);
			break;
	
		case BMI_AUDIO_RLEDON:
			bmi_slot_gpio_set_value (slot, RED_LED, 0);
			break;
	
		case BMI_AUDIO_GLEDOFF:
			bmi_slot_gpio_set_value (slot, GREEN_LED, 1);
			break;
	
		case BMI_AUDIO_GLEDON:
			bmi_slot_gpio_set_value (slot, GREEN_LED, 0);
			break;
	
		case BMI_AUDIO_SPKON:
			set_speaker_amp(audio->bdev, 1);
			break;

		case BMI_AUDIO_SPKOFF:
			set_speaker_amp(audio->bdev, 0);
			break;

		case BMI_AUDIO_SETRST:
			bmi_slot_gpio_direction_out (slot, GPIO_RESET, 0);
			break;
	
		case BMI_AUDIO_CLRRST:
			bmi_slot_gpio_direction_out (slot, GPIO_RESET, 1);
			break;
	
		case BMI_AUDIO_GETSTAT:
			{
			int read_data;
	
			if (ReadByte_IOX (audio->iox, IOX_INPUT_REG, &iox_data) < 0)
				return -ENODEV;
			
			read_data = iox_data | (bmi_slot_gpio_get_all(slot) << 8);
			if (put_user(read_data, (int __user *) arg))
				return -EFAULT;
			}
			break;
	
		case BMI_AUDIO_ACTIVATE:
			if (!audio->active) {
				activate_slot(audio->bdev);
			} else {
				printk(KERN_INFO "bmi_audio: slot%d already active\n", slot);
			}
			break;

		case BMI_AUDIO_DEACTIVATE:
			if (audio->active) {
				deactivate_slot(audio->bdev);
			} else {
				printk(KERN_INFO "bmi_audio: slot%d already inactive\n", slot);
			}
			break;

/* TODO FIXME suspend/resume snd_dev
		case BMI_AUDIO_SUSPEND:
			mxc_alsa_audio_suspend(audio->bdev);
			break;

		case BMI_AUDIO_RESUME:
			mxc_alsa_audio_resume(audio->bdev);
			break;
*/


		default:
			return -ENOTTY;
	}

	return 0;
}

// control file operations
struct file_operations cntl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = cntl_ioctl, 
	.open = cntl_open, 
	.release = cntl_release, 
};

/*
 * Input interrupt handler and support routines
 */

// work handler
void bmi_audio_input_work (struct work_struct *work)
{
	struct bmi_audio *audio = container_of(work, struct bmi_audio, work);
	unsigned char iox_data, changed;
	int irq = audio->bdev->slot->status_irq;
	int slot = audio->bdev->slot->slotnum;

	if (bmi_slot_module_present (slot) == 0) {
		printk(KERN_INFO 
                       "bmi_audio: bmi_audio_input work called with no bdev active (slot %d)\n", 
                       slot);
		goto out;
	}

	// IOX status
	if (ReadByte_IOX (audio->iox, IOX_INPUT_REG, &iox_data) < 0) {
		printk(KERN_ERR "Error reading iox\n");
		goto out;
	}
#ifdef INPUT
	iox_data &= IOX_INPUT_MASK;
	changed = iox_data ^ audio->iox_data;
        if (!changed)
                goto out;

        /* Create individual events for those keys/jacks that have
         * changed
         */
	if (changed & GETSTAT_VOLP)
                input_report_key(audio->input_dev, KEY_VOLUMEUP,
                                 !(iox_data & GETSTAT_VOLP));
	if (changed & GETSTAT_VOLD)
                input_report_key(audio->input_dev, KEY_VOLUMEDOWN,
                                 !(iox_data & GETSTAT_VOLD));
	if (changed & GETSTAT_LI_INS)
                input_report_switch(audio->input_dev, SW_JACK_PHYSICAL_INSERT,
                                    iox_data & GETSTAT_LI_INS);
	if (changed & GETSTAT_MIC_INS)
                input_report_switch(audio->input_dev, SW_MICROPHONE_INSERT,
                                    !(iox_data & GETSTAT_MIC_INS));
	if (changed & GETSTAT_LO_INS)
                input_report_switch(audio->input_dev, SW_LINEOUT_INSERT,
                                    iox_data & GETSTAT_LO_INS);
	if (changed & GETSTAT_HP_INS)
                input_report_switch(audio->input_dev, SW_HEADPHONE_INSERT,
                                    iox_data & GETSTAT_HP_INS);

	input_sync(audio->input_dev);
#endif
	audio->iox_data = iox_data;

out:
	enable_irq (irq);
}

/**
 * module_irq_handler - interrupt service routine
 * @param irq number
 * @param dev_id 
 *
 * We can't acknolwedge from interrupt context since the i2c
 * iox device may sleep, so we disable the interrupt and
 * schedule a delayed work procedure which will handle the
 * work and re-enable interrupts
 */ 
static irqreturn_t module_irq_handler (int irq, void *dev_id)
{
	struct bmi_audio *audio = dev_id;

	disable_irq_nosync (irq);
	schedule_work(&audio->work);

	return IRQ_HANDLED;
}

#ifdef AIC3X_CONTROLS
/**
 * controls:
 *    speaker amp on/off
 *    volume control - see sound/soc/sh/siu_dai.c?
 */
static int bugaudio_spk_func;

/* handler for setting codec pins based on external machine controls
 * (ie spk amp setting)
 */
static void bugaudio_ext_control(struct snd_soc_codec *codec)
{
	if (bugaudio_spk_func)
		snd_soc_dapm_enable_pin(codec, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(codec, "Ext Spk");

	snd_soc_dapm_sync(codec);
}

static int bugaudio_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	snd_pcm_hw_constraint_minmax(runtime,
				     SNDRV_PCM_HW_PARAM_CHANNELS, 2, 2);
	bugaudio_ext_control(codec);

	return 0;
}


static int bugaudio_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = bugaudio_spk_func;

	return 0;
}

static int bugaudio_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (bugaudio_spk_func == ucontrol->value.integer.value[0])
		return 0;

	bugaudio_spk_func = ucontrol->value.integer.value[0];
	bugaudio_ext_control(codec);

	return 1;
}

/** dapm event for speaker
 * enable/disable speaker amp when not needed
 * this is part of the pop/click reduction stuff
 * if spk is on, after audio playback has stopped for a sec or so
 * this will get called with an event to turn off the speaker amp
 */
static int bugaudio_spk_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *k, int event)
{
	int slot;

	for (slot = 0; slot < BMI_NUM_SLOTS; slot++) {
		if (&snd_soc_bugaudio[slot] == w->codec->card)
			break;
	}
	if (&snd_soc_bugaudio[slot] != w->codec->card) {
		printk(KERN_ERR "bmi_audio: unknown slot for widget %s\n",
		       w->name);
		return -ENODEV;
	}
	if (!bmi_audio[slot].bdev)
		return -ENODEV;
	return set_speaker_amp(bmi_audio[slot].bdev, SND_SOC_DAPM_EVENT_ON(event));
}

static const struct snd_soc_dapm_widget aic3x_bugaudio_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", bugaudio_spk_event),
};

/* this maps widgets/controls to a codec line
 * in this case, 'Ext Spk' is mapped to HPLCOM/HPRCOM
 */
static const struct snd_soc_dapm_route bugaudio_audio_map[] = {
	{"Ext Spk", NULL, "HPLCOM"},
	{"Ext Spk", NULL, "HPRCOM"},
};

static const char *spk_function[] = {"Off", "On"};

static const struct soc_enum bugaudio_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_function), spk_function),
};

/* map controls to functions */
static const struct snd_kcontrol_new aic3x_bugaudio_controls[] = {
	/* create a 'Speaker Function' control that can have 'On' or 'Off' state
	* and whos get/set functions are defined
	*/
	SOC_ENUM_EXT("Speaker Function", bugaudio_enum[0],
		     bugaudio_get_spk, bugaudio_set_spk),
};

/** Dynamic Audio Power Management (DAPM)
 * bugaudio_aic3x_init - init machine specific features and controls
 * @codec - the codec instance to attach to.
 *
 * Attach our controls and configure the necessary codec
 * mappings for our sound card instance.
 * 
 * @see Documentation/sound/alsa/dapm.txt
 * @see soc-dapm.h for convenience macros
 * @see sound/soc/omap/rx51.c for good exmaples
 */
static int bugaudio_aic3x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int err;

	/* Set up NC codec pins */
	// cant tell on schematic if this is used or should be NC
	//snd_soc_dapm_nc_pin(codec, "MIC3L");
	
	/* Add bugaudio specific controls */
	err = snd_soc_add_controls(codec, aic3x_bugaudio_controls,
				   ARRAY_SIZE(aic3x_bugaudio_controls));
	if (err < 0)
		return err;
	
	/* Add bugaudio specific widgets (spk amp control) */
	err = snd_soc_dapm_new_controls(codec, aic3x_bugaudio_dapm_widgets,
				   ARRAY_SIZE(aic3x_bugaudio_dapm_widgets));
	if (err < 0)
		return err;

        snd_soc_dapm_enable_pin(codec, "LLOUT");
        snd_soc_dapm_enable_pin(codec, "RLOUT");
        snd_soc_dapm_disable_pin(codec, "MONO_LOUT");
        snd_soc_dapm_enable_pin(codec, "HPLOUT");
        snd_soc_dapm_enable_pin(codec, "HPROUT");
        snd_soc_dapm_enable_pin(codec, "HPLCOM");
        snd_soc_dapm_enable_pin(codec, "HPRCOM");

	/* Set up bugaudio specific audio path audio map */
	snd_soc_dapm_add_routes(codec, bugaudio_audio_map,
				ARRAY_SIZE(bugaudio_audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}
#endif // AIC3X_CONTROLS

/**
 * bugaudio_hw_params - update hardware parameters
 * @substream: The audio substream instance.
 * @params: The parameters requested.
 *
 * Update the codec data routing and configuration settings
 * from the supplied ata.
 */
static int bugaudio_hw_params(struct snd_pcm_substream *substream,
                              struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
        unsigned int div;
	int ret;

        /**
         * The Codec is clocked by the McBSP clock, and the McBSP is
         * unable to pad the data words.  McBSP requests frame data at
         * the beginning of the frame and not at the beginning of each
         * of the data phases.  That means that the McBSP bit clock
         * must be set to match the effective bit clock required by
         * the configured sample rate (i.e. at 44.1 khz with 16 bit
         * sample data per channel, the bit clock must be 44100 * 32 =
         * 1411200 Hz).
         *
         * The DMA transfer mode dictates that both the left and the
         * right channel are sent without any pauses between them.
         * The Codec, in contrast, reads sample data both at the
         * rising and the falling edge, ignoring any bits sent after
         * the configured sample width.
         *
         * Due to hardware restrictions which inhibits the use of the
         * Codec clock to drive the McBSP, it is clocked from
         * CORE_48M_FCLK (48 Mhz).  This results in errors in the
         * resulting sample rate.  For 16 bit sample width, the errors
         * for popular sample rates are as follows:
         *
         * Sample Rate  Error
         * 96000        4,00%
         * 48000        0,80%
         * 44100        0,04%
         * 32000        1,87%
         * 22050        0,04%
         * 16000        0,80%
         * 11025        0,04%
         *  8000        0,27%
         *
         * (For the curious: The McBSP clock lines run through a
         * unidirectional buffer before going to the BMI socket in the
         * BUGbase).
         */

        div = 96000000 / (params_rate(params) * 32);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
                div /= 2;
		break;
	default:
                printk(KERN_ERR "Unsupported sample format %d\n", params_format(params));
		return -EINVAL;
	}

        if (div > 256) {
                /* maximum permitted clock divisor is 256, as
                 * snd_soc_dai_set_clkdiv subtracts one from the value
                 * and the divisor field is 8 bits wide.
                 */
                printk(KERN_ERR "Unsupported sample rate %d Hz (too low)\n", params_rate(params));
                return -EINVAL;
        }

	/* Set codec DAI configuration - bus clock slave */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
        /* The BUGaudio module has a 12 Mhz crystal */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 12000000, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec sysclk\n");
		return ret;
	}

	/* Set the McBSP clock source and freq - McBSP is clocked from CORE_96M_FCLK */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
				     96000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu sysclk\n");
		return ret;
	}

	/* Set the McBSP clock divisor */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, div);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu sysclk\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops bugaudio_ops = {
#ifdef AIC3X_CONTROLS
	.startup = bugaudio_startup,
#endif
	.hw_params = bugaudio_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU 
 *
 * sound/soc/omap/omap-mcbsp.c declares snd_soc_dai omap_mcbsp_dai[5]
 * for OMAP3430 (1 element for each McBSP available)
 *
 * care needs to be taken to make sure each concurant use of McBSP needs
 * to be linked to a unique instance (not to be confused with physical
 * mapping to McBSP).
 *
 * Note that sound/soc/omap3/omap3bug.c (BugBase audio support 'omap3bug'
 * ALSA card) uses link0 so we use link1 for shared slot0/2 and link3 for
 * shared slot1/3
 */
static struct snd_soc_dai_link bugaudio_dai[] = {
	// slot0 - McBSP3
	{
		.name		= "TLV320AIC31",
		.stream_name	= "AIC31",
		.cpu_dai_name	= "omap-mcbsp-dai.2",
		.platform_name	= "omap-pcm-audio",
		.codec_dai_name	= "tlv320aic3x-hifi",
		.codec_name	= "tlv320aic3x-codec.4-0018",
#ifdef AIC3X_CONTROLS
		.init		= &bugaudio_aic3x_init,
#endif
		.ops		= &bugaudio_ops,
	},
	// slot1 - McBSP4 (NB: this is a dummy, it cant exist)
	{
		.name		= "TLV320AIC31",
		.stream_name	= "AIC31",
		.cpu_dai_name	= "omap-mcbsp-dai.3",
		.platform_name	= "omap-pcm-audio",
		.codec_dai_name	= "tlv320aic3x-hifi",
		.codec_name	= "tlv320aic3x-codec.5-0018",
#ifdef AIC3X_CONTROLS
		.init		= &bugaudio_aic3x_init,
#endif
		.ops		= &bugaudio_ops,
	},
	// slot2 - McBSP3
	{
		.name		= "TLV320AIC31",
		.stream_name	= "AIC31",
		.cpu_dai_name	= "omap-mcbsp-dai.2",
		.platform_name	= "omap-pcm-audio",
		.codec_dai_name	= "tlv320aic3x-hifi",
		.codec_name	= "tlv320aic3x-codec.6-0018",
#ifdef AIC3X_CONTROLS
		.init		= &bugaudio_aic3x_init,
#endif
		.ops		= &bugaudio_ops,
	},
	// slot3 - McBSP4
	{
		.name		= "TLV320AIC31",
		.stream_name	= "AIC31",
		.cpu_dai_name	= "omap-mcbsp-dai.3",
		.platform_name	= "omap-pcm-audio",
		.codec_dai_name	= "tlv320aic3x-hifi",
		.codec_name	= "tlv320aic3x-codec.7-0018",
#ifdef AIC3X_CONTROLS
		.init		= &bugaudio_aic3x_init,
#endif
		.ops		= &bugaudio_ops,
	},
};

/* Audio card (machine driver)
 * TODO: the name field is used as the 'long_name' of the ALSA device
 *       and the short name gets auto-generated by filtering out anything
 *       other than [a-zA-Z0-9].  Would like to figure out how to set
 *       short name to same as long name (w/o '_' filtered out)
 */
static struct snd_soc_card snd_soc_bugaudio[] = {
	// slot0
	{
		.name = "bmi_audio_dev_m0",
		.owner = THIS_MODULE,
		.dai_link = &bugaudio_dai[0],
		.num_links = 1,
	},
	// slot1
	{
		.name = "bmi_audio_dev_m1",
		.owner = THIS_MODULE,
		.dai_link = &bugaudio_dai[1],
		.num_links = 1,
	},
	// slot2
	{
		.name = "bmi_audio_dev_m2",
		.owner = THIS_MODULE,
		.dai_link = &bugaudio_dai[2],
		.num_links = 1,
	},
	// slot3
	{
		.name = "bmi_audio_dev_m3",
		.owner = THIS_MODULE,
		.dai_link = &bugaudio_dai[3],
		.num_links = 1,
	},
};

/** activate slot
 * - take into account shared resources - do not activate if another 
 *   shared slot is active
 * - register codec on i2c bus
 * - register soc-audio device
 * - set active state
 * @return 0 on success
 */
static int activate_slot(struct bmi_device *bdev) {
	int slot = bdev->slot->slotnum;
	struct bmi_audio *audio = &bmi_audio[slot];
	int rc = 0;

	printk(KERN_INFO "bmi_audio: activate slot%d\n", slot);

	// check for opposite side already active and assign McBSP resource
	// (slot0/slot2 share McBSP3 and slot3 uses McBSP4, slot1 not compat)
	switch (slot) {
		case 0:
			if (bmi_audio[2].active == 1) {
				printk(KERN_INFO "bmi_audio: slot%d activation failed, slot2 is active\n", slot);
				return -EBUSY;
			}
			break;
		case 1: // this should never happen - slot1 is dedicated for
			// video and is physically keyed differently than PIM
			printk(KERN_INFO "bmi_audio: slot%d activation failed - unsupported slot for module\n", slot);
			return -EINVAL;
		case 2:
			if (bmi_audio[0].active == 1) {
				printk(KERN_INFO "bmi_audio: slot%d activation failed, slot0 is active\n", slot);
				return -EBUSY;
			}
			break;
		case 3:
			break;
	}

	/* probe for the codec */
	audio->codec = i2c_new_device(bdev->slot->adap, &codec_info);
	if (!audio->codec) {
		printk(KERN_ERR "Codec not found\n");
		return -ENODEV;
	}

	/* create a new platform device */
	audio->snd_device = platform_device_alloc("soc-audio", slot);
	if (!audio->snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		rc = -ENOMEM;
		goto err1;
	}

	platform_set_drvdata(audio->snd_device, &snd_soc_bugaudio[slot]);

	rc = platform_device_add(audio->snd_device);
	if (rc)
		goto err2;

	audio->active = 1;

	return 0;

err2:
	printk(KERN_ERR "unable to add platform device\n");
	platform_device_put(audio->snd_device);
	audio->snd_device = NULL;
err1:
	i2c_unregister_device(audio->iox);
	audio->iox = NULL;

	return rc;
}

/** deactivate_slot
 * - unregister soc-audio device
 * - unregister codec on i2c bus
 * - clear active state
 */
static void deactivate_slot(struct bmi_device *bdev) {
	int slot = bdev->slot->slotnum;
	struct bmi_audio *audio = &bmi_audio[slot];

	if (audio->active) {
		printk(KERN_INFO "bmi_audio: deactivate slot%d\n", slot);

		// unregister soc-audio device
		platform_device_unregister(audio->snd_device);
		audio->snd_device = NULL;

		// unregister codec instance
		i2c_unregister_device(audio->codec);
		audio->codec = NULL;

		audio->active = 0;
	}
}

/*
 * This function initializes the driver in terms of BMI and
 * CODEC functionality.
 *
 * @return              0 on success, >0 otherwise.
 */
static int bmi_audio_probe (struct bmi_device *bdev)
{
	int rc = 0;
	struct bmi_audio *audio;
	struct i2c_adapter *adap;
	struct class *bmi_class;
	int slot;
	int irq;

	// bmi set-up
	slot = bdev->slot->slotnum;
	adap = bdev->slot->adap;
	audio = &bmi_audio[slot];
	audio->bdev = 0;
	audio->active = 0;

	//printk("\n\n%s: slot%d audio=%p\n", __func__, bdev->slot->slotnum, audio);
	
	// Create 1 minor device
	cdev_init (&audio->cdev, &cntl_fops);
	rc = cdev_add (&audio->cdev, MKDEV(major, slot), 1);
	if (rc)
		return rc;

	// Create class device 
	bmi_class = bmi_get_class();
	audio->class_dev = device_create (bmi_class, NULL, MKDEV(major, slot),
					  audio, "bmi_audio_ctrl_m%i", slot);  
	if (IS_ERR(audio->class_dev)) {                                
		printk (KERN_ERR "Unable to create class_device for bmi_audio_m%i; errno = %ld\n",
		       slot, PTR_ERR(audio->class_dev));             
		rc = -ENODEV;
		goto err1;
	}                                                            

	// bind driver and bmi_device 
	audio->bdev = bdev;

	bmi_device_set_drvdata (bdev, audio);

	// Initialize GPIOs (turn LED's on )
	bmi_slot_gpio_direction_out (slot, RED_LED, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GREEN_LED, 0);	// Green LED=ON
	bmi_slot_gpio_direction_out (slot, GPIO_RESET, 0);	// Assert RST = 0;
	bmi_slot_gpio_direction_in (slot, GPIO_SPARE);			// unused

	mdelay (200);

	// turn LED's off
	bmi_slot_gpio_set_value (slot, RED_LED, 1); // Red LED=OFF 
	bmi_slot_gpio_set_value (slot, GREEN_LED, 1); // Green LED=OFF 

	// release reset on codec 
	bmi_slot_gpio_set_value (slot, GPIO_RESET, 1);	// Reset = 1

	// activate slot (if not conflinciting with shared resources)
	activate_slot(bdev);
	
	// init workqueue for delayed interrupt processing
	INIT_WORK(&audio->work, bmi_audio_input_work);

	// i2c
	audio->iox = i2c_new_device(bdev->slot->adap, &iox_info);
	if (!audio->iox) {
		printk(KERN_ERR "IOX not found\n");
		goto err2;
	}

	// configure IOX output (default spk amp on)
	WriteByte_IOX (audio->iox, IOX_OUTPUT_REG, 0x01);
	// configure IOX input mask
	WriteByte_IOX (audio->iox, IOX_CONTROL, IOX_INPUT_MASK);
	// clear interrupts and obtain initial input state
	ReadByte_IOX (audio->iox, IOX_INPUT_REG, &audio->iox_data);

	printk (KERN_INFO "bmi_audio: slot%d IOX=0x%x\n", slot,
		audio->iox_data);

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (audio->int_name, "bmi_audio_stat_m%d", slot);
	if (request_irq (irq, &module_irq_handler, IRQF_TRIGGER_FALLING,
			 audio->int_name, audio)) {
		printk (KERN_ERR "bmi_audio: Can't allocate irq %d or find audio in slot %d\n", irq, slot); 
		rc = -EBUSY;
		goto err2;
	}

#ifdef INPUT
	// Allocate and Register input device - bmi_audio_status_m[0123]
	sprintf(audio->inp_name, "bmi_audio_status_m%d", slot);
	audio->input_dev = input_allocate_device();
	if (!audio->input_dev) {
		printk (KERN_ERR "bmi_audio: allocation of %s failed\n",
			audio->inp_name);
		rc = -ENOMEM;
		goto err2;
	}

	// set up input device
	audio->input_dev->name = audio->inp_name;
	audio->input_dev->phys = audio->inp_name;

	platform_set_drvdata(audio->input_dev, &bmi_audio[slot]);

        /* reports switches for jacks */
        set_bit(EV_SW, audio->input_dev->evbit);
        set_bit(SW_JACK_PHYSICAL_INSERT, audio->input_dev->swbit);
        set_bit(SW_MICROPHONE_INSERT, audio->input_dev->swbit);
        set_bit(SW_LINEOUT_INSERT, audio->input_dev->swbit);
        set_bit(SW_HEADPHONE_INSERT, audio->input_dev->swbit);

        /* reports keys for volume buttons */
        set_bit(EV_KEY, audio->input_dev->evbit);
        set_bit(KEY_VOLUMEUP, audio->input_dev->keybit);
        set_bit(KEY_VOLUMEDOWN, audio->input_dev->keybit);

	// register input device
	if (input_register_device (audio->input_dev)) {
		printk (KERN_ERR "bmi_audio: failed registering %s\n",
			audio->inp_name);
		rc = -ENODEV;
		goto err3;
	}

        /* Report current input switch state, otherwise applications
         * won't see an event for removing a plug that was plugged in
         * when the driver was initialized
         */
        input_report_switch(audio->input_dev, SW_JACK_PHYSICAL_INSERT,
                            audio->iox_data & GETSTAT_LI_INS);
        input_report_switch(audio->input_dev, SW_MICROPHONE_INSERT,
                            !(audio->iox_data & GETSTAT_MIC_INS));
        input_report_switch(audio->input_dev, SW_LINEOUT_INSERT,
                            audio->iox_data & GETSTAT_LO_INS);
        input_report_switch(audio->input_dev, SW_HEADPHONE_INSERT,
                            audio->iox_data & GETSTAT_HP_INS);

#endif // INPUT

       	// power stablization delay
        mdelay (500);
	return 0;

#ifdef INPUT
err3:
	input_free_device(audio->input_dev);
#endif
err2:
	deactivate_slot(bdev);
	device_destroy (bmi_class, MKDEV(major, slot));
	audio->class_dev = NULL;                               
err1:
	cdev_del (&audio->cdev);
	return -ENODEV;
}

// remove PIM
void bmi_audio_remove(struct bmi_device *bdev)
{
	int slot = bdev->slot->slotnum;
	struct bmi_audio *audio = &bmi_audio[slot];
	int i;
	struct class *bmi_class;
	int irq = bdev->slot->status_irq;

	//printk("\n\n%s slot%d active=%d audio=%p\n", __func__, slot, audio->active, audio);

	deactivate_slot(bdev);

	// release iox device
	i2c_unregister_device(audio->iox);
	audio->iox = NULL;

	// remove input interrupt scheduled work
	free_irq (irq, audio);
	cancel_work_sync(&audio->work);

	// deconfigure GPIO
	for (i = 0; i < 4; i++)
		bmi_slot_gpio_direction_in(slot, i);

	// remove/unregister input device
	input_unregister_device(audio->input_dev);
	input_free_device(audio->input_dev);

	// remove class device
	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	// clean slot structure
	cdev_del (&audio->cdev);
	audio->class_dev = NULL;
	audio->bdev = NULL;
	audio->active = 0;

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);

	return;
}

// BMI device ID table
static struct bmi_device_id bmi_audio_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_AUDIO, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	// terminate list
};
MODULE_DEVICE_TABLE(bmi, bmi_audio_tbl);

int	bmi_audio_resume (struct device *dev);
int	bmi_audio_suspend (struct device *dev);

static struct dev_pm_ops bmi_audio_pm =
{
	.resume = bmi_audio_resume,
	.suspend = bmi_audio_suspend,
};

static struct bmi_driver bmi_audio_driver = {
	.name = "bmi_audio", 
	.id_table = bmi_audio_tbl, 
	.probe = bmi_audio_probe,
	.remove = bmi_audio_remove,
	.pm = &bmi_audio_pm,
#if 1 // old MXC
	.driver = {
		   .name = "pim_ALSA",
		   },
#endif
};

/*
 *	BMI functions
 */

// probe - insert PIM

/*
 *	PM routines
 */

int bmi_audio_resume(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_audio: Resume..\n");
	bmi_slot_uart_enable(bmi_dev->slot->slotnum);
	bmi_slot_spi_enable(bmi_dev->slot->slotnum);
	// i2c?
	return 0;
}

int bmi_audio_suspend(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_audio: Suspend..\n");
	bmi_slot_uart_disable(bmi_dev->slot->slotnum);
	bmi_slot_spi_disable(bmi_dev->slot->slotnum);
	// i2c?

	return 0;
}

/**************************************************************************
 * Module initialization and termination functions.
 *
 * Note that if this code is compiled into the kernel, then the
 * module_init() function will be called within the device_initcall()
 * group.
 **************************************************************************
 */

/*
 * @name Audio Driver Loading/Unloading Functions
 * These non-exported internal functions are used to support the audio
 * device driver initialization and de-initialization operations.
 */

/*
 * @brief This is the audio device driver initialization function.
 *
 * This function is called by the kernel when this device driver is first
 * loaded.
 */

static int __init bmi_audio_init (void)
{
	int rc = 0;
	dev_t dev_id;
	int i;

	printk (KERN_INFO "BMI Audio driver loading...\n");

	// clear bmi devices and active bits
	for (i = 0; i < BMI_NUM_SLOTS; i++)
		memset(&bmi_audio[i], 0, sizeof(bmi_audio[i]));

	// register fixed regulators needed by codec
	for (i = 0; i < ARRAY_SIZE(fixed_regulator_devices); i++) {
		rc = platform_device_register(&fixed_regulator_devices[i]);
		if (rc < 0) {
			printk(KERN_ERR "Failed registering regulator device\n");
			return rc;
		}
	}

	// alloc char driver with 4 minor numbers (1 for each slot)
	rc = alloc_chrdev_region (&dev_id, 0, 4, "BMI AUDIO Driver"); 
	if (rc) {
		printk (KERN_ERR "bmi_audio_init: Can't allocate chrdev region\n"); 
		rc = -ENODEV;
		goto err1;
	}
	major = MAJOR(dev_id);

	// register with BMI
	rc = bmi_register_driver (&bmi_audio_driver); 
	if (rc) {
		printk (KERN_ERR "bmi_audio: Can't register bmi_audio_driver\n");
		goto err2;
	}

	printk (KERN_INFO "bmi_audio: BMI_AUDIO Driver v%s \n", BMIAUDIO_VERSION);

	if (fcc_test)
		printk (KERN_INFO "bmi_audio: FCC Test mode enabled\n");
	return 0;

err2:
	unregister_chrdev_region (dev_id, 4);
err1:
	// unregister regulators
	for (i = 0; i < ARRAY_SIZE(fixed_regulator_devices); i++) {
		platform_device_unregister(&fixed_regulator_devices[i]);
	}
	return rc;
}

/*
 * @brief This is the audio device driver de-initialization function.
 *
 * This function is called by the kernel when this device driver is about
 * to be unloaded.
 */
static void __exit bmi_audio_exit (void)
{
	dev_t dev_id;
	int i;

	printk(KERN_INFO "BMI Audio driver unloading...\n");

	// remove bmi functionality
	bmi_unregister_driver (&bmi_audio_driver);

	// remove control cdev
	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);

	// unregister regulators
	for (i = 0; i < ARRAY_SIZE(fixed_regulator_devices); i++) {
		platform_device_unregister(&fixed_regulator_devices[i]);
	}

	printk(KERN_INFO "BMI Audio driver unloaded.\n");
	return;
}

/*
 * Module entry points and description information.
 */

module_init (bmi_audio_init);
module_exit (bmi_audio_exit);

module_param(fcc_test, ushort, S_IRUGO);
MODULE_PARM_DESC(fcc_test, "FCC Test code enable");

MODULE_DESCRIPTION("BMI driver for ALSA");
MODULE_AUTHOR("Tim Harvey <tim.harvey@buglabs.com>");
MODULE_AUTHOR("EnCADIS Design, Inc. <p.giacomini@encadis.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("bmi_audio_ctrl_m[0123]");
MODULE_SUPPORTED_DEVICE("bmi_audio_status_m[0123]");

