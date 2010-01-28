/*
 *
 * Glue audio driver for the TI OMAP730 & TI TWL3016 (Syren) CODEC
 *          (based on omap1610-tsc2101.c)
 * Jean Pihet <j-pihet@ti.com>
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */
/*
 *
 *   Copyright (C) 2007, 2008 E28 SH Limited,
 *
 *   Revision History:
 *  		      Modification    
 *	    Date             Description of Changes
 *	------------      -------------------------
 *	June 25,2007       E28 for SmartCore Platform
 *  March 23, 2008     Change for VRIO power saving.
 * Jul 28, 2008        Bypass EAC when EAC's clock is shut down
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/input.h>

#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/arch/io.h>
#include <asm/hardware.h>
#include <asm/arch/clock.h>

#include "omap850-audio.h"
#include <asm/irq.h>
#include <linux/miscdevice.h>
#include "syren_spi.h"
//#include "omap730_syren.h"

#define CONFIG_CEE

//undef DEBUG
#define DEBUG

void syren_clocks_on(void);

#ifdef DEBUG
#define DPRINTK( x... )  printk(KERN_WARNING x)
#define FN_IN printk("%s start\n", __FUNCTION__)
#define FN_OUT(n) printk("%s end(%d)\n", __FUNCTION__, n)

#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(n)
#endif /* DEBUG */

#define AUDIO_NAME		"OMAP730_SYREN"

#define AUDIO_RATE_DEFAULT	44100

#define REC_MASK ( SOUND_MASK_LINE | SOUND_MASK_MIC )
#define DEV_MASK ( REC_MASK | SOUND_MASK_PCM )

#define SET_LINE   1
#define SET_PCM    2

#define DEFAULT_VOLUME       	100 //100 // 100 0x7f 81	// 0 dB gain
#define DEFAULT_INPUT_VOLUME 	0	// mute inputs

#define OUTPUT_VOLUME_MIN 	0x00
#define OUTPUT_VOLUME_MAX 	0x7F
#define OUTPUT_VOLUME_RANGE 	(OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define OUTPUT_VOLUME_MASK 	OUTPUT_VOLUME_MIN

#define INPUT_VOLUME_MIN 	0x00
#define INPUT_VOLUME_MAX 	0x7F
#define INPUT_VOLUME_RANGE 	(INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK 	INPUT_VOLUME_MAX

#define	DMA_DEFAULT_VOLUME	0xFF		// // Max Vol: 0xFF +12db	Min Vol:0xE7 0dB
#define	MIXER_DEFAULT_VOLUME	0x67	// 0dB gain

#define EARPHONE_JACK_HARDKEY	(252 << 16)
#define SNDCTL_DSP_AUDIOVOICE				0x2242
#define SNDCTL_DSP_VOICEBANDCTL				0x2243
#define SNDCTL_DSP_OUTPUTCTL				0x2244
#define SNDCTL_DSP_SIDETONE				0x2245
#define SNDCTL_DSP_AUDIOCODEC				0x2246
#define SNDCTL_DSP_ECHOCANC				0x2247
#define SNDCTL_DSP_AUDIOPGA				0x2248
#define SNDCTL_DSP_AUDIOBT				0x2249
#define SNDCTL_DSP_DUMP_REG				0x2250
#define SNDCTL_DSP_ENHECHOCANC				0x2251
#define SNDCTL_DSP_AUDIOPROFILE				0x2252
#define SNDCTL_DSP_ABB_READ				0x2260
#define SNDCTL_DSP_ABB_WRITE				0x2261
#define SNDCTL_DSP_BTCTRL				0x2270
#define SNDCTL_DSP_BYPASSCTRL				0x2271
#define SNDCTL_DSP_CHGSWITCH				0x2280
#define SNDCTL_DSP_SYREN_READ				0x2281
#define GPIO_NEW_SETUP					0x1242
#define GPIO_GET_OUTPUT					0x1243
#define GPIO_GET_INPUT					0x1244

typedef struct {
	unsigned long pin;	// pin num	
	unsigned long pin_dir;	// direction
	unsigned long pin_def;	//default
} gpio_para;

// Structure format for the GC AUDIO VOICE BAND CONTROL request
typedef struct
{
    u16    vbctl1;
    u16	   vbctl2;
} audio_voicebandctl;

// Structure format for the GC AUDIO VOICE request
typedef struct
{
    //u8     Uplink;
    u8     Mute;
    //u8     Volume;
} audio_voice;

typedef struct
{
    u8     Uplink;
    u8     Gain;
} audio_pga;

// Structure format for the GC Stereo Codec request
typedef struct
{
    u8     on_off;
    u8     AudioMode;
    u8     Gain;
    u8     SampleFrequency;
} audio_codec;

typedef struct
{
    u16     Flag;
    u16     Para1;
    u16     Para2;
    u16     Para3;
    u16     Para4;
    u16     Para5;
} audio_enhechocanc;

typedef struct
{
	u16	  Page;
	u16	  RegisterAddr;
	u16	  Value;
} abb_register_write;

typedef struct
{
	u16	  Page;
	u16	  RegisterAddr;
} abb_register_read;

/*
  Local prototypes.
*/
static void omap850_audio_init(void *dummy);
static void omap850_audio_shutdown(void *dummy);
//FIXME static int omap850_audio_ioctl(struct inode *inode, struct file *file,
//                                 uint cmd, ulong arg);
static int omap850_getclock(void);
static void omap850_setclock(int on_off);


#ifdef CONFIG_CEE  /* MVL-CEE */
#include <linux/device.h>

static int omap850_audio_suspend(struct device * dev, u32 state, u32 level);
static int omap850_audio_resume(struct device * dev, u32 level);

extern u32 GTI_IOControl(u32 hOpenContext, u32 dwCode, u8* pBufIn, u32 dwLenIn, u8* pBufOut, u32 dwLenOut, u32* pdwActualOut);
static int GSM_WriteAbb(u16 Page,u16 RegisterAddr,u16 Value);
static u16 GSM_ReadAbb(u16 Page,u16 RegisterAddr);
static int GSM_SetupAudio( u8 on_off, u8 AudioMode,u8 Gain, u8 SampleFrequency);
static int GTI_Audio_Echocanc(u16 Algorithm);
static int GTI_Audio_EnhEchocanc(u16 Flag, u16 Para1, u16 Para2, u16 Para3, u16 Para4, u16 Para5);
static int GTI_Output_CTRL(u16 vauoctl);
static int GTI_Voiceband_Ctl(u16 vbctl1, u16 vbctl2);
static int GTI_Audio_Voice(u8 Mute);
static int GTI_Audio_PGA(u8 Uplink, u8 Gain);
static int GTI_Audio_Sidetone(u8 Volume);
static int GTI_Audio_Profileload(u8 Profile);
static int bt_control(int on_off);
static void modem_bypass(int on_off);
static struct device_driver audio_driver_ldm = {
       name:      "omap850-syren",
       probe:     NULL,
       remove:    NULL,
};

static struct device audio_device_ldm = {
       bus_id: "I2S_Audio",
       driver: NULL,
};

static void audio_ldm_driver_register(void)
{
   FN_IN;
   FN_OUT(0);
}

static void audio_ldm_device_register(void)
{
   FN_IN;
   FN_OUT(0);
}

static void audio_ldm_driver_unregister(void)
{
   FN_IN;
   FN_OUT(0);
}

static void audio_ldm_device_unregister(void)
{
   FN_IN;
   FN_OUT(0);
}
#endif /* MVL-CEE */

static u8 syren_codec_onoff;;
static audio_stream_t output_stream = {
		name:		"SYREN out",
};

static audio_stream_t input_stream = {
		name:		"SYREN in",
};

static audio_state_t audio_state = {
	output_stream:	&output_stream,
	input_stream:	&input_stream,
	hw_init:	omap850_audio_init,
	hw_shutdown:	omap850_audio_shutdown,
// FIXME	client_ioctl:	omap850_audio_ioctl,

};

struct {
        u8 volume;
        u8 line;
        u8 mic;
        int mod_cnt;
} eac_local;


static void eac_dump(void)
{
#ifdef DEBUG

	#define DBPRINT(name, addr) \
	printk(KERN_ALERT "%s: %s(@0x%08x), 0x%04x\n", __FUNCTION__, name, addr, omap_readw(addr));\
	udelay(1000);


// FIXME:	DBPRINT("DPLL1_CTL_REG", DPLL1_CTL_REG);
//	DBPRINT("ARM_SYSST", ARM_SYSST);
//	DBPRINT("ARM_CKCTL", ARM_CKCTL);
//	DBPRINT("PCC_CTRL_REG", PCC_CTRL_REG);
//	DBPRINT("ARM_RSTCT2", ARM_RSTCT2);
//	DBPRINT("ARM_IDLECT1", ARM_IDLECT1);
//	DBPRINT("ARM_IDLECT2", ARM_IDLECT2);
//	DBPRINT("ARM_IDLECT3", ARM_IDLECT3);
//	DBPRINT("OMAP_DMA_GCR_REG", OMAP_DMA_GCR);
// FIXME:	DBPRINT("PERSEUS2_MODE_1", PERSEUS2_MODE_1);
//	DBPRINT("ULPD_SOFT_DISABLE_REQ_REG", SOFT_DISABLE_REQ_REG);
//	DBPRINT("ULPD_CAM_CLK_CTRL", CAM_CLK_CTRL);
	DBPRINT("EAC_AGCTR", EAC_AGCTR);
	DBPRINT("EAC_AGCFR", EAC_AGCFR);
	DBPRINT("EAC_AGCFR2", EAC_AGCFR2);
	DBPRINT("EAC_CPTCTL", EAC_CPTCTL);
	DBPRINT("EAC_CPCFR1", EAC_CPCFR1);
	DBPRINT("EAC_CPCFR2", EAC_CPCFR2);
	DBPRINT("EAC_CPCFR3", EAC_CPCFR3);
	DBPRINT("EAC_CPCFR4", EAC_CPCFR4);
	DBPRINT("EAC_AMVCTR", EAC_AMVCTR);
	DBPRINT("EAC_AM1VCTR", EAC_AM1VCTR);
	DBPRINT("EAC_AM2VCTR", EAC_AM2VCTR);
	DBPRINT("EAC_AM3VCTR", EAC_AM3VCTR);
	DBPRINT("EAC_AMSCFR", EAC_AMSCFR);
//	DBPRINT("EAC_MPCTR", EAC_MPCTR);
//	DBPRINT("EAC_MPMCCFR", EAC_MPMCCFR);
//	DBPRINT("EAC_BPCTR", EAC_BPCTR);
//	DBPRINT("EAC_BPMCCFR", EAC_BPMCCFR);
//	DBPRINT("SOFT_REQ_REG", SOFT_REQ_REG);
//	DBPRINT("PCC_PERIPH_CLOCK_SOURCE_SEL", PCC_PERIPH_CLOCK_SOURCE_SEL);
#endif
}


static void omap850_eac_write(__u32 addr, __u16 data) 
{
	DPRINTK("%s: addr 0x%08x, data 0x%04x\n", __FUNCTION__, (int) addr, data);
	omap_writew(data, addr);
}

static u16 omap850_eac_read(__u32 addr) 
{
	__u16 data;
	
	data = omap_readw(addr);
	DPRINTK("%s: addr 0x%08x, data 0x%04x\n", __FUNCTION__, (int) addr, data);

	return data;
}

static void eac_update(void)
{
        u16 volume, line;
	u16 mixer1_vol, mixer2_vol, mixer3_vol;
	u16 reg1_vol, reg2_vol, reg3_vol;
	//int clock_enabled;

	FN_IN;

	// Enable the EAC clock if needed
//	if (!(clock_enabled = omap850_getclock()))
//		omap850_setclock(1);

	// PCM -> Mixer 2B & 3B
	// LINE & MIC -> Mixer 1A & 3A

       	// Convert % to Gain
  	volume = ((eac_local.volume * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;
	line = ((eac_local.line * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;

	// Calc the register value (Mixer1 B & Mixer2 A to 0)
	mixer1_vol = (line << MIXER_x_A_GAIN_OFFSET) & MIXER_x_A_MASK;
	mixer2_vol = (volume << MIXER_x_B_GAIN_OFFSET) & MIXER_x_B_MASK;
	mixer3_vol = (line << MIXER_x_A_GAIN_OFFSET) & MIXER_x_A_MASK;
	mixer3_vol |= (volume << MIXER_x_B_GAIN_OFFSET) & MIXER_x_B_MASK;

	// Use some read/write/read algotithm to prevent strange EAC mixer registers access problem
	reg1_vol = omap850_eac_read(EAC_AM1VCTR);
	reg2_vol = omap850_eac_read(EAC_AM2VCTR);
	reg3_vol = omap850_eac_read(EAC_AM3VCTR);
	// Apply volume to Mixer1
       	omap850_eac_write(EAC_AM1VCTR, mixer1_vol);
	// Apply volume to Mixer2
	omap850_eac_write(EAC_AM2VCTR, mixer2_vol);
	// Apply volume to Mixer3
       	omap850_eac_write(EAC_AM3VCTR, mixer3_vol);
	mdelay(1);
	// Check the registers values
	reg1_vol = omap850_eac_read(EAC_AM1VCTR);
	reg2_vol = omap850_eac_read(EAC_AM2VCTR);
	reg3_vol = omap850_eac_read(EAC_AM3VCTR);

	// Feedback on the actual mixer settings
	if (reg1_vol != mixer1_vol || reg2_vol != mixer2_vol || reg3_vol != mixer3_vol)
	{
		eac_local.volume = (100 * ((reg2_vol & MIXER_x_B_MASK) >> MIXER_x_B_GAIN_OFFSET) - OUTPUT_VOLUME_MIN) / OUTPUT_VOLUME_RANGE;
		eac_local.line = eac_local.mic = (100 * ((reg1_vol & MIXER_x_A_MASK) >> MIXER_x_A_GAIN_OFFSET) - INPUT_VOLUME_MIN) / INPUT_VOLUME_RANGE;
	}

	// Disable the EAC clock if it was disabled when entering the function
	//if (!clock_enabled)
	//	omap850_setclock(0);

	FN_OUT(0);
}
#if 0
static int mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
        int val;
        int gain;
        int ret = 0;
        int nr = _IOC_NR(cmd);

	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	FN_IN;
        DPRINTK("%s: IOC_NR=0x%08x, IOC_DIR=0x%08x\n", __FUNCTION__, _IOC_NR(cmd), _IOC_DIR(cmd));
	if (_IOC_TYPE(cmd) != 'M')
	{ 
		switch (cmd) {
		case SNDCTL_DSP_OUTPUTCTL:
		{
			u16 val_tmp;
			ret = get_user(val_tmp, (int *) arg);
			if (ret)
				return ret;
			
			ret = GTI_Output_CTRL(val_tmp);
			FN_OUT(5);
			return ret;
		}

		case SNDCTL_DSP_ABB_WRITE:
		{
	 		abb_register_write new_ctl;
	
			if (copy_from_user(&new_ctl, (abb_register_write *) arg,sizeof(abb_register_write)))
				return -EFAULT;
			else
			{
				ret = GSM_WriteAbb(new_ctl.Page, new_ctl.RegisterAddr, new_ctl.Value);
			}
			FN_OUT(44);
			return ret;
		}

		case SNDCTL_DSP_ABB_READ:
		{
	 		abb_register_read new_ctl;
	 		u16 value_read;
	
			if (copy_from_user(&new_ctl, (abb_register_read *) arg,sizeof(abb_register_read)))
				return -EFAULT;
			else
			{
				value_read = GSM_ReadAbb(new_ctl.Page, new_ctl.RegisterAddr);
			}
			FN_OUT(44);
			return put_user(value_read, (u16 *)arg);
		}
		
		case SNDCTL_DSP_VOICEBANDCTL:
		{
		 	audio_voicebandctl new_ctl;
		
			if (copy_from_user(&new_ctl, (audio_voicebandctl *) arg,sizeof(audio_voicebandctl)))
				return -EFAULT;
			else
			{
				ret = GTI_Voiceband_Ctl(new_ctl.vbctl1, new_ctl.vbctl2); 
			}
			FN_OUT(6);
			return ret;
		}
		
		case SNDCTL_DSP_AUDIOVOICE:
		{
		 	audio_voice new_ctl;
		
			if (copy_from_user(&new_ctl, (audio_voice *) arg,sizeof(audio_voice)))
				return -EFAULT;
			else
			{
				ret = GTI_Audio_Voice(new_ctl.Mute);
			}
			FN_OUT(6);
			return ret;
		}
		
		case SNDCTL_DSP_AUDIOPGA:
		{
		 	audio_pga new_ctl;
		
			if (copy_from_user(&new_ctl, (audio_pga *) arg,sizeof(audio_pga)))
				return -EFAULT;
			else
			{
				ret = GTI_Audio_PGA(new_ctl.Uplink, new_ctl.Gain);
			}
			FN_OUT(6-2);
			return ret;
		}
			
		case SNDCTL_DSP_SIDETONE:
		{
			u8 val_tmp;
			ret = get_user(val_tmp, (u8 *) arg);
			if (ret)
				return ret;
			
			ret = GTI_Audio_Sidetone(val_tmp);
			FN_OUT(7);
			return ret;
		}	
	
		case SNDCTL_DSP_ECHOCANC:
		{
			u16 val_tmp;
			ret = get_user(val_tmp, (int *) arg);
			if (ret)
				return ret;
			
			ret = GTI_Audio_Echocanc(val_tmp);
			FN_OUT(8);
			return ret;
		}
	
		case SNDCTL_DSP_ENHECHOCANC:
		{
		 	audio_enhechocanc new_ctl;
		
			if (copy_from_user(&new_ctl, (audio_enhechocanc *) arg,sizeof(audio_enhechocanc)))
				return -EFAULT;
			else
			{
				ret = GTI_Audio_EnhEchocanc(new_ctl.Flag, new_ctl.Para1, new_ctl.Para2, new_ctl.Para3, new_ctl.Para4, new_ctl.Para5);
			}
			FN_OUT(85);
			return ret;
		}
			
		case SNDCTL_DSP_AUDIOCODEC:
		{
		 	audio_codec new_ctl;
		
			if (copy_from_user(&new_ctl, (audio_codec *) arg,sizeof(audio_codec)))
				return -EFAULT;
			else
			{
				ret = GSM_SetupAudio( new_ctl.on_off, new_ctl.AudioMode, new_ctl.Gain, new_ctl.SampleFrequency);
			}
			FN_OUT(9);
			return ret;
		}
	
		case SNDCTL_DSP_AUDIOPROFILE:
		{
			u8 val_tmp;
			ret = get_user(val_tmp, (u8 *) arg);
			if (ret)
				return ret;
			
			GTI_Audio_Profileload(val_tmp);
			FN_OUT(9-5);
			return 0;
		}	
	
		case SNDCTL_DSP_BTCTRL:
		{
			u8 val_tmp;
			ret = get_user(val_tmp, (u8 *) arg);
			if (ret)
				return ret;
			
			ret = bt_control(val_tmp);
			FN_OUT(10);
			return ret;
		}
		
		case SNDCTL_DSP_BYPASSCTRL:
		{
			u8 val_tmp;
			ret = get_user(val_tmp, (u8 *) arg);
			if (ret)
				return ret;
			
			modem_bypass(val_tmp);
			FN_OUT(10);
			return 0;
		}
			
		case SNDCTL_DSP_CHGSWITCH:
		{
			u16 val_tmp;
			ret = get_user(val_tmp, (u16 *) arg);
			if (ret)
				return ret;
			
			omap850_eac_write((u16 *) EAC_AMSCFR, val_tmp);
			FN_OUT(10);
			return 0;
		}
	
		case SNDCTL_DSP_AUDIOBT:
		{

			return 0;
		}
		
		default:
			/*
			 * command without audio path business still expect be handled in "dsp" device.
			 */
			return -EINVAL;		
		}
	}		 

	/*
	 * Now we accept mixer (type 'M') ioctls.
	 */

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;
                
		strncpy(mi.id, "SYREN", sizeof(mi.id));
		strncpy(mi.name, "TI SYREN", sizeof(mi.name));
		mi.modify_counter = eac_local.mod_cnt;
		FN_OUT(1);
		return copy_to_user((void *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;

                // Ignore separate left/right channel for now,
                // even the codec does support it.
		gain = val & 255;

		switch (nr) {
		case SOUND_MIXER_PCM:
			eac_local.volume = gain;
			eac_local.mod_cnt++;
			eac_update();
			break;

		case SOUND_MIXER_MIC:
		case SOUND_MIXER_LINE:
			eac_local.line = eac_local.mic = gain;
			eac_local.mod_cnt++;
			eac_update();
			break;

		case SOUND_MIXER_RECSRC:
			break;

		default:
			ret = -EINVAL;
		}
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = 0;

		switch (nr) {
		case SOUND_MIXER_PCM:        val = eac_local.volume;	break;
		case SOUND_MIXER_LINE:       val = eac_local.line;	break;
		case SOUND_MIXER_MIC:        val = eac_local.mic;	break;
		case SOUND_MIXER_RECSRC:     val = REC_MASK;	break;
		case SOUND_MIXER_RECMASK:    val = REC_MASK;	break;
		case SOUND_MIXER_DEVMASK:    val = DEV_MASK;	break;
		case SOUND_MIXER_CAPS:       val = 0;		break;
		case SOUND_MIXER_STEREODEVS: val = 0;		break;
		default:	val = 0;     ret = -EINVAL;	break;
		}

		if (ret == 0)
			ret = put_user(val, (int *)arg);
	}
 out:
	FN_OUT(0);
	return ret;

}

static struct file_operations omap850_mixer_fops = {
	ioctl:		mixer_ioctl,
	owner:		THIS_MODULE
};
#endif /* 0 */

/*
 * Audio interface
 */

static long audio_samplerate = AUDIO_RATE_DEFAULT;

static void omap850_set_samplerate(long val)
{
        int fsint = 0;


	FN_IN;
	
	/* We don't want to mess with clocks when frames are in flight */
        // TODO - could call dma_flush_all, or could poll on
        // enable bit to wait for DMA writes to stop.

	/* wait for any frame to complete */
	udelay(125);

	DPRINTK("%s %d\n", __FUNCTION__, (int) val);

#if 0
        /*
                Note: according to the issues which can't support MT function and
        some critic code will be pushed back from V.1.14 to V.1.12!
                dannywang,050710!
        */
        omap850_setclock(1);
#endif
	fsint = omap850_eac_read(EAC_AGCFR) & ~EAC_AGCFR_FSINT_MASK;

        /*
         * We have the following clock sources:
         * 13.000 MHz
         *
         *  Available sampling rates:
         *  (48kHz,) 44.1kHz, 22 kHz, 11kHz, 8kHz
         */
        //if (val >= 48000)
	//  {
        //        val = 48000;
	//	fsint = 0x0004;
	//  }
        //else 
	if (val >= 44100)
	  {
                val = 44100;
		fsint |= EAC_AGCFR_FSINT_44KHZ;
	  }
        else if (val >= 22050)
	  {
                val = 22050;
		fsint |= EAC_AGCFR_FSINT_22KHZ;
	  }
        else if (val >= 11025)
	  {
                val = 11025;
		fsint |= EAC_AGCFR_FSINT_11KHZ;
	  }
        else
	  {
                val = 8000;
		fsint |= EAC_AGCFR_FSINT_8KHZ;
	  }

	omap850_eac_write(EAC_AGCFR, fsint);
	audio_samplerate = val;
	FN_OUT(0);
}

static int omap850_getclock(void)
{
	return (omap850_eac_read(EAC_AGCTR) & EAC_AGCTR_MCLK_EN);
}

/* Warning: init/deinit operation order is important to keep the codec in sync
	with the I2S frames.
   Init:   Enable Syren codec (GTI_SetupAudio(1))
           Enable EAC Codec Port & Clocks (omap850_setclock(1))
           Enable DMA
   Deinit: Disable DMA
           Disable EAC Codec Port & Clocks (omap850_setclock(0))
	   Disable Syren codec (GTI_SetupAudio(0))
*/
static void omap850_setclock(int on_off)
{
    u16 agctr_temp, soft_temp;
    u16 temp;
    
    agctr_temp = omap850_eac_read(EAC_AGCTR) & ~EAC_AGCTR_RESERVED;
    soft_temp = omap850_eac_read(SOFT_REQ_REG);

    if (on_off)
    {
	// Enable clock & disable low power
	agctr_temp |= EAC_AGCTR_MCLK_EN;
	agctr_temp &= ~EAC_AGCTR_EACPWD;
	soft_temp |= SOFT_REQ_REG_EAC12M_DPLL_REQ;
    	omap850_eac_write(EAC_AGCTR, agctr_temp);
    	mdelay(1);
    	omap850_eac_write(SOFT_REQ_REG, soft_temp);

	// Enable C-Port
	temp = omap850_eac_read(EAC_CPTCTL);
	temp |= EAC_CPTCTL_CPEN;
	omap850_eac_write(EAC_CPTCTL, temp);	
    }
    else 
    {
//FIXME   	modem_bypass(1);  	
	// Disable C-Port
	temp = omap850_eac_read(EAC_CPTCTL);
	temp &= ~EAC_CPTCTL_CPEN;
	omap850_eac_write(EAC_CPTCTL, temp);	

	// Disable clock & enable low power
	agctr_temp &= ~EAC_AGCTR_MCLK_EN;
	agctr_temp |= EAC_AGCTR_EACPWD;
	soft_temp &= ~SOFT_REQ_REG_EAC12M_DPLL_REQ;
    	omap850_eac_write(SOFT_REQ_REG, soft_temp);
    	omap850_eac_write(EAC_AGCTR, agctr_temp);
    	mdelay(1);
    }

}

static void omap850_audio_init(void *dummy)
{
	FN_IN;

        /*  
            Configure the DMA channel and EAC
        */

	// DMA configuration already done
	// Setup the I2S codec through the ARM7.
//	GTI_SetupAudio(1);
	// Enable EAC clocks
#if 1
        /*
                Note: according to the issues which can't support MT function and
        some critic code will be pushed back from V.1.14 to V.1.12!
        */
        omap850_setclock(1);
                                                                                                                             
        // Setup K switches for Phone Call + play/record
        // omap850_eac_write((u16 *) EAC_AMSCFR, EAC_AMSCFR_DEFAULT_SWITCHES);
                                                                                                                             
        omap850_set_samplerate(audio_samplerate);
                                                                                                                             
#endif

	eac_dump();

	FN_OUT(0);
}

static void omap850_audio_shutdown(void *dummy)
{
	u16 temp;

        /* 
           Turn off codec after it is done.  
           Can't do it immediately, since it may still have
           buffered data.

           Wait 20ms (arbitrary value) and then turn it off.
        */
        
	FN_IN;
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(2);

	// Audio Global Control Register 2
	temp = omap850_eac_read(EAC_AGCTR);
	// DMA read and write operation disabled
	temp &= ~(EAC_AGCTR_DMAWEN | EAC_AGCTR_DMAREN);
	omap850_eac_write(EAC_AGCTR, temp);
	
	// Shutdown the I2S codec through the ARM7.
	omap850_setclock(0);
	// Disable EAC clocks
	//GTI_SetupAudio(0);

	FN_OUT(0);
}

#ifdef CONFIG_CEE  /* MVL-CEE */
extern void sleep_mode_ctrl(unsigned char mode);
extern int vrio_mode_ctrl(unsigned char mode);

static int omap850_audio_suspend(struct device *dev, u32 state, u32 level)
{
  extern void audio_ldm_suspend(void *data);

  FN_IN;/* FIXME
  switch(level)
  {
     case SUSPEND_POWER_DOWN: 
*/
       /* Turn off power to omap850_audio */
       /* FIXME
       audio_ldm_suspend(&audio_state);      
       omap850_setclock(0);
       sleep_mode_ctrl(4);
       vrio_mode_ctrl(0);*/
       //GTI_SetupAudio(0);

//        break;
//   }

  FN_OUT(0);
  return 0;
}

static int omap850_audio_resume(struct device *dev, u32 level)
{
  extern void audio_ldm_resume(void *data);

  FN_IN;
/* FIXME:
  switch(level)
  {
     case RESUME_POWER_ON: */

       /* Turn on power to omap850_audio */

       //GTI_SetupAudio(1);
   /*    omap850_setclock(1);
       audio_ldm_resume(&audio_state);
       vrio_mode_ctrl(1);
       break;
  }*/

  FN_OUT(0);
  return 0;
}

#endif

#if 0
static int omap850_audio_ioctl(struct inode *inode, struct file *file,
                                uint cmd, ulong arg)
{
	long val;
	int ret = 0;

	DPRINTK("%s 0x%08x\n", __FUNCTION__, cmd);

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic omap-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *) arg);
		if (ret)
			return ret;
		/* the SYREN is stereo only */
		ret = (val == 0) ? -EINVAL : 1;
		FN_OUT(1);
		return put_user(ret, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
		/* the AIC23 is stereo only */
		FN_OUT(2);
		return put_user(2, (long *) arg);

	case SNDCTL_DSP_SPEED:
		ret = get_user(val, (long *) arg);
		if (ret) break;
		omap850_set_samplerate(val);
		/* fall through */

	case SOUND_PCM_READ_RATE:
		FN_OUT(3);
		return put_user(audio_samplerate, (long *) arg);

	case SNDCTL_DSP_SETFMT:
	case SNDCTL_DSP_GETFMTS:
		/* we can do 16-bit only */
		FN_OUT(4);
		return put_user(AFMT_S16_LE, (long *) arg);
	case SNDCTL_DSP_ABB_WRITE:
	{
	 	abb_register_write new_ctl;
	
		if (copy_from_user(&new_ctl, (abb_register_write *) arg,sizeof(abb_register_write)))
			return -EFAULT;
		else
		{
			ret = GSM_WriteAbb(new_ctl.Page, new_ctl.RegisterAddr, new_ctl.Value);
		}
		FN_OUT(44);
		return ret;
	}

	case SNDCTL_DSP_ABB_READ:
	{
	 	abb_register_read new_ctl;
	 	u16 value_read;
	
		if (copy_from_user(&new_ctl, (abb_register_read *) arg,sizeof(abb_register_read)))
			return -EFAULT;
		else
		{
			value_read = GSM_ReadAbb(new_ctl.Page, new_ctl.RegisterAddr);
		}
		FN_OUT(44);
		return put_user(value_read, (u16 *)arg);
	}
		
	case SNDCTL_DSP_OUTPUTCTL:
	{
		u16 val_tmp;
		ret = get_user(val_tmp, (int *) arg);
		if (ret)
			return ret;
		
		ret = GTI_Output_CTRL(val_tmp);
		FN_OUT(5);
		return ret;
	}

	case SNDCTL_DSP_VOICEBANDCTL:
	{
	 	audio_voicebandctl new_ctl;
	
		if (copy_from_user(&new_ctl, (audio_voicebandctl *) arg,sizeof(audio_voicebandctl)))
			return -EFAULT;
		else
		{
			ret = GTI_Voiceband_Ctl(new_ctl.vbctl1, new_ctl.vbctl2); 
		}
		FN_OUT(6);
		return ret;
	}

	case SNDCTL_DSP_AUDIOVOICE:
	{
	 	audio_voice new_ctl;
	
		if (copy_from_user(&new_ctl, (audio_voice *) arg,sizeof(audio_voice)))
			return -EFAULT;
		else
		{
			ret = GTI_Audio_Voice(new_ctl.Mute);
		}
		FN_OUT(6);
		return ret;
	}

	case SNDCTL_DSP_AUDIOPGA:
	{
	 	audio_pga new_ctl;
	
		if (copy_from_user(&new_ctl, (audio_pga *) arg,sizeof(audio_pga)))
			return -EFAULT;
		else
		{
			ret = GTI_Audio_PGA(new_ctl.Uplink, new_ctl.Gain);
		}
		FN_OUT(6-2);
		return ret;
	}
		
	case SNDCTL_DSP_SIDETONE:
	{
		u8 val_tmp;
		ret = get_user(val_tmp, (u8 *) arg);
		if (ret)
			return ret;
		
		ret = GTI_Audio_Sidetone(val_tmp);
		FN_OUT(7);
		return ret;
	}	

	case SNDCTL_DSP_ECHOCANC:
	{
		u16 val_tmp;
		ret = get_user(val_tmp, (int *) arg);
		if (ret)
			return ret;
		
		ret = GTI_Audio_Echocanc(val_tmp);
		FN_OUT(8);
		return ret;
	}

	case SNDCTL_DSP_ENHECHOCANC:
	{
	 	audio_enhechocanc new_ctl;
	
		if (copy_from_user(&new_ctl, (audio_enhechocanc *) arg,sizeof(audio_enhechocanc)))
			return -EFAULT;
		else
		{
			ret = GTI_Audio_EnhEchocanc(new_ctl.Flag, new_ctl.Para1, new_ctl.Para2, new_ctl.Para3, new_ctl.Para4, new_ctl.Para5);
		}
		FN_OUT(85);
		return ret;
	}
		
	case SNDCTL_DSP_AUDIOCODEC:
	{
	 	audio_codec new_ctl;
	
		if (copy_from_user(&new_ctl, (audio_codec *) arg,sizeof(audio_codec)))
			return -EFAULT;
		else
		{
			ret = GSM_SetupAudio( new_ctl.on_off, new_ctl.AudioMode, new_ctl.Gain, new_ctl.SampleFrequency);
		}
		FN_OUT(9);
		return ret;
	}

	case SNDCTL_DSP_AUDIOPROFILE:
	{
		u8 val_tmp;
		ret = get_user(val_tmp, (u8 *) arg);
		if (ret)
			return ret;
		
		GTI_Audio_Profileload(val_tmp);
		FN_OUT(9-5);
		return 0;
	}	

	case SNDCTL_DSP_BTCTRL:
	{
		u8 val_tmp;
		ret = get_user(val_tmp, (u8 *) arg);
		if (ret)
			return ret;
		
		ret = bt_control(val_tmp);
		FN_OUT(10);
		return ret;
	}
	
	case SNDCTL_DSP_BYPASSCTRL:
	{
		u8 val_tmp;
		ret = get_user(val_tmp, (u8 *) arg);
		if (ret)
			return ret;
		
		modem_bypass(val_tmp);
		FN_OUT(10);
		return 0;
	}
		
	case SNDCTL_DSP_CHGSWITCH:
	{
		u16 val_tmp;
		ret = get_user(val_tmp, (u16 *) arg);
		if (ret)
			return ret;
		
		omap850_eac_write((u16 *) EAC_AMSCFR, val_tmp);
		FN_OUT(10);
		return 0;
	}			
	case SNDCTL_DSP_DUMP_REG:
	{
#ifdef DEBUG
	DBPRINT("PERSEUS2_IO_CONF0", REG32(PERSEUS2_IO_CONF0));
	DBPRINT("PERSEUS2_IO_CONF1", REG32(PERSEUS2_IO_CONF1));
	DBPRINT("PERSEUS2_IO_CONF2", REG32(PERSEUS2_IO_CONF2));
	DBPRINT("PERSEUS2_IO_CONF3", REG32(PERSEUS2_IO_CONF3));
	DBPRINT("PERSEUS2_IO_CONF4", REG32(PERSEUS2_IO_CONF4));
	DBPRINT("PERSEUS2_IO_CONF5", REG32(PERSEUS2_IO_CONF5));
	DBPRINT("PERSEUS2_IO_CONF6", REG32(PERSEUS2_IO_CONF6));
	DBPRINT("PERSEUS2_IO_CONF7", REG32(PERSEUS2_IO_CONF7));
	DBPRINT("PERSEUS2_IO_CONF8", REG32(PERSEUS2_IO_CONF8));
	DBPRINT("PERSEUS2_IO_CONF9", REG32(PERSEUS2_IO_CONF9));
	DBPRINT("PERSEUS2_IO_CONF10", REG32(PERSEUS2_IO_CONF10));
	DBPRINT("PERSEUS2_IO_CONF11", REG32(PERSEUS2_IO_CONF11));
	DBPRINT("PERSEUS2_IO_CONF12", REG32(PERSEUS2_IO_CONF12));
	DBPRINT("PERSEUS2_IO_CONF13", REG32(PERSEUS2_IO_CONF13));
	
	DBPRINT("PERSEUS2_MODE_1", REG32(PERSEUS2_MODE_1));
	
	DBPRINT("MPUIO_KBR_LATCH", REG32(MPUIO_KBR_LATCH));
	DBPRINT("MPUIO_KBC_REG", REG32(MPUIO_KBC_REG));
		
#if 0	
	DBPRINT("GPIO_MOTOR", REG32(GPIO_DATA_OUTPUT_REG + ((GPIO_MOTOR_ENABLE_NUM / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE)));

	//DBPRINT("GPIO_HEADSET 0", (*(REG32(GPIO_DATA_INPUT_REG + ((GPIO_HEADSET_DETECT_NUM / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE)))) >> (GPIO_HEADSET_DETECT_NUM % OMAP730_GPIO_NB)) & 1);
	//DBPRINT("GPIO_HEADSET 1",REG32(GPIO_DATA_INPUT_REG + ((GPIO_HEADSET_DETECT_NUM / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE)));
	DBPRINT("GPIO_HOOK_DET_IRQ_NUM 1",GPIO_DATA_INPUT_REG + ((GPIO_HOOK_DET_IRQ_NUM / OMAP730_GPIO_NB) * OMAP730_GPIO_SIZE));
	
	DBPRINT("DPLL1_CTL_REG", DPLL1_CTL_REG);
	DBPRINT("ARM_SYSST", ARM_SYSST);
	DBPRINT("ARM_CKCTL", ARM_CKCTL);
	DBPRINT("PCC_CTRL_REG", PCC_CTRL_REG);
	DBPRINT("ARM_RSTCT2", ARM_RSTCT2);
	DBPRINT("ARM_IDLECT1", ARM_IDLECT1);
	DBPRINT("ARM_IDLECT2", ARM_IDLECT2);
	DBPRINT("ARM_IDLECT3", ARM_IDLECT3);
	DBPRINT("OMAP_DMA_GCR_REG", OMAP_DMA_GCR_REG);
	DBPRINT("PERSEUS2_MODE_1", PERSEUS2_MODE_1);
	DBPRINT("ULPD_SOFT_DISABLE_REQ_REG", SOFT_DISABLE_REQ_REG);
	DBPRINT("ULPD_CAM_CLK_CTRL", CAM_CLK_CTRL);
	DBPRINT("EAC_AGCTR", EAC_AGCTR);
	DBPRINT("EAC_AGCFR", EAC_AGCFR);
	DBPRINT("EAC_AGCFR2", EAC_AGCFR2);
	DBPRINT("EAC_CPTCTL", EAC_CPTCTL);
	DBPRINT("EAC_CPCFR1", EAC_CPCFR1);
	DBPRINT("EAC_CPCFR2", EAC_CPCFR2);
	DBPRINT("EAC_CPCFR3", EAC_CPCFR3);
	DBPRINT("EAC_CPCFR4", EAC_CPCFR4);
	DBPRINT("EAC_AMVCTR", EAC_AMVCTR);
	DBPRINT("EAC_AM1VCTR", EAC_AM1VCTR);
	DBPRINT("EAC_AM2VCTR", EAC_AM2VCTR);
	DBPRINT("EAC_AM3VCTR", EAC_AM3VCTR);
	DBPRINT("EAC_AMSCFR", EAC_AMSCFR);
	DBPRINT("EAC_MPCTR", EAC_MPCTR);
	DBPRINT("EAC_MPMCCFR", EAC_MPMCCFR);
	DBPRINT("EAC_BPCTR", EAC_BPCTR);
	DBPRINT("EAC_BPMCCFR", EAC_BPMCCFR);
	DBPRINT("SOFT_REQ_REG", SOFT_REQ_REG);
	DBPRINT("PCC_PERIPH_CLOCK_SOURCE_SEL", PCC_PERIPH_CLOCK_SOURCE_SEL);
#endif
#endif

		return 0;
	}	
	//#define MPUIO_KBR_LATCH            (OMAP730_ARMIO_BASE + 0x08)
	//#define MPUIO_KBC_REG              (OMAP730_ARMIO_BASE + 0x0a)

	case SNDCTL_DSP_AUDIOBT:
	{
		*((volatile u32*) PERSEUS2_IO_CONF8) = 0x1111ddd1;
		return 0;
	}

	case GPIO_NEW_SETUP:
	{
	 	gpio_para new_gpio;
	
		if (copy_from_user(&new_gpio, (gpio_para *) arg,sizeof(new_gpio)))
			return -EFAULT;
		else
		{
			GPIO_SET_DIR(new_gpio.pin, new_gpio.pin_dir);    \
			GPIO_SET_STATE(new_gpio.pin, new_gpio.pin_def);   
		}
		
	}
		return 0;

	case GPIO_GET_INPUT:
	{
		int val;
        	int ret = 0;
        	int result;
		ret = get_user(val, (int *)arg);
		//GPIO_SET_STATE(val, 0);
		result = GPIO_GET_INPUT_STATE(val);
		printk("GPIO %d data is 0x%x \n", val, result);
	}
		return 0;	

	case GPIO_GET_OUTPUT:
	{
		int val;
        	int ret = 0;
        	int result;
		ret = get_user(val, (int *)arg);
		//GPIO_SET_STATE(val, 0);
		result = GPIO_GET_OUTPUT_STATE(val);
		printk("GPIO %d data is 0x%x \n", val, result);
	}
		return 0;
		
	case SNDCTL_DSP_SYREN_READ:
		return put_user(syren_codec_onoff, (u8 *) arg);
					
	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		FN_OUT(12);
		return mixer_ioctl(inode, file, cmd, arg);
	}

	FN_OUT(0);
	return ret;
}
#endif /*0*/
static int omap850_audio_open(struct inode *inode, struct file *file)
{
	//syren_clocks_on();
	return omap_audio_attach(inode, file, &audio_state);
}

#ifdef EAC_PROC_SUPPORT
/*
	DBPRINT("EAC_AGCTR", EAC_AGCTR);
	DBPRINT("EAC_AGCFR", EAC_AGCFR);
	DBPRINT("EAC_AGCFR2", EAC_AGCFR2);
	DBPRINT("EAC_CPTCTL", EAC_CPTCTL);
	DBPRINT("EAC_CPCFR1", EAC_CPCFR1);
	DBPRINT("EAC_CPCFR2", EAC_CPCFR2);
	DBPRINT("EAC_CPCFR3", EAC_CPCFR3);
	DBPRINT("EAC_CPCFR4", EAC_CPCFR4);
	DBPRINT("EAC_AMVCTR", EAC_AMVCTR);
	DBPRINT("EAC_AM1VCTR", EAC_AM1VCTR);
	DBPRINT("EAC_AM2VCTR", EAC_AM2VCTR);
	DBPRINT("EAC_AM3VCTR", EAC_AM3VCTR);
	DBPRINT("EAC_AMSCFR", EAC_AMSCFR);
	DBPRINT("EAC_MPCTR", EAC_MPCTR);
	DBPRINT("EAC_MPMCCFR", EAC_MPMCCFR);
	DBPRINT("EAC_BPCTR", EAC_BPCTR);
	DBPRINT("EAC_BPMCCFR", EAC_BPMCCFR);
*/
static struct proc_dir_entry *eac_dir = NULL;
static struct proc_dir_entry *AGCTR_entry = NULL;

static int create_eac_proc(void)
{
	eac_dir = proc_mkdir("eac", NULL);
	AGCTR_entry = create_proc_entry("AGCTR", S_IFREG | 0644, eac_dir);
	if(AGCTR_entry == NULL)
		return -EINVAL;


	return 0;
}
#endif
/*
 * Missing fields of this structure will be patched with the call
 * to omap_audio_attach().
 */
static struct file_operations omap850_audio_fops = {
	open:		omap850_audio_open,
	owner:		THIS_MODULE
};

#if 0
static int GSM_WriteAbb(u16 Page,u16 RegisterAddr,u16 Value)
{
	CSMI_INFO_GC_ABB_REGISTER_REQ   infoGcReq;
	CSMI_INFO_GC_ABB_REGISTER_RES	infoGcRes;
	u32 ret;
	DPRINTK("in GSM_WriteAbb\n");
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_ABB_REGISTER_REQ));
	memset(&infoGcRes, 0, sizeof(CSMI_INFO_GC_ABB_REGISTER_RES));
	
	infoGcReq.PrimitiveCode = GC_ABB_REGISTER_REQ;

	infoGcReq.WriteAccess = 1; //0=read | 1=write
	infoGcReq.Page = Page + 1;
	infoGcReq.RegisterAddr = RegisterAddr;
	infoGcReq.Value = Value; //0=read | 1=write

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_ABB_REGISTER_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     (u8*) &infoGcRes,
			     sizeof(infoGcRes),
			     NULL);
	if((ret == FALSE) || (infoGcRes.Value == 0xFFFF))
		DPRINTK("Write ABB failed\n");
	else
		DPRINTK("Write ABB success\n");
	return ((ret != FALSE) && (infoGcRes.Value == 0x0000)) ? 0:-1;
}

static u16 GSM_ReadAbb(u16 Page,u16 RegisterAddr)
{
	CSMI_INFO_GC_ABB_REGISTER_REQ   infoGcReq;
	CSMI_INFO_GC_ABB_REGISTER_RES	infoGcRes;
	u32 ret;
	DPRINTK("in GSM_ReadAbb\n");
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_ABB_REGISTER_REQ));
	memset(&infoGcRes, 0, sizeof(CSMI_INFO_GC_ABB_REGISTER_RES));
		
	infoGcReq.PrimitiveCode = GC_ABB_REGISTER_REQ;

	infoGcReq.WriteAccess = 0; //0=read | 1=write
	infoGcReq.Page = Page + 1;
	infoGcReq.RegisterAddr = RegisterAddr;
	infoGcReq.Value = 0; //0=read | 1=write

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_ABB_REGISTER_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     (u8*) &infoGcRes,
			     sizeof(infoGcRes),
			     NULL);
	if((ret == FALSE) || (infoGcRes.Value == 0xFFFF))
		DPRINTK("Read ABB failed\n");
	else
	{
		DPRINTK("Read ABB success\n");
		printk("register value = %d\n", infoGcRes.Value);
	}
	return ((ret != FALSE) && (infoGcRes.Value != 0xFFFF)) ? infoGcRes.Value:0;
}
/*
 *  GSM_SetupAudio
 *
 *  Setup the audio codec on the ARM7 side.
 *
 *  Param: TRUE/FALSE for respectively ON/OFF
 *
 *  Default params:
 *   - ControlCode = GC_STEREO_ON
 *   - AudioMode = GC_SPEAKER_MODE
 *   - Gain = 16
 *   - SampleFrequency = GC_SAMPLE_FREQ_48000
 *
 *  Returns 0 if OK, -1 if not OK
 */
static int GSM_SetupAudio( u8 on_off, u8 AudioMode,u8 Gain, u8 SampleFrequency)
{
	CSMI_INFO_GC_AUDIO_STEREO_REQ   infoGcReq;
	CSMI_INFO_GC_AUDIO_STEREO_RES	infoGcRes;
	u32 ret;
	DPRINTK("in GSM_SetupAudio\n");
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_STEREO_REQ));
	memset(&infoGcRes, 0, sizeof(CSMI_INFO_GC_AUDIO_STEREO_RES));
		
	infoGcReq.PrimitiveCode = GC_AUDIO_STEREO_CODEC_REQ;
//	infoGcReq.ControlCode = (on_off == TRUE) ? GC_STEREO_ON:GC_STEREO_OFF;
//	infoGcReq.AudioMode = GC_SPEAKER_MODE;
//	infoGcReq.Gain = 16;
//	infoGcReq.SampleFrequency = GC_SAMPLE_FREQ_48000;
	infoGcReq.ControlCode = on_off;//(on_off == 1) ? GC_STEREO_ON:GC_STEREO_OFF;
	infoGcReq.AudioMode = AudioMode;
	infoGcReq.Gain = Gain;
	infoGcReq.SampleFrequency = SampleFrequency;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_AUDIO_STEREO_CODEC_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     (u8*) &infoGcRes,
			     sizeof(infoGcRes),
			     NULL);
	if((ret == FALSE) || (infoGcRes.Result != TRUE))
		DPRINTK("setup audio failed\n");
	else if ((ret != FALSE) && (infoGcRes.Result == TRUE))
	{
		if (on_off == 1)
			syren_codec_onoff = 1;
		if (on_off == 0)
			syren_codec_onoff = 0;
	}
	return ((ret != FALSE) && (infoGcRes.Result == TRUE)) ? 0:-1;
}

static int GTI_Audio_Echocanc(u16 Algorithm)
{
	CSMI_INFO_GC_AUDIO_ECHOCANC_REQ   infoGcReq;
	u32 ret;
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_ECHOCANC_REQ));
	
	infoGcReq.PrimitiveCode = GC_AUDIO_ECHOCANC_REQ;
	infoGcReq.Algorithm = Algorithm;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_AUDIO_ECHOCANC_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     NULL,
			     0,
			     NULL);
	if(ret == FALSE)
		DPRINTK("audio echocanc failed!\n");
	else
		DPRINTK("audio echocanc success!\n");
	return (ret != FALSE) ? 0:-1;
}

static int GTI_Output_CTRL(u16 vauoctl)
{
	CSMI_INFO_GC_AUDIO_OUTPUTCTL_REQ   infoGcReq;
	CSMI_INFO_GC_AUDIO_OUTPUTCTL_RES	infoGcRes;

	u32 ret;
	FN_IN;
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_OUTPUTCTL_REQ));
	memset(&infoGcRes, 0, sizeof(CSMI_INFO_GC_AUDIO_OUTPUTCTL_RES));
	
	infoGcReq.PrimitiveCode = GC_AUDIO_OUTPUTCTL_REQ;
	infoGcReq.vauoctl = vauoctl;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL, //CSMI_SendToMailbox(CSMI_PORT portIndex,CSMI_MAILBOX *pP)
			     GC_AUDIO_OUTPUTCTL_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     (u8*) &infoGcRes,
			     sizeof(infoGcRes),
			     NULL);
	if((ret == FALSE) || (infoGcRes.Result != TRUE))
		printk("audio output ctrl failed!\n");
	else
		DPRINTK("audio output ctrl success!\n");
	FN_OUT(0);
	return ((ret != FALSE) && (infoGcRes.Result == TRUE)) ? 0:-1;
}

static int GTI_Voiceband_Ctl(u16 vbctl1, u16 vbctl2)
{
	CSMI_INFO_GC_AUDIO_VOICEBANDCTL_REQ   infoGcReq;
	CSMI_INFO_GC_AUDIO_VOICEBANDCTL_RES	infoGcRes;
	u32 ret;
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_VOICEBANDCTL_REQ));
	memset(&infoGcRes, 0, sizeof(CSMI_INFO_GC_AUDIO_VOICEBANDCTL_RES));
	
	infoGcReq.PrimitiveCode = GC_AUDIO_VOICEBANDCTL_REQ;
	infoGcReq.vbctl1 = vbctl1;
	infoGcReq.vbctl2 = vbctl2;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_AUDIO_VOICEBANDCTL_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     (u8*) &infoGcRes,
			     sizeof(infoGcRes),
			     NULL);
	if((ret == FALSE) || (infoGcRes.Result != TRUE))
		DPRINTK("audio voice band ctrl failed!\n");
	else
		DPRINTK("audio voice band ctrl success!\n");
	return ((ret != FALSE) && (infoGcRes.Result == TRUE)) ? 0:-1;
}

//static int GTI_Audio_Voice(u8 Uplink, u8 Mute, u8 Volume)
static int GTI_Audio_Voice(u8 Mute)
{
	CSMI_INFO_GC_AUDIO_VOICE_REQ   infoGcReq;
	u32 ret;
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_VOICE_REQ));
	
	infoGcReq.PrimitiveCode = GC_AUDIO_VOICE_REQ;
	//infoGcReq.Uplink = Uplink;
	infoGcReq.Mute = Mute;
	//infoGcReq.Volume = Volume;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_AUDIO_VOICE_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     NULL,
			     0,
			     NULL);
	if(ret == FALSE)
		DPRINTK("audio voice band ctrl failed!\n");
	else
		DPRINTK("audio voice band ctrl success!\n");
	return (ret != FALSE) ? 0:-1;
}

static int GTI_Audio_EnhEchocanc(u16 Flag, u16 Para1, u16 Para2, u16 Para3, u16 Para4, u16 Para5)
{
	CSMI_INFO_GC_AUDIO_ENHECHOCANC_REQ   infoGcReq;
	u32 ret;
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_ENHECHOCANC_REQ));
	
	infoGcReq.PrimitiveCode = GC_AUDIO_ENHECHOCANC_REQ;
	infoGcReq.Flag = Flag;
	infoGcReq.P1 = Para1;
	infoGcReq.P2 = Para2;
	infoGcReq.P3 = Para3;
	infoGcReq.P4 = Para4;
	infoGcReq.P5 = Para5;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_AUDIO_ENHECHOCANC_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     NULL,
			     0,
			     NULL);
	if(ret == FALSE)
		DPRINTK("AUDIO_ENHECHOCANC failed!\n");
	else
		DPRINTK("AUDIO_ENHECHOCANC success!\n");
	return (ret != FALSE) ? 0:-1;	
}

static int GTI_Audio_PGA(u8 Uplink, u8 Gain)
{
	CSMI_INFO_GC_AUDIO_PGA_REQ   infoGcReq;
	u32 ret;
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_PGA_REQ));
	
	infoGcReq.PrimitiveCode = GC_AUDIO_PGA_REQ;
	infoGcReq.Uplink = Uplink;
	infoGcReq.Gain = Gain;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_AUDIO_PGA_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     NULL,
			     0,
			     NULL);
	if(ret == FALSE)
		DPRINTK("audio PGA failed!\n");
	else
		DPRINTK("audio PGA success!\n");
	return (ret != FALSE) ? 0:-1;
}

static int GTI_Audio_Sidetone(u8 Volume)
{
	CSMI_INFO_GC_AUDIO_SIDETONE_REQ   infoGcReq;
	u32 ret;
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_SIDETONE_REQ));
	
	infoGcReq.PrimitiveCode = GC_AUDIO_SIDETONE_REQ;
	infoGcReq.Volume = Volume;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_AUDIO_SIDETONE_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     NULL,
			     0,
			     NULL);
	if(ret == FALSE)
		DPRINTK("audio voice band ctrl failed!\n");
	else
		DPRINTK("audio voice band ctrl success!\n");
	return (ret != FALSE) ? 0:-1;
}

static int GTI_Audio_Profileload(u8 Profile)
{
	CSMI_INFO_GC_AUDIO_PROFILE_LOAD_REQ	infoGcReq;
	CSMI_INFO_GC_AUDIO_PROFILE_LOAD_RES	infoGcRes;
	u32 ret;
	
	memset(&infoGcReq, 0, sizeof(CSMI_INFO_GC_AUDIO_PROFILE_LOAD_REQ));
	memset(&infoGcRes, 0, sizeof(CSMI_INFO_GC_AUDIO_PROFILE_LOAD_RES));
		
	infoGcReq.PrimitiveCode = GC_AUDIO_PROFILE_LOAD_REQ;
	infoGcReq.Profile = Profile;

	ret =  GTI_IOControl(CSMI_PORT_INDEX_GSMCTRL,
			     GC_AUDIO_PROFILE_LOAD_REQ,
			     (u8*) &infoGcReq,
			     sizeof(infoGcReq),
			     (u8*) &infoGcRes,
			     sizeof(infoGcRes),
			     NULL);
	if((ret == FALSE) || (infoGcRes.Result != TRUE))
		DPRINTK("audio profile load failed!\n");
	else
		DPRINTK("audio profile load success!\n");
	return ((ret != FALSE) && (infoGcRes.Result == TRUE)) ? 0:-1;	
}
static int bt_control(int on_off)
{
	DPRINTK("on_off = %d\n",on_off);
	switch(on_off)
	{
		case 0: // disable
			// system setup to support EAC 
			// while also can diable EAC using bypass path when calling
			// D_SYREN_VOICE: SCLK, SDO, SDI, FSYNC
		    	//*((volatile __u32 *) PERSEUS2_IO_CONF2) &= 0xFFFFFF1F;
			*((volatile u32*) PERSEUS2_IO_CONF2) &= 0xFFFFFF0F;	
			*((volatile u32*) PERSEUS2_IO_CONF2) |= 0x00000010;	
			*((volatile u32*) PERSEUS2_IO_CONF8) = 0x1111ddd1;
			*((volatile u32*) PERSEUS2_IO_CONF9) = 0x15bbbddd;
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFF3;
			
			// INTERNAL_GSM_VOICE_SOURCE: Internal EAC Modem AuSPI
			// bits 1:0 = 01: Internal EAC modem AuSPI
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFFC;
			*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000001; 									
		break;
		
		case 1: // bt enable
			*((volatile u32*) PERSEUS2_IO_CONF2) &= 0xFFFFFF0F;	
			*((volatile u32*) PERSEUS2_IO_CONF2) |= 0x000000D0;	
			*((volatile u32*) PERSEUS2_IO_CONF8) = 0x3333ddd1; //0x2222ddd1; // 0x3333ddd1
			*((volatile u32*) PERSEUS2_IO_CONF9) = 0x15bbbddd;
			// INTERNAL_EAC_BT_AUSPI_SOURC: Pins BT AuSPI : SCLK, SDI, FSYNC
			// bits 3:2 = 01: Pins MPU_SPI: MPU_SCLK, MPU_SDI,
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFF3;
			*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000004;
			
			// INTERNAL_GSM_VOICE_SOURCE: Internal EAC Modem AuSPI
			// bits 1:0 = 01: Internal EAC modem AuSPI
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFFC;
			*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000001; 
#if 0	// HIGH END app
			// INTERNAL_GSM_VOICE_SOURCE: Internal EAC Modem AuSPI
			// bits 1:0 = 01: Internal EAC modem AuSPI
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFFC;
			*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000001; 
			
			// INTERNAL_EAC_BT_AUSPI_SOURC: Pins BT AuSPI : SCLK, SDI, FSYNC
			// bits 3:2 = 00: Pins MPU_SPI: MPU_SCLK, MPU_SDI,
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFF3;
			*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000000;
			
			// D_EAC pins: D_SPI1_SEN0, D_SPI1_SDI, D_SPI1_SDO, D_SPI1_SCLK
			// bits 31:29 = 001 27:25 = 001 23:21 = 001 19:17 = 001
			*((volatile u32*) PERSEUS2_IO_CONF8) = 0x2222ddd1; // 0x3333ddd1
			
			// D_SMC_PWR: TSPACT_9, D_SPI1_SEN1 SEN1
			// bits 3:1 = 000
			*((volatile u32*) PERSEUS2_IO_CONF9) = 0x15bbbdd2;

			*((volatile u32*) PERSEUS2_IO_CONF4) = 0x11111111; // 27:25 for GPIO INT of FLIP STAT
			*((volatile u32*) PERSEUS2_IO_CONF3) = 0x1d119911; // 27:25 FP_INT GPIO
				
			// D_SYREN_VOICE: VCLKRX, VDX, VDR, VFSRX
		    	// bits 7:5 = 110: GPIO_22, GPIO_23, GPIN_3, GPIO_24
		    	*((volatile u32*) PERSEUS2_IO_CONF2) = 0x111111D0;		    	
		
#endif													
		break;
		
		default:
		break;		
	}
	return 0;	
}

static void modem_bypass(int on_off)
{
	switch(on_off)
	{
		case 0: // EAC support
    	*((volatile __u32 *) PERSEUS2_IO_CONF2) &= 0xFFFFFF1F;


	// D_CRESET: GPIO36 used for headset detection
			// INTERNAL_EAC_BT_AUSPI_SOURC: Pins BT AuSPI : SCLK, SDI, FSYNC
			// SPI used for EAC_BT_SPI bits 3:2 = 00
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFF3;
			
			// INTERNAL_GSM_VOICE_SOURCE: Internal EAC Modem AuSPI
			// bits 1:0 = 01
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFFC;
			*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000001;
		break;
		
		case 1: // on not-eac
			// system setup to connect the modem directly to SYREN VSP
			// and the EAC isn't used when calling
			// D_SYREN_VOICE: VCLKRX, VDX, VDR, VFSRX
			// D_SYREN_VOICE 7:5 = 001
		    	*((volatile __u32 *) PERSEUS2_IO_CONF2) &= 0xFFFFFF0F;
		    	*((volatile __u32 *) PERSEUS2_IO_CONF2) |= 0x00000030;
			// INTERNAL_EAC_BT_AUSPI_SOURC: Pins BT AuSPI : SCLK, SDI, FSYNC
			// SPI used for EAC_BT_SPI bits 3:2 = 00
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFF3;
			
			// INTERNAL_GSM_VOICE_SOURCE: Internal EAC Modem AuSPI
			// bits 1:0 = 00
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFFC;
		break;
		
		case 2: // bt enable
			*((volatile u32*) PERSEUS2_IO_CONF2) &= 0xFFFFFF0F;
		        *((volatile u32*) PERSEUS2_IO_CONF2) |= 0x000000D0;

		        *((volatile u32*) PERSEUS2_IO_CONF8) &= 0x0000FFFF; //0x2222ddd1; // 0x3333ddd1
		        *((volatile u32*) PERSEUS2_IO_CONF8) |= 0x33330000; //0x2222ddd1; // 0x3333ddd1	

			// bits 3:2 = 01: Pins MPU_SPI: MPU_SCLK, MPU_SDI,
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFF3;
			*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000004;

		        // INTERNAL_GSM_VOICE_SOURCE: Internal EAC Modem AuSPI
			// bits 1:0 = 01: Internal EAC modem AuSPI
			*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFFC;
			*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000001;
		break;
				
		default:
		break;		
	}
	return;	
}
#endif /* 0 */
static int audio_dev_id, mixer_dev_id;

static void syren_audio_init(void)
{
	        FN_IN;
		        /* p. 5-15, syren master clock = 13Mhz, not 32khz */
		//      syren_write_reg(0, SYREN_TOGBR2, SYREN_TOGBR2_ACTS, SYREN_TOGBR2_ACTS);
		        syren_direct_io(0, SYREN_TOGBR2, SYREN_TOGBR2_ACTS & SYREN_TOGBR2_ACTS,1);
			        //syren_write_regite_reg
			        /* p. 5-38, set sample rate to 44.1khz */
			//      syren_write_reg(1, SYREN_VAUDCTRL, SYREN_VAUDCTRL_SRW_441, SYREN_VAUDCTRL_SR_MASK);
			        syren_direct_io(1, SYREN_VAUDCTRL, SYREN_VAUDCTRL_SRW_441 & SYREN_VAUDCTRL_SR_MASK,1);

				        /* p. 5-40, set gain to 0db */
				//      syren_write_reg(1, SYREN_VAUSCTRL, 0, 0xffff);
				        syren_direct_io(1, SYREN_VAUSCTRL, 0 & 0xffff, 1);

					        /* p. 5-39, set audio outputs to headset L/R and speaker*/
					//      syren_write_reg(1, SYREN_VAUOCTRL, SYREN_VAUOCTRL_SPK | SYREN_VAUOCTRL_HS,
					//                                      SYREN_VAUOCTRL_AUDIO_MASK);
					        syren_direct_io(1, SYREN_VAUOCTRL, (SYREN_VAUOCTRL_SPK | SYREN_VAUOCTRL_HS) & SYREN_VAUOCTRL_AUDIO_MASK,1);

}

void syren_clocks_on(void)
{
	        FN_IN;
		        /* p. 5-15, enable audio codec clock in PWDNREG */
		//      syren_write_reg(0, SYREN_TOGBR2, SYREN_TOGBR2_AUDS, SYREN_TOGBR2_AUDS);
		        syren_direct_io(0, SYREN_TOGBR2, SYREN_TOGBR2_AUDS & SYREN_TOGBR2_AUDS, 1);

			        /* p. 5-41, power up audio PLL */
			//      syren_write_reg(1, SYREN_VAUDPLL, SYREN_VAUDPLL_AUPLLON, SYREN_VAUDPLL_AUPLLON);
			        syren_direct_io(1, SYREN_VAUDPLL, SYREN_VAUDPLL_AUPLLON & SYREN_VAUDPLL_AUPLLON, 1);

				        /* from TI sysboot sample code - delay 0.000157 sec here */
				        udelay(SYREN_DELAY_USEC);

					        /* p. 5-41, activate I2S interface */
					//      syren_write_reg(1, SYREN_VAUDPLL, SYREN_VAUDPLL_AUPLLON | SYREN_VAUDPLL_I2SON,
					//                                                              SYREN_VAUDPLL_AUPLLON | SYREN_VAUDPLL_I2SON);
					        syren_direct_io(1, SYREN_VAUDPLL, (SYREN_VAUDPLL_AUPLLON | SYREN_VAUDPLL_I2SON) & (SYREN_VAUDPLL_AUPLLON | SYREN_VAUDPLL_I2SON),1);
}

void syren_clocks_off(void)
{
	        FN_IN;
		        /* p. 5-15, disable audio codec clock in PWDNREG */
		//      syren_write_reg(0, SYREN_TOGBR2, SYREN_TOGBR2_AUDR, SYREN_TOGBR2_AUDR);
		        syren_direct_io(0, SYREN_TOGBR2, SYREN_TOGBR2_AUDR & SYREN_TOGBR2_AUDR, 1);

			        /* p. 5-41, power down audio PLL */
			//      syren_write_reg(1, SYREN_VAUDPLL, 0, SYREN_VAUDPLL_AUPLLON);
			        syren_direct_io(1, SYREN_VAUDPLL, 0 & SYREN_VAUDPLL_AUPLLON, 1);

				        /* from TI sysboot sample code - delay 0.000157 sec here */
				        udelay(SYREN_DELAY_USEC);

					        /* p. 5-41, deactivate I2S interface */
					//      syren_write_reg(1, SYREN_VAUDPLL, 0, SYREN_VAUDPLL_AUPLLON | SYREN_VAUDPLL_I2SON);
					        syren_direct_io(1, SYREN_VAUDPLL, 0 & (SYREN_VAUDPLL_AUPLLON | SYREN_VAUDPLL_I2SON), 1);

}

#define OMAP730_UPLD_PCC_POWER_CTRL             (OMAP730_PCC_UPLD_CTRL_BASE+0x050)
#define OMAP730_UPLD_PCC_CAM_CLK_CTRL   (OMAP730_PCC_UPLD_CTRL_BASE+0x07c)



static void eac_init(void)
{
	/* p.650 - Get EAC signals - CSYNC, CSCLK, CDO */
	set_bit_range_32(OMAP730_IO_CONF_4, 23, 21, 0);

	/* p.649 - Get EAC CDI signal */
	set_bit_range_32(OMAP730_IO_CONF_4, 27, 25, 0);

	/* p.649 - Get EAC MCLK signal */
	set_bit_range_32(OMAP730_IO_CONF_4, 31, 29, 0);

	/* p.1641 - system clock enabled */
	set_bit_range_32(OMAP730_UPLD_PCC_CAM_CLK_CTRL, 2, 2, 1);

	/* p.659 - Set GPIO 145 to CLK_13M_REQ */
	set_bit_range_32(OMAP730_IO_CONF_10, 31, 29, 0);

	/* XXX eac - disable codec port - TI sysboot does not do
	    * this but i don't think unload/reload will work without it */
	printk("EAC_CPTCTL:%x\n", omap_readw(EAC_CPTCTL));
	set_bit_range_16(EAC_CPTCTL, 3, 3, 0);
	printk("EAC_CPTCTL:%x\n", omap_readw(EAC_CPTCTL));


	/* p. 15-97 - set audio clock source to 13Mhz */
	set_bit_range_16(EAC_AGCFR, 5, 4, 0x2);

	/* p. 15-94 - close K1 switch (audio playback)
	    *            and K5 switch (audio record from mic)
	     */
	// XXX all input switches closed, still nothing -- syren?
	set_bit_range_16(EAC_AMSCFR, 11, 0, (1<<0)|(1<<4)|(1<<5)|(1<<6)|(1<<7));

	/* p. 15-83 - select I2S mode */
	set_bit_range_16(EAC_CPCFR1, 2, 0, 0x4);
	/* p. 15-83 - 2 time slots per frame (I2S) */
	set_bit_range_16(EAC_CPCFR1, 7, 3, 0x1);

	/* p. 15-84 - 20 serial clock cycle time slot length */
	set_bit_range_16(EAC_CPCFR2, 2, 0, 0x3);
	/* p. 15-84 - 16 data bits per time slot */
	set_bit_range_16(EAC_CPCFR2, 5, 3, 0x1);
	/* p. 15-84 - time slot 0 length same as TSLL */
	set_bit_range_16(EAC_CPCFR2, 7, 6, 0x0);

	/* p. 15-99 - FSINT2 is 44.1khz */
	set_bit_range_16(EAC_AGCFR2, 2, 0, 0x3);

//	omap850_eac_write(EAC_AMVCTR, (DMA_DEFAULT_VOLUME << EAC_AMVCTR_RD_DMA_OFFSET) | (DMA_DEFAULT_VOLUME << EAC_AMVCTR_WR_DMA_OFFSET));
	/* set default volume */
	//set_bit_range_16(EAC_AMVCTR, 15, 0, (0xe7 << EAC_AMVCTR_RD_DMA_OFFSET) | (0xe7 << EAC_AMVCTR_WR_DMA_OFFSET));
	
	omap_writew(0x3099, EAC_AMVCTR);
	//eac_local.volume = 0x99;
	//eac_local.line = eac_local.volume;

	//eac_update();
	omap_writew(0x3067, EAC_AM1VCTR);                                                                                                                      
	omap_writew(0x3067, EAC_AM2VCTR);                                                                                                                      
	omap_writew(0x3000, EAC_AM3VCTR);   
	printk("EAC_AMVCTR : 0x%04x\n", omap_readw(EAC_AMVCTR));	
	/* p. 15-97 - little endian audio data */
	set_bit_range_16(EAC_AGCFR, 8, 8, 0);
	/* p. 15-97 - 16-bit audio data */
	set_bit_range_16(EAC_AGCFR, 9, 9, 1);
	/* p. 15-97 - stereo audio data */
	set_bit_range_16(EAC_AGCFR, 10, 10, 1);

	/* p. 15-85 - CP_SCLK is an input */
	set_bit_range_16(EAC_CPCFR3, 1, 1, 1);
	/* p. 15-85 - CP_SYNC is an input */
	set_bit_range_16(EAC_CPCFR3, 0, 0, 1);

	/* p. 15-98 - enable audio */
	set_bit_range_16(EAC_AGCTR, 1, 1, 1);

	/* p. 15-86 - enable codec port (disables access to config regs) */
	set_bit_range_16(EAC_CPTCTL, 3, 3, 1);
}


#define EAC_BPMCCFR_DEFAULT_SLAVE_NOCOMP_13BITS 0x00EC
static int __init omap850_syren_init(void)
{
	u16 temp;
	int ret = 0;
	struct file_operations *op = NULL;
	u16 reg = 0;
	FN_IN;
	
	eac_dump();
	eac_init();
	/*
	 * Pins multiplexing
	 */

	//modem_bypass(0);
	/*
	 * UPLD Clocks
	 */
/*	
	reg = omap_readw(SOFT_REQ_REG);
	reg |= SOFT_REQ_REG_EAC12M_DPLL_REQ;
	omap_writew(reg, SOFT_REQ_REG);

	reg = omap_readw(PCC_PERIPH_CLOCK_SOURCE_SEL);
	reg |= PCC_PERIPH_SOURCE_EAC_CLK_SOURCE;
	omap_writew(reg, PCC_PERIPH_CLOCK_SOURCE_SEL);

	reg = omap_readw(CAM_CLK_CTRL);
	reg |= CAM_CLK_CTRL_SYSTEM_CLK_EN;
	omap_writew(reg, CAM_CLK_CTRL);

*/	
	/*
	 * GPIO pins setup to detect headset
	 */
	// TODO: GPIO36 for headset detection


	/*
	 * EAC setup
	 */

	// Audio Global Control Register 2
//	temp = omap850_eac_read(EAC_AGCTR);
	// EAC in powerdown mode
//	temp |= EAC_AGCTR_EACPWD;
	// Audio processing disabled
//	temp &= ~EAC_AGCTR_AUDEN;
//	omap850_eac_write(EAC_AGCTR, temp);

	// Audio Global Configuration Register
//	temp = omap850_eac_read(EAC_AGCFR) & EAC_AGCFR_RESERVED;
	// stereo, 16 bit audio file
//	temp |= EAC_AGCFR_B8_16 | EAC_AGCFR_MN_ST;
	// clock setting
//	temp |= EAC_AGCFR_AUD_CKSRC_12MHZ;
//	omap850_eac_write(EAC_AGCFR, temp);
//	printk(KERN_ALERT "Test value\n");
	// EAC rev2 Intermediate sample frequency for DMA read and write operations
//	omap850_set_samplerate(AUDIO_RATE_DEFAULT);

	// set clock on
//	omap850_setclock(1);

	// Audio Mixer Switchs Configuration Register
//	omap850_eac_write(EAC_AMSCFR, 0xFEF); //0xFEF); //k5 = 0 becoz c-codec no input EAC_AMSCFR_DEFAULT_SWITCHES);
	//omap850_eac_write((u16 *) EAC_AMSCFR, 0x0A00);

	// Set default volume
	// Default DMA volume
//	omap850_eac_write(EAC_AMVCTR, (DMA_DEFAULT_VOLUME << EAC_AMVCTR_RD_DMA_OFFSET) | (DMA_DEFAULT_VOLUME << EAC_AMVCTR_WR_DMA_OFFSET));
	// Line (GSM) & Mic input volume control
    //    eac_local.line = eac_local.mic = DEFAULT_INPUT_VOLUME;
//		eac_local.line = eac_local.mic = DEFAULT_VOLUME;

	// MPU volume control
  //      eac_local.volume = DEFAULT_VOLUME;
    //    eac_update();
	// No sidetone
//	temp = omap850_eac_read(EAC_ASTCTR);
//	temp &= ~EAC_ASTCTR_ATTEN;
//	omap850_eac_write(EAC_ASTCTR, temp);

	// Audio processing enable
//	temp = omap850_eac_read(EAC_AGCTR);
//	temp |= EAC_AGCTR_AUDEN;
//	omap850_eac_write(EAC_AGCTR, temp);

	/*
	 * Codec port setup
	 */

	// CODEC Port Interface Control and Status Register
//	temp = omap850_eac_read(EAC_CPTCTL) & EAC_CPTCTL_RESERVED;
	// CODEC RESET release , clear RECEIVE DATA REGISTER FULL and TRANSMIT DATA REGISTER EMPTY
//	temp |= EAC_CPTCTL_CRST | EAC_CPTCTL_TXE | EAC_CPTCTL_RXF;
	// C_PORT ENABLE Disabled to configure some registers
//	temp &= ~EAC_CPTCTL_CPEN;
//	omap850_eac_write(EAC_CPTCTL, temp);

	// Codec Port Configuration Register 1
	// Codec-Port interface mode: I2S mode, Number of time slots per audio frame: 2 time slots per frame
//	omap850_eac_write(EAC_CPCFR1, EAC_CPCFR1_MODE_I2S);

	// CODEC PORT CONFIGURATION REGISTER 2
//	omap850_eac_write(EAC_CPCFR2, EAC_CPCFR2_I2S_20BITS);

	// CODEC PORT INTERFACE CONFIGURATION REGISTER 3
//	omap850_eac_write(EAC_CPCFR3, EAC_CPCFR3_I2S_INPUT);

	// CODECPORT INTERFACE CONFIGURATION REGISTER 4
	// DIVB Calc: (12000000/(2*16*44100))-1=7
//	omap850_eac_write(EAC_CPCFR4, EAC_CPCFR4_I2S_DIV7);

	// CODEC Port Interface Control and Status Register
//	temp = omap850_eac_read(EAC_CPTCTL) & EAC_CPTCTL_RESERVED;
	// C_PORT ENABLE Enabled
//	temp |= EAC_CPTCTL_CPEN;
//	omap850_eac_write(EAC_CPTCTL, temp);

	/*
	 * Modem port setup
	 */

	// Modem Port Control Register
//	omap850_eac_write(EAC_MPCTR, EAC_MPCTR_DISABLEALL);

	// Modem Port Main channel Configuration Register
//	omap850_eac_write(EAC_MPMCCFR, EAC_MPMCCFR_DEFAULT_MASTER_NOCOMP_16BITS);

	// Modem Port Control Register
//	temp = omap850_eac_read(EAC_MPCTR);
//	temp |= EAC_MPCTR_PRE_MC_16 | EAC_MPCTR_MC_EN;	// Prescaler and enable
//	omap850_eac_write(EAC_MPCTR, temp);
//	temp |= EAC_MPCTR_CKEN;				// Clocks running
//	omap850_eac_write(EAC_MPCTR, temp);

	/*
	 * Bluetooth port setup
	 */

	// Bluetooth Port Control Register
//	omap850_eac_write(EAC_BPCTR, EAC_BPCTR_DISABLEALL);

	// Bluetooth Port Main channel Configuration Register
//	omap850_eac_write(EAC_BPMCCFR, EAC_BPMCCFR_DEFAULT_SLAVE_NOCOMP_13BITS);

	// Modem Port Control Register
//	temp = omap850_eac_read(EAC_BPCTR);
//	temp |= EAC_BPCTR_PRE_MC_16 | EAC_BPCTR_MC_EN;	// Prescaler and enable
//	omap850_eac_write(EAC_BPCTR, temp);
//	temp |= EAC_BPCTR_CKEN;				// Clocks running
//	omap850_eac_write(EAC_BPCTR, temp);

#ifdef CONFIG_CEE  /* MVL-CCE */
//FIXME         audio_ldm_device_register();
//FIXME         audio_ldm_driver_register();
#endif /* MVL-CCE */

	/*
	 * Driver init
	 */
	syren_spi_init();
	syren_audio_init();
	syren_clocks_on();
	op = &omap850_audio_fops;
		
	/* add missing file operations from audio */	
	op = omap_audio_get_fops();

	omap850_audio_fops.release = op->release;
	omap850_audio_fops.write = op->write;
	omap850_audio_fops.read	= op->read;
	omap850_audio_fops.mmap	= op->mmap;
	omap850_audio_fops.poll	= op->poll;
	omap850_audio_fops.ioctl = op->ioctl;

	ret = register_chrdev(199, "omap850_syren", &omap850_audio_fops);

	if (ret < 0)
		goto end;
	/* init semaphore */
	init_MUTEX(&audio_state.sem);
end:
		
	//GSM_SetupAudio(0, 0x03, 0, 6);
	//GTI_Output_CTRL(768);
	//GSM_SetupAudio(1, 0x03, 0, 6);
	eac_dump();
	//GTI_Output_CTRL();

#ifdef EAC_PROC_SUPPORT
		if(retval = create_eac_proc())
			return retval;
#endif
	printk(KERN_ALERT "Module omap850-syren init\n");
	FN_OUT(0);
	return ret;
}

static void __exit omap850_syren_exit(void)
{
	omap_audio_clear_buf(audio_state.output_stream);
	omap_audio_clear_buf(audio_state.input_stream);
	unregister_chrdev(199, "omap850_syren");
	syren_clocks_off();

#ifdef CONFIG_CEE  /* MVL-CCE */
        audio_ldm_device_unregister();
        audio_ldm_driver_unregister();
#endif /* MVL-CCE */
	//free_irq(INT_SYREN, NULL);
	//free_irq(INT_EARPHONE_JACK, NULL);
	//free_irq(INT_HOOK_DETECT,NULL);
	
//FIXME 	unregister_sound_dsp(audio_dev_id);
//FIXME 	unregister_sound_mixer(mixer_dev_id);
}


module_init(omap850_syren_init);
module_exit(omap850_syren_exit);

MODULE_AUTHOR("Jean Pihet");
MODULE_DESCRIPTION("Glue audio driver for the TI OMAP730 & TI TWL3016 (Syren) CODEC");
MODULE_LICENSE("GPL");
