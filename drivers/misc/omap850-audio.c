/*
 *  linux/drivers/sound/omap730-audio.c -- audio interface for the OMAP730 chip
 *  Author:     Jean Pihet <j-pihet@ti.com>
 * 
 *  From linux/drivers/sound/pxa-audio.c -- audio interface for the Cotula chip
 *  Author:	Nicolas Pitre
 *  Created:	Aug 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
/*
 *
 *   Copyright (C) 2007 E28 SH Limited,
 *
 *   Revision History:
 *  		      Modification    
 *	    Date             Description of Changes
 *	------------      -------------------------
 *	June 25,2007       E28 for SmartCore Platform
 *  March 13, 2008     add timeout mechanism for sync return
 *  
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/arch/dma.h>
// #include <asm/dma.h>
#include "omap850-audio.h"

/*
 * Buffering mechanism
 *
 * Since the P2 DMA channels (linked or not) are not dynamic (i.e. the registers cannot be
 * updated on the fly), we have to use fixed DMA addresses and parameters.
 *
 * So the principle for audio playback operation is:
 * - use AUDIO_NBFRAGS_DEFAULT user buffers of fixed size AUDIO_FRAGSIZE_DEFAULT to buffer the user
 *   data,
 * - use one physical channel and one DMAable buffer for data transfer to the audio device (EAC).
 *   This buffer is divided into two parts of fixed size AUDIO_FRAGSIZE_DEFAULT,
 * - use the HALF_FRAME and FRAME interrupts to alternately copy one of the user buffers 
 *   into the DMA buffer half that is not in use,
 * - the buffers management is done through the read (user buffers to DMA) and write
 *   (user space to user buffers) buffer indexes in a circular fashion.
 *
 * The audio record principle is comparable, only the data transfer direction changes.
 *
 * This principle is not ideal because we memcpy data in the irq handler, but used because:
 * - sufficient buffering is achieved for user data, allowing good performance under
 *   heavy system load,
 * - the minimum amount of work is done in the irq handler: small amount of data (usually 8KB)
 *   is memcpy'ed. This keeps the irq handling as fast as possible.
 */

#define AUDIO_NBFRAGS_DMA		2	// DMA buffer (two halves)
#define AUDIO_NBFRAGS_USER_DEFAULT	8	// User buffers
#define AUDIO_FRAGSIZE_DEFAULT		8192	// Fixed size

#define AUDIO_SILENCE			0x00

#define DEBUG_FN

#ifdef DEBUG_FN
#define FN_IN() printk("[omap_audio.c:[%s] start\n", __FUNCTION__)
#define FN_OUT(n) printk("[omap_audio.c:[%s] end(%d)\n", __FUNCTION__ , n)
#else
#define FN_IN 
#define FN_OUT(n) 
#endif
#undef DEBUG
// #define DEBUG

static spinlock_t bufindex_lock = SPIN_LOCK_UNLOCKED;
volatile unsigned short max_dma_count = 0;


/* dump registers for DMA */
void dump_regs(char *msg, int ch)
{
#ifdef DEBUG

#define DBPRINT(name, addr) \
	printk(KERN_ALERT "%s: %s(@0x%08x)\n", __FUNCTION__, name, addr);

	printk(KERN_ALERT"***** %s: %s *****\n", __FUNCTION__, msg);

	printk(KERN_ALERT"******** DMA channel %d settings ********\n", ch);
	DBPRINT("OMAP_DMA_CSDP_REG", OMAP_DMA_CSDP_REG(ch));
	DBPRINT("OMAP_DMA_CCR_REG", OMAP_DMA_CCR_REG(ch));
	DBPRINT("OMAP_DMA_CCR2_REG", OMAP_DMA_CCR2_REG(ch));
	DBPRINT("OMAP_DMA_CICR_REG", OMAP_DMA_CICR_REG(ch));
	DBPRINT("OMAP_DMA_CSR_REG", OMAP_DMA_CSR_REG(ch));
	DBPRINT("OMAP_DMA_CSSA_L_REG", OMAP_DMA_CSSA_L_REG(ch));
	DBPRINT("OMAP_DMA_CSSA_U_REG", OMAP_DMA_CSSA_U_REG(ch));
	DBPRINT("OMAP_DMA_CDSA_L_REG", OMAP_DMA_CDSA_L_REG(ch));
	DBPRINT("OMAP_DMA_CDSA_U_REG", OMAP_DMA_CDSA_U_REG(ch));
	DBPRINT("OMAP_DMA_CEN_REG", OMAP_DMA_CEN_REG(ch));
	DBPRINT("OMAP_DMA_CFN_REG", OMAP_DMA_CFN_REG(ch));
	DBPRINT("OMAP_DMA_CLNK_CTRL_REG", OMAP_DMA_CLNK_CTRL_REG(ch));
	DBPRINT("OMAP_DMA_LCH_CTRL_REG", OMAP_DMA_LCH_CTRL_REG(ch));
#endif
}


static void start_dma(audio_stream_t *s)
{
	u16 temp;
	FN_IN();
	/* Prepare the first DMA buffer data */
        if (s->output) {
		/* Copy the next user buffer to the first half of the DMA buffer */
		/* The other buffers will be handled in the irq handler */
		memcpy(s->dma_buf.vaddr, s->user_buf[s->dma_frag].data, s->fragsize);
		if (++s->dma_frag >= s->nbfrags)
			s->dma_frag = 0;
	}

	/* Start DMA channel */
	/* omap_start_dma() from the 2.6. kernel is equivalent to 
	 * omap_resume_dma() from the 2.4 kernel.
	 */

	omap_start_dma(s->dma_ch);
	s->dma_running = 1;

	/* Audio Global Control Register 2: start sync transfer */
	temp = omap_readw(EAC_AGCTR);
        if (s->output) {
		/* DMA write operation enabled */
		temp |= EAC_AGCTR_DMAREN;
        }
        else {
		/* DMA read operation enabled */
		temp |= EAC_AGCTR_DMAWEN;
	}
	omap_writew(temp, EAC_AGCTR);
	FN_OUT(0);
}

static void stop_dma(audio_stream_t *s)
{
	u16 temp;
	FN_IN();

	s->dma_running = 0;

	/* Audio Global Control Register 2: stop sync transfer */
	temp = omap_readw(EAC_AGCTR);
        if (s->output) {
		/* DMA write operation disabled */
		temp &= ~EAC_AGCTR_DMAREN;
        }
        else {
		/* DMA read operation disabled */
		temp &= ~EAC_AGCTR_DMAWEN;
	}
	omap_writew(temp, EAC_AGCTR);

	/* Stop DMA channel */
	omap_stop_dma(s->dma_ch);
	FN_OUT(0);
}

  
void audio_configure_dma_channel(audio_stream_t *s)
{

	struct omap_dma_channel_params params;

	params.data_type = OMAP_DMA_DATA_TYPE_S16;	/* data type 16 */
	params.elem_count = s->fragsize;		/* two buffers, 16 bits of s->fragsize */
	params.frame_count = 1; /* only one frame */
	params.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	/*FIXME: not sure if this is OK */
// 	params.read_prio = DMA_CH_PRIO_HIGH;
// 	params.write_prio = DMA_CH_PRIO_HIGH;
	
	if (s->output) {
        	params.src_start = s->dma_buf.dma_addr;
		params.src_port = OMAP_DMA_PORT_EMIFF;
		params.src_amode = OMAP_DMA_AMODE_POST_INC;
		params.dst_start = EAC_ADRDR;
        	params.dst_port = OMAP_DMA_PORT_TIPB;
		params.dst_amode = OMAP_DMA_AMODE_CONSTANT;
        } else {
		params.src_start = EAC_ADWDR;
		params.src_port = OMAP_DMA_PORT_TIPB;
		params.src_amode = OMAP_DMA_AMODE_CONSTANT;
		params.dst_start = s->dma_buf.dma_addr;
        	params.dst_port = OMAP_DMA_PORT_EMIFF;
		params.dst_amode = OMAP_DMA_AMODE_POST_INC;
        }
	
        omap_set_dma_params(s->dma_ch, &params);
	/* enable interrupts which are handled in irq handler */
	omap_enable_dma_irq(s->dma_ch, OMAP_DMA_FRAME_IRQ);
	omap_disable_dma_irq(s->dma_ch, OMAP_DMA_BLOCK_IRQ);
	omap_enable_dma_irq(s->dma_ch, OMAP_DMA_HALF_IRQ);
	dump_regs("audio_configure_dma_channel()", s->dma_ch);
}


/*
 * This function frees all buffers
 */
#define audio_clear_buf omap_audio_clear_buf

void omap_audio_clear_buf(audio_stream_t * s)
{
	int frag;

	if (!s->user_buf)
		return;

	/* Ensure DMA isn't running */
	stop_dma(s);

	/* free user buffers */
	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_user_buf_t *b = &s->user_buf[frag];
		if (b->data)
			kfree(b->data);
	}
	
	/* free DMA buffer */
	if (s->dma_buf.vaddr)
		dma_free_coherent(NULL, s->fragsize * AUDIO_NBFRAGS_DMA, s->dma_buf.vaddr, s->dma_buf.dma_addr);

	/* free buffer structure array */
	kfree(s->user_buf);
	s->user_buf = NULL;
}

/*
 * This function allocates the DMA buffer and user buffers
 * according to the current number of fragments and fragment size.
 */
static int audio_setup_buf(audio_stream_t * s)
{
	int frag;
	char *dma_buf = NULL;
	dma_addr_t dma_buf_phys = 0;

	if (s->user_buf)
		return -EBUSY;

	/* Our buffer structure array */
	s->user_buf = kzalloc(sizeof(audio_user_buf_t) * s->nbfrags, GFP_KERNEL);
	if (!s->user_buf) 
		goto err;
	
	/* Our user buffers */
	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_user_buf_t *b = &s->user_buf[frag];
		b->data = kzalloc(s->fragsize, GFP_KERNEL);
		if (!b->data)
			goto err;
		/* handle buffer pointers */
		b->offset = 0;
	}

	/* Our actual DMA buffer
	 * Let's allocate non-cached memory for DMA buffers.
	 */
	dma_buf = dma_alloc_coherent(NULL, s->fragsize * AUDIO_NBFRAGS_DMA, &dma_buf_phys, 0);
	if (!dma_buf)
		goto err;
	
	memzero(dma_buf, s->fragsize * AUDIO_NBFRAGS_DMA);
	s->dma_buf.vaddr = dma_buf;
	s->dma_buf.dma_addr = dma_buf_phys;

	// Reconfigure the DMA to reflect the DMA physical address and buffer size
	audio_configure_dma_channel(s);

#ifdef DEBUG 
	printk(KERN_ALERT "%s: s=%p\n", __FUNCTION__, s);
	printk(KERN_ALERT "%s: s->user_buf=%p, s->dma_buf.vaddr=%p, s->dma_buf.dma_addr=0x%08x, s->fragsize=0x%08x\n",__FUNCTION__, s->user_buf, s->dma_buf.vaddr, s->dma_buf.dma_addr, s->fragsize);

	for (frag = 0; frag < s->nbfrags; frag++) {
		printk(KERN_ALERT "%s: s->user_buf[%d].data=0x%08x\n", __FUNCTION__, frag, *(s->user_buf[frag].data));
	}
#endif

	/* Init stream struct */
	s->usr_frag = s->dma_frag = 0;
	s->dma_running = 0;
	s->bytecount = 0;
	s->fragcount = 0;
	sema_init(&s->sem, (s->output) ? s->nbfrags-1 : 0);
	return 0;

err:
	printk(KERN_ALERT "omap-audio: unable to allocate audio memory\n ");
	audio_clear_buf(s);
	return -ENOMEM;
}

/*
 * DMA interrupt handler
 */
static void audio_dma_irq(int lch, u16 ch_status, void *data)
{
	audio_stream_t *s = (audio_stream_t *)data;
	int ch;
	
	ch = s->dma_ch;
	printk(KERN_ALERT "DMA IRQ retrieved, ch_status %x\n", ch_status);
	
	if (!s->user_buf) {
		printk(KERN_ALERT"OMAP730 AUDIO DMA: wow... received IRQ for channel %d but no buffer exists\n", ch);
		return;
	}

	// Process the half frame interrupt:
	//  now the first half of the buffer is not in use by the DMA anymore
	if (ch_status & OMAP_DMA_HALF_IRQ) {
		printk(KERN_ALERT "DMA OMAP_DMA_HALF_IRQ retrieved\n");
		if (s->output)	{
			// Check if we have free buffers
			if (USER_BUF_IS_EMPTY(s)) {
				printk(KERN_ALERT "USER_BUF_IS_EMPTY\n");
				stop_dma(s);
			} else {
				// Copy the next user buffer to the first half of the DMA buffer
				memcpy(s->dma_buf.vaddr, s->user_buf[s->dma_frag].data, s->fragsize);

				if (++s->dma_frag >= s->nbfrags)
					s->dma_frag = 0;
			}
		} else {
			// Check if we have free buffers
			if (!USER_BUF_READ_HAS_PLACE(s)) {
				stop_dma(s);
			} else {
				// Copy the first half of the DMA buffer to the next user buffer
				memcpy(s->user_buf[s->dma_frag].data, s->dma_buf.vaddr, s->fragsize);

				if (++s->dma_frag >= s->nbfrags)
					s->dma_frag = 0;
			}
		}

		// Wake-up waiting processes
		/*FIXME: where is down called ? */
		if (!s->mapped)
			up(&s->sem);

		/* Accounting */
		s->bytecount += s->fragsize;
		s->fragcount++;

		/* ... and for polling processes */
		wake_up(&s->frag_wq);
	}

	// Process the end of frame interrupt:
	//  now the second half of the buffer is not in use by the DMA anymore
	if (ch_status & OMAP_DMA_FRAME_IRQ) {
		printk(KERN_ALERT "DMA OMAP_DMA_FRAME_IRQ retrieved\n");
		if (s->output)	{
			// Check if we have free buffers
			if (USER_BUF_IS_EMPTY(s)) {
				stop_dma(s);
			} else {
				// Copy the next user buffer to the second half of the DMA buffer
				memcpy(s->dma_buf.vaddr + s->fragsize, s->user_buf[s->dma_frag].data, s->fragsize);

				if (++s->dma_frag >= s->nbfrags)
					s->dma_frag = 0;
			}
		} else {
			if (!USER_BUF_READ_HAS_PLACE(s)) {
				stop_dma(s);
			} else {
				// Copy the first half of the DMA buffer to the next user buffer
				// Copy the second half of the DMA buffer to the next user buffer
				memcpy(s->user_buf[s->dma_frag].data, s->dma_buf.vaddr + s->fragsize, s->fragsize);

				if (++s->dma_frag >= s->nbfrags)
					s->dma_frag = 0;
			}
		}

		// Wake-up waiting processes
		if (!s->mapped)
			up(&s->sem);

		/* Accounting */
		s->bytecount += s->fragsize;
		s->fragcount++;

		/* ... and for polling processes */
		wake_up(&s->frag_wq);
	}
}

/*
 * Validate and sets up buffer fragments, etc.
 */
static int audio_set_fragments(audio_stream_t *s, int val)
{
	if (s->mapped || s->dma_running)
		return -EBUSY;
	if (s->user_buf)
		audio_clear_buf(s);
	s->nbfrags = (val >> 16) & 0x7FFF;
	val &= 0xffff;
	if (val < 5)
		val = 5;
	if (val > 15)
		val = 15;
	s->fragsize = 1 << val;
	if (s->nbfrags < 2)
		s->nbfrags = 2;
	if (s->nbfrags * s->fragsize > 256 * 1024)
		s->nbfrags = 256 * 1024 / s->fragsize;
	if (audio_setup_buf(s))
		return -ENOMEM;
	return val|(s->nbfrags << 16);
}


/*
 * The fops functions
 */

static int audio_write(struct file *file, const char *buffer,
		       size_t count, loff_t * ppos)
{
	const char *buffer0 = buffer;
	audio_state_t *state = (audio_state_t *)file->private_data;
	audio_stream_t *s = state->output_stream;
	int chunksize, ret = 0;
        unsigned long flags;

	
	printk(KERN_ALERT "%s count=%d\n", __FUNCTION__, count);
	
	if (*ppos != file->f_pos) {
		printk(KERN_ALERT"FPOS not ppos ppos=0x%x fpos =0x%x\n", (u32) * ppos,
		       (u32) file->f_pos);
		return -ESPIPE;
	}
	
	if (s->mapped)
		return -ENXIO;
	if (!s->user_buf && audio_setup_buf(s))
		return -ENOMEM;

	while (count > 0) {
		audio_user_buf_t *b = &s->user_buf[s->usr_frag];

		/* Grab a fragment */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (down_interruptible(&s->sem))
				break;
		}
		/* Feed the current buffer */
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
		if (copy_from_user(b->data + b->offset, buffer, chunksize)) {
			up(&s->sem);
			return -EFAULT;
		}
		b->offset += chunksize;
		buffer += chunksize;
		count -= chunksize;
		if (b->offset < s->fragsize) {
			up(&s->sem);
			break;
		}

		spin_lock_irqsave(&bufindex_lock, flags);
		/* move the index to the next fragment */
		if (++s->usr_frag >= s->nbfrags)
			s->usr_frag = 0;
		spin_unlock_irqrestore(&bufindex_lock, flags);

		/* 
		 * Activate DMA on current buffer.
		 */
		b->offset = 0;
		if (!s->dma_running) {
			start_dma(s);
		}
	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;
	return ret;
}


static int audio_read(struct file *file, char *buffer,
		      size_t count, loff_t * ppos)
{
	char *buffer0 = buffer;
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->input_stream;
	int chunksize, ret = 0;
        unsigned long flags;


	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (s->mapped)
		return -ENXIO;
	if (!s->user_buf && audio_setup_buf(s))
		return -ENOMEM;

	while (count > 0) {
		audio_user_buf_t *b = &s->user_buf[s->usr_frag];

		/* prime DMA */
		if (!s->dma_running) {
			start_dma(s);
		}

		/* Wait for a buffer to become full */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (down_interruptible(&s->sem))
				break;
		}

		/* Grab data from current buffer */
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
		if (copy_to_user(buffer, b->data + b->offset, chunksize)) {
			up(&s->sem);
			return -EFAULT;
		}
		b->offset += chunksize;
		buffer += chunksize;
		count -= chunksize;
		if (b->offset < s->fragsize) {
			up(&s->sem);
			break;
		}

		/* 
		 * Make this buffer available for DMA again.
		 */
		b->offset = 0;

		/* move the index to the next fragment */
		spin_lock_irqsave(&bufindex_lock, flags);
		if (++s->usr_frag >= s->nbfrags)
			s->usr_frag = 0;
		spin_unlock_irqrestore(&bufindex_lock, flags);
	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;
	return ret;
}


static int audio_sync(struct file *file)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->output_stream;
	audio_user_buf_t *b;
	DECLARE_WAITQUEUE(wait, current);
	int pad_size;
        unsigned long flags;


	if (!(file->f_mode & FMODE_WRITE) || !s->user_buf || s->mapped)
		return 0;

	/*
	 * Send current buffer if it contains data.  Be sure to send
	 * a full sample count. As we have to work with fixed size
	 * buffers, just pad ... Ark too bad !
	 *
	 * Note that this is bad for really short samples (beep beep).
	 */
	b = &s->user_buf[s->usr_frag];
	pad_size = s->fragsize - b->offset;

	if (pad_size <= 0)
		return 0;

	memset(b->data + b->offset, AUDIO_SILENCE, pad_size);

	spin_lock_irqsave(&bufindex_lock, flags);
	/* move the index to the next fragment */
	if (++s->usr_frag >= s->nbfrags)
		s->usr_frag = 0;
	spin_unlock_irqrestore(&bufindex_lock, flags);

	if (!s->dma_running) {
		start_dma(s);
	}

	/* Wait for DMA to complete. */
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&s->frag_wq, &wait);
	max_dma_count = s->nbfrags;
	while (s->dma_running && !signal_pending(current) && (max_dma_count--)) {
		schedule_timeout(10);
		set_current_state(TASK_INTERRUPTIBLE);
	}

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&s->frag_wq, &wait);

	return 0;
}


static unsigned int audio_poll(struct file *file,
			       struct poll_table_struct *wait)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *is = state->input_stream;
	audio_stream_t *os = state->output_stream;
	unsigned int mask = 0;

	if (file->f_mode & FMODE_READ) {
		/* Start audio input if not already active */
		if (!is->user_buf && audio_setup_buf(is))
			return -ENOMEM;
		if (!is->dma_running) {
			start_dma(is);
		}
		poll_wait(file, &is->frag_wq, wait);
	}

	if (file->f_mode & FMODE_WRITE) {
		if (!os->user_buf && audio_setup_buf(os))
			return -ENOMEM;
		poll_wait(file, &os->frag_wq, wait);
	}

	if (file->f_mode & FMODE_READ)
		if (( is->mapped && is->bytecount > 0) ||
		    (!is->mapped && atomic_read(&is->sem.count) > 0))
			mask |= POLLIN | POLLRDNORM;

	if (file->f_mode & FMODE_WRITE)
		if (( os->mapped && os->bytecount > 0) ||
		    (!os->mapped && atomic_read(&os->sem.count) > 0))
			mask |= POLLOUT | POLLWRNORM;

	return mask;
}


static int audio_ioctl( struct inode *inode, struct file *file,
			uint cmd, ulong arg)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	long val;

	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_GETBLKSIZE:
		if (file->f_mode & FMODE_WRITE)
			return put_user(os->fragsize, (int *)arg);
		else
			return put_user(is->fragsize, (int *)arg);

	case SNDCTL_DSP_GETCAPS:
		val = DSP_CAP_REALTIME|DSP_CAP_TRIGGER|DSP_CAP_MMAP;
		if (is && os)
			val |= DSP_CAP_DUPLEX;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETFRAGMENT:
		if (get_user(val, (long *) arg))
			return -EFAULT;
		if (file->f_mode & FMODE_READ) {
			int ret = audio_set_fragments(is, val);
			if (ret < 0)
				return ret;
			ret = put_user(ret, (int *)arg);
			if (ret)
				return ret;
		}
		if (file->f_mode & FMODE_WRITE) {
			int ret = audio_set_fragments(os, val);
			if (ret < 0)
				return ret;
			ret = put_user(ret, (int *)arg);
			if (ret)
				return ret;
		}
		return 0;

	case SNDCTL_DSP_SYNC:
		return audio_sync(file);

	case SNDCTL_DSP_SETDUPLEX:
		return 0;

	case SNDCTL_DSP_POST:
		return 0;

	case SNDCTL_DSP_GETTRIGGER:
		val = 0;
		if (file->f_mode & FMODE_READ && !USER_BUF_IS_EMPTY(is))
			val |= PCM_ENABLE_INPUT;
		if (file->f_mode & FMODE_WRITE && !USER_BUF_IS_EMPTY(os))
			val |= PCM_ENABLE_OUTPUT;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETTRIGGER:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (file->f_mode & FMODE_READ) {
			if (val & PCM_ENABLE_INPUT) {
				if (!is->user_buf && audio_setup_buf(is))
					return -ENOMEM;
				if (!is->dma_running)
					start_dma(is);
			} else {
				stop_dma(is);
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			if (val & PCM_ENABLE_OUTPUT) {
				if (!os->user_buf && audio_setup_buf(os))
					return -ENOMEM;
				if (!os->dma_running)
					start_dma(os);
			} else {
				stop_dma(os);
			}
		}
		return 0;

	case SNDCTL_DSP_GETOSPACE:
	case SNDCTL_DSP_GETISPACE:
	    {
		audio_buf_info inf = { 0, };
		audio_stream_t *s = (cmd == SNDCTL_DSP_GETOSPACE) ? os : is;

		if ((s == is && !(file->f_mode & FMODE_READ)) ||
		    (s == os && !(file->f_mode & FMODE_WRITE)))
			return -EINVAL;
		if (!s->user_buf && audio_setup_buf(s))
			return -ENOMEM;
		inf.bytes = atomic_read(&s->sem.count) * s->fragsize;
		inf.bytes -= s->user_buf[s->usr_frag].offset;
		inf.fragments = inf.bytes / s->fragsize;
		inf.fragsize = s->fragsize;
		inf.fragstotal = s->nbfrags;
		return copy_to_user((void *)arg, &inf, sizeof(inf));
	    }

	case SNDCTL_DSP_GETOPTR:
	case SNDCTL_DSP_GETIPTR:
	    {
		count_info inf = { 0, };
		audio_stream_t *s = (cmd == SNDCTL_DSP_GETOPTR) ? os : is;
		dma_addr_t ptr;
		int bytecount, offset;
		unsigned long flags;

		if ((s == is && !(file->f_mode & FMODE_READ)) ||
		    (s == os && !(file->f_mode & FMODE_WRITE)))
			return -EINVAL;
		if (s->dma_running) {
			local_irq_save(flags);
			ptr = (s->output) ? OMAP_DMA_CDAC_REG(s->dma_ch) : OMAP_DMA_CSAC_REG(s->dma_ch);
			offset = ptr - s->dma_buf.dma_addr;
			if (offset >= s->fragsize)
				offset = s->fragsize - 4;
		} else {
			local_save_flags(flags);
			offset = 0;
		}
		inf.ptr = s->dma_frag * s->fragsize + offset;
		bytecount = s->bytecount + offset;
		s->bytecount = -offset;
		inf.blocks = s->fragcount;
		s->fragcount = 0;

		local_save_flags(flags);
		if (bytecount < 0)
			bytecount = 0;
		inf.bytes = bytecount;
		return copy_to_user((void *)arg, &inf, sizeof(inf));
	    }

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		return 0;

	case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_WRITE) 
			audio_clear_buf(os);
		if (file->f_mode & FMODE_READ)
			audio_clear_buf(is);
		return 0;

	default:
		return state->client_ioctl(inode, file, cmd, arg);
	}

	return 0;
}


static int audio_mmap(struct file *file, struct vm_area_struct *vma)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *s;
	unsigned long size, vma_addr;

	int i, ret;

	if (vma->vm_pgoff != 0)
		return -EINVAL;

	if (vma->vm_flags & VM_WRITE) {
		if (!state->wr_ref)
			return -EINVAL;;
		s = state->output_stream;
	} else if (vma->vm_flags & VM_READ) {
		if (!state->rd_ref)
			return -EINVAL;
		s = state->input_stream;
	} else return -EINVAL;

	if (s->mapped)
		return -EINVAL;
	size = vma->vm_end - vma->vm_start;
	if (size != s->fragsize * s->nbfrags)
		return -EINVAL;
	if (!s->user_buf && audio_setup_buf(s))
		return -ENOMEM;
	vma_addr = vma->vm_start;
	for (i = 0; i < s->nbfrags; i++) {
		audio_user_buf_t *buf = &s->user_buf[i];
		if (!buf->data)
			continue;

		ret = remap_pfn_range(vma, vma_addr, s->dma_buf.dma_addr,
				       s->fragsize * AUDIO_NBFRAGS_DMA, vma->vm_page_prot);
		if (ret)
			return ret;
		vma_addr += s->fragsize * AUDIO_NBFRAGS_DMA;
	}
	// Start DMA here ???
	//for (i = 0; i < s->nbfrags; i++)
	//	s->user_buf[i].dma_desc->ddadr &= ~DDADR_STOP;
	s->mapped = 1;
	return 0;
}

#ifdef CONFIG_PM
static int audio_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void
*data)
{
#if 0
        audio_state_t *state = pm_dev->data;
        audio_stream_t *is = state->input_stream;
        audio_stream_t *os = state->output_stream;

        FN_IN;
        switch (req) {
        case PM_SUSPEND: /* enter D1-D3 */
                if (is) {
			stop_dma(is);
                }
                if (os) {
			stop_dma(os);
                }
                if (state->hw_shutdown)
                        state->hw_shutdown(state);
                break;
        case PM_RESUME:  /* enter D0 */
                if (state->hw_init)
                        state->hw_init(state);
                if (os) {
			start_dma(os);
                }
                if (is) {
			start_dma(is);
                }
                break;
        }
        FN_OUT(0);
#endif
        return 0;
}

#endif
/* Power Management Functions for Linux Device Model  */

void audio_ldm_suspend(void *data)
{
#if 0
        audio_state_t *state = data;
        audio_stream_t *is = state->input_stream;
        audio_stream_t *os = state->output_stream;

        if (is)
        {
		stop_dma(is);
        }
        if (os)
        {
		stop_dma(os);
        }
        if (state->hw_shutdown)
                state->hw_shutdown(state);
#endif
}

void audio_ldm_resume(void *data)
{
#if 0
        audio_state_t *state = data;
        audio_stream_t *is = state->input_stream;
        audio_stream_t *os = state->output_stream;

        if (state->hw_init)
                state->hw_init(state);

        if (os)
        { 
		start_dma(os);
        }
        if (is)
        {
		start_dma(is);
        }
#endif
}

static int audio_release(struct inode *inode, struct file *file)
{
	audio_state_t *state = file->private_data;

	down(&state->sem);

	if (file->f_mode & FMODE_READ) {
		stop_dma(state->input_stream);
		audio_clear_buf(state->input_stream);
		omap_free_dma(state->input_stream->dma_ch);
		state->rd_ref = 0;
	}

	if (file->f_mode & FMODE_WRITE) {
		//audio_sync(file);
		audio_clear_buf(state->output_stream);
		omap_free_dma(state->output_stream->dma_ch);
		state->wr_ref = 0;
	}

	if ((state->rd_ref == 0) && (state->wr_ref == 0)) {
		if (state->hw_shutdown)
			state->hw_shutdown(state);
	}

	up(&state->sem);
	return 0;
}

int omap_audio_attach(struct inode *inode, struct file *file,
			 audio_state_t *state)
{
	audio_stream_t *is = state->input_stream;
	audio_stream_t *os = state->output_stream;
	int dma_ch;
	int err;
	int dma_regs;

	down(&state->sem);

	/* access control */
	err = -ENODEV;
	if ((file->f_mode & FMODE_WRITE) && !os)
		goto out;
	if ((file->f_mode & FMODE_READ) && !is)
		goto out;
	err = -EBUSY;
	if ((file->f_mode & FMODE_WRITE) && (state->wr_ref || state->rd_ref))
		goto out;
	if ((file->f_mode & FMODE_READ) && (state->rd_ref || state->wr_ref))
		goto out;

	/* request DMA channel */
	if (file->f_mode & FMODE_WRITE) {
		err = omap_request_dma(OMAP850_DMA_EAC_PLAY, "Audio TX DMA", audio_dma_irq, os, &dma_regs);
		dma_ch = dma_regs;
		if (err < 0)
			goto out;
		os->dma_ch = dma_ch;
		printk(KERN_ALERT "DMA channel output stream = %d\n", dma_ch);
	}
	if (file->f_mode & FMODE_READ) {
		err = omap_request_dma(OMAP850_DMA_EAC_REC, "Audio RX DMA", audio_dma_irq, is, &dma_regs);
		dma_ch = dma_regs;
		if (err < 0) {
			if (file->f_mode & FMODE_WRITE) {
				omap_free_dma(os->dma_ch);
			}
			goto out;
		}
		is->dma_ch = dma_ch;
		printk(KERN_ALERT "DMA channel input stream = %d\n", dma_ch);
	}

	file->private_data	= state;

	if ((file->f_mode & FMODE_WRITE)) {
		state->wr_ref = 1;
		os->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		os->nbfrags = AUDIO_NBFRAGS_USER_DEFAULT;
		os->output = 1;
		os->mapped = 0;
		init_waitqueue_head(&os->frag_wq);
	}
	if (file->f_mode & FMODE_READ) {
		state->rd_ref = 1;
		is->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		is->nbfrags = AUDIO_NBFRAGS_USER_DEFAULT;
		is->output = 0;
		is->mapped = 0;
		init_waitqueue_head(&is->frag_wq);
	}

	if ((!(file->f_mode & FMODE_WRITE)) && (!(file->f_mode & FMODE_READ))) {
		err = -ENODEV;
		goto out;
        }

	if (state->hw_init)
		state->hw_init(state);

	err = 0;

out:
	up(&state->sem);
	return err;
}


/* File Ops structure */
static struct file_operations omap850_audio_fops = {
	/*.open will be used in main driver */
	.release	= audio_release,
	.write		= audio_write,
	.read		= audio_read,
	.mmap		= audio_mmap,
	.poll		= audio_poll,
	.ioctl		= audio_ioctl,
	.owner		= THIS_MODULE
};

/* return file_operations structure */
struct file_operations *omap_audio_get_fops(void)
{
	return &omap850_audio_fops;
}

MODULE_LICENSE("GPL");

EXPORT_SYMBOL(omap_audio_get_fops);
EXPORT_SYMBOL(omap_audio_attach);
EXPORT_SYMBOL(omap_audio_clear_buf);
EXPORT_SYMBOL(audio_ldm_resume);
EXPORT_SYMBOL(audio_ldm_suspend);
