/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * drivers/analog/adc.c
 *
 *   Copyright (C) 2008-2009, 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from drivers/can.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/fs/fs.h>
#include <tinyara/arch.h>
#include <tinyara/kmalloc.h>
#include <tinyara/semaphore.h>
#include <tinyara/analog/adc.h>

#include <tinyara/irq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int     adc_open(FAR struct file *filep);
static int     adc_close(FAR struct file *filep);
static ssize_t adc_read(FAR struct file *fielp, FAR char *buffer,
			size_t buflen);
static int     adc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int     adc_receive(FAR struct adc_dev_s *dev, uint8_t ch,
			   int32_t data);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct file_operations g_adc_fops = {
	adc_open,	/* open */
	adc_close,	/* close */
	adc_read,	/* read */
	0,		/* write */
	0,		/* seek */
	adc_ioctl,	/* ioctl */
#ifndef CONFIG_DISABLE_POLL
	0,		/* poll */
#endif
};

static const struct adc_callback_s g_adc_callback = {
	.au_receive = adc_receive
};

typedef struct adc_client_state_s {
  int               read_idx;    /* zero based sample no. */
  adc_read_policy_t read_policy; /* oldest, newest */
  //adc_data_type_t   data_type; /* {s,u}{8,16,32} */
  struct adc_msg_s  last_sample; /* */
  int               bytes_left;  /* how much data is left to read from last_sample */
} adc_client_state_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static adc_client_state_t *adc_create_clientstate(
    int read_index, adc_read_policy_t read_policy) {
  adc_client_state_t *client = (adc_client_state_t *)kmm_malloc(sizeof(adc_client_state_t));
  client->read_idx = read_index;
  client->read_policy = read_policy;
  client->bytes_left = 0;
  return client;
}

/****************************************************************************
 * Name: adc_open
 *
 * Description:
 *   This function is called whenever the ADC device is opened.
 *
 ****************************************************************************/
static int adc_open(FAR struct file *filep)
{
	FAR struct inode     *inode = filep->f_inode;
	FAR struct adc_dev_s *dev   = inode->i_private;
	uint8_t               tmp;
	int                   ret   = OK;

	/*
	 * If the port is the middle of closing, wait until the close is
	 * finished
	 */
	if (sem_wait(&dev->ad_closesem) != OK) {
		ret = -errno;
	} else {
		/*
		 * Increment the count of references to the device.  If this
		 * the first time that the driver has been opened for this
		 * device, then initialize the device.
		 */
		tmp = dev->ad_ocount + 1;
		if (tmp == 0) {
			/* More than 255 opens; uint8_t overflows to zero */
			ret = -EMFILE;
		} else {
			/*
			 * Check if this is the first time that the driver
			 * has been opened.
			 */
			if (tmp == 1) {
				/*
				 * Yes.. perform one time hardware
				 * initialization.
				 */
				irqstate_t flags = irqsave();
				ret = dev->ad_ops->ao_setup(dev);
				if (ret == OK) {
					/* Mark the FIFOs empty */
					dev->ad_fifo.head = 0;

					/*
					 * Finally, Enable the ADC RX
					 * interrupt
					 */
					dev->ad_ops->ao_rxint(dev, true);

					/* Save the new open count on success */
					dev->ad_ocount = tmp;
				}

				irqrestore(flags);
			}
            if (ret == OK) {
              // TODO: handle multiple WROK opens, maybe (how device control behaves?)
              if (filep->f_oflags & O_RDOK) {
                int oldest_sample_index = 0;
                if (dev->ad_fifo.head >= CONFIG_ADC_FIFOSIZE) {
                  oldest_sample_index = dev->ad_fifo.head - CONFIG_ADC_FIFOSIZE + 1;
                }
                filep->f_priv = adc_create_clientstate(
                    oldest_sample_index, ADC_READ_OLDEST);
                // TODO: handle error[s]
              } else {
                filep->f_priv = NULL;
              }
            }
		}

		sem_post(&dev->ad_closesem);
	}

	return ret;
}

/****************************************************************************
 * Name: adc_close
 *
 * Description:
 *   This routine is called when the ADC device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/
static int adc_close(FAR struct file *filep)
{
	FAR struct inode     *inode = filep->f_inode;
	FAR struct adc_dev_s *dev   = inode->i_private;
	irqstate_t            flags;
	int                   ret = OK;

	if (sem_wait(&dev->ad_closesem) != OK) {
		ret = -errno;
	} else {
		/*
		 * Decrement the references to the driver.  If the reference
		 * count will decrement to 0, then uninitialize the driver.
		 */
		if (dev->ad_ocount > 1) {
			dev->ad_ocount--;
			sem_post(&dev->ad_closesem);
		} else {
			/* There are no more references to the port */
			dev->ad_ocount = 0;

			/* Free the IRQ and disable the ADC device */
			flags = irqsave(); /* Disable interrupts */
			dev->ad_ops->ao_shutdown(dev); /* Disable the ADC */
			irqrestore(flags);

			sem_post(&dev->ad_closesem);
		}
        if (filep->f_priv) {
            kmm_free(filep->f_priv);
        }
	}

	return ret;
}

static int adc_waitfor_samples(FAR struct adc_dev_s *dev,
                               int blocking,
                               FAR adc_client_state_t *client) {
  while (client->read_idx > dev->ad_fifo.head) {
    if (!blocking) {
      return -EAGAIN;
    } else {
      /* Wait for a message to be received */
      ++dev->ad_nrxwaiters;
      int ret = sem_wait(&dev->ad_fifo.sem);
      --dev->ad_nrxwaiters;
      if (ret < 0) {
        return -errno;
      }
    }
  }
  return 0;
}

static ssize_t adc_partial_read(FAR adc_client_state_t *client,
                                FAR char *buffer, size_t buflen) {
  const uint8_t *puc = (const uint8_t *)&client->last_sample;
  size_t offset = sizeof(client->last_sample) - client->bytes_left;
  size_t chunk = client->bytes_left < buflen ? client->bytes_left : buflen;
  memcpy(buffer, puc + offset, chunk);
  client->bytes_left -= chunk;
  return buflen;
}

static ssize_t adc_read_samples(FAR struct adc_fifo_s *fifo,
                                FAR adc_client_state_t *client,
                                char *buffer,
                                size_t buflen) {
  int requested_samples = buflen / sizeof(struct adc_msg_s);
  if (requested_samples > fifo->head - client->read_idx) {
    // There are less samples than requested, clip the request.
    requested_samples = fifo->head - client->read_idx;
  }
  if (requested_samples == 0) return 0;

  int index = client->read_idx % CONFIG_ADC_FIFOSIZE;
  int sample_count = 0;
  while (sample_count++ < requested_samples) {
    memcpy(buffer, &fifo->samples[index], sizeof(struct adc_msg_s));
    buffer += sizeof(struct adc_msg_s);
    buflen -= sizeof(struct adc_msg_s);
    if (++index == CONFIG_ADC_FIFOSIZE) index = 0;
  }
  client->read_idx += sample_count;
  return sample_count * sizeof(struct adc_msg_s);
}

/****************************************************************************
 * Name: adc_read
 ****************************************************************************/
static ssize_t adc_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
    FAR adc_client_state_t *client = filep->f_priv;
	FAR struct inode     *inode = filep->f_inode;
	FAR struct adc_dev_s *dev   = inode->i_private;
	irqstate_t            flags;
	int                   ret   = 0;

    if (!buflen) return 0;
    if (!buffer) return -EFAULT;
    if (!client) return -EINVAL;

	avdbg("buflen: %d\n", (int)buflen);
    // In case client did a partial read before, continue where they left.
    if (client->bytes_left > 0) {
      return adc_partial_read(client, buffer, buflen);
    }

    flags = irqsave();
    // Wait for the queue to have some data for us
    ret = adc_waitfor_samples(
        dev, ((filep->f_oflags) & O_NONBLOCK) == 0, client);
    if (ret < 0) {
      goto return_with_irqdisabled;
    }

    // Adjust client index; it might have fallen behind the window.
    if (client->read_idx < dev->ad_fifo.head - CONFIG_ADC_FIFOSIZE + 1) {
      client->read_idx = dev->ad_fifo.head - CONFIG_ADC_FIFOSIZE + 1;
    }

    // Now read as many samples as the client requested
    // (but not more than the queue holds).
    ret = adc_read_samples(&dev->ad_fifo, client, buffer, buflen);

    if (ret == 0) {
      // Partial read. Copy the sample into client state first.
      int index = client->read_idx % CONFIG_ADC_FIFOSIZE;
      memcpy(&client->last_sample, &dev->ad_fifo.samples[index],
          sizeof(struct adc_msg_s));
      client->bytes_left = sizeof(struct adc_msg_s);
      ++client->read_idx;
      ret = adc_partial_read(client, buffer, buflen);
    }

return_with_irqdisabled:
	irqrestore(flags);

	avdbg("Returning: %d\n", ret);
	return ret;
}

/****************************************************************************
 * Name: adc_ioctl
 ****************************************************************************/
static int adc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode *inode = filep->f_inode;
	FAR struct adc_dev_s *dev = inode->i_private;
	int ret;

    // TODO: handle data format/policy.
	ret = dev->ad_ops->ao_ioctl(dev, cmd, arg);
	return ret;
}

/****************************************************************************
 * Name: adc_receive
 ****************************************************************************/
static int adc_receive(FAR struct adc_dev_s *dev, uint8_t ch, int32_t data)
{
	FAR struct adc_fifo_s *fifo = &dev->ad_fifo;

    // Add the sample at the head of the queue.
    // Do not worry about overwriting previous samples, as we'd
    // rather produce more up-to-date samples than  stall the capturing.

    int index = fifo->head % CONFIG_ADC_FIFOSIZE;

	irqstate_t flags = irqsave();

    fifo->samples[index].am_channel = ch;
    fifo->samples[index].am_data = data;
    ++fifo->head;

	irqrestore(flags);

	if (dev->ad_nrxwaiters > 0) {
	  sem_post(&fifo->sem);
	}
	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_register
 ****************************************************************************/
int adc_register(FAR const char *path, FAR struct adc_dev_s *dev)
{
	int ret;

	DEBUGASSERT(path != NULL && dev != NULL);

	/* Bind the upper-half callbacks to the lower half ADC driver */
	DEBUGASSERT(dev->ad_ops != NULL && dev->ad_ops->ao_bind != NULL);
	ret = dev->ad_ops->ao_bind(dev, &g_adc_callback);
	if (ret < 0) {
		avdbg("ERROR: Failed to bind callbacks: %d\n", ret);
		return ret;
	}

	/* Initialize the ADC device structure */
	dev->ad_ocount = 0;

	/* Initialize semaphores */
	sem_init(&dev->ad_fifo.sem, 0, 0);
	sem_init(&dev->ad_closesem, 0, 1);

	/*
	 * The receive semaphore is used for signaling and, hence,
	 * should not have priority inheritance enabled.
	 */
	sem_setprotocol(&dev->ad_fifo.sem, SEM_PRIO_NONE);

	/* Reset the ADC hardware */
	DEBUGASSERT(dev->ad_ops->ao_reset != NULL);
	dev->ad_ops->ao_reset(dev);

	/* Register the ADC character driver */
	ret = register_driver(path, &g_adc_fops, 0444, dev);
	if (ret < 0) {
		sem_destroy(&dev->ad_fifo.sem);
		sem_destroy(&dev->ad_closesem);
	}

	return ret;
}
