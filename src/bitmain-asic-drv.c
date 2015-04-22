/*
 *  Bitmain asic driver
 *
 *  Copyright (C) 2007-2008 Gabor Juhos <juhosg@openwrt.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 * ---------------------------------------------------------------------------
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/workqueue.h>


#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/mach-ath79/ar71xx_regs.h>
#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/irq.h>
#include "bitmain-asic.h"
#include "sha2.h"

#define DRV_NAME	"bitmain-asic"
#define DRV_DESC	"Bitmain asic driver"
#define DRV_VERSION	"0.1.1"

#define	PRNT_TIMER_EN	0


#define	TIMER_NUM	2
#define TIMER_INTERRUPT		(15 + TIMER_NUM)

#define RESET_BASE	(0x18000000 + 0x00060000)
#define RESET_SIZE	0x100

#define GENERAL_TIMER	(0x94+8*(TIMER_NUM-1))
#define GENERAL_TIMER_RELOAD	(GENERAL_TIMER + 0x04)
//AHB 时钟200 MHz
#define	TIME_40MS			(40*200*1000)
#define TASK_BUFFER_NUMBER	100
//防止full，预留位置，存储已发送的上几个数据
#define TASK_PRE_LEFT		5
extern struct file_operations *bitmain_fops_p;

#if PRNT_TIMER_EN
static struct timer_list prnt_timer;
#endif

typedef struct
{
	uint8_t	core_num;
	uint8_t	asic_num;
	uint8_t	fan_pwm_data;
	uint8_t	timeout_data;
	uint16_t frequency;

	uint8_t	voltage;
	uint8_t	chain_check_time;
	uint8_t	chip_address;
	uint8_t	reg_address;
}ASIC_CONFIGURE;
typedef struct __BT_AS_info
{
	spinlock_t			lock;
	struct mutex		result_lock;
	void __iomem		*virt_addr;
	unsigned			irq;
	struct workqueue_struct *usb_wq;
	struct work_struct usb_sdata_work;
	struct delayed_work usb_rdata_work;
	ASIC_TASK_P			task_buffer;
	unsigned char		task_last_num;
	unsigned char		task_current_num;
	unsigned int		task_buffer_size;
	unsigned int		task_buffer_wr;
	unsigned int		task_buffer_rd;
	bool				task_buffer_full;
	bool				usb_opened;
	bool				get_status;
	ASIC_CONFIGURE	asic_configure;
	struct BITMAIN_STATUS_DATA asic_status_data;
}*BT_AS_INFO;

static struct __BT_AS_info bitmain_asic_dev;
struct inode bitmain_inode;
struct file bitmain_file;

static unsigned long bitmain_is_open;

uint8_t asic_return_bytes = 4;
// --------------------------------------------------------------
//      CRC16 check table
// --------------------------------------------------------------
const uint8_t chCRCHTalbe[] =                                 // CRC high byte table
{
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40
};

const uint8_t chCRCLTalbe[] =                                 // CRC low byte table
{
 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
 0x41, 0x81, 0x80, 0x40
};
uint16_t CRC16(const uint8_t* p_data, uint16_t w_len)
{
	uint8_t chCRCHi = 0xFF; // CRC high byte initialize
	uint8_t chCRCLo = 0xFF; // CRC low byte initialize
	uint16_t wIndex = 0;    // CRC cycling index

	while (w_len--) {
		wIndex = chCRCLo ^ *p_data++;
		chCRCLo = chCRCHi ^ chCRCHTalbe[wIndex];
		chCRCHi = chCRCLTalbe[wIndex];
	}
	return ((chCRCHi << 8) | chCRCLo);
}

#if PRNT_TIMER_EN
static void Prnt(unsigned long data)
{
	BT_AS_INFO dev = &bitmain_asic_dev;
	printk("Prnt general time = {%u}\n", readl(dev->virt_addr + GENERAL_TIMER));
	prnt_timer.expires = jiffies + 30000*HZ/1000;//30s
	add_timer(&prnt_timer);
	//bitmain_asic_open_usb(dev);
}
#endif

int cmd_check(uint8_t *data)//OK 1; failure 0
{
	BITMAIN_TASK_P bt = (BITMAIN_TASK_P)data;
	uint16_t r_crc = 0;
	uint16_t crc = bt->crc;
	uint16_t length = 0;
	if(bt->token_type == BM_TX_TASK)
	{
		#if HAVE_NUM
		uint16_t len = 4 + bt->work_num*sizeof(struct ASIC_TASK);
		#else
		uint16_t len = le16_to_cpu(bt->length)+ 2;
		#endif
		//crc = data[bt->length] | (data[bt->length+1]<<8);
		crc = data[len] | (data[len+1]<<8);
		length = le16_to_cpu(bt->length) + 2;
	}
	else if(bt->token_type == BM_TX_CONF)
	{
		BITMAIN_CONFIGURE_P btc = (BITMAIN_CONFIGURE_P)data;
		length = btc->length;
		crc = cpu_to_le16(btc->crc);
	}
	else if(bt->token_type == BM_GET_STATUS)
	{
		BITMAIN_GET_STATUS_P bgs = (BITMAIN_GET_STATUS_P)data;
		length = bgs->length;
		crc = cpu_to_le16(bgs->crc);
	}
	else
	{
		printk("Tx token err {%#x}\n", bt->token_type);
		return 0;
	}
	if(crc == (r_crc=CRC16(data, length)))//length 去除了type和length
	{
		//rintf("OK: crc{%#x}r_crc{%#x}\n", crc, r_crc);
		return 1;
	}
	else
	{
		printk("Err:token{%#x} crc{%#x}r_crc{%#x}len{%#x}\n",
			bt->token_type, crc, r_crc,length);
		if(bt->token_type == BM_TX_TASK)
		{
			#if HAVE_NUM
			printk("work_num {%d}\n", bt->work_num);
			#else
			printk("work_num {%d}\n", (le16_to_cpu(bt->length) - 3)/sizeof(struct ASIC_TASK));
			#endif
		}
		return 0;
	}
}

void rev(unsigned char *s, size_t l)
{
	size_t i, j;
	unsigned char t;

	for (i = 0, j = l - 1; i < j; i++, j--) {
		t = s[i];
		s[i] = s[j];
		s[j] = t;
	}
}
unsigned char CRC5(unsigned char *ptr,unsigned char len)
{
    unsigned char i,j,k;
    unsigned char crc=0x09;

    j=0x80;
    k=0;
    for(i=0;i<len;i++){
        if(crc&0x10){
            crc<<=1;
            crc^=0x09;
        }
        else crc<<=1;
        if(*ptr&j) crc^=0x09;
        j>>=1;
        k++;
        if(k==8){
            j=0x80;
            k=0;
            ptr++;
        }
    }
    return(crc&0x1f);
}

int bitmain_asic_get_status(char* buf, char mode, char chip_addr, char reg_addr)
{
	memset(buf, 0, 4);
	buf[0] = 4;
	buf[1] = chip_addr;
	buf[2] = reg_addr;
	buf[3] = CRC5(buf, 4);
	if(mode)//all
		buf[0] |= 0x80;
	return 4;
}
int bitmain_asic_inactive(char* buf)
{
	memset(buf, 0, 4);
	buf[0] = 5;
	buf[3] = CRC5(buf, 4);
	buf[0] |= 0x80;
	return 4;
}

int bitmain_asic_set_addr(char* buf, char mode, char chip_addr)
{
	memset(buf, 0, 4);
	buf[0] = 1;
	buf[1] = chip_addr;
	buf[3] = CRC5(buf, 4);
	buf[0] |= 0x80;
	return 4;
}

int bitmain_asic_set_frequency(char* buf, char mode, char chip_addr , char reg_addr, char reg_value)
{
	memset(buf, 0, 4);
	buf[0] = 1;
	buf[1] = chip_addr;
	buf[3] = CRC5(buf, 4);
	if(mode)//all
		buf[0] |= 0x80;
	return 4;
}

void bitmain_usb_sdata(struct work_struct *work)
{
	BT_AS_INFO dev= container_of(work, struct __BT_AS_info, usb_sdata_work);
	void __iomem *base = ath79_gpio_base;
	static char value=0;
	unsigned char offset = 19;
	char task_work[] = {0x46, 0x79, 0xba, 0x4e, 0xc9, 0x98, 0x76, 0xbf, 0x4b, 0xfe, 0x08, 0x60, 0x82, 0xb4, 0x00, 0x25,
						0x4d, 0xf6, 0xc3, 0x56, 0x45, 0x14, 0x71, 0x13, 0x9a, 0x3a, 0xfa, 0x71, 0xe4, 0x8f, 0x54, 0x4a,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x87, 0x32, 0x0b, 0x1a, 0x14, 0x26, 0x67, 0x4f, 0x2f, 0xa7, 0x22, 0xce
		};
	uint8_t asic_work[64] = {0};
	if (value)
		__raw_writel(1 << offset, base + AR71XX_GPIO_REG_SET);
	else
		__raw_writel(1 << offset, base + AR71XX_GPIO_REG_CLEAR);

	__raw_writel(__raw_readl(base + AR71XX_GPIO_REG_OE) | (1 << offset),
		     base + AR71XX_GPIO_REG_OE);
	value = !value;
	if(bitmain_fops_p != NULL)
	{
		memcpy(asic_work, &dev->task_buffer[dev->task_buffer_rd].midstate, 32);
		memcpy(asic_work + 52, &dev->task_buffer[dev->task_buffer_rd].data, 12);
		rev(asic_work, 32);
		rev(asic_work + 52, 12);
		if(0)//bitmain_mode == BITMAIN_MULTI_MODE
		{
			asic_work[51] = le32_to_cpu(dev->task_buffer[dev->task_buffer_rd].work_id) & 0x1f; //bit[0-4]
		}
		if(bitmain_fops_p->write(&bitmain_file, task_work, sizeof(task_work), NULL) != sizeof(task_work))
	//	if(bitmain_fops_p->write(&bitmain_file, asic_work, sizeof(asic_work), NULL) != sizeof(asic_work))
			printk("usb send data error!!!\n");
		dev->task_last_num = dev->task_current_num;
		dev->task_current_num = dev->task_buffer_rd;
		dev->task_buffer_rd++;
		if(dev->task_buffer_rd >= dev->task_buffer_size)
		{
			dev->task_buffer_rd = 0;
		}
		dev->task_buffer_full = false;
	}

}
static __inline void flip80(void *dest_p, const void *src_p)
{
	uint32_t *dest = dest_p;
	const uint32_t *src = src_p;
	int i;

	for (i = 0; i < 20; i++)
		dest[i] = swab32(src[i]);
}

static __inline void flip32(void *dest_p, const void *src_p)
{
	uint32_t *dest = dest_p;
	const uint32_t *src = src_p;
	int i;

	for (i = 0; i < 8; i++)
		dest[i] = swab32(src[i]);
}

static __inline void flip_swab(void *dest_p, const void *src_p, unsigned int length)
{
	uint32_t *dest = dest_p;
	const uint32_t *src = src_p;
	int i;

	for (i = 0; i < length/4; i++)
		dest[i] = swab32(src[i]);
}

#if 1
const char g_full_data[] = { "0000000258007a06037bb0c899e253afc369f10c9e7762a8aa73d33b00000034000000009d5146878f7fda9f7f7f76f1c1aa6751f679e3aff3f722b2030b0eac"
						"814f5d975201f3ba1972dbf2"
						"ae319135"
						"000000800000000000000000000000000000000000000000000000000000000000000000000000000000000080020000"
					   };
const char g_midstate[] = {"2e26c4440504a801393d373204e87cc02828ba7fd6f191add1b0ff01a9302ac0"};//需要反序 (现在左侧为高位，右侧为低位 )
const char g_data[] = {"f2db7219baf30152975d4f81"};
const char g_nonce[] = {"359131ae"};
#else
const char g_full_data[] = { "0000000258007a06037bb0c899e253afc369f10c9e7762a8aa73d33b00000034000000009d5146878f7fda9f7f7f76f1c1aa6751f679e3aff3f722b2030b0eac"
						"814f5d975201f3ba1972dbf2"
						"ae319135"
						"000000800000000000000000000000000000000000000000000000000000000000000000000000000000000080020000"
					   };
const char g_midstate[] = {"9a91ac02214239593aadae3e0e1ce8c6c8a7509542148383653981d2698b67c1"};//需要反序 (现在左侧为高位，右侧为低位 )
const char g_data[] = {"29a4001a4edce351072fb87f"};
const char g_nonce[] = {"92ec9b6e"};
#endif

static void regen_hash(void)
{
	unsigned char full_data_hex[80];
	uint32_t *data32 = (uint32_t *)full_data_hex;
	unsigned char swap[80];
	uint32_t *swap32 = (uint32_t *)swap;
	unsigned char hash1[32];
	uint32_t *hash2_32 = (uint32_t *)hash1;
	unsigned char hash2[32];
	uint8_t i;

	hex2bin(full_data_hex, g_full_data, sizeof(full_data_hex));
	flip80(swap32, data32);
	sha2(swap, 80, hash1);
	#if 0
	printk("first sha2 out->\n");
	for(i=0; i< sizeof(hash1); i++)
	{
		printk("[%d]{%#x}\n", i, hash1[i]);
	}
	#endif
	sha2(hash1, 32, hash2);
	flip32(hash1, hash2);
	if(be32toh(hash2_32[7]) == 0)
		printk("hash ok\n");
	else
		printk("test hash error\n");
}


/* Returns 1 if meets current buffer work, 0 if last buffer work */
static int hashtest(ASIC_TASK_P asic_task, uint32_t nonce)
{
	unsigned char hash1[32];
	unsigned char hash2[32];
	uint32_t *hash2_32 = (uint32_t *)hash1;
	uint8_t i;
	__attribute__ ((aligned (4)))  sha2_context ctx;
	/*
	memcpy(ctx.state, asic_task->midstate, 32);
	rev((unsigned char*)ctx.state, sizeof(ctx.state));
	*/
	memcpy(hash1, asic_task->midstate, 32);
	//rev(hash1, 32);
	flip_swab(ctx.state, hash1, 32);
	ctx.total[0] = 80;
	ctx.total[1] = 0;
	memcpy(hash1, (void*)asic_task->data, 12);
	//rev(hash1, 12);
	flip_swab(ctx.buffer, hash1, 12);
	memcpy(hash1, &nonce, 4);

	flip_swab(ctx.buffer + 12, hash1, 4);
	sha2_finish( &ctx, hash1);
	#if 0
	printk("first sha2 out->\n");
	for(i=0; i< sizeof(hash1); i++)
	{
		printk("[%d]{%#x}\n", i, hash1[i]);
	}
	#endif
	memset( &ctx, 0, sizeof( sha2_context ) );
	sha2(hash1, 32, hash2);

	flip32(hash1, hash2);
	//printk("hash2_32[7]{%#x}hash2_32[0]{%#x}\n", hash2_32[7], hash2_32[0]);
	if (be32toh(hash2_32[7]) != 0) {
		printk("not work{%#x} nonce\n", le32_to_cpu(asic_task->work_id));
		return 0;
	}
	return 1;
}

static struct ASIC_RESULT	asic_result[8];
#define ASIC_RESULT_NUM		(sizeof(asic_result)/sizeof(asic_result[0]))
static uint8_t asic_result_wr = 0, asic_result_rd = 0, asic_result_full = 0;

static uint32_t	asic_result_status[8];
#define ASIC_RESULT_STATUS_NUM		(sizeof(asic_result_status)/sizeof(asic_result_status[0]))
static uint8_t asic_result_status_wr = 0, asic_result_status_rd = 0, asic_result_status_full = 0;

void bitmain_usb_rdata(struct work_struct *work)
{
	BT_AS_INFO dev= container_of(work, struct __BT_AS_info, usb_rdata_work);
	void __iomem *base = ath79_gpio_base;
	int ret = 0;
	uint8_t read_buffer[64];
	static uint8_t save_buffer[64];
	static uint8_t rd_p = 0, wr_p = 0;
	uint8_t	left = 0;
	uint32_t nonce = 0;
	uint8_t i,r_num;
	bool hw_err = false;
	if(bitmain_fops_p != NULL)
	{
		if((ret = bitmain_fops_p->read(&bitmain_file, read_buffer, sizeof(read_buffer), NULL)) < 0)
		{
			if(ret != -ETIMEDOUT)
				printk("usb read error ret{%d}!!!\n", ret);
		}
		else if(ret != 0)
		{
			#if 0
			uint8_t i=0;
			printk("\n");
			printk("read data from usb");
			for(i=0;i<ret;i++)
			{
				if(!(i%16))
					printk("\n");
				printk("i{%d}={%#x}, ", i, read_buffer[i]);
			}
			printk("\n");
			#endif
			memcpy(save_buffer + wr_p, read_buffer, ret);
			wr_p += ret;
			mutex_lock(&dev->result_lock);
			while((wr_p - rd_p)>= asic_return_bytes)
			{
				uint8_t data = save_buffer[rd_p+4];
				uint32_t work_id = 0;
				if(asic_return_bytes == 5)
				{
					if((data & 0xe0) == 0x80)//nonce
					{
						if((data & 0x1f) == (char)(le32_to_cpu(dev->task_buffer[dev->task_last_num].work_id) & 0x1f))
							work_id = dev->task_buffer[dev->task_last_num].work_id;
						else if((data & 0x1f) == (char)(le32_to_cpu(dev->task_buffer[dev->task_current_num].work_id) & 0x1f))
							work_id = dev->task_buffer[dev->task_current_num].work_id;
						else
						{
							rd_p++;
							continue;
						}

						//save work nonce
						if(asic_result_full)
						{
							printk("No sapce to save asic return nonce!!\n");
							break;
						}
						else
						{
							asic_result[asic_result_wr].work_id = work_id;
							asic_result[asic_result_wr++].nonce = /*htobe32*/((save_buffer[rd_p+0]<<0) +
								(save_buffer[rd_p+1]<<8) + (save_buffer[rd_p+2]<<16) + (save_buffer[rd_p+3]<<24));
							if(asic_result_wr >= ASIC_RESULT_NUM)
								asic_result_wr = 0;
							if(asic_result_wr == asic_result_rd)
								asic_result_full = 1;

							rd_p += asic_return_bytes;

						}
					}
					else if(( data & 0xe0) == 0x00 && (CRC5(save_buffer+rd_p, 4) == (data&0x1f)))//status
					{
						asic_result_status[asic_result_status_wr++] = /*htobe32*/((save_buffer[rd_p+0]<<0) +
								(save_buffer[rd_p+1]<<8) + (save_buffer[rd_p+2]<<16) + (save_buffer[rd_p+3]<<24));
							if(asic_result_status_wr >= ASIC_RESULT_NUM)
								asic_result_status_wr = 0;
							if(asic_result_status_wr == asic_result_status_rd)
								asic_result_status_full = 1;

							rd_p += asic_return_bytes;
					}
					else
						rd_p++;
				}
				else
				{
					//save work nonce
					if(asic_result_full)
					{
						printk("No sapce to save asic return nonce!!\n");
						break;
					}
					else
					{
						nonce = (save_buffer[rd_p+0]<<24) +
							(save_buffer[rd_p+1]<<16) + (save_buffer[rd_p+2]<<8) + (save_buffer[rd_p+3]<<0);
						if(hashtest(&dev->task_buffer[dev->task_last_num], cpu_to_le32(nonce)))
							work_id = dev->task_buffer[dev->task_last_num].work_id;
						else if(hashtest(&dev->task_buffer[dev->task_current_num], cpu_to_le32(nonce)))
							work_id = dev->task_buffer[dev->task_current_num].work_id;
						else
						{
							#if 0
							printk("asic nonce error!!!dev->task_last_num = %d,dev->task_current_num = %d\n",dev->task_last_num,dev->task_current_num);
							printk("nonce{%#x},middstate:last{%#x}current{%#x}\n", nonce, dev->task_buffer[dev->task_last_num].midstate[0],
								dev->task_buffer[dev->task_current_num].midstate[0]);
							#endif
							hw_err = true;
							for(i=1; i<(TASK_PRE_LEFT+1); i++)
							{
								if((dev->task_last_num-i) == 0)
								{
									r_num = dev->task_buffer_size - 1;
								}
								else
								{
									r_num = dev->task_last_num-i;
								}

								if(hashtest(&dev->task_buffer[r_num], cpu_to_le32(nonce)))
								{
									work_id = dev->task_buffer[r_num].work_id;
									printk("i=%d, midstate[%#x]\n", i, dev->task_buffer[r_num].midstate[0]);
									hw_err = false;
									break;
								}
							}
						}
						if(hw_err == false)
						{
							asic_result[asic_result_wr].work_id = work_id;
							asic_result[asic_result_wr++].nonce = cpu_to_le32(nonce);
							if(asic_result_wr >= ASIC_RESULT_NUM)
								asic_result_wr = 0;
							if(asic_result_wr == asic_result_rd)
								asic_result_full = 1;
						}
						else
						{
							hw_err = false;
						}
						rd_p += asic_return_bytes;

					}
				}
			}
			mutex_unlock(&dev->result_lock);
			left = wr_p - rd_p;
			memcpy(save_buffer, &save_buffer[rd_p], left);
			wr_p = left;
			rd_p = 0;
		}
		//schedule_delayed_work(&dev->usb_rdata_work,40 * HZ/1000);
	}
}


static irqreturn_t btmain_asic_interrupt(int irq, void *dev_id)
{
	int result = IRQ_NONE;
	BT_AS_INFO dev = &bitmain_asic_dev;
	void __iomem *base = ath79_gpio_base;
	static char value=0;
	unsigned char offset = 18;
	//printk("enter %s\n", __func__);
	if (1)
	{
		if (value)
			__raw_writel(1 << offset, base + AR71XX_GPIO_REG_SET);
		else
			__raw_writel(1 << offset, base + AR71XX_GPIO_REG_CLEAR);

		__raw_writel(__raw_readl(base + AR71XX_GPIO_REG_OE) | (1 << offset),
			     base + AR71XX_GPIO_REG_OE);
		value = !value;
		if((dev->usb_opened) && (dev->task_buffer_full || (dev->task_buffer_wr != dev->task_buffer_rd)))
		{
			//tasklet_schedule(&bitmain_usb_tasklet);
				/* Schedule a new measurement */
			if(queue_work(dev->usb_wq,	&dev->usb_sdata_work) != 1)
				printk("usb_sdata_work in queue\n");

			if(schedule_delayed_work(&dev->usb_rdata_work, 1 * HZ/1000) != 1)
				printk("usb_rdata_work in queue\n");
		}
		result = IRQ_HANDLED;
	}
	return result;
}


static void bitmain_asic_open_usb(BT_AS_INFO dev)
{
	if((dev->usb_opened == false) && (bitmain_fops_p != NULL))
	{
		bitmain_inode.i_rdev = MKDEV(192, 0);
		if(0 == bitmain_fops_p->open(&bitmain_inode, &bitmain_file))
		{
			writel(TIME_40MS, dev->virt_addr + GENERAL_TIMER_RELOAD);
			writel(TIME_40MS, dev->virt_addr + GENERAL_TIMER);
			//writel(500*200*1000, dev->virt_addr + GENERAL_TIMER_RELOAD );
			dev->usb_opened = true;
			enable_irq(dev->irq);
			printk("bitmain_asic_open_usb OK!!\n");
		}
		else
		{
			printk("bitmain_asic_open_usb error!!\n");
		}

	}
	else if(((dev->usb_opened == true) && (bitmain_fops_p == NULL))|| (!test_bit(0, &bitmain_is_open)))
	{
		disable_irq(dev->irq);
		bitmain_fops_p->release(&bitmain_inode, &bitmain_file);
		dev->usb_opened = false;
		printk("bitmain_asic close usb !!\n");
		//writel(0xffffffff, dev->virt_addr + GENERAL_TIMER_RELOAD );
	}
	return;
}

static int bitmain_asic_open(struct inode *inode, struct file *file)
{
	BT_AS_INFO dev = &bitmain_asic_dev;
	/* only allow one at a time */
	if (test_and_set_bit(0, &bitmain_is_open))
		return -EBUSY;
	file->private_data = dev;
	memset(dev, 0, sizeof(bitmain_asic_dev));
	dev->virt_addr = ioremap_nocache(RESET_BASE, RESET_SIZE);
	dev->task_buffer_size = TASK_BUFFER_NUMBER + TASK_PRE_LEFT;
	dev->task_buffer = (ASIC_TASK_P)kmalloc(sizeof(*dev->task_buffer)*dev->task_buffer_size, GFP_KERNEL);
	//dev->task_buffer_wr = 10;
	dev->asic_status_data.data_type = BM_STATUS_DATA;
	dev->asic_status_data.core_num = 8;
	dev->asic_status_data.asic_num = 32;
	mutex_init(&dev->result_lock);
	#if PRNT_TIMER_EN
	init_timer(&prnt_timer);
	prnt_timer.function = Prnt;
	prnt_timer.expires = jiffies + 500*HZ/1000;
	add_timer(&prnt_timer);
	#endif
	writel(0xffffffff, dev->virt_addr + GENERAL_TIMER_RELOAD );
	if (request_irq(TIMER_INTERRUPT, btmain_asic_interrupt, NULL, DRV_NAME, NULL)) {
		printk(KERN_WARNING "%s: unable to allocate interrupt.",
			DRV_NAME);
		goto out1;
	}
	dev->usb_wq = create_singlethread_workqueue(DRV_NAME);
	/* Init work for send data using usb */
	INIT_WORK(&dev->usb_sdata_work, bitmain_usb_sdata);
	INIT_DELAYED_WORK(&dev->usb_rdata_work, bitmain_usb_rdata);
	printk("bitmain_asic_init ok\n");
	return 0;
out2:
	free_irq(TIMER_INTERRUPT, NULL);
out1:
	return -1;
	printk("bitmain_asic_open ok\n");
	return 0;
}

static int bitmain_asic_close(struct inode *inode, struct file *file)
{
	BT_AS_INFO dev = &bitmain_asic_dev;
	clear_bit(0, &bitmain_is_open);
	bitmain_asic_open_usb(dev);
	destroy_workqueue(&dev->usb_sdata_work);
	#if PRNT_TIMER_EN
	del_timer(&prnt_timer);
	#endif
	kfree((void*)(dev->task_buffer));
	free_irq(TIMER_INTERRUPT, NULL);
	printk("bitmain_asic_close\n");
	return 0;
}
static ssize_t bitmain_asic_write(struct file *file, const char __user *user_buffer,
				size_t writesize, loff_t *ppos)
{
	BT_AS_INFO dev = file->private_data;
	struct BITMAIN_TASK txtask;
	uint8_t task_work_num, need_cpy_task_work;
	int retval = 0;
	bool asic_reset = false;
	spin_lock(&dev->lock);
	bitmain_asic_open_usb(dev);
	if (copy_from_user((void*)&txtask, user_buffer, writesize)) {
	retval = -EFAULT;
	goto error;
	}
	if(cmd_check((uint8_t*)&txtask)) //crc16 ok
	{
		switch(txtask.token_type)
		{
			case BM_TX_TASK:
				if(txtask.new_block)
				{
					mutex_lock(&dev->result_lock);
					dev->task_buffer_wr = dev->task_buffer_rd;
					dev->task_buffer_full = false;
					printk("New blok\n");
					mutex_unlock(&dev->result_lock);
				}
				#if HAVE_NUM
				task_work_num = txtask.work_num;
				#else
				task_work_num = (le16_to_cpu(txtask.length) - 6)/sizeof(*dev->task_buffer);
				#endif
				if((dev->task_buffer_wr + task_work_num) >= dev->task_buffer_size)
				{
					need_cpy_task_work = dev->task_buffer_size - dev->task_buffer_wr;
					memcpy(dev->task_buffer+dev->task_buffer_wr, &txtask.asic_task[0],
						need_cpy_task_work * sizeof(*dev->task_buffer));
					task_work_num -= need_cpy_task_work;
					memcpy(dev->task_buffer, &txtask.asic_task[need_cpy_task_work], task_work_num * sizeof(*dev->task_buffer));
					dev->task_buffer_wr = task_work_num;
					//printk("split asic_task[0].work_id %#x wr{%d}\n", le32_to_cpu(txtask.asic_task[0].work_id), dev->task_buffer_wr);
				}
				else
				{
					memcpy(dev->task_buffer+dev->task_buffer_wr, &txtask.asic_task[0],
						task_work_num * sizeof(*dev->task_buffer));
					dev->task_buffer_wr += task_work_num;
					//printk("asic_task[0].work_id %#x wr{%d}rd{%d}\n", le32_to_cpu(txtask.asic_task[0].work_id), dev->task_buffer_wr,dev->task_buffer_rd);
				}
				if(dev->task_buffer_wr == dev->task_buffer_rd)
					dev->task_buffer_full = true;
				//printk("dev->task_buffer_wr = %d\n", dev->task_buffer_wr);
				break;
			case BM_TX_CONF:
			{
				BITMAIN_CONFIGURE_P bt_conf = (BITMAIN_CONFIGURE_P)&txtask;
				dev->asic_configure.asic_num = bt_conf->asic_num;
				dev->asic_configure.core_num = bt_conf->core_num;
				if(bt_conf->reset)
				{
					asic_reset = true;
				}
				if(bt_conf->fan_eft)
				{
					dev->asic_configure.fan_pwm_data = bt_conf->fan_pwm_data;
				}
				if(bt_conf->timeout_eft)
				{
					dev->asic_configure.timeout_data = bt_conf->timeout_data;
				}
				if(bt_conf->fan_eft)
				{
					dev->asic_configure.frequency= bt_conf->frequency;
				}
				if(bt_conf->voltage_eft)
				{
					dev->asic_configure.voltage= bt_conf->voltage;
				}
				if(bt_conf->chain_check_time_eft)
				{
					dev->asic_configure.chain_check_time= bt_conf->chain_check_time;
				}
				if(bt_conf->chip_config_eft)
				{
					dev->asic_configure.chip_address= bt_conf->chip_address;
					dev->asic_configure.reg_address= bt_conf->reg_address;
				}
				printk("Set bitmain configure\n");
			}
				break;
			case BM_GET_STATUS:
			{
				BITMAIN_GET_STATUS_P bt_gt_status = (BITMAIN_GET_STATUS_P)&txtask;
				if(bt_gt_status->detect_get)
				{
					asic_reset = true;
					printk("Detect device\n");
				}
				if(bt_gt_status->chip_status_eft)
				{
					dev->asic_configure.chip_address = bt_gt_status->chip_address;
					dev->asic_configure.reg_address = bt_gt_status->reg_addr;
				}
				dev->get_status = true;
				//printk("Get status\n");
			}
				break;
			default:
				break;
		}
		if(asic_reset)
		{
			mutex_lock(&dev->result_lock);
			asic_result_status_rd = asic_result_status_wr = asic_result_status_full = 0;
			asic_result_rd = asic_result_wr = asic_result_full = 0;
			//dev->task_buffer_wr = dev->task_buffer_rd = 0;
			mutex_unlock(&dev->result_lock);
		}
	}
	else
	{
		retval = -EINVAL;
	}
	spin_unlock(&dev->lock);
	return writesize;
error:
	return retval;
}

static int create_rx_status_struct(struct BITMAIN_STATUS_DATA *rx_status_data,bool chip_value_eft, uint32_t reg_value,
	uint8_t fifo_sapce,char *temp, int temp_num, char* fan, int fan_num)
{
	uint16_t crc16;
	rx_status_data->chip_value_eft = chip_value_eft;
	rx_status_data->reg_value = reg_value;
	rx_status_data->temp_num = temp_num;
	rx_status_data->fifo_space = fifo_sapce;
	if((rx_status_data->temp_num !=0) && (temp != NULL))
	{
		memcpy(((char*)rx_status_data + 12), temp, temp_num);
	}
	rx_status_data->fan_num = fan_num;
	if((rx_status_data->fan_num !=0) && (fan != NULL))
	{
		memcpy(((char*)rx_status_data + 12 + rx_status_data->temp_num), fan, fan_num);
	}
	rx_status_data->length = 12 + rx_status_data->temp_num+rx_status_data->fan_num;
	crc16 = CRC16((const uint8_t*)rx_status_data, rx_status_data->length);
	*((char*)rx_status_data + rx_status_data->length) = crc16 & 0xff;
	*((char*)rx_status_data + rx_status_data->length + 1) = (char)(crc16>>8);
	return rx_status_data->length+2;
}

static ssize_t bitmain_asic_read(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	BT_AS_INFO dev = file->private_data;
	int retval = 0;
	uint8_t fifo_space;
	uint8_t i;
	spin_lock(&dev->lock);
	bitmain_asic_open_usb(dev);
	mutex_lock(&dev->result_lock);
	if(dev->get_status)
	{
		dev->get_status = false;
		if(dev->task_buffer_full)
		{
			fifo_space = 0;
		}
		else
		{
			if(dev->task_buffer_wr >= dev->task_buffer_rd)
			{
				fifo_space =dev->task_buffer_size - (dev->task_buffer_wr - dev->task_buffer_rd);
			}
			else
			{
				fifo_space = dev->task_buffer_rd - dev->task_buffer_wr;
			}
			//防止full，预留位置，存储已发送的上几个数据
			if(fifo_space >= TASK_PRE_LEFT)
				fifo_space -=TASK_PRE_LEFT;
			else
				fifo_space = 0;
		}
		if(asic_result_status_full || (asic_result_status_rd != asic_result_status_wr))
		{
			asic_result_status_full = 0;
			printk("status return\n");
			retval = create_rx_status_struct(&dev->asic_status_data, true, asic_result_status[asic_result_status_rd++],
				fifo_space, NULL, 0, NULL, 0);
		}
		else
		{
			retval = create_rx_status_struct(&dev->asic_status_data, false, 0,
				fifo_space, NULL, 0, NULL, 0);
		}
		copy_to_user(userbuf, (void*)&dev->asic_status_data, retval);
	}
	else if(asic_result_full || (asic_result_rd != asic_result_wr))
	{
		struct BITMAIN_RESULT bitmain_result;
		uint8_t	nonce_num = 0;
		uint16_t crc16;
		bitmain_result.data_type = BM_RX_NONCE;
		asic_result_full = 0;
		do
		{
			memcpy(&bitmain_result.nonce[nonce_num], &asic_result[asic_result_rd], sizeof(asic_result[0]));
			nonce_num++;
			asic_result_rd++;
			if(asic_result_rd >= ASIC_RESULT_NUM)
				asic_result_rd = 0;
		}
		while((asic_result_rd != asic_result_wr)&&(nonce_num <= 8));
		bitmain_result.nonce_num = nonce_num;
		bitmain_result.length = nonce_num*(sizeof(asic_result[0])) + 4;
		crc16 = CRC16((const uint8_t*)&bitmain_result, bitmain_result.length);
		*((char*)&bitmain_result + bitmain_result.length) = crc16 & 0xff;
		*((char*)&bitmain_result + bitmain_result.length + 1) = (char)(crc16>>8);
		retval = bitmain_result.length + 2;
		copy_to_user(userbuf, (void*)&bitmain_result, retval);
		printk("read %d nonce\n", nonce_num);
	}
	mutex_unlock(&dev->result_lock);
	spin_unlock(&dev->lock);
	return retval;
}
/*
 * Handle commands from user-space.
 */
static long bitmain_asic_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int ret = -ENOTTY;
	int time;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {

	}
	return ret;
}

static const struct file_operations btasic_fops = {
	.owner = THIS_MODULE,
	.read = bitmain_asic_read,
	.write = bitmain_asic_write,
	.open = bitmain_asic_open,
	.release = bitmain_asic_close,
	.unlocked_ioctl = bitmain_asic_ioctl,
};

static struct miscdevice bitmain_asic = {
	MISC_DYNAMIC_MINOR,
	DRV_NAME,
	&btasic_fops
};
static int __init bitmain_asic_init(void)
{
	struct ASIC_TASK  asic_task;
	uint32_t nonce;
	uint8_t i;
	uint8_t *u8p;
	asic_task.work_id = 0xffffffff;
	hex2bin(asic_task.midstate, g_midstate, sizeof(asic_task.midstate));
	hex2bin(asic_task.data, g_data, sizeof(asic_task.data));
	hex2bin(&nonce, g_nonce, sizeof(nonce));
	regen_hash();
	rev(asic_task.midstate, sizeof(asic_task.midstate));
	rev(asic_task.data, sizeof(asic_task.data));
	rev(&nonce, sizeof(nonce));
	#if 0
	printk("nonce-->{%#x}\n", nonce);
	printk("nonce-->\n");
	u8p = &nonce;
	for(i = 0; i < sizeof(nonce); i++)
	{
		printk("{%d}{%#x}, ", i, u8p[i]);
		if(0 == (i%16))
			printk("\n");
	}
	#endif
	if(hashtest(&asic_task, nonce))
		printk("hashtest OK\n");
	else
		printk("hashtest Error!!!!!!!!!!\n");
	#if 0
	BT_AS_INFO dev = &bitmain_asic_dev;
	memset(dev, 0, sizeof(bitmain_asic_dev));
	dev->virt_addr = ioremap_nocache(RESET_BASE, RESET_SIZE);
	dev->task_buffer_size = TASK_BUFFER_NUMBER;
	dev->task_buffer = (ASIC_TASK_P)kmalloc(sizeof(*dev->task_buffer)*dev->task_buffer_size, GFP_KERNEL);
	//dev->task_buffer_wr = 10;

	mutex_init(&dev->result_lock);
	init_timer(&prnt_timer);
	prnt_timer.function = Prnt;
	prnt_timer.expires = jiffies + 500*HZ/1000;
	add_timer(&prnt_timer);
	writel(0xffffffff, dev->virt_addr + GENERAL_TIMER_RELOAD );
	if (request_irq(TIMER_INTERRUPT, btmain_asic_interrupt, NULL, DRV_NAME, NULL)) {
		printk(KERN_WARNING "%s: unable to allocate interrupt.",
			DRV_NAME);
		goto out1;
	}

	if (misc_register(&bitmain_asic)) {
		printk(KERN_ERR "%s: failed to register device\n",
		       DRV_NAME);
		goto out2;
	}

	dev->usb_wq = create_singlethread_workqueue(DRV_NAME);
	/* Init work for send data using usb */
	INIT_WORK(&dev->usb_sdata_work, bitmain_usb_sdata);
	INIT_DELAYED_WORK(&dev->usb_rdata_work, bitmain_usb_rdata);
	printk("bitmain_asic_init ok\n");
	return 0;

out2:
	free_irq(TIMER_INTERRUPT, NULL);
out1:
	return -1;
	#endif
	if (misc_register(&bitmain_asic)) {
	printk(KERN_ERR "%s: failed to register device\n",
	       DRV_NAME);
	return -1;
	}
	else
	{
		printk(KERN_ERR "%s: success to register device\n",
	       DRV_NAME);
		return 0;
	}
}
module_init(bitmain_asic_init);

static void __exit bitmain_asic_exit(void)
{
	#if 0
	BT_AS_INFO dev = &bitmain_asic_dev;
	kfree((void*)(dev->task_buffer));
	misc_deregister(&bitmain_asic);
	free_irq(TIMER_INTERRUPT, NULL);
	#endif
	misc_deregister(&bitmain_asic);
}
module_exit(bitmain_asic_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Xuelei <xuelei_51@126.com>");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VERSION);
