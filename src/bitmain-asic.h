#ifndef __BITMAIN_ASIC_H__
#define __BITMAIN_ASIC_H__

#define HAVE_NUM		0
#define PACKED __attribute__( ( packed, aligned(4) ) )

#define BITMAIN_DEFAULT_ASIC_NUM 0xA

#define	BM_TX_CONF		0x51
#define	BM_TX_TASK		0x52
#define	BM_GET_STATUS	0x53
#define	BM_STATUS_DATA	0xa1
#define	BM_RX_NONCE		0xa2


#define	CNF_REST	(0x01<<0)

#define	CNF_FAN		(0x01<<1)
#define	CNF_TIMEOUT	(0x01<<2)
#define	CNF_FREQUENCY		(0x01<<3)
#define	CNF_VOLTAGE		(0x01<<4)
#define	CNF_CCHECKT		(0x01<<5)
#define	CNF_CHIP_CNF		(0x01<<6)

struct BITMAIN_CONFIGURE{
	uint8_t	token_type;
	uint8_t	length;
	/*
	uint8_t	rccvftfr;
	*/
	uint8_t reset                :1;
	uint8_t fan_eft              :1;
	uint8_t timeout_eft          :1;
	uint8_t frequency_eft        :1;
	uint8_t voltage_eft          :1;
	uint8_t chain_check_time_eft :1;
	uint8_t chip_config_eft      :1;
	uint8_t reserved1            :1;
	uint8_t reserved2;
	/*
	reset	uint8_t£º1
	fan_eft uint8_t£º1
	timeout_eft uint8_t£º1
	frequency_eft	uint8_t£º1
	voltage_eft uint8_t£º1
	chain_check_time_eft	uint8_t£º1
	chip_config_eft uint8_t£º1
	Reserved	uint8_t£º1
	*/
	uint8_t	core_num;
	uint8_t	asic_num;
	uint8_t	fan_pwm_data;
	uint8_t	timeout_data;

	uint16_t	frequency;
	uint8_t	voltage;
	uint8_t	chain_check_time;

	uint32_t	reg_data;

	uint8_t	chip_address;
	uint8_t	reg_address;
	uint16_t crc;
}PACKED;
typedef struct BITMAIN_CONFIGURE*	BITMAIN_CONFIGURE_P;


struct ASIC_TASK
{
	uint32_t	work_id;
	uint8_t	midstate[32];
	uint8_t	data[12];
}PACKED;
typedef struct ASIC_TASK* ASIC_TASK_P;

#define NEW_BLK	0x01
struct BITMAIN_TASK {
	uint8_t token_type;
	uint8_t reserved1;
	uint16_t length;
	uint8_t new_block            :1;
	uint8_t reserved2            :7;
	uint8_t reserved3[3];
	struct ASIC_TASK asic_task[8];
	uint16_t crc;
}PACKED;

#if 0
struct BITMAIN_TASK {
	uint8_t token_type;
	#if HAVE_NUM
	uint8_t length;
	uint8_t new_block            :1;
	uint8_t reserved1            :7;

	/*uint8_t	rnew_block;*/
	uint8_t	work_num;
	#else
	uint16_t length;
	uint8_t new_block            :1;
	uint8_t reserved1            :7;
	#endif
	struct ASIC_TASK asic_task[8];
	uint16_t	crc;
}PACKED;
#endif
typedef struct BITMAIN_TASK*	BITMAIN_TASK_P;

#define	DETECT_GET		0x02
#define 	GET_CHIP_ST	0x01
struct BITMAIN_GET_STATUS {
	uint8_t token_type;
	uint8_t length;

	uint8_t chip_status_eft      :1;
	uint8_t detect_get		:1;
	uint8_t reserved1            :6;

	/*uint8_t rchipd_eft;*/
	uint8_t reserved;
	//uint32_t reg_data;
	uint8_t chip_address;
	uint8_t reg_addr;
	uint16_t crc;
}PACKED;
typedef struct BITMAIN_GET_STATUS*	BITMAIN_GET_STATUS_P;

struct BITMAIN_STATUS_DATA_HEADER {
	uint8_t data_type;
	uint8_t length;
	/*
	uint8_t chip_reg_value_eft      :1;
	uint8_t reserved1            :7;
	*/
	uint8_t rchipregval_eft;
	uint8_t reserved;
	uint32_t reg_value;
	uint8_t core_num;
	uint8_t asic_num;
	uint8_t temp_sensor_num;
	uint8_t fan_num;
	uint32_t fifo_space;
	/*
	uint8_t temp[temp_sensor_num];
	uint8_t fan[fan_num];
	uint16_t crc;
	*/
}PACKED;

struct BITMAIN_STATUS_DATA {
	//struct BITMAIN_STATUS_DATA_HEADER status_header;
	uint8_t data_type;
	uint8_t length;

	uint8_t chip_value_eft		:1;
	uint8_t reserved1			:7;

	/*uint8_t rchipregval_eft;*/
	uint8_t fifo_space;
	uint32_t reg_value;
	uint8_t core_num;
	uint8_t asic_num;
	uint8_t temp_num;
	uint8_t fan_num;
	//uint32_t fifo_space;
	uint8_t temp[10];
	uint8_t fan[10];
	uint16_t crc;
}PACKED;

struct ASIC_RESULT{
	uint32_t	work_id;
	uint32_t	nonce;
};

struct BITMAIN_RESULT {

	uint8_t		data_type;
	uint8_t		length;
	uint8_t		reserved;
	uint8_t		nonce_num;
	struct ASIC_RESULT nonce[8];
	uint16_t	crc;
}PACED;

struct ASIC_WORK
{
	uint8_t midstate[32];
	uint8_t pad[64-32-12];
	uint8_t data[12];
	uint32_t	nonce2;
}PACKED;
typedef struct ASIC_WORK*	ASIC_WORK_P;

#define be32toh		be32_to_cpu

#endif
