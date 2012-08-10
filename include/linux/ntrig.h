/* include/linux/Ntrig.h - NTRIG_G41 Touch driver
 *
 * Copyright (C) 2010 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_NTRIG_H
#define _LINUX_NTRIG_H

#define NTRIG_NAME "ntrig"
#define NTRIG_PACKET_SIZE (136)

#define NTG_PMB_FF	0xFFFFFFFF
#define NTG_PREAMBLE 	0x7EE75AA5
#define NTG_CAL_STATE	0x212121
#define NTG_CAL_FAIL	0x424242
#define NTG_MASK	0x7FFFFF
#define NTG_CALSTART	(0)
#define NTG_CALPASS	(1)
#define NTG_CALDEFEAT	(2)

/*#define COUNTER_DEBUG 1*/
#define TRACKLIB_MAX_FINGERS	(6)
#define MTM_REPORT_SIZE		(94)
#define NTRIG			(0xD1)
#define CK_CNT			(6)

#ifdef COUNTER_DEBUG
#define NTRIG_IOCTL_READ_DATA    _IOWR(NTRIG, 0x1, unsigned char[MTM_REPORT_SIZE+1+4])
#else
#define NTRIG_IOCTL_READ_DATA    _IOWR(NTRIG, 0x1, unsigned char[MTM_REPORT_SIZE+1])
#endif

#define NTRIG_IOCTL_SEND_DATA	  _IOW(NTRIG, 0x2, NtrTrackedFingersReport)

#define ANALY_IOC_MAGIC	(0xD2)
#define ANALY_INIT	_IOR(ANALY_IOC_MAGIC, 0x00, char)
#define ANALY_BASE1	_IOR(ANALY_IOC_MAGIC, 0x01, unsigned char)
#define ANALY_BASE2	_IOR(ANALY_IOC_MAGIC, 0x02, unsigned char)
#define ANALY_BASE3	_IOR(ANALY_IOC_MAGIC, 0x03, unsigned char)
#define ANALY_BASE4	_IOR(ANALY_IOC_MAGIC, 0x04, unsigned char)
#define ANALY_OPFREQ	_IOR(ANALY_IOC_MAGIC, 0x05, char)
#define ANALY_ACTIVE	_IOR(ANALY_IOC_MAGIC, 0x06, unsigned char)
#define ANALY_FINGER	_IOR(ANALY_IOC_MAGIC, 0x07, unsigned char)
#define ANALY_PEN	_IOR(ANALY_IOC_MAGIC, 0x08, char)
#define ANALY_MAXNR	(8)

#define DEBUG_ANGENT_UART_REPORT_SIZE	(1952)
#define DEBUG_ANGENT_ALL_REPORT_SIZE	(2048)
#define DEBUG_ANGENT_REPORT_SIZE	(1024)

#define CMD_GOTO_BL_VAL_LOAD_FW		(0x0100)
#define CMD_GOTO_BL_VAL_STAY_IN_BL	(0x0101)

#define DFU_CMD_NOT_RELEVANT_DATA 	(0x0)

#define DFU_CMD_START_BYTE		(0x7E)
#define DFU_CMD_GROUP			(0x7)
#define DFU_CMD_FW_TYPE			(0x1)

enum DfuMessageType {
	DFU_CMD_MSG_TYPE_CALL 		= 0x01,
	DFU_CMD_MSG_TYPE_CALL_RESPONSE 	= 0x81
};

enum MessageGroup {
	BootGroup = 0x01,
	CMDGroup = 0x05,
	DfuGroup = 0x07,
	AnaGroup = 0x0B,
};

enum BootGroup {
	BOOTLOADER_VERSION	= 0x01,
};

enum DfuMessageCode {
	DFU_CMD_MSG_CODE_START		= 0x10,
	DFU_CMD_MSG_CODE_BLOCK_HEADER	= 0x11,
	DFU_CMD_MSG_CODE_DATA_FRAME	= 0x12,
	DFU_CMD_MSG_CODE_DATA_COMPLETE	= 0x13,
	DFU_CMD_MSG_CODE_LOAD_FW	= 0x14,
	DFU_CMD_MSG_CODE_TEST_FW	= 0x17,
	DFU_CMD_MSG_CODE_SET_SELF_VALID	= 0x19,
};

enum CmdMessageCode {
	CMD_MSG_CODE_TURNOFF_USB	= 0x07,
	CMD_CONFIG_FETCH_SETTING	= 0x08,
	CMD_CONFIG_SETTING	 	= 0x09,
	CMD_CONFIG_ALGO_FETCH		= 0x0F,
	CMD_CONFIG_ALGO_SETTING		= 0x10,
	CMD_OPERATE_FREQ            	= 0x11,
};

enum AnaMessgaeCode {
	ANA_GET_SN	= 0x01,
	ANA_SEN_CONF	= 0x02,
	ANA_GET_BASEMAP	= 0x03,
	ANA_GET_ACTMAP	= 0x04,
	ANA_GET_FINMAP	= 0x05,
	ANA_GET_PEN	= 0x06,
};

enum OTFCstate {
	OTFCDisable	= 0x00,
	OTFCEnable	= 0x01,
	OTFCRevert	= 0x02,
};

enum FHstate {
	FHauto	= 0x00,
	FHmanu	= 0x01,
};

enum DebugLevel {
	Dbg_L1 = 0x01,
	Dbg_L2 = 0x02,
	Dbg_L3 = 0x04,
	Dbg_L4 = 0x08,
	Dbg_L5 = 0x10,
	Dbg_L6 = 0x20,
	Dbg_L7 = 0x40,
	Dbg_L8 = 0x80,
	Dbg_L9 = 0x100,
};

enum RX_STATE {
	CHK5A,
	CHKPMB,
	FINISH,
	FINLOOP,
	CHKLEN,
	CHKDATA,
};

enum ST_STATE {
	st_START,
	st_CONTI,
	st_FINISH,
};

#pragma pack(1)

typedef struct {
	char startByte;
	short int address;
	short int totalLength;
	char msgType;
	char msgGroup;
	char msgCode;
	unsigned long returnCode;
} ntg_CmdDfuHeader;

typedef struct {
	ntg_CmdDfuHeader dfuHeader;
	char reserved1;
	char reserved2;
	unsigned long startAddress;
	char fwType;
	char checksum;
} ntg_CmdDfuStart;

typedef struct {
	ntg_CmdDfuHeader dfuHeader;
	unsigned long dataLength;
	unsigned long destinationInRam;
	char lastBlockIndication;
	char checksum;
} ntg_CmdDfuSendBlock;

struct ntg_touch_header {
	u8  type;
	u16 length;
	u8  flag;
	u8  channel;
	u8  function;
};

struct ntg_touch_ver {
	u8 repId;
	u8 reserved_0;
	u32 value;
	u16 reserved_1;
};

struct ntg_touch_Stouch {
	u8  repID;
	u8  bit;
	u16 posX;
	u16 posY;
	u16 posDx;
	u16 posDy;
};

struct ntg_touch_Ptouch {
	u8  repID;
	u8  bit;
	u16 posX;
	u16 posY;
	u16 tip;
	u16 unused;
};

struct ntg_touch_Mtouch {
    u8  bit;
    u16 cid;
    u16 posX;
    u16 posY;
    u16 posDx;
    u16 posDy;
    u32 vdefine;
};

struct ntg_AnaPen {
    u8  pState;
    u16 pTh;
    u16 mag;
    u8  pPress;
    u16 posX;
    u16 posY;
    u8  pBtn;
};

struct ntg_debug_agent_uart {
	struct 	ntg_touch_header  header;
	u8	data[DEBUG_ANGENT_UART_REPORT_SIZE];
};

struct ntg_touch_event {
	struct ntg_touch_header  header;
	struct ntg_touch_ver	 version;
	struct ntg_touch_Stouch  stouch;
	struct ntg_touch_Ptouch  ptouch;
	ntg_CmdDfuStart  DfuStCmd;
};

struct ntg_DfuCmd{
	u8 startAddr;
	u16 Addr;
	u16 Length;
	u8 msgType;
	u8 msgGup;
	u8 msgCode;
	u32 retCode;
	u8 reseved0;
	u8 reserved1;
};

struct ntrig_spi_platform_data {
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;
	uint16_t fwtwozero;
	uint8_t orientate;
	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t abs_width_min;
	uint8_t abs_width_max;
	int	spi_enable;
	int	irq_gpio;
	int	(*power)(int on);
	int	(*pmicgpio_config)(void);
	bool	esdFlag;
};

typedef struct NtrTrackedFinger_tag {
	unsigned short	TrackID;
	unsigned short	posX;
	unsigned short	posY;
	unsigned short	posDx;
	unsigned short	posDy;
	unsigned char	IsRemoved;
	unsigned char	IsPalm;
} NtrTrackedFinger;

typedef struct NtrTrackedFingersReport_tag {
	unsigned char FingerCount;
	unsigned char Reserved0;
	unsigned char Reserved1;
	unsigned char Reserved2;
	NtrTrackedFinger Fingers[TRACKLIB_MAX_FINGERS];

} NtrTrackedFingersReport;

#pragma pack()

#endif
