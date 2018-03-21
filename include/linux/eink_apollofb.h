
enum eink_apollo_controls {
	H_CD = 0,
	H_RW = 1,
	H_DS = 2,
	H_ACK = 3,
	H_WUP = 4,
	H_NRST = 5,

	/* next pins using by Netronix EB600 */
	H_PWR = 6,
	H_D0 = 7,
	H_D1 = 8,
	H_D2 = 9,
	H_D3 = 10,
	H_D4 = 11,
	H_D5 = 12,
	H_D6 = 13,
	H_D7 = 14,
};

struct eink_apollo_operations {
	int (*initialize)(void);
	void (*set_ctl_pin)(unsigned int pin, unsigned char val);
	int (*get_ctl_pin)(unsigned int pin);
	void (*write_value)(unsigned char val);
	unsigned char (*read_value)(void);
	int (*send_data_fast)(void*, unsigned char data);
};

/* Apollo controller commands */
#define APOLLO_WRITE_TO_FLASH		0x01
#define APOLLO_READ_FROM_FLASH		0x02
#define APOLLO_WRITE_REGISTER		0x10
#define APOLLO_READ_REGISTER		0x11
#define APOLLO_READ_TEMPERATURE		0x21
#define APOLLO_LOAD_PICTURE		0xA0
#define APOLLO_STOP_LOADING		0xA1
#define APOLLO_DISPLAY_PICTURE		0xA2
#define APOLLO_ERASE_DISPLAY		0xA3
#define APOLLO_INIT_DISPLAY		0xA4
#define APOLLO_RESTORE_IMAGE		0xA5
#define APOLLO_GET_STATUS		0xAA
#define APOLLO_LOAD_PARTIAL_PICTURE	0xB0
#define APOLLO_DISPLAY_PARTIAL_PICTURE	0xB1
#define APOLLO_VERSION_NUMBER		0xE0
#define APOLLO_DISPLAY_SIZE		0xE2
#define APOLLO_RESET			0xEE
#define APOLLO_NORMAL_MODE		0xF0
#define APOLLO_SLEEP_MODE		0xF1
#define APOLLO_STANDBY_MODE		0xF2
#define APOLLO_SET_DEPTH		0xF3
#define APOLLO_ORIENTATION		0xF5
#define APOLLO_POSITIVE_PICTURE		0xF7
#define APOLLO_NEGATIVE_PICTURE		0xF8
#define APOLLO_AUTO_REFRESH		0xF9
#define APOLLO_CANCEL_AUTO_REFRESH	0xFA
#define APOLLO_SET_REFRESH_TIMER	0xFB
#define APOLLO_MANUAL_REFRESH		0xFC
#define APOLLO_READ_REFRESH_TIMER	0xFD

#define APOLLO_WAVEFORMS_FLASH_SIZE	(1024 * 1024 * 2)

#define APOLLO_STATUS_MODE_MASK		(1 << 0)
#define APOLLO_STATUS_MODE_SLEEP	0x01
#define APOLLO_STATUS_MODE_NORMAL	0x00

struct eink_apollofb_platdata {
	struct eink_apollo_operations ops;
	unsigned long defio_delay;
};

#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC       _IOW('F', 0x20, __u32)
#endif
#define EINK_APOLLOFB_IOCTL_SET_AUTOREDRAW _IOW('F', 0x21, unsigned int)
#define EINK_APOLLOFB_IOCTL_FORCE_REDRAW _IO('F', 0x22)
#define EINK_APOLLOFB_IOCTL_SHOW_PREVIOUS _IO('F', 0x23)
