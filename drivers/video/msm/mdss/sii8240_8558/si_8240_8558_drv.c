/*

SiI8558 / SiI8240 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/

/*
   @file si_8240_8558_drv.c
*/
//#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <mach/debug_display.h>

#include "si_fw_macros.h"
#include "si_app_devcap.h"
#include "si_mhl_defs.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_8240_8558_internal_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "si_8240_8558_drv.h"
#include "mhl_linux_tx.h"
#include "platform.h"
#include "si_tpi_regs.h"
#include "si_8240_8558_regs.h"
#include "../mdss_hdmi_mhl.h"
#include "../mdss_panel.h"
#include "../mdss_hdmi_tx.h"
#include "../mdss_hdmi_hdcp.h"

/* external functions */
extern bool ap_hdcp_success;
extern struct hdmi_hdcp_ctrl *hdcp_ctrl_global;

/* Local functions */
static	int	int_4_isr(struct drv_hw_context *hw_context, uint8_t int_4_status);
static	int	int_5_isr(struct drv_hw_context *hw_context, uint8_t int_5_status);
static	int	int_8_isr(struct drv_hw_context *hw_context, uint8_t intr_8_status);
static	int	int_9_isr(struct drv_hw_context *hw_context, uint8_t int_9_status);
static	int	hdcp_isr(struct drv_hw_context *hw_context, uint8_t tpi_int_status);
static	int	g2wb_isr(struct drv_hw_context *hw_context, uint8_t intr_stat);
static	int	mhl_cbus_isr(struct drv_hw_context *hw_context, uint8_t cbus_int);
static	int	mhl_cbus_err_isr(struct drv_hw_context *hw_context, uint8_t cbus_err_int);
static	int	acc_switch_int_0(struct drv_hw_context *hw_context, uint8_t reg_asw_int0);
static	int	acc_switch_int_1(struct drv_hw_context *hw_context, uint8_t reg_asw_int1);
static	void	enable_intr(struct drv_hw_context *hw_context, uint8_t intr_num, uint8_t intr_mask);
static	void	disconnect_mhl(struct drv_hw_context *hw_context,bool do_interrupt_clear, bool discovery_enable);
static	void	start_hdcp(struct drv_hw_context *hw_context);
static	void	stop_video(struct drv_hw_context *hw_context);
static	void	init_transcode_mode(struct drv_hw_context *hw_context);
static	int		get_device_rev(struct drv_hw_context *hw_context);
static	void	board_reset(struct drv_hw_context *hw_context,
							uint16_t hwResetPeriod,
							uint16_t hwResetDelay);
static	void	unmute_video(struct drv_hw_context *hw_context);
static	int	set_hdmi_params(struct mhl_dev_context *dev_context);
static	int	get_cbus_connection_status(struct drv_hw_context *hw_context);

#ifdef CONFIG_MHL_MONITOR_WORKAROUND
int		start_video(struct drv_hw_context *hw_context, void *edid_parser_context);
#else
static	int	start_video(struct drv_hw_context *hw_context, void *edid_parser_context);
#endif

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
void AppVbusControl(struct drv_hw_context *hw_context, uint8_t dev_cat);
#endif

/* Local data */
#define	HDCP_RPTR_CTS_DELAY_MS	2875
#define	HDCP_ERROR_THRESHOLD	5
static	int	hdcp_bksv_err_count    = 0;
static	int	hdcp_reneg_err_count   = 0;
static	int	hdcp_link_err_count    = 0;
static	int	hdcp_suspend_err_count = 0;

static enum {
	  ddc_bypass_off=0
	, ddc_bypass_on
	, ddc_bypass_start_video
}ddc_bypass_state=ddc_bypass_off;

enum {
	waiting_for_bc_done_fw_trig =0
	,waiting_for_bc_done_fw_trig_special
	,charging_dcp
	,charging_cdp
	,charging_sdp
	,charging_ps2
	,charging_sony
	,charging_apple_1_0A
	,no_downstream_charger
}firmware_triggered_bc_detection_state=waiting_for_bc_done_fw_trig;

typedef enum{
	bc_done_dcp=0
	,bc_done_cdp
	,bc_done_sdp
	,bc_done_ps2
	,bc_done_ps2_special_charger
	,bc_done_sony
	,bc_done_apple_1_0A
}firmware_triggered_bc_detection_event;
struct rid_entry{
	uint32_t	flags;
	unsigned char 	asw_manual_ctrl_value;
	char		*description;
};
enum rid_entry_flags {
	 flag_none		= 0x00000000
	,flag_audio_poll	= 0x00000001
	,flag_mcpc_button	= 0x00000002
	,flag_audio_button	= 0x00000004
	,flag_no_buttons	= 0x00000008
	,flag_mic		= 0x00000010
	,flag_usb		= 0x00000020
	,flag_audio		= 0x00000040
	,flag_uart		= 0x00000080
	,flag_boot		= 0x00000100
};
#define AUDIO_POPSUP_SWCFG \
	( BIT_ASW_MAN_CTRL_AUD_POPRES_ON \
	| BIT_ASW_MAN_CTRL_AUDIO_ON )

#define AUDIO_BUTTON_MICVB_SWCFG \
	( BIT_ASW_MAN_CTRL_AUD_POPRES_ON \
	| BIT_ASW_MAN_CTRL_MIC_VBUS_ON \
	| BIT_ASW_MAN_CTRL_AUDIO_ON )

#define AUDIO_BUTTON_MICID_SWCFG \
	( BIT_ASW_MAN_CTRL_AUD_POPRES_ON \
	| BIT_ASW_MAN_CTRL_MIC_ID_ON \
	| BIT_ASW_MAN_CTRL_AUDIO_ON )

#define MHL_SWCFG 0
#define SWCFG_OFF      0
#define OPEN_IMPEDANCE 0x1F

enum	{
	 TABLE_MCPC_SHORTED_RESERVED = 0
	,TABLE_MCPC_2000_OHM_END_BUTTON
	,TABLE_MCPC_2604_OHM_RESERVED
	,TABLE_MCPC_3208_OHM_RESERVED
	,TABLE_MCPC_4014_OHM_RESERVED
	,TABLE_MCPC_4820_OHM_RESERVED
	,TABLE_MCPC_6030_OHM_RESERVED
	,TABLE_MCPC_8030_OHM_RESERVED
	,TABLE_MCPC_10030_OHM_RESERVED
	,TABLE_MCPC_12030_OHM_RESERVED
	,TABLE_MCPC_14460_OHM_RESERVED
	,TABLE_MCPC_17260_OHM_RESERVED
	,TABLE_MCPC_20500_OHM_RESERVED
	,TABLE_MCPC_24070_OHM_RESERVED
	,TABLE_MCPC_28700_OHM_RESERVED
	,TABLE_MCPC_34000_OHM_RESERVED
	,TABLE_MCPC_40200_OHM_47K_RCSW
	,TABLE_MCPC_49900_OHM_47K_RCSW

	,TABLE_MCPC_OPEN = OPEN_IMPEDANCE
};

enum	{
	 TABLE_ONE_SHORTED = 0
	,TABLE_ONE_2000_OHM_END_BUTTON
	,TABLE_ONE_2604_OHM_S1_BUTTON
	,TABLE_ONE_3208_OHM_S2_BUTTON
	,TABLE_ONE_4014_OHM_S3_BUTTON
	,TABLE_ONE_4820_OHM_S4_BUTTON
	,TABLE_ONE_6030_OHM_S5_BUTTON
	,TABLE_ONE_8030_OHM_S6_BUTTON
	,TABLE_ONE_10030_OHM_S7_BUTTON
	,TABLE_ONE_12030_OHM_S8_BUTTON
	,TABLE_ONE_14460_OHM_S9_BUTTON
	,TABLE_ONE_17260_OHM_S10_BUTTON
	,TABLE_ONE_20500_OHM_S11_BUTTON
	,TABLE_ONE_24070_OHM_S12_BUTTON

	,TABLE_ONE_OPEN = OPEN_IMPEDANCE
};
#define MCPC_FLAGS (flag_mcpc_button | flag_audio_poll | flag_audio)

struct rid_entry tableOne[] = {
     {flag_usb           ,BIT_ASW_MAN_CTRL_USB_D_ON,"T1 - SHORTED (USB)"                   }/* 0x00 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 2K - Audio Send - End Button"    }/* 0x01 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 2.604K - Audio Remote S1 Button" }/* 0x02 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 3.208K - Audio Remote S2 Button" }/* 0x03 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 4.014K - Audio Remote S3 Button" }/* 0x04 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 4.82K - Audio Remote S4 Button"  }/* 0x05 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 6.03K - Audio Remote S5 Button"  }/* 0x06 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 8.03K - Audio Remote S6 Button"  }/* 0x07 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 10.03K - Audio Remote S7 Button" }/* 0x08 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 12.03K - Audio Remote S8 Button" }/* 0x09 */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 14.46K - Audio Remote S9 Button" }/* 0x0A */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 17.26K - Audio Remote S10 Button"}/* 0x0B */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 20.5K - Audio Remote S11 Button" }/* 0x0C */
    ,{flag_audio_button  ,AUDIO_POPSUP_SWCFG       ,"T1 - 24.07K - Audio Remote S12 Button"}/* 0x0D */
    ,{flag_uart          ,BIT_ASW_MAN_CTRL_UART_ON ,"T1 - 28.7K - Factory Boot OFF - UART" }/* 0x0E */
    ,{flag_uart          ,BIT_ASW_MAN_CTRL_UART_ON ,"T1 - 34K - Factory Boot ON - UART"    }/* 0x0F */
    ,{flag_usb			 ,BIT_ASW_MAN_CTRL_USB_D_ON,"T1 - 40.2K - Factory Boot OFF - USB"  }/* 0x10 */
    ,{flag_usb			 ,BIT_ASW_MAN_CTRL_USB_D_ON,"T1 - 49.9K - Factory Boot ON - USB"   }/* 0x11 */
    ,{flag_none          ,SWCFG_OFF                ,"T1 - 64.9K - Reserved"                }/* 0x12 */
    ,{flag_none			 ,SWCFG_OFF                ,"T1 - 80.07K - Reserved"               }/* 0x13 */
    ,{flag_none			 ,SWCFG_OFF                ,"T1 - 102K - Reserved"                 }/* 0x14 */
    ,{flag_none          ,SWCFG_OFF                ,"T1 - 121K - Reserved"                 }/* 0x15 */
    ,{flag_none          ,SWCFG_OFF                ,"T1 - 150K - Reserved"                 }/* 0x16 */
    ,{flag_none          ,SWCFG_OFF                ,"T1 - 200K - Reserved"                 }/* 0x17 */
    ,{flag_no_buttons|flag_mic|flag_audio
                         ,AUDIO_BUTTON_MICID_SWCFG ,"T1 - 255K - Audio with mic' on ID --> no polling"    }/* 0x18 */
    ,{flag_none			 ,SWCFG_OFF                ,"T1 - 301K - Reserved"                 }/* 0x19 */
    ,{flag_no_buttons    ,SWCFG_OFF                ,"T1 - 365K - Reserved"                 }/* 0x1A */
    ,{flag_none          ,SWCFG_OFF                ,"T1 - 442K - Reserved"                 }/* 0x1B */
    ,{flag_none			 ,SWCFG_OFF                ,"T1 - 523K - Reserved"                 }/* 0x1C */
    ,{flag_none			 ,SWCFG_OFF                ,"T1 - 619K - Reserved"                 }/* 0x1D */
    ,{flag_audio_poll|flag_mic|flag_audio
                         ,AUDIO_BUTTON_MICVB_SWCFG ,"T1 - 1001K - Audio with mic' on VBUS" }/* 0x1E */
    ,{flag_none			 ,SWCFG_OFF                ,"T1 - OPEN - Reserved"                 }/* 0x1F */
    ,{flag_none          ,MHL_SWCFG                ,"T1 - 1K - MHL"                        }/* 0x20 */
    ,{flag_none          ,MHL_SWCFG                ,"T1 - USER SPEC"                       }/* 0x21 */
	,{flag_none          ,MHL_SWCFG                ,"T1 - no connection"                   }/* 0x22 = MAX_IMPEDANCE_VALUES - 1 */
};

/* Impedance measurement table for SiI8558 */
struct rid_entry tableOneMCPC[] = {
     {flag_usb         ,BIT_ASW_MAN_CTRL_USB_D_ON  ,"T1'MCPC - SHORTED - Reserved" }/* 0x00 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 2K - Reserved"        }/* 0x01 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 2.604K - Reserved"    }/* 0x02 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 3.208K - Reserved"    }/* 0x03 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 4.014K - Reserved"    }/* 0x04 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 4.82K - Reserved"     }/* 0x05 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 6.03K - Reserved"     }/* 0x06 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 8.03K - Reserved"     }/* 0x07 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 10.03K - Reserved"    }/* 0x08 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 12.03K - Reserved"    }/* 0x09 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 14.46K - Reserved"    }/* 0x0A */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 17.26K - Reserved"    }/* 0x0B */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 20.5K - Reserved"     }/* 0x0C */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 24.07K - Reserved"    }/* 0x0D */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 28.7K - Reserved"     }/* 0x0E */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 34K - Reserved"       }/* 0x0F */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 40.2K - RCSW 47K"     }/* 0x10 */
    ,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 49.9K - RCSW 47K"     }/* 0x11 */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 64.9K - Reserved"     }/* 0x12 */
	,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 80.07K - Reserved"    }/* 0x13 - needs audio polling */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 102K - Reserved"      }/* 0x14 - needs audio polling */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 121K - Reserved"      }/* 0x15 */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 150K - Ridm 180K"     }/* 0x16 */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 200K - Ridm 180K"     }/* 0x17 */
    ,{MCPC_FLAGS       ,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 255K - RFSEL1 + RCSW" 	}/* 0x18 */
    ,{MCPC_FLAGS       ,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 301K - RFSEL1 + RCSW" 	}/* 0x19 */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 365K - RIDr 390"      	}/* 0x1A - needs audio polling */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 442K - RIDr 390"      	}/* 0x1B */
    ,{flag_none        ,SWCFG_OFF                  ,"T1'MCPC - 523K - Reserved" 	 	}/* 0x1C *//* for demo only */
    ,{MCPC_FLAGS       ,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 619K - RFSEL2 + RCSWT"	}/* 0x1D */
    ,{MCPC_FLAGS       ,AUDIO_POPSUP_SWCFG         ,"T1'MCPC - 1001K - RFSEL2 + RCSWT"	}/* 0x1E - needs audio polling */
    ,{flag_none		   ,SWCFG_OFF                  ,"T1'MCPC - OPEN - Reserved"			}/* 0x1F */
    ,{flag_none        ,MHL_SWCFG                  ,"T1'MCPC - 1K - MHL"                }/* 0x20 */
    ,{flag_none        ,MHL_SWCFG                  ,"T1'MCPC - USER SPEC"               }/* 0x21 */
	,{flag_none        ,MHL_SWCFG                  ,"T1' - no connection"               }/* 0x22 = MAX_IMPEDANCE_VALUES - 1 */
};

struct rid_entry tableTwo[] = {
	 {flag_usb         ,BIT_ASW_MAN_CTRL_USB_D_ON  ,"T2 - SHORTED"                          }/* 0x00 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 2K - Audio Send-End Button"       }/* 0x01 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 2.604K - Audio Remote S1 Button"  }/* 0x02 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 3.208K - Audio Remote S2 Button"  }/* 0x03 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 4.014K - Audio Remote S3 Button"  }/* 0x04 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 4.82K - Audio Remote S4 Button"   }/* 0x05 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 6.03K - Audio Remote S5 Button"   }/* 0x06 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 8.03K - Audio Remote S6 Button"   }/* 0x07 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 10.03K - Audio Remote S7 Button"  }/* 0x08 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 12.03K - Audio Remote S8 Button"  }/* 0x09 */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 14.46K - Audio Remote S9 Button"  }/* 0x0A */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 17.26K - Audio Remote S10 Button" }/* 0x0B */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 20.5K - Audio Remote S11 Button"  }/* 0x0C */
	,{flag_audio_button,AUDIO_POPSUP_SWCFG         ,"T2 - 24.07K - Audio Remote S12 Button" }/* 0x0D */
	,{flag_uart        ,BIT_ASW_MAN_CTRL_UART_ON   ,"T2 - 56K - Factory Boot OFF-UART"      }/* 0x0E */
	,{flag_uart        ,BIT_ASW_MAN_CTRL_UART_ON   ,"T2 - 100K - Factory Boot ON-UART"      }/* 0x0F */
	,{flag_usb         ,BIT_ASW_MAN_CTRL_USB_D_ON  ,"T2 - 130K - Factory Boot OFF-USB"      }/* 0x10 */
	,{flag_usb         ,BIT_ASW_MAN_CTRL_USB_D_ON  ,"T2 - 180K - Factory Boot ON-USB"       }/* 0x11 */
	,{flag_none        ,SWCFG_OFF                  ,"T2 - 240K - Reserved"                  }/* 0x12 */
	,{flag_none        ,SWCFG_OFF                  ,"T2 - 330K - Reserved"                  }/* 0x13 */
	,{flag_none        ,SWCFG_OFF                  ,"T2 - 430K - Reserved"                  }/* 0x14 */
	,{flag_none        ,SWCFG_OFF                  ,"T2 - 620K - Reserved"                  }/* 0x15 */
	,{flag_none        ,SWCFG_OFF                  ,"T2 - 910K - Reserved"                  }/* 0x16 */
	,{flag_none        ,SWCFG_OFF                  ,"T2 - OPEN - Reserved"                  }/* 0x17 */
	,{flag_boot        ,SWCFG_OFF                  ,"T2 - N/A - Reserved"                   }/* 0x18 */
	,{flag_boot        ,SWCFG_OFF                  ,"T2 - N/A - Reserved"                   }/* 0x19 */
	,{flag_boot        ,SWCFG_OFF                  ,"T2 - N/A - Reserved"                   }/* 0x1A */
	,{flag_boot        ,SWCFG_OFF                  ,"T2 - N/A - Reserved"                   }/* 0x1B */
	,{flag_boot        ,SWCFG_OFF                  ,"T2 - N/A - Reserved"                   }/* 0x1C */
	,{flag_boot        ,SWCFG_OFF                  ,"T2 - N/A - Reserved"                   }/* 0x1D */
	,{flag_boot        ,SWCFG_OFF                  ,"T2 - N/A - Reserved"                   }/* 0x1E */
	,{flag_boot        ,SWCFG_OFF                  ,"T2 - N/A - Reserved"                   }/* 0x1F */
	,{flag_none        ,MHL_SWCFG                  ,"T2 - 1K - MHL"                         }/* 0x20 */
	,{flag_none        ,MHL_SWCFG                  ,"T2 - USER SPEC"                        }/* 0x21 */
	,{flag_none        ,MHL_SWCFG                  ,"T2 - no connection"                   }/* 0x22 = MAX_IMPEDANCE_VALUES - 1 */
};

char *table_asw_mon[]={
	 "0000:  MHL"
	,"0001:  AUDIO without Microphone"
	,"0010:  AUDIO with Microphone on VBUS"
	,"0011:  AUDIO with Microphone on ID"
	,"0100 : USB (Not Factory cable)"
	,"0101:  USB Factory cable"
	,"0110:  UART (Not Factory cable )"
	,"0111:  UART Factory cableh  "
	,"1000:  Switch Off"
};
#define NUM_RID_ENTRIES (sizeof(tableOne)/sizeof(tableOne[0]))
#define MHL_INDEX (NUM_RID_ENTRIES -3)
#define RID_INDEX_USER (NUM_RID_ENTRIES -2)
#define NO_CONNECTION_INDEX (NUM_RID_ENTRIES -1)
static int	rid_index= NO_CONNECTION_INDEX;
static struct rid_entry	*rid_table;

#define	DDC_ABORT_THRESHOLD		10
static	int		ddc_abort_count = 0;
static uint8_t asw_test_ctrl_default_bits=BIT_ASWTEST_DEFAULT;

#define	MSC_ABORT_THRESHOLD		10
static	int		msc_abort_count = 0;

struct	intr_tbl	{
	uint8_t	mask;
	uint8_t	mask_page;
	uint8_t	mask_offset;
	uint8_t	stat_page;
	uint8_t	stat_offset;
	int	(*isr)(struct drv_hw_context *, uint8_t int_5_status);
	char	name[5];
};

struct	intr_tbl	g_intr_tbl[] = {
		 {0, REG_INTR4_MASK, REG_INTR4, int_4_isr, "DISC"}
		,{0, REG_CBUS_MDT_INT_0_MASK, REG_CBUS_MDT_INT_0, g2wb_isr, "G2WB"}
		,{0, REG_CBUS_INT_0_MASK, REG_CBUS_INT_0, mhl_cbus_isr, "MSC "}
		,{0, REG_CBUS_INT_1_MASK, REG_CBUS_INT_1, mhl_cbus_err_isr, "MERR"}
		,{0, REG_INTR8_MASK, REG_INTR8, int_8_isr, "INFR"}
		,{0, REG_TPI_INTR_ST0_ENABLE, REG_TPI_INTR_ST0, hdcp_isr, "HDCP"}
		,{0, REG_INTR9_MASK, REG_INTR9, int_9_isr, "EDID"}
		,{0, REG_INTR5_MASK, REG_INTR5, int_5_isr, "CKDT"}
		,{0,REG_ASW_INT0_MASK,REG_ASW_INT0,acc_switch_int_0,"ASW0"}
		,{0,REG_ASW_INT1_MASK,REG_ASW_INT1,acc_switch_int_1,"ASW1"}
};

typedef enum {
	 INTR_DISC	= 0
	,INTR_G2WB	= 1
	,INTR_MSC	= 2
	,INTR_MERR	= 3
	,INTR_INFR	= 4
	,INTR_HDCP	= 5
	,INTR_EDID	= 6
	,INTR_CKDT	= 7
	,INTR_ASW0	= 8
	,INTR_ASW1	= 9
	,MAX_INTR	= 10
}intr_nums_t;

extern struct mhl_drv_info drv_info;

#define SILICON_IMAGE_ADOPTER_ID 322

#define TX_HW_RESET_PERIOD	10	/* 10 ms. */
#define TX_HW_RESET_DELAY	100
#define TX_EDID_POLL_MAX 	256

static uint8_t colorSpaceTranslateInfoFrameToHw[] = {
		BIT_TPI_INPUT_FORMAT_RGB,
		BIT_TPI_INPUT_FORMAT_YCbCr422,
		BIT_TPI_INPUT_FORMAT_YCbCr444,
		BIT_TPI_INPUT_FORMAT_INTERNAL_RGB	/* reserved for future */
};

#ifdef ENABLE_GEN2 //(
static	void	enable_gen2_write_burst(struct drv_hw_context *hw_context)
{
	/*  enable Gen2 Write Burst interrupt, MSC and EDID interrupts. */

	if(hw_context->ready_for_mdt) {
		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_RCV_TIMEOUT, 100);	/* 2 second timeout */
		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_RCV_CONTROL, BIT_CBUS_MDT_RCV_CONTROL_RCV_EN_ENABLE);
		enable_intr(hw_context, INTR_G2WB, BIT_MDT_RXFIFO_DATA_RDY);

		hw_context->gen2_write_burst = true;
	}
}

static void disable_gen2_write_burst(struct drv_hw_context *hw_context)
{
	/*  disable Gen2 Write Burst engine to perform it using legacy WRITE_BURST */
	mhl_tx_write_reg(hw_context, REG_CBUS_MDT_RCV_CONTROL, BIT_CBUS_MDT_RCV_CONTROL_RCV_EN_DISABLE);
	enable_intr(hw_context, INTR_G2WB, 0);
	hw_context->gen2_write_burst = false;
}
#endif //)
int __si_8240_8558_drv_ddc_bypass_control(char *file,int iLine,struct drv_hw_context *hw_context,int bypass)
{
	int ret_val=0;
	struct mhl_dev_context	*dev_context;
	dev_context = get_mhl_device_context(hw_context);
	switch (ddc_bypass_state) {
	case ddc_bypass_off:
		if (bypass){
			int hpd_ctrl;
			hpd_ctrl = mhl_tx_read_reg(hw_context, REG_HPD_CTRL);
			if (BIT_HPD_CTRL_HPD_HIGH & hpd_ctrl){
				stop_video(hw_context);
				ret_val = mhl_tx_modify_reg(hw_context,REG_DCTL,BIT_DCTL_EXT_DDC_SEL,BIT_DCTL_EXT_DDC_SEL);
				ddc_bypass_state=ddc_bypass_on;
			} else {
				MHL_TX_DBG_ERR(,"\n\n\n ERROR!!!! DDC bypass request while upstream HPD driven low.  Called from %s:%d\n\n",file,iLine);
				ret_val=-1;
			}
		} else {
			MHL_TX_DBG_ERR(,"unexpected DDC bypass event from %s:%d\n",file,iLine);
		}
		break;
	case ddc_bypass_on:
		if (bypass) {
			MHL_TX_DBG_ERR(,"unexpected DDC bypass event from %s:%d\n",file,iLine);
		} else {
			ret_val = mhl_tx_modify_reg(hw_context,REG_DCTL,BIT_DCTL_EXT_DDC_SEL,0);
			ddc_bypass_state=ddc_bypass_off;
		}
		break;
	case ddc_bypass_start_video:
		if (bypass) {
			MHL_TX_DBG_ERR(,"unexpected DDC bypass event from %s:%d\n",file,iLine);
		} else {
			ret_val = mhl_tx_modify_reg(hw_context,REG_DCTL,BIT_DCTL_EXT_DDC_SEL,0);
			ddc_bypass_state=ddc_bypass_off;
			start_video(hw_context,hw_context->intr_info->edid_parser_context);
		}
		break;
	}
	return ret_val;
}

static void si_mhl_tx_drv_reset_ddc_fifo(struct drv_hw_context *hw_context)
{
	uint8_t	ddc_status;

	ddc_status = mhl_tx_read_reg(hw_context, REG_DDC_STATUS);

	mhl_tx_modify_reg(hw_context, REG_TPI_SEL,
					  BIT_TPI_SEL_SW_TPI_EN_MASK,
					  BIT_TPI_SEL_SW_TPI_EN_NON_HW_TPI);
	if (BIT_DDC_STATUS_DDC_NO_ACK & ddc_status) {
		MHL_TX_DBG_ERR(hw_context, "Clearing DDC ack status\n");
		mhl_tx_write_reg(hw_context, REG_DDC_STATUS,
						 ddc_status & ~BIT_DDC_STATUS_DDC_NO_ACK);
	}
	mhl_tx_modify_reg(hw_context, REG_DDC_CMD,
					  BIT_DDC_CMD_COMMAND_MASK,
					  BIT_DDC_CMD_COMMAND_CLEAR_FIFO);

	mhl_tx_modify_reg(hw_context, REG_TPI_SEL,
					  BIT_TPI_SEL_SW_TPI_EN_MASK,
					  BIT_TPI_SEL_SW_TPI_EN_HW_TPI);
}

bool si_mhl_tx_drv_issue_edid_read_request(struct drv_hw_context *hw_context,
						uint8_t block_number)
{
	uint8_t reg_val;
	reg_val = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
	if ( BIT_CBUS_HPD & reg_val ) {
		MHL_TX_DBG_INFO(hw_context,
				"\n\tRequesting EDID block:%d\n"	\
				"\tcurrentEdidRequestBlock:%d\n"	\
				"\tedidFifoBlockNumber:%d\n",
				block_number,
				hw_context->current_edid_request_block,
				hw_context->edid_fifo_block_number);

		/* Setup auto increment and kick off read */
		mhl_tx_write_reg(hw_context, REG_EDID_CTRL
				, BIT_EDID_CTRL_EDID_PRIME_VALID_DISABLE
				| BIT_EDID_CTRL_FIFO_SELECT_EDID
				| BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
				| BIT_EDID_CTRL_EDID_MODE_EN_ENABLE
				);

		si_mhl_tx_drv_reset_ddc_fifo(hw_context);
		/* Setup which block to read */
		if( 0 == block_number) {
			/* Enable EDID interrupt */
			enable_intr(hw_context, INTR_EDID,
						   ( BIT_INTR9_DEVCAP_DONE_MASK
							| BIT_INTR9_EDID_DONE_MASK
							| BIT_INTR9_EDID_ERROR
						   ));
			mhl_tx_write_reg(hw_context, REG_TPI_CBUS_START,
							 BIT_TPI_CBUS_START_EDID_READ_BLOCK_0);
		} else {
			uint8_t param = (1 << (block_number-1));
			MHL_TX_DBG_INFO(hw_context, "EDID HW Assist: Programming "\
							"Reg %02X:%02x to %02X\n",
							REG_EDID_HW_ASSIST_READ_BLOCK_ADDR, param);
			mhl_tx_write_reg(hw_context, REG_EDID_HW_ASSIST_READ_BLOCK_ADDR,
							 param);
		}

		return true;
	} else {
		MHL_TX_DBG_INFO(hw_context,
				"\n\tNo HPD for EDID block request:%d\n"	\
				"\tcurrentEdidRequestBlock:%d\n"			\
				"\tedidFifoBlockNumber:%d\n",
				block_number,
				hw_context->current_edid_request_block,
				hw_context->edid_fifo_block_number);
		return false;
	}
}

/*
 * si_mhl_tx_drv_send_cbus_command
 *
 * Write the specified Sideband Channel command to the CBUS.
 * such as READ_DEVCAP, SET_INT, WRITE_STAT, etc.
 * Command can be a MSC_MSG command (RCP/RAP/RCPK/RCPE/RAPK), or another command
 * Parameters:
 *              req     - Pointer to a cbus_req_t structure containing the
 *                        command to write
 * Returns:     true    - successful write
 *              false   - write failed
 */
bool si_mhl_tx_drv_send_cbus_command(struct drv_hw_context *hw_context, struct cbus_req *req)
{
	bool	success = true;
	uint8_t	block_write_buffer [3];	// used for efficient block writes

#ifdef ENABLE_GEN2 //(
	/* Disable h/w automation of WRITE_BURST until this command completes */
	disable_gen2_write_burst(hw_context);
#endif //)
	switch (req->command) {
	case MHL_SET_INT:
		MHL_TX_DBG_INFO(hw_context, "SET_INT reg: 0x%02x data: 0x%02x\n",
				req->reg, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, req->reg);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
				BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);
#ifdef CONFIG_MHL_DONGLE_WORKAROUND
		if((req->reg == MHL_RCHANGE_INT) && (req->reg_data == MHL_INT_DCAP_CHG)){
			MHL_TX_DBG_INFO(hw_context,"Sleep 50ms for dongle reading DCAP");
			msleep(50);
		}
#endif
		break;

	case MHL_WRITE_STAT:
		MHL_TX_DBG_INFO(hw_context,
				"WRITE_STAT (0x%02x, 0x%02x)\n",
				req->reg,
				req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, req->reg);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
				BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);
		break;

	case MHL_READ_DEVCAP:
		MHL_TX_DBG_INFO(hw_context,"Trigger DEVCAP Read\n");
		/* Enable DEVCAP_DONE interrupt */
		enable_intr(hw_context, INTR_EDID, BIT_INTR9_DEVCAP_DONE);

		/* don't call si_mhl_tx_drv_reset_ddc_fifo here */

		/* read the entire DEVCAP array in one command */
		mhl_tx_write_reg(hw_context, REG_TPI_CBUS_START,
				BIT_TPI_CBUS_START_DEVCAP_READ_START);
		break;

	case MHL_READ_EDID_BLOCK:
		hw_context->current_edid_request_block = 0;
		success = si_mhl_tx_drv_issue_edid_read_request(hw_context,
				hw_context->current_edid_request_block);
		break;

	case MHL_GET_STATE:			/* 0x62 - */
	case MHL_GET_VENDOR_ID:		/* 0x63 - for vendor id */
	case MHL_SET_HPD:			/* 0x64	- Set Hot Plug Detect */
	case MHL_CLR_HPD:			/* 0x65	- Clear Hot Plug Detect */
	case MHL_GET_SC1_ERRORCODE:	/* 0x69	- Get channel 1 command error code */
	case MHL_GET_DDC_ERRORCODE:	/* 0x6A	- Get DDC channel command error code */
	case MHL_GET_MSC_ERRORCODE:	/* 0x6B	- Get MSC command error code */
	case MHL_GET_SC3_ERRORCODE:	/* 0x6D	- Get channel 3 command error code */
		MHL_TX_DBG_INFO(hw_context, "Sending MSC command %02x, %02x, %02x\n",
				req->command, req->reg, req->reg_data);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_CMD_OR_OFFSET, req->command);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_COMMAND_START,
				BIT_CBUS_MSC_PEER_CMD);
		break;

	case MHL_MSC_MSG:
		MHL_TX_DBG_INFO(hw_context,
				"MHL_MSC_MSG sub cmd: 0x%02x data: 0x%02x\n",
				req->msg_data[0], req->msg_data[1]);
		block_write_buffer[0] = req->command;
		block_write_buffer[1] = req->msg_data[0];
		block_write_buffer[2] = req->msg_data[1];

		/*
		   mhl_tx_write_reg(hw_context,REG_CBUS_MSC_CMD_OR_OFFSET, req->command);
		   mhl_tx_write_reg(hw_context,REG_CBUS_MSC_1ST_TRANSMIT_DATA, req->msg_data[0]);
		   mhl_tx_write_reg(hw_context,REG_CBUS_MSC_2ND_TRANSMIT_DATA, req->msg_data[1]);
		   */
		mhl_tx_write_reg_block(hw_context,REG_CBUS_MSC_CMD_OR_OFFSET, 3, block_write_buffer);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_COMMAND_START,
				BIT_CBUS_MSC_MSG);
		break;

	case MHL_WRITE_BURST:
		MHL_TX_DBG_INFO(hw_context, "MHL_WRITE_BURST offset: 0x%02x "\
				"length: 0x%02x\n",
				req->offset, req->length);

		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET,
				req->offset + REG_CBUS_MHL_SCRPAD_BASE);

		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_WRITE_BURST_DATA_LEN,
				req->length -1);

		/* Now copy all bytes from array to local scratchpad */
		mhl_tx_write_reg_block(hw_context, REG_CBUS_WB_XMIT_DATA_0,
				req->length, req->msg_data);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_COMMAND_START,
				BIT_CBUS_MSC_WRITE_BURST);
		break;

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
	case MHL_CHECK_HTC_DONGLE_CHARGER:
	{
		uint8_t devcap = 0;
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x02);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START, BIT_CBUS_MSC_READ_DEVCAP);
		msleep(1);
		devcap = mhl_tx_read_reg(hw_context, REG_CBUS_PRI_RD_DATA_1ST);
		MHL_TX_DBG_ERR(NULL,"===EXECUTE MHL_READ_DEVCAT, DEVCAT = %#02x\n", devcap);

		AppVbusControl(hw_context, devcap);
		break;
	}
#endif
	default:
		MHL_TX_DBG_ERR(hw_context, "Unsupported command 0x%02x detected!\n",
				req->command);
		success = false;
		break;
	}

    return (success);
}

void si_mhl_tx_drv_set_3d_mode(struct drv_hw_context *hw_context, bool do_3D,
							   _3D_structure_e three_d_mode)
{
	if (do_3D) {
		if (tdsFramePacking == three_d_mode) {
			MHL_TX_DBG_INFO(hw_context, "using frame packing\n");

			mhl_tx_write_reg(hw_context, REG_VID_OVRRD ,
					REG_VID_OVRRD_DEFVAL |
					BIT_VID_OVRRD_3DCONV_EN_FRAME_PACK);
		} else {
			MHL_TX_DBG_INFO(hw_context, "NOT using frame packing\n");
			mhl_tx_write_reg(hw_context, REG_VID_OVRRD, REG_VID_OVRRD_DEFVAL);
		}
	} else {
		MHL_TX_DBG_INFO(hw_context, "NOT using frame packing\n");
		mhl_tx_write_reg(hw_context, REG_VID_OVRRD , REG_VID_OVRRD_DEFVAL);
	}
}

uint16_t si_mhl_tx_drv_get_incoming_horizontal_total(struct drv_hw_context *hw_context)
{
	uint16_t ret_val;

	ret_val = (((uint16_t)mhl_tx_read_reg(hw_context, REG_HRESH)) <<8) |
				(uint16_t)mhl_tx_read_reg(hw_context, REG_HRESL);
	return ret_val;
}

uint16_t si_mhl_tx_drv_get_incoming_vertical_total(struct drv_hw_context *hw_context)
{
	uint16_t ret_val;

	ret_val = (((uint16_t)mhl_tx_read_reg(hw_context, REG_VRESH)) <<8) |
				(uint16_t)mhl_tx_read_reg(hw_context, REG_VRESL);
	return ret_val;
}

int si_mhl_tx_drv_get_edid_fifo_next_block(struct drv_hw_context *hw_context,
					   uint8_t *edid_buf)
{
	int ret_val;
	uint8_t offset;

	offset = EDID_BLOCK_SIZE * (hw_context->edid_fifo_block_number & 0x01);

	MHL_TX_DBG_INFO(hw_context, "%x %x",(unsigned int)hw_context,(unsigned int)edid_buf);
	hw_context->edid_fifo_block_number++;

	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, offset);

#if 0 //( unnecessary here
    // choose EDID instead of devcap to appear at the FIFO
	mhl_tx_write_reg(hw_context
			, REG_EDID_CTRL
			, BIT_EDID_CTRL_EDID_PRIME_VALID_DISABLE
			| BIT_EDID_CTRL_FIFO_SELECT_EDID
			| BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
			| BIT_EDID_CTRL_EDID_MODE_EN_ENABLE
			);
#endif //)

	ret_val = mhl_tx_read_reg_block(hw_context
		, REG_EDID_FIFO_RD_DATA
		, EDID_BLOCK_SIZE
		, edid_buf);

	DUMP_EDID_BLOCK(0,edid_buf, EDID_BLOCK_SIZE)
	ret_val = mhl_tx_read_reg(hw_context,REG_CBUS_STATUS);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR(hw_context, "%d", ret_val);
		return ne_NO_HPD;
	} else if (BIT_CBUS_HPD & ret_val) {
		MHL_TX_DBG_INFO(hw_context,
					"Done reading EDID from FIFO using HW_ASSIST ret_val:0x%02x\n",ret_val);
		return 0;
	} else {
		MHL_TX_DBG_INFO(hw_context, "No HPD ret_val:0x%02x\n",ret_val);
		return ne_NO_HPD;
	}
}

int si_mhl_tx_drv_get_scratch_pad(struct drv_hw_context *hw_context,
					uint8_t start_reg, uint8_t *data, uint8_t length)
{
	if ((start_reg + length) > (int)MHL_SCRATCHPAD_SIZE)
		return -1;

	memcpy(data, &hw_context->write_burst_data[start_reg], length);

	return 0;
}

static	bool packed_pixel_available(struct mhl_dev_context *dev_context)
{
	if ((MHL_DEV_VID_LINK_SUPP_PPIXEL & DEVCAP_VAL_VID_LINK_MODE) &&
		(dev_context->dev_cap_cache.mdc.vid_link_mode &
		 MHL_DEV_VID_LINK_SUPP_PPIXEL)) {
		return true;
	}
	return false;
}
#define SIZE_AVI_INFOFRAME				14
static uint8_t calculate_avi_info_frame_checksum(hw_avi_payload_t *payload)
{
	uint8_t checksum;

	checksum = 0x82 + 0x02 + 0x0D;	/* these are set by the hardware */
	return calculate_generic_checksum(payload->ifData, checksum, SIZE_AVI_INFOFRAME);
}
static int is_valid_avi_info_frame(struct mhl_dev_context *dev_context,
					avi_info_frame_t *avif)
{
	uint8_t	checksum;

	checksum = calculate_generic_checksum((uint8_t *)avif, 0, sizeof(*avif));
	if (0 != checksum) {
		MHL_TX_DBG_ERR(dev_context, "AVI info frame checksum is: 0x%02x "
					   "should be 0\n", checksum);
		return 0;

	} else if (0x82 != avif->header.type_code) {
		MHL_TX_DBG_ERR(dev_context, "Invalid AVI type code: 0x%02x\n",
						avif->header.type_code);
		return 0;

	} else if (0x02 != avif->header.version_number) {
		MHL_TX_DBG_ERR(dev_context, "Invalid AVI version: 0x%02x\n",
					   avif->header.version_number);
		return 0;

	} else if (0x0D != avif->header.length) {
		return 0;

	} else {
		return 1;
	}
}

static int is_valid_vsif(struct mhl_dev_context *dev_context,
					vendor_specific_info_frame_t *vsif)
{
	uint8_t	checksum;
	/*
		Calculate the checksum assuming that the payload
			includes the checksum
	*/
	checksum = calculate_generic_checksum((uint8_t *)vsif, 0,
			sizeof(vsif->header) + vsif->header.length );
	if (0 != checksum) {
		MHL_TX_DBG_WARN(dev_context, "VSIF info frame checksum is: 0x%02x "
					   "should be 0\n", checksum);
		/*
			Try again, assuming that the header includes the checksum.
		*/
		checksum = calculate_generic_checksum((uint8_t *)vsif, 0,
			sizeof(vsif->header) + vsif->header.length
			+ sizeof(vsif->payLoad.checksum));
		if (0 != checksum) {
			MHL_TX_DBG_ERR(dev_context, "VSIF info frame checksum "
					"(adjusted for checksum itself) is: 0x%02x "
						   "should be 0\n", checksum);
			return 0;
		}
	}
	if (0x81 != vsif->header.type_code) {
		MHL_TX_DBG_ERR(dev_context, "Invalid VSIF type code: 0x%02x\n",
					   vsif->header.type_code);
		return 0;

	} else if (0x01 != vsif->header.version_number) {
		MHL_TX_DBG_ERR(dev_context, "Invalid VSIF version: 0x%02x\n",
					   vsif->header.version_number);
		return 0;

	} else {
		return 1;
	}
}
static	void	print_vic_modes(struct drv_hw_context *hw_context,uint8_t vic)
{
	int	i;
	struct	vic_name {
		uint8_t		vic;
		char		name[10];
	} vic_name_table[] = {
				  {2, "480P"}
				 ,{4, "720P60"}
				 ,{5, "1080i60"}
				 ,{6, "480i"}
				 ,{16,"1080P60"}
				 ,{17,"576P50"}
				 ,{19,"720P50"}
				 ,{20,"1080i50"}
				 ,{21,"576i50"}
				 ,{31,"1080P50"}
				 ,{32,"1080P24"}
				 ,{33,"1080P25"}
				 ,{34,"1080P30"}
				 ,{0,""} /* to handle the case where the VIC is not found in the table */
	};
#define	NUM_VIC_NAMES	(sizeof(vic_name_table)/sizeof(vic_name_table[0]) )
	/* stop before the terminator */
	for (i = 0; i < (NUM_VIC_NAMES - 1); i++) {
		if (vic == vic_name_table[i].vic) {
			break;
		}
	}
	MHL_TX_DBG_ERR(hw_context, "VIC = %d (%s)\n", vic, vic_name_table[i].name);
}

static void set_mhl_zone_settings(struct mhl_dev_context *dev_context,
					uint32_t pixel_clock_frequency)
{
	struct drv_hw_context	*hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	/*
	 * Zone control setting as per pixel clock frequency
	 */
	if (pixel_clock_frequency > 75000000) {
		MHL_TX_DBG_INFO(hw_context,"zone control: D0\n");
		mhl_tx_write_reg(hw_context,REG_ZONE_CTRL_SW_RST,0xD0);
	} else {
		MHL_TX_DBG_INFO(hw_context,"zone control: E0\n");
		mhl_tx_write_reg(hw_context,REG_ZONE_CTRL_SW_RST,0xE0);
	}

	MHL_TX_DBG_INFO(hw_context,"pixel clock:%d %04x rev %02x\n",
					pixel_clock_frequency,
					hw_context->chip_device_id,
					hw_context->chip_rev_id);
	/*
	 * TODO: Does 8558 need this setting for low frequency modes?
	 *
	 * don't do this for rev 0.0 of 8240 and 8558
	 * Modes below 30MHz need a different zone control
	 */
	if (hw_context->chip_rev_id > 0) {
		if (pixel_clock_frequency < 30000000)
			mhl_tx_write_reg(hw_context, REG_TXMZ_CTRL2, 0x01);
		else
			mhl_tx_write_reg(hw_context, REG_TXMZ_CTRL2, 0x00);
	}
	/*
	 * MSC WRITE_STATUS is required to prepare sink for new mode
	 */
	si_mhl_tx_set_status(dev_context, MHL_STATUS_REG_LINK_MODE,
				dev_context->link_mode);

}

/*
 * This function return the swing value by different pixel clock.
 * If the debug_swiing_value has been set, the return it.
 */
static u8 get_swing_value(struct mhl_dev_context *dev_context, uint32_t pixel_clock_frequency)
{
	if (dev_context->debug_swing_value != -1)
		return dev_context->debug_swing_value;

	if (pixel_clock_frequency < 74000000)
		return dev_context->swing_value[0];
	else if (pixel_clock_frequency < 148000000)
		return dev_context->swing_value[1];
	else
		return dev_context->swing_value[2];
}

/*
 * This function must not be called for DVI mode.
 */
static int	set_hdmi_params(struct mhl_dev_context *dev_context)
{
	uint32_t			pixel_clock_frequency;
	uint32_t			threeDPixelClockRatio;
	uint8_t				packedPixelNeeded = 0;
	uint8_t				fp_3d_mode;
	AviColorSpace_e			input_clr_spc = acsRGB;
	uint8_t				output_clr_spc =acsRGB;
	avi_info_frame_data_byte_4_t	input_video_code;
	struct drv_hw_context		*hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	enum {
		 use_avi_vic
		,use_hardware_totals
	}timing_info_basis=use_avi_vic;

	/* Extract VIC from incoming AVIF */
	input_video_code = hw_context->current_avi_info_frame.payLoad.hwPayLoad.namedIfData.
											  ifData_u.bitFields.VIC;

	/*
	 * From VSIF bytes, figure out if we need to perform
	 * frame packing or not. This helps decide if packed pixel
	 * (16-bit) is required or not in conjunction with the VIC.
	 */
	threeDPixelClockRatio = 1;
	fp_3d_mode = REG_VID_OVRRD_DEFVAL;

	/*
		HDMI spec. v1.4:
		"When transmitting any 2D video format of section 6.3 above, an HDMI Source shall set
		the VIC field to the Video Code for that format. See CEA-861-D section 6.4 for detatils.  The
		additional VIC values from 60 to 64 are defined in Table 8-4.
		When transmitting any 3D video format using the 3D_Structure field in the HDMI Vendor
		Specific InfoFrame, an HDMI Source shall set the AVI InfoFrame VIC field to satisfy the
		relation described in section 8.2.3.2.
		When transmitting any extended video format indicated through use of the HDMI_VIC field
		in the HDMI Vendor Specific InfoFrame or any other format which is not described in the
		above cases, and HDMI Source shall set the AVI InfoFrame VIC field to zero."
	*/
	if (hw_context->valid_vsif) {
			MHL_TX_DBG_WARN(, "valid HDMI VSIF\n");
		if (hvfExtendedResolutionFormatPresent == hw_context->current_vs_info_frame.
				payLoad.pb4.HDMI_Video_Format) {
			/* HDMI_VIC should contain one of 0x01 through 0x04 */
			MHL_TX_DBG_ERR(,"HDMI extended resolution not supported for 8240/8558\n");
			return false;
		} else {
			print_vic_modes(hw_context, (uint8_t) input_video_code.VIC);
			if (0 == input_video_code.VIC) {
				MHL_TX_DBG_ERR(,"AVI VIC is zero!!!\n");
				return false;
			}
			if (hvf3DFormatIndicationPresent == hw_context->current_vs_info_frame.
					payLoad.pb4.HDMI_Video_Format) {

				MHL_TX_DBG_INFO(dev_context,"VSIF indicates 3D\n");
				if (tdsFramePacking == hw_context->current_vs_info_frame.
							payLoad.pb5.ThreeDStructure.threeDStructure) {

					MHL_TX_DBG_INFO(dev_context, "mhl_tx: tdsFramePacking\n");
					threeDPixelClockRatio = 2;

					fp_3d_mode |= BIT_VID_OVRRD_3DCONV_EN_FRAME_PACK;
				}
			}
		}
	}else{ /* no VSIF */
		if (0 == input_video_code.VIC) {
			/*
				This routine will not be called until we positively know (from the downstream EDID)
					that the sink is HDMI.
				We do not support DVI only sources.  The upstream source is expected to choose between
					HDMI and DVI based upon the EDID that we present upstream.
				The other information in the infoframe, even if it is non-zero, is not helpful for
					determining the pixel clock frequency.
				So we try as best we can to infer the pixel clock from the HTOTAL and VTOTAL registers.
			*/
			timing_info_basis = use_hardware_totals;
			MHL_TX_DBG_WARN(,"no VSIF and AVI VIC is zero!!! trying HTOTAL/VTOTAL\n");
		} else {
			print_vic_modes(hw_context, (uint8_t) input_video_code.VIC);
		}
	}

	mhl_tx_write_reg(hw_context, REG_VID_OVRRD, fp_3d_mode);
	/* Do not remember previous VSIF */
	hw_context->valid_vsif = 0;
	hw_context->valid_avif = 0;

	/* make a copy of avif */
	hw_context->outgoingAviPayLoad  = hw_context->current_avi_info_frame.payLoad.hwPayLoad;

	/* compute pixel frequency */
	switch (timing_info_basis) {
	case use_avi_vic:
		pixel_clock_frequency = si_edid_find_pixel_clock_from_AVI_VIC(
								dev_context->edid_parser_context,
								hw_context->current_avi_info_frame.
								payLoad.hwPayLoad.namedIfData.
								ifData_u.bitFields.VIC.VIC);
		break;
	case use_hardware_totals:
		pixel_clock_frequency = si_mhl_tx_find_timings_from_totals(
				dev_context->edid_parser_context);
		if (0 == pixel_clock_frequency) {
			MHL_TX_DBG_ERR(,"VIC was zero and totals not supported\n");
			return false;
		}
		break;
	}

	/* extract input color space */
	input_clr_spc = hw_context->current_avi_info_frame.payLoad.hwPayLoad.namedIfData.
											  ifData_u.bitFields.pb1.colorSpace;

	MHL_TX_DBG_INFO(dev_context, "input_clr_spc = %02X infoData[0]:%02X\n",
		input_clr_spc,
		hw_context->current_avi_info_frame.payLoad.hwPayLoad.namedIfData.ifData_u.
			infoFrameData[0]);

	/*
	 * decide about packed pixel mode
	 */
	pixel_clock_frequency *= threeDPixelClockRatio;
	MHL_TX_DBG_INFO(hw_context, "pixel clock:%u\n",
			pixel_clock_frequency);

	if (qualify_pixel_clock_for_mhl(dev_context->edid_parser_context,
		pixel_clock_frequency, 24)) {
		MHL_TX_DBG_INFO(hw_context, "OK for 24 bit pixels\n");
	} else {
		/* not enough bandwidth for uncompressed video */
		if (si_edid_sink_supports_YCbCr422(dev_context->edid_parser_context)) {
			MHL_TX_DBG_INFO(hw_context, "Sink supports YCbCr422\n");

			if (qualify_pixel_clock_for_mhl(
				dev_context->edid_parser_context, pixel_clock_frequency, 16)) {
				/* enough for packed pixel */
				packedPixelNeeded = 1;
			} else {
				MHL_TX_DBG_ERR(hw_context,"unsupported video mode."
						"pixel clock too high %s\n"
						,si_peer_supports_packed_pixel(dev_context)
							? "" :"(peer does not support packed pixel)."
						);
				return	false;
			}
		} else {
			MHL_TX_DBG_ERR(hw_context,"unsupported video mode."
					"Sink doesn't support 4:2:2.\n");
			return	false;
		}
	}

	/*
	 * Determine output color space if it needs to be 4:2:2 or same as input
	 */
	output_clr_spc = input_clr_spc;

	if (packedPixelNeeded){
		if (packed_pixel_available(dev_context)) {
			u8 swing_value = get_swing_value(dev_context, pixel_clock_frequency);
			MHL_TX_DBG_INFO(hw_context, "setting packed pixel mode\n");

			dev_context->link_mode = MHL_STATUS_PATH_ENABLED | MHL_STATUS_CLK_MODE_PACKED_PIXEL;

			/* enforcing 4:2:2 if packed pixel. */
			output_clr_spc = BIT_EDID_FIELD_FORMAT_YCbCr422;

			mhl_tx_write_reg(hw_context
					, REG_VID_MODE
					, REG_VID_MODE_DEFVAL | BIT_VID_MODE_m1080p_ENABLE);

			mhl_tx_modify_reg(hw_context, REG_MHLTX_CTL4,
						BIT_MHLTX_CTL4_MHL_CLK_RATIO_MASK | BIT_DATA_SWING_CTL_MASK,
						BIT_MHLTX_CTL4_MHL_CLK_RATIO_2X | swing_value);

			mhl_tx_write_reg(hw_context, REG_MHLTX_CTL6, 0x60);

		} else {
			MHL_TX_DBG_ERR(hw_context,
				"unsupported video mode. Packed Pixel not available on sink."
				"Sink's link mode = 0x%02x\n",
				dev_context->dev_cap_cache.mdc.vid_link_mode);
			return false;
		}
	} else {
		u8 swing_value = get_swing_value(dev_context, pixel_clock_frequency);
		MHL_TX_DBG_INFO(hw_context, "normal Mode ,Packed Pixel mode disabled \n");

		dev_context->link_mode = MHL_STATUS_PATH_ENABLED | MHL_STATUS_CLK_MODE_NORMAL;

		mhl_tx_write_reg(hw_context
				, REG_VID_MODE
				, REG_VID_MODE_DEFVAL | BIT_VID_MODE_m1080p_DISABLE);

		mhl_tx_modify_reg(hw_context, REG_MHLTX_CTL4,
						  BIT_MHLTX_CTL4_MHL_CLK_RATIO_MASK | BIT_DATA_SWING_CTL_MASK,
						  BIT_MHLTX_CTL4_MHL_CLK_RATIO_3X | swing_value);

		mhl_tx_write_reg(hw_context, REG_MHLTX_CTL6, 0xA0);
	}

	/* Set input color space */
	mhl_tx_write_reg(hw_context
			, REG_TPI_INPUT
			, colorSpaceTranslateInfoFrameToHw[input_clr_spc]);

	/* Set output color space */
	if (packedPixelNeeded) {
		mhl_tx_write_reg(hw_context
				, REG_TPI_OUTPUT
				, colorSpaceTranslateInfoFrameToHw[output_clr_spc] | BIT_TPI_OUTPUT_QUAN_RANGE_LIMITED | BIT_TPI_OUTPUT_709_CONVERSION);
	} else {
		mhl_tx_write_reg(hw_context
				, REG_TPI_OUTPUT
				, colorSpaceTranslateInfoFrameToHw[output_clr_spc]);
	}

	set_mhl_zone_settings(dev_context,pixel_clock_frequency);
	/*
	 * Prepare outgoing AVIF for later programming the registers
	 *
	 * the checksum itself is included in the calculation.
	 */
	hw_context->outgoingAviPayLoad.namedIfData.checksum = 0;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.bitFields.pb1.colorSpace
		= output_clr_spc;
	hw_context->outgoingAviPayLoad.namedIfData.checksum =
		calculate_avi_info_frame_checksum(&hw_context->outgoingAviPayLoad);

	return true;
}

/*
	process_info_frame_change
		called by the MHL Tx driver when a
        new AVI info frame is received from upstream
		OR
		called by customer's SOC video driver when a mode change is desired.
*/

void process_info_frame_change(struct drv_hw_context *hw_context,
				vendor_specific_info_frame_t *vsif, avi_info_frame_t *avif)
{
	bool mode_change = false;
	struct mhl_dev_context	*dev_context;

	dev_context = container_of((void *)hw_context, struct mhl_dev_context,
							   drv_context);
	if (NULL != vsif) {
		if(is_valid_vsif(dev_context, vsif)) {
			hw_context->current_vs_info_frame = *vsif;
			hw_context->valid_vsif = 1;
			mode_change = true;
		}
	}
	if (NULL != avif) {
		if(is_valid_avi_info_frame(dev_context, avif)) {
			hw_context->current_avi_info_frame = *avif;
			hw_context->valid_avif = 1;
			mode_change = true;
		}
	}
	if (mode_change) {
		int cstat_p3;
		int bits_of_interest;
		cstat_p3 = mhl_tx_read_reg(hw_context, REG_TMDS_CSTAT_P3);
		bits_of_interest = cstat_p3 & (BIT_TMDS_CSTAT_P3_PDO_MASK |BIT_TMDS_CSTAT_P3_SCDT);

		if ((BIT_TMDS_CSTAT_P3_PDO_CLOCK_DETECTED |BIT_TMDS_CSTAT_P3_SCDT)
				== bits_of_interest) {
			start_video(hw_context,dev_context->edid_parser_context);
		}
	}
}

#define dump_edid_fifo(hw_context, block_number) /* do nothing */

static int init_rx_regs(struct drv_hw_context *hw_context)
{
	/* power up the RX */
	if (hw_context->chip_device_id == DEVICE_ID_8558) {
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0x7D);
	} else {
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0x75);
	}
	/* TODO: add to PR. Default for 2A4 is 0x0f */
	mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL3, 0x00);

	/* Due to packet overflow, AVIF write using TPI:0C fails.
	Until ready, we drop all packets*/
	mhl_tx_write_reg(hw_context, REG_PKT_FILTER_0, 0xFF);
	mhl_tx_write_reg(hw_context, REG_PKT_FILTER_1, 0xFF);

	mhl_tx_modify_reg(hw_context, REG_HDMI_CLR_BUFFER,
				BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_EN_MASK,
				BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_EN_CLEAR);

	return 0;
}

static void bch_force_avif(struct drv_hw_context *hw_context)
{
	mhl_tx_write_reg(hw_context, REG_HDMI_BCH_CORRECTED_THRESHOLD, 0x01);
	mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL4, 0x30);
}
static void bch_setup(struct drv_hw_context *hw_context)
{
	/*
	 * This workaround ensures NEW_AVI interrupt when AVI does not change
	 * but due to downstream HDMI or MHL hot plug, we toggle upstream HPD
	 * and expect a new AVI.
	 */
	/* Clear BCH register for infr workaround */
	mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL4, 0);
	mhl_tx_write_reg(hw_context, REG_HDMI_BCH_CORRECTED_THRESHOLD, 0);

	/* Clear NEW_AVI interrupt caused by above two lines.*/
	mhl_tx_write_reg(hw_context,
			g_intr_tbl[INTR_INFR].stat_page,
			g_intr_tbl[INTR_INFR].stat_offset,
			BIT_INTR8_CEA_NEW_AVI);

}

static int drive_hpd_high(struct drv_hw_context *hw_context)
{
	hpd_control_mode mode;
	int ret_val=-1;
	mode =  platform_get_hpd_control_mode();
	if (HPD_CTRL_OPEN_DRAIN == mode)
		ret_val = mhl_tx_write_reg(hw_context, REG_HPD_CTRL, 0x75);
	else if (HPD_CTRL_PUSH_PULL == mode)
		ret_val = mhl_tx_write_reg(hw_context, REG_HPD_CTRL, 0x35);
	if (0 == get_config(hw_context, TRANSCODE_MODE)) {
		bch_force_avif(hw_context);
	}
	return ret_val;
}

static int drive_hpd_low(struct drv_hw_context *hw_context)
{
	hpd_control_mode mode;
	int ret_val=-1;
	mode =  platform_get_hpd_control_mode();
	if (0 == get_config(hw_context, TRANSCODE_MODE)) {
		mhl_tx_modify_reg(hw_context
			, REG_EDID_CTRL
			, BIT_EDID_CTRL_EDID_PRIME_VALID_MASK
			, BIT_EDID_CTRL_EDID_PRIME_VALID_DISABLE
			);
	}
	if (HPD_CTRL_OPEN_DRAIN == mode)
		ret_val = mhl_tx_write_reg(hw_context, REG_HPD_CTRL, 0x55);
	else if (HPD_CTRL_PUSH_PULL == mode)
		ret_val = mhl_tx_write_reg(hw_context, REG_HPD_CTRL, 0x15);

	if (0 == get_config(hw_context, TRANSCODE_MODE)) {
		bch_setup(hw_context);
	}
	return ret_val;
}

int si_mhl_tx_drv_set_upstream_edid(struct drv_hw_context *hw_context,
									uint8_t *edid, uint16_t length)
{
	uint8_t	reg_val;

	reg_val = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
	if (!(BIT_CBUS_HPD & reg_val)) {
		return -1;
	}

#ifdef NEVER //(
	if(si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG
				, TMDS_OUTPUT_CONTROL_POWER_DOWN
				| AV_MUTE_MUTED
				| TMDS_OUTPUT_MODE_HDMI
				);
	} else {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG
				, TMDS_OUTPUT_CONTROL_POWER_DOWN
				| AV_MUTE_MUTED
				| TMDS_OUTPUT_MODE_DVI
				);
	}
#endif //)


	init_rx_regs(hw_context);

	/* choose EDID instead of devcap to appear at the FIFO */
	mhl_tx_write_reg(hw_context
			, REG_EDID_CTRL
			, BIT_EDID_CTRL_EDID_PRIME_VALID_DISABLE
			| BIT_EDID_CTRL_FIFO_SELECT_EDID
			| BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
			| BIT_EDID_CTRL_EDID_MODE_EN_ENABLE
			);

	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, 0);
	mhl_tx_write_reg_block(hw_context, REG_EDID_FIFO_WR_DATA, length, edid);

	mhl_tx_write_reg(hw_context
					, REG_EDID_CTRL
					, BIT_EDID_CTRL_EDID_PRIME_VALID_ENABLE
					| BIT_EDID_CTRL_FIFO_SELECT_EDID
					| BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
					| BIT_EDID_CTRL_EDID_MODE_EN_ENABLE
					);

	MHL_TX_DBG_INFO(hw_context, "Expose EDID\n");

	/* Enable CKDT interrupt only. For HDMI we must first wait for
	 * stable CKDT, apply infoframe workaround then enable INTR_INFR
	 */
	enable_intr(hw_context
			, INTR_CKDT
			, (BIT_INTR5_CKDT_CHANGE | BIT_INTR5_SCDT_CHANGE));

	/* Disable EDID interrupt */
	enable_intr(hw_context, INTR_EDID, 0);

	/* Enable h/w automation of WRITE_BURST */
	hw_context->ready_for_mdt = true;

#ifdef ENABLE_GEN2 //(
	enable_gen2_write_burst(hw_context);
#endif //)

	/*
		Before exposing the EDID to upstream device, setup to drop all packets.
		This ensures we do not get Packet Overflow interrupt.
		Dropping all packets means we still get the AVIF interrupts which is crucial.
		Packet filters must be disabled until after TMDS is enabled.
	*/
	mhl_tx_write_reg(hw_context, REG_PKT_FILTER_0, 0xFF);
	mhl_tx_write_reg(hw_context, REG_PKT_FILTER_1, 0xFF);

	/*
	 * enable infoframe interrupt
	 */
	enable_intr(hw_context
				,INTR_INFR
				,(BIT_INTR8_CEA_NEW_AVI | BIT_INTR8_CEA_NEW_VSI)
				);

	/* HPD was held low all this time. Now we send an HPD high */
	drive_hpd_high(hw_context);
	return 0;
}



static void power_up(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "called\n");

	if (hw_context->chip_device_id == DEVICE_ID_8558) {
		// Refer to the Power Up and Power Down flowchart in PR.

		// Assert SRST
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, (0));

		// Turn the power switch ON
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, (BIT_MASTER_POWER_CTRL));

		// Wait 50 ms
		msleep(50);

		// Deassert SRST
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, (BIT_OSC_EN | BIT_MASTER_POWER_CTRL));

		// Disable Isolation
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, (BIT_OSC_EN | BIT_ISO_EN | BIT_MASTER_POWER_CTRL));

		// Enable Tx Core. Defer enabling Rx core until MHL is found.
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, (BIT_PDNTX12 | BIT_OSC_EN | BIT_ISO_EN | BIT_PCLK_EN | BIT_MASTER_POWER_CTRL));
	}
	else
	{
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, (BIT_PDNTX12 | BIT_OSC_EN | BIT_PCLK_EN | BIT_MASTER_POWER_CTRL));

		/* Toggle power strobe on 8240 */
		mhl_tx_modify_reg(hw_context, REG_DISC_CTRL1, BIT_DISC_CTRL1_STROBE_OFF, 0);
	}
}

#define	MHL_LOGICAL_DEVICE_MAP		(MHL_DEV_LD_GUI)
#define DEVCAP_REG(x) REG_CBUS_DEVICE_CAP_0 | DEVCAP_OFFSET_##x

uint8_t	dev_cap_values[] = {
				DEVCAP_VAL_DEV_STATE
			,	DEVCAP_VAL_MHL_VERSION
			,	DEVCAP_VAL_DEV_CAT
			,	DEVCAP_VAL_ADOPTER_ID_H
			,	DEVCAP_VAL_ADOPTER_ID_L
			,	DEVCAP_VAL_VID_LINK_MODE
			,	DEVCAP_VAL_AUD_LINK_MODE
			,	DEVCAP_VAL_VIDEO_TYPE
			,	DEVCAP_VAL_LOG_DEV_MAP
			,	DEVCAP_VAL_BANDWIDTH
			,	DEVCAP_VAL_FEATURE_FLAG
			,	0
			,	0
			,	DEVCAP_VAL_SCRATCHPAD_SIZE
			,	DEVCAP_VAL_INT_STAT_SIZE
			,	DEVCAP_VAL_RESERVED
};

static int init_regs(struct drv_hw_context *hw_context)
{
	int ret_val = 0;

	MHL_TX_DBG_INFO(hw_context, "called\n");

	/* default values for flags */
	hw_context->video_ready = false;
	hw_context->video_path = 1;
	hw_context->ready_for_mdt = false;
	hw_context->audio_poll_enabled = false;

	hw_context->rx_hdmi_ctrl2_defval = REG_RX_HDMI_CTRL2_DEFVAL_DVI;
	/*
	 * This workaround ensures NEW_AVI interrupt when AVI does not change
	 * but due to downstream HDMI or MHL hot plug, we toggle upstream HPD
	 * and expect a new AVI.
	 */
	/* Clear BCH register for infr workaround */
	mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL4, 0);
	mhl_tx_write_reg(hw_context, REG_HDMI_BCH_CORRECTED_THRESHOLD, 0);

	/* Clear NEW_AVI interrupt caused by above two lines.*/
	mhl_tx_write_reg(hw_context,
			g_intr_tbl[INTR_INFR].stat_page,
			g_intr_tbl[INTR_INFR].stat_offset,
			BIT_INTR8_CEA_NEW_AVI);

	/* Drop the Hot Plug to upstream and hide EDID */
	drive_hpd_low(hw_context);
	mhl_tx_write_reg(hw_context, REG_EDID_CTRL, BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE);

	/*
	 * 8240 has correct default value for REG_DCTL
	 * wake pulses necessary in all modes
	 * No OTG, Discovery pulse proceed, Wake pulse not bypassed
	 */
	mhl_tx_write_reg(hw_context, REG_DISC_CTRL9
						, BIT_DC9_WAKE_DRVFLT
						| BIT_DC9_DISC_PULSE_PROCEED
						);

	if (get_config(hw_context, TRANSCODE_MODE)) {

		init_transcode_mode(hw_context);

		if (hw_context->chip_device_id == DEVICE_ID_8558) {
			mhl_tx_write_reg(hw_context, REG_DCTL,
							 BIT_DCTL_EXT_DDC_SEL |
							 BIT_DCTL_TRANSCODE_ON |
							 BIT_DCTL_TLCK_PHASE_INVERTED);
		}

	} else {
		/* Enable TxPLL Clock */
		mhl_tx_write_reg(hw_context, REG_TMDS_CLK_EN, 0x01);

		/* Enable Tx Clock Path & Equalizer */
		mhl_tx_write_reg(hw_context, REG_TMDS_CH_EN, 0x11);

		/* Enable TPI */
		ret_val = mhl_tx_read_reg(hw_context, REG_TPI_SEL);
		ret_val &= ~BIT_TPI_SEL_SW_TPI_EN_MASK;
		ret_val |= BIT_TPI_SEL_SW_TPI_EN_HW_TPI;
		mhl_tx_write_reg(hw_context, REG_TPI_SEL, ret_val);

		mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG, 0);

		/*
		 * TODO: TPI:0xBB has bad default. Need to document in the PR
		 * If this initialization is not changed, we have unstable HDCP
		 * (while testing 9612 as source)
		 */
		mhl_tx_write_reg(hw_context, REG_TPI_HW_OPT3, 0x76);

		/* TX Source termination ON */
		mhl_tx_write_reg(hw_context, REG_MHLTX_CTL1,
						 BIT_MHLTX_CTL1_TX_TERM_MODE_100DIFF |
						 BIT_MHLTX_CTL1_DISC_OVRIDE_ON);

		mhl_tx_write_reg(hw_context, REG_MHLTX_CTL3,
						 (REG_MHLTX_CTL3_DEFVAL & ~BIT_MHLTX_CTL3_DAMPING_SEL_MASK)
						 | BIT_MHLTX_CTL3_DAMPING_SEL_150_OHM);

		mhl_tx_modify_reg(hw_context, REG_MHLTX_CTL4,
						  BIT_CLK_SWING_CTL_MASK | BIT_DATA_SWING_CTL_MASK,
						  0x33);

		/*
		 * TODO: Is SRST required?
		 */
		/* synchronous s/w reset */
		mhl_tx_write_reg(hw_context, REG_ZONE_CTRL_SW_RST, 0x60);
		mhl_tx_write_reg(hw_context, REG_ZONE_CTRL_SW_RST, 0xD0);

		if (hw_context->chip_device_id == DEVICE_ID_8558) {

			/* Tx data termination for 100 Ohm */
			mhl_tx_write_reg(hw_context, REG_MHLTX_CFG_TERM, 0xE0);

			/* Enable Tx outputs */
			mhl_tx_write_reg(hw_context, REG_MHLTX_CFG_OE, 0x3C);

			/* Ignore VBUS */
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL8,0x01);

			/* Enable CBUS discovery */
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL1,VAL_DISC_CTRL1_DEFAULT | BIT_DISC_CTRL1_MHL_DISCOVERY_ENABLE);

		} else if (hw_context->chip_device_id == DEVICE_ID_8240) {

			/* Ignore VBUS, wait for usbint_clr */
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL8,0x03);

			mhl_tx_write_reg(hw_context, REG_DISC_CTRL2, 0xA5);

			/* Enable CBUS discovery */
	    	mhl_tx_write_reg(hw_context, REG_DISC_CTRL1,VAL_DISC_CTRL1_DEFAULT | BIT_DISC_CTRL1_STROBE_OFF | BIT_DISC_CTRL1_MHL_DISCOVERY_ENABLE);

			/* MHL CBUS discovery */
			/* TODO: This line is redundant. Remove it after CBUS CTS etc. */
//	    	mhl_tx_write_reg(hw_context, REG_DISC_CTRL3, BIT_DC3_DEFAULT);

		}

		if (hw_context->chip_device_id == DEVICE_ID_8558) {
			/* MHL Auto FIFO Reset */
			mhl_tx_write_reg(hw_context, REG_PWD_SRST, 0x80);
		}

		/* enable transcode mode of operation */
		mhl_tx_write_reg(hw_context, REG_DCTL,
						 BIT_DCTL_TRANSCODE_OFF |
						 BIT_DCTL_TLCK_PHASE_INVERTED);

		/* enable CKDT. Interrupt will be enabled once we have parsed EDID */
		mhl_tx_modify_reg(hw_context, REG_TMDS_CCTRL,
								BIT_TMDS_CCTRL_CKDT_EN,
								 BIT_TMDS_CCTRL_CKDT_EN);

		/* set time base for one second to be 60Hz/4*5 + 4*/
		mhl_tx_write_reg(hw_context
				, REG_TPI_HDCP_TIMER_1_SEC
				, 79
				);
	}
	/* setup local DEVCAP and few more CBUS registers. */
	{
			/*
			 * Fill-in DEVCAP device ID values with those read
			 * from the transmitter.
			 */
			dev_cap_values[DEVCAP_OFFSET_DEVICE_ID_L] = 0;
			dev_cap_values[DEVCAP_OFFSET_DEVICE_ID_H] = 0;

			/* Setup local DEVCAP registers */
			mhl_tx_write_reg_block(hw_context, DEVCAP_REG(DEV_STATE),
								   ARRAY_SIZE(dev_cap_values), dev_cap_values);

			/*
			 * Make sure MDT registers are initialized and the MDT
			 * transmit/receive are both disabled.
			 */
			mhl_tx_write_reg(hw_context, REG_CBUS_MDT_XMIT_TIMEOUT, 100);

			/* Clear transmit FIFOs and make sure MDT transmit is disabled */
			mhl_tx_write_reg(hw_context, REG_CBUS_MDT_XMIT_CONTROL, 0x03);

			/* Disable MDT transmit preemptive handshake option */
			mhl_tx_write_reg(hw_context, REG_CBUS_MDT_XFIFO_STAT, 0x00);

			mhl_tx_write_reg(hw_context, REG_CBUS_MDT_RCV_TIMEOUT, 100);

			/* 2013-03-01 bugzilla 27180 */
			mhl_tx_write_reg(hw_context, REG_CBUS_LINK_XMIT_BIT_TIME, 0x1D);
	}
#ifdef ENABLE_GEN2 //(
	/*
		Disable h/w automation of WRITE_BURST.
		3D packets are handled using legacy WRITE_BURST method.
	*/
	disable_gen2_write_burst(hw_context);
#endif //)
	hw_context->ready_for_mdt = false;

	return ret_val;
}

void si_mhl_tx_drv_disable_video_path(struct drv_hw_context *hw_context)
{
	/* If video was already being output */
	if(hw_context->video_ready && (0 == (AV_MUTE_MUTED &
			 mhl_tx_read_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG)))) {

		/* stop hdcp and video and remember */
		stop_video(hw_context);
		hw_context->video_path = 0;
	}
}

void si_mhl_tx_drv_enable_video_path(struct drv_hw_context *hw_context)
{
	uint8_t	mask = (TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK);
	uint8_t	reg;

	/* if a path_en = 0 had stopped the video, restart it unless done already. */
	if(hw_context->video_ready && (0 == hw_context->video_path)) {
		/* remember ds has enabled our path */
		hw_context->video_path = 1;

		reg  = mhl_tx_read_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG);

		if(mask == (mask & reg)) {
			start_video(hw_context,hw_context->intr_info->edid_parser_context);
		}
	}
}

void si_mhl_tx_drv_content_off(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(dev_context, "RAP CONTENT_OFF video %sready\n",hw_context->video_ready?"":"NOT ");
	/* If video was already being output */
	if(hw_context->video_ready && (0 == (AV_MUTE_MUTED &
			 mhl_tx_read_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG)))) {

		MHL_TX_DBG_INFO(dev_context, "RAP CONTENT_OFF\n");
		/* stop hdcp and video and remember */
		stop_video(hw_context);
	}
}

void si_mhl_tx_drv_content_on(struct drv_hw_context *hw_context)
{
	uint8_t	mask = (TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK);
	uint8_t	reg;

	/* if a path_en = 0 had stopped the video, restart it unless done already. */
	if(hw_context->video_ready) {

		reg  = mhl_tx_read_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG);

		if(mask == (mask & reg)) {
			start_video(hw_context,hw_context->intr_info->edid_parser_context);
		}
	}
}

static void ensure_avif_part2(struct drv_hw_context *hw_context)
{
	uint8_t	new_ckdt;

	 /*Disable TMDS output power because input is unstable. */
	mhl_tx_modify_reg(hw_context, REG_TMDS_CCTRL, BIT_TMDS_CCTRL_TMDS_OE, 0);

	if (1 == hw_context->ckdt_done) {
		mhl_tx_write_reg(hw_context, REG_MHLTX_CTL2,
			 	 	  hw_context->saved_reg_mhltx_ctl2);
	}

	new_ckdt = mhl_tx_read_reg(hw_context, REG_TMDS_CSTAT_P3);
	new_ckdt &= BIT_TMDS_CSTAT_P3_PDO_MASK;

	if (BIT_TMDS_CSTAT_P3_PDO_CLOCK_DETECTED == new_ckdt) {
		/* Mask out CKDT interrupt */
		mhl_tx_modify_reg(hw_context, REG_INTR5_MASK,
						  BIT_INTR5_CKDT_CHANGE, 0);

	}
}
#ifdef	EXAMPLE_ONLY	// These functions are not called from anywhere.
static	void	mute_video(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "AV muted\n");
	mhl_tx_modify_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG,
						AV_MUTE_MASK,
						AV_MUTE_MUTED);
}
#endif	//	EXAMPLE_ONLY

static void unmute_video(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "AV unmuted.\n");

	if(si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_MODE_HDMI);
	} else {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_MODE_DVI);
	}
	/* Now we can entertain PATH_EN */
	hw_context->video_ready = 1;
}

/*
	Sequence of operations in this function is important.
	1. Turn off HDCP interrupt
	2. Turn of TMDS output
	3. Turn off HDCP engine/encryption
	4. Clear any leftover HDCP interrupt
*/
static void stop_video(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "stop video\n");

	/* Turn off HDCP interrupt */
	enable_intr(hw_context, INTR_HDCP, (0x00));

	/* We must maintain the output bit (bit 0) to allow just one bit change
	 * later when BKSV read is triggered. */
	if(si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG
				, TMDS_OUTPUT_CONTROL_POWER_DOWN
				| AV_MUTE_MUTED
				| TMDS_OUTPUT_MODE_HDMI
				);
	} else {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG
				, TMDS_OUTPUT_CONTROL_POWER_DOWN
				| AV_MUTE_MUTED
				| TMDS_OUTPUT_MODE_DVI
				);
	}
	/* stop hdcp engine */
	mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG, 0);

	/* clear any leftover hdcp interrupt */
	mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page, g_intr_tbl[INTR_HDCP].stat_offset, 0xff);
}


static void	start_hdcp(struct drv_hw_context *hw_context)
{
	struct mhl_dev_context *dev_context;
	dev_context = container_of((void *)hw_context, struct mhl_dev_context, drv_context);

	MHL_TX_DBG_INFO(hw_context, "start_hdcp\n");

	if (dev_context->hdcp_status == 0) {
		unmute_video(hw_context);
		return;
	}

	/* First stop hdcp and video */
	//stop_video(hw_context);

#if 0 //(
	/* Came here too often, pause a bit. */
	if( (hdcp_bksv_err_count > HDCP_ERROR_THRESHOLD) ||
			(hdcp_link_err_count > HDCP_ERROR_THRESHOLD) ||
			(hdcp_reneg_err_count > HDCP_ERROR_THRESHOLD) ||
			(hdcp_suspend_err_count > HDCP_ERROR_THRESHOLD))
	{
		MHL_TX_DBG_ERR(hw_context, "Too many HDCP Errors: bksv_err= %d, reneg_err= %d, link_err= %d, suspend_err= %d\n",
				hdcp_bksv_err_count, hdcp_reneg_err_count, hdcp_link_err_count, hdcp_suspend_err_count);
		hdcp_bksv_err_count = hdcp_reneg_err_count = hdcp_link_err_count = hdcp_suspend_err_count = 0;

		//msleep(10 * 1000);

		/*
		 * Check CKDT and SCDT.
		 * It is possible that lack of clock instability is why hdcp is not
		 * succeeding after repeated retries
		 *
		 * Do not continue this anymore. Returning without restarting HDCP
		 * ensures this thread is completely killed.
		 */
		return;
	}
#endif //)
	/* Ensure we get HDCP interrupts now onwards. Clear interrupt first. */
	mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page, g_intr_tbl[INTR_HDCP].stat_offset, 0xff);

	if(get_cbus_connection_status(hw_context)) {
		/* Enable HDCP interrupt */
		enable_intr(hw_context, INTR_HDCP,
				 (  BIT_TPI_INTR_ST0_HDCP_AUTH_STATUS_CHANGE_EVENT
			  	  | BIT_TPI_INTR_ST0_HDCP_SECURITY_CHANGE_EVENT
			  	  | BIT_TPI_INTR_ST0_BKSV_DONE
			  	  | BIT_TPI_INTR_ST0_BKSV_ERR
				 ));
		msleep(250);
		/*
		 * Chip requires only bit 4 to change for BKSV read
		 * No other bit should change.
		 */
		mhl_tx_modify_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG,
					TMDS_OUTPUT_CONTROL_MASK,
					TMDS_OUTPUT_CONTROL_ACTIVE);
	}
}

/*
 * start_video
 *
 *
 */

#ifdef CONFIG_MHL_MONITOR_WORKAROUND
int start_video(struct drv_hw_context *hw_context, void *edid_parser_context)
#else
static	int start_video(struct drv_hw_context *hw_context, void *edid_parser_context)
#endif
{
	struct mhl_dev_context	*dev_context;
	dev_context = get_mhl_device_context(hw_context);
	/*
	 * stop hdcp and video
	 * this kills any hdcp thread going on already
	 */
	stop_video(hw_context);

	/*
	 * if path has been disabled by PATH_EN = 0 return with error;
	 * When enabled, this function will be called again.
	 * if downstream connection has been lost (CLR_HPD), return with error.
	 */
	if ((0 == hw_context->video_path)
			|| (0 == get_cbus_connection_status(hw_context))
			|| (false == dev_context->misc_flags.flags.rap_content_on)) {
		return false;
	}
	if (get_config(hw_context,TRANSCODE_MODE)) {

		unmute_video(hw_context);

		return true;
	}
	if (ddc_bypass_on == ddc_bypass_state) {
		ddc_bypass_state = ddc_bypass_start_video;
		MHL_TX_DBG_ERR(,"waiting for DDC bypass disable\n");
		return false;
	}
	/*
	 * For DVI, output video w/o infoframe; No video settings are changed.
	 */
	if (si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		/*
		 * setup registers for packed pixel, 3D, colors and AVIF
		 * using incoming info frames. VSIF sent out by the h/w.
		 */
		mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2
				, hw_context->rx_hdmi_ctrl2_defval = REG_RX_HDMI_CTRL2_DEFVAL_HDMI
				| BIT_RX_HDMI_CTRL2_VSI_MON_SEL_AVI_INFOFRAME);
		if(false == set_hdmi_params(dev_context)) {
			/* Do not disrupt an ongoing video for bad incoming infoframes */
			return false;
		}
	} else {
		uint32_t pixel_clock_frequency;
		mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2
				, hw_context->rx_hdmi_ctrl2_defval = REG_RX_HDMI_CTRL2_DEFVAL_DVI
				| BIT_RX_HDMI_CTRL2_VSI_MON_SEL_AVI_INFOFRAME);
		pixel_clock_frequency = si_mhl_tx_find_timings_from_totals(
				dev_context->edid_parser_context);
		set_mhl_zone_settings(dev_context,pixel_clock_frequency);
	}

	MHL_TX_DBG_INFO(hw_context,"Start HDCP Authentication\n");

	start_hdcp(hw_context);
	/*
	 * TODO: Check if the following block has to be performed again at
	 * HDCP restart.
	 *
	 * Start sending out AVIF now.
	 * Also ensure other appropriate frames are being forwarded or dropped.
	*/
	if (si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		/*
		 * Send infoframe out
		 */
		mhl_tx_write_reg_block(hw_context, REG_TPI_AVI_CHSUM,
							   sizeof(hw_context->outgoingAviPayLoad.ifData),
							   (uint8_t*)&hw_context->outgoingAviPayLoad.ifData);
		/*
		 * Change packet filters to drop AIF and GCP.
		 * TODO: Describe this in the PR appropriately.
		*/
		mhl_tx_write_reg(hw_context, REG_PKT_FILTER_0, 0xA5);
		/*
		 * enable HDMI to MHL VSIF conversion ( clear bit 7)
		 * and drop generic infoframes  ( set bit 1)
		 */
		mhl_tx_write_reg(hw_context, REG_PKT_FILTER_1,0x02);

	}
	return true;
}
static int hdcp_isr(struct drv_hw_context *hw_context, uint8_t tpi_int_status)
{
	uint8_t		query_data;

	query_data = mhl_tx_read_reg(hw_context, TPI_HDCP_QUERY_DATA_REG);
	MHL_TX_DBG_INFO(hw_context, "R3D= %02x R29= %02x\n", tpi_int_status, query_data);

	if (BIT_TPI_INTR_ST0_BKSV_DONE & tpi_int_status) {
		if (PROTECTION_TYPE_MASK & query_data) {
			int temp;
			if (!ap_hdcp_success) {
				if (!hdmi_hpd_status()) {
					pr_info("sii8240: HDMI hpd is low\n");
				} else {
					pr_info("sii8240: workaround for drm\n");
					mhl_tx_modify_reg(hw_context,REG_DCTL,BIT_DCTL_EXT_DDC_SEL,BIT_DCTL_EXT_DDC_SEL);
					hdmi_hdcp_set_state(HDCP_STATE_AUTHENTICATING);
					if (hdmi_hdcp_authentication_from_mhl()) {
						pr_err("%s: HDMI HDCP Auth Part I failed\n", __func__);
						ap_hdcp_success = false;
					} else {
						ap_hdcp_success = true;
					}
					mhl_tx_modify_reg(hw_context,REG_DCTL,BIT_DCTL_EXT_DDC_SEL,0);
					hdmi_hdcp_set_state(HDCP_STATE_AUTHENTICATED);
					msleep(100);
				}
			}

			/*
				TODO: Document this as SWWA 27396.
				Wait for bottom five bits of debug 6 register to change before
				reading query_data (0x29) register.
			*/
			do{
				temp = mhl_tx_read_reg(hw_context,REG_TPI_HW_DBG6) & 0x1F;
			} while (temp == 2);
			if (temp < 2)
				return 0; /* TPI hdcp state machine has restarted, just wait for another BKSV_DONE */
			query_data = mhl_tx_read_reg(hw_context, TPI_HDCP_QUERY_DATA_REG);

			/*
				If the downstream device is a repeater, enforce a 5 second delay
				to pass HDCP CTS 1B-03.
				TODO: Describe this in the PR.
			*/
			if (HDCP_REPEATER_YES== (HDCP_REPEATER_MASK & query_data)){
				msleep(HDCP_RPTR_CTS_DELAY_MS);
			}

			// Start authentication here
			mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG,
					  BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MAX
					| BIT_TPI_HDCP_CONTROL_DATA_DOUBLE_RI_CHECK_ENABLE);
		}
	} else if ( BIT_TPI_INTR_ST0_BKSV_ERR & tpi_int_status) {
		hdcp_bksv_err_count++;
		start_hdcp(hw_context);
	} else if (BIT_TPI_INTR_ST0_HDCP_SECURITY_CHANGE_EVENT & tpi_int_status) {
		int link_status;

		link_status = query_data & LINK_STATUS_MASK;

		switch (link_status) {
		case LINK_STATUS_NORMAL:
			unmute_video(hw_context);
			break;

		case LINK_STATUS_LINK_LOST:
			hdcp_link_err_count++;
			start_hdcp(hw_context);
			break;
		case LINK_STATUS_RENEGOTIATION_REQ:
			MHL_TX_DBG_INFO(hw_context
					, "tpi BSTATUS2: 0x%x\n"
					, mhl_tx_read_reg(hw_context,REG_TPI_BSTATUS2)
					);
			hdcp_reneg_err_count++;
			/* Disabling TMDS output here will disturb the clock */
			mhl_tx_modify_reg(hw_context
					, TPI_SYSTEM_CONTROL_DATA_REG
					, AV_MUTE_MASK
					, AV_MUTE_MUTED);

			/* send TPI hardware to HDCP_Prep state */
			mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG, 0);
			break;
		case LINK_STATUS_LINK_SUSPENDED:
			hdcp_suspend_err_count++;
			start_hdcp(hw_context);
			break;
		}
	} /* Check if HDCP state has changed: */
	else if (BIT_TPI_INTR_ST0_HDCP_AUTH_STATUS_CHANGE_EVENT & tpi_int_status) {
		uint8_t new_link_prot_level;

		new_link_prot_level = (uint8_t)
				(query_data & (EXTENDED_LINK_PROTECTION_MASK |
							LOCAL_LINK_PROTECTION_MASK));

		switch (new_link_prot_level) {
		case (EXTENDED_LINK_PROTECTION_NONE | LOCAL_LINK_PROTECTION_NONE):
			hdcp_link_err_count++;
			start_hdcp(hw_context);
			break;

		case EXTENDED_LINK_PROTECTION_SECURE:
		case LOCAL_LINK_PROTECTION_SECURE:
		case (EXTENDED_LINK_PROTECTION_SECURE | LOCAL_LINK_PROTECTION_SECURE):
			unmute_video(hw_context);
			break;
		}
	}
	return 0;
}

static int int_8_isr(struct drv_hw_context *hw_context, uint8_t intr_8_status)
{
	vendor_specific_info_frame_t	vsif;
	avi_info_frame_t		avif;

	/* Resolution change interrupt (NEW_AVIF or NEW_VSIF) */
	if (BIT_INTR8_CEA_NEW_VSI & intr_8_status) {
			MHL_TX_DBG_ERR(hw_context, "got NEW_VSI\n");

			mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2
					, hw_context->rx_hdmi_ctrl2_defval
					| BIT_RX_HDMI_CTRL2_VSI_MON_SEL_VS_INFOFRAME);

			mhl_tx_read_reg_block(hw_context
					, REG_RX_HDMI_MON_PKT_HEADER1
					, sizeof(vsif)
					, (uint8_t *)&vsif
					);
	}

	if (BIT_INTR8_CEA_NEW_AVI & intr_8_status) {

			MHL_TX_DBG_ERR(hw_context, "got NEW_AVIF\n");
			/*
			 * If this is the first time after ckdt,
			 * we need to restore tmds_oe registers
			 */
			if (2 != hw_context->ckdt_done) {
				ensure_avif_part2(hw_context);
				hw_context->ckdt_done = 2;
			}
			/*
			 * Read AVIF packet
			 */
			mhl_tx_write_reg(hw_context, REG_RX_HDMI_CTRL2
					, hw_context->rx_hdmi_ctrl2_defval
					| BIT_RX_HDMI_CTRL2_VSI_MON_SEL_AVI_INFOFRAME);

			mhl_tx_read_reg_block(hw_context
						, REG_RX_HDMI_MON_PKT_HEADER1
						, sizeof(avif)
						, (uint8_t *)&avif
						);
	}

	switch (intr_8_status &(BIT_INTR8_CEA_NEW_VSI|BIT_INTR8_CEA_NEW_AVI)) {
        case BIT_INTR8_CEA_NEW_VSI:
            process_info_frame_change(hw_context,&vsif,NULL);
            break;
        case BIT_INTR8_CEA_NEW_AVI:
            process_info_frame_change(hw_context,NULL,&avif);
            break;
        case (BIT_INTR8_CEA_NEW_VSI|BIT_INTR8_CEA_NEW_AVI):
            process_info_frame_change(hw_context ,&vsif ,&avif);
            break;
	}
	return 0;
}

static int int_9_isr(struct drv_hw_context *hw_context, uint8_t int_9_status)
{
	if (int_9_status) {
		mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_EDID].stat_page,
					g_intr_tbl[INTR_EDID].stat_offset, int_9_status);
		if (BIT_INTR9_DEVCAP_DONE & int_9_status) {
			hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
			hw_context->intr_info->msc_done_data = 0;
		}

		if (BIT_INTR9_EDID_DONE & int_9_status) {
			int ddcStatus;
			ddcStatus = mhl_tx_read_reg(hw_context,REG_DDC_STATUS);
			if (BIT_DDC_STATUS_DDC_NO_ACK & ddcStatus) {

				/* Restart EDID read from block 0 */
				if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
					hw_context->current_edid_request_block=0)) {
					hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
					hw_context->intr_info->msc_done_data =1;
				}
			} else {
				int num_extensions;

				MHL_TX_DBG_INFO(hw_context, "EDID block read complete\n");
				num_extensions =si_mhl_tx_get_num_cea_861_extensions(
								hw_context->intr_info->edid_parser_context,
								hw_context->current_edid_request_block);
				if (num_extensions < 0)	{
					MHL_TX_DBG_ERR(hw_context,"edid problem:%d\n",num_extensions);
					if (ne_NO_HPD == num_extensions) {
						// no HPD, so start over
						hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
						hw_context->intr_info->msc_done_data =1;
					} else {
						si_mhl_tx_drv_reset_ddc_fifo(hw_context);
						/* Restart EDID read from block 0 */
						if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
								hw_context->current_edid_request_block=0)) {
							// Notify the component layer with error
							hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
							hw_context->intr_info->msc_done_data =1;
						}
					}
				} else if (hw_context->current_edid_request_block < num_extensions) {
					/* EDID read next block */
					if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
						++hw_context->current_edid_request_block)) {
						/* Notify the MHL module with error */
						hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
						hw_context->intr_info->msc_done_data =1;
					}
				} else {
					/* Inform MHL module of EDID read MSC command completion */
					hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
					hw_context->intr_info->msc_done_data =0;
				}
			}
		}

		if (BIT_INTR9_EDID_ERROR & int_9_status) {
			uint8_t cbus_status;
			/*
			 * un-comment next two lines for showing TPI debug
			 * registers when readblock is done.
			uint8_t debugStatus[7];
			mhl_tx_read_reg_block(REG_TPI_HW_DBG1,sizeof(debugStatus), debugStatus);
			 */
			cbus_status = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);

			MHL_TX_DBG_INFO(hw_context, "EDID read error, retrying\n");

			hw_context->edid_fifo_block_number = 0;
			hw_context->current_edid_request_block = 0;
			si_mhl_tx_drv_reset_ddc_fifo(hw_context);
			if (BIT_CBUS_HPD & cbus_status) {
				/* Restart EDID read from block 0 */
				if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
						hw_context->current_edid_request_block=0)) {
					/* Notify the MHL module of error */
					hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
					hw_context->intr_info->msc_done_data =1;
				}
			} else {
				/* Notify the MHL module with error */
				hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
				hw_context->intr_info->msc_done_data =1;
			}
		}
	}
	return int_9_status;
}

void si_mhl_tx_read_devcap_fifo(struct drv_hw_context *hw_context,
					MHLDevCap_u *dev_cap_buf)
{
	MHL_TX_DBG_INFO(hw_context,"called\n");

	/* Enable EDID interrupt */
	enable_intr(hw_context, INTR_EDID,
				   ( BIT_INTR9_DEVCAP_DONE_MASK
				    | BIT_INTR9_EDID_DONE_MASK
				    | BIT_INTR9_EDID_ERROR
				   ));

	/* choose devcap instead of EDID to appear at the FIFO */
	mhl_tx_write_reg(hw_context
			, REG_EDID_CTRL
			, BIT_EDID_CTRL_EDID_PRIME_VALID_DISABLE
			| BIT_EDID_CTRL_FIFO_SELECT_DEVCAP
			| BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
			| BIT_EDID_CTRL_EDID_MODE_EN_ENABLE
			);
	mhl_tx_write_reg(hw_context, REG_EDID_FIFO_ADDR, 0);

	/* Retrieve the DEVCAP register values from the FIFO */
	mhl_tx_read_reg_block(hw_context, REG_EDID_FIFO_RD_DATA,
						  DEVCAP_SIZE, dev_cap_buf->devcap_cache);

	mhl_tx_write_reg(hw_context, REG_EDID_CTRL, BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE);

	MHL_TX_DBG_INFO(hw_context, "\n\ngot DEVCAP\n\n");
}
static int get_cbus_connection_status(struct drv_hw_context *hw_context)
{
	return (BIT_CBUS_HPD & mhl_tx_read_reg(hw_context, REG_CBUS_STATUS));
}

static int mhl_cbus_err_isr(struct drv_hw_context *hw_context, uint8_t cbus_err_int)
{
	int	ret_val = 0;
	uint8_t ddc_abort_reason = 0;
	uint8_t msc_abort_reason = 0;

	/*
	 * Three possible errors could be asserted.
	 * 94[2] = DDC_ABORT
	 * 94[3] = ABORT while receiving a command - MSC_RCV_ERROR
	 * 			The ABORT reasons are in 9A
	 * 94[6] = ABORT while sending a command - MSC_SEND_ERROR
	 * 			The ABORT reasons are in 9C
	 */
	if (cbus_err_int & BIT_CBUS_DDC_ABRT) {
		/*
		 * For DDC ABORTs, options are
		 * 1. reset DDC block. This will hamper a broken HDCP or EDID.
		 * 2. if error persists, reset the chip. In any case video is not
		 *    working. So it should not cause blinks.
		 * In either case, call an API to let SoC know what happened.
		 *
		 * Only option 1 has been implemented here.
		 */

		ddc_abort_reason = mhl_tx_read_reg(hw_context, REG_CBUS_DDC_ABORT_INT);

		MHL_TX_DBG_ERR(hw_context, "CBUS DDC ABORT. Reason = %02X\n",
						ddc_abort_reason);

		if(DDC_ABORT_THRESHOLD < ++ddc_abort_count) {
			si_mhl_tx_drv_reset_ddc_fifo(hw_context);
			ddc_abort_count = 0;
			MHL_TX_DBG_ERR(hw_context, "DDC fifo has been reset.%s\n"
						,((BIT_CBUS_DDC_PEER_ABORT & ddc_abort_reason)
							&&
						  (ddc_abort_count >= DDC_ABORT_THRESHOLD)
						 ) ? "  Please reset sink device!!!"
						   : ""
						);
		}
	}
	if (cbus_err_int & BIT_CBUS_MSC_ABORT_RCVD) {
		/*
		 * For MSC Receive time ABORTs
		 * - Defer submission of new commands by 2 seconds per MHL specs
		 * - This is not even worth reporting to SoC. Action is in the hands of peer.
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CBUS_ABORT;

		msc_abort_reason = mhl_tx_read_reg(hw_context, REG_MSC_RCV_ERROR);

		++msc_abort_count;

		MHL_TX_DBG_ERR(hw_context, "#%d: ABORT during MSC RCV. Reason = %02X\n",
				msc_abort_count, msc_abort_reason);
	}
	if (cbus_err_int & BIT_CBUS_CMD_ABORT) {
		/*
		 * 1. Defer submission of new commands by 2 seconds per MHL specs
		 * 2. For API operations such as RCP/UCP etc., report the situation to
		 *    SoC and let the decision be there. Internal retries have been already
		 *    done.
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CBUS_ABORT;

		msc_abort_reason = mhl_tx_read_reg(hw_context, REG_CBUS_MSC_MT_ABORT_INT);

		MHL_TX_DBG_ERR(hw_context, "CBUS ABORT during MSC SEND. Reason = %02X\n",
						msc_abort_reason);

		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_MT_ABORT_INT,msc_abort_reason);
	}
	/*
	 * Print the reason for information
	 */
	if (msc_abort_reason) {
		if (BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "Retry threshold exceeded\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "Protocol Error\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "Translation layer timeout\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "Undefined opcode\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "MSC Peer sent an ABORT\n");
		}
	}
	return(ret_val);
}

/*
 * mhl_cbus_isr
 *
 * Only when MHL connection has been established. This is where we have the
 * first looks on the CBUS incoming commands or returned data bytes for the
 * previous outgoing command.
 *
 * It simply stores the event and allows application to pick up the event
 * and respond at leisure.
 *
 *	return values:
 *		0	- MHL interrupts (all of them) have been cleared
 *				- calling routine should exit
 *		1	- MHL interrupts (at least one of them) may not have been cleared
 *				- calling routine should proceed with interrupt processing.
 */
static int mhl_cbus_isr(struct drv_hw_context *hw_context, uint8_t cbus_int)
{

	if (cbus_int & ~BIT_CBUS_HPD_RCVD) {
		/* bugzilla 27396
		   Logic to detect missed HPD interrupt.
		   Do not clear BIT_CBUS_HPD_RCVD yet.
		   */
		mhl_tx_write_reg(hw_context, REG_CBUS_INT_0,cbus_int & ~BIT_CBUS_HPD_RCVD);
	}

	if (BIT_CBUS_HPD_RCVD & cbus_int) {
		uint8_t cbus_status;
		uint8_t status;

		/* Check if a SET_HPD came from the downstream device. */
		cbus_status = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
		status = cbus_status & BIT_CBUS_HPD;

		if (BIT_CBUS_HPD & (hw_context->cbus_status ^ cbus_status)) {

			/* bugzilla 27396
			   No HPD interrupt has been missed yet.
			   Clear BIT_CBUS_HPD_RCVD.
			   */
			mhl_tx_write_reg(hw_context, REG_CBUS_INT_0, BIT_CBUS_HPD_RCVD);
			MHL_TX_DBG_INFO(hw_context, "HPD change\n");
		} else {
			MHL_TX_DBG_ERR(hw_context, "missed HPD change\n");

			/* leave the BIT_CBUS_HPD_RCVD interrupt uncleared, so that
			 * 	we get another interrupt
			 */
			/* whatever we missed, it's the inverse of what we got */
			status ^= BIT_CBUS_HPD;
			cbus_status ^= BIT_CBUS_HPD;
		}

		MHL_TX_DBG_ERR(hw_context, "DS HPD changed to %02X\n", status);

		hw_context->intr_info->flags |= DRV_INTR_FLAG_HPD_CHANGE;
		hw_context->intr_info->hpd_status = status;

		if (0 == get_config(hw_context, TRANSCODE_MODE)) {
			// non-transcode mode
			if (0 == status) {
				struct mhl_dev_context	*dev_context;
				dev_context = get_mhl_device_context(hw_context);
				MHL_TX_DBG_ERR(hw_context, "got CLR_HPD\n\n");
				drive_hpd_low(hw_context);

				if (hw_context->chip_device_id == DEVICE_ID_8558) {
					mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0x5D);
				} else {
					mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0x55);
				}
				hw_context->current_edid_request_block = 0;
#ifdef ENABLE_GEN2 //(
				/*
				   At DS HPD low:
				   Disable h/w automation of WRITE_BURST.
				   3D packets are handled using legacy WRITE_BURST method.
				   */
				disable_gen2_write_burst(hw_context);
#endif //)
				hw_context->ready_for_mdt = false;

				/* default values for video */
				hw_context->video_ready = false;
				hw_context->video_path = 1;
				/*
				 * This cannot wait for the upper layer to notice DRV_INTR_FLAG_HPD_CHANGE.
				 * stop_video relies on the result.
				 */
				si_edid_reset(dev_context->edid_parser_context);
			} else {
				MHL_TX_DBG_INFO(hw_context, "\n\nGot SET_HPD\n\n");
			}
			/* if DS sent CLR_HPD or SET_HPD, ensure video is not there */
			stop_video(hw_context);
		}
		hw_context->cbus_status = cbus_status;
	}

	if (BIT_CBUS_MSC_MT_DONE & cbus_int) {
		MHL_TX_DBG_INFO(hw_context, "MSC_REQ_DONE\n");

		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
		hw_context->intr_info->msc_done_data =
			mhl_tx_read_reg(hw_context, REG_CBUS_PRI_RD_DATA_1ST);

#ifdef ENABLE_GEN2 //(
		/* Enable h/w automation of WRITE_BURST */
		enable_gen2_write_burst(hw_context);
#endif //)
	}
	if (BIT_CBUS_MSC_MT_DONE_NACK & cbus_int){
		MHL_TX_DBG_ERR(hw_context,"MSC_MT_DONE_NACK\n");
		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_NAK;
	}

	if (BIT_CBUS_MSC_MR_WRITE_STAT & cbus_int) {

		/* read status bytes */
		mhl_tx_read_reg_block(hw_context, REG_CBUS_WRITE_STAT_0,
				ARRAY_SIZE(hw_context->intr_info->write_stat),
				hw_context->intr_info->write_stat);

		if(MHL_STATUS_DCAP_RDY & hw_context->intr_info->write_stat[0]) {
			MHL_TX_DBG_INFO(hw_context, "\n\ngot DCAP_RDY\n\n");

			/* Enable EDID interrupt */
			enable_intr(hw_context, INTR_EDID,
					( BIT_INTR9_DEVCAP_DONE_MASK
					  | BIT_INTR9_EDID_DONE_MASK
					  | BIT_INTR9_EDID_ERROR
					));
		}
		/*
		 * Save received write_stat info for later
		 * post interrupt processing
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_WRITE_STAT;
	}

	if ((BIT_CBUS_MSC_MR_MSC_MSG & cbus_int)) {
		/*
		 * Save received MSC message info for later
		 * post interrupt processing
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_RECVD;
		mhl_tx_read_reg_block(hw_context,
				REG_CBUS_MSC_MR_MSC_MSG_RCVD_1ST_DATA,
				ARRAY_SIZE(hw_context->intr_info->msc_msg),
				hw_context->intr_info->msc_msg);

		MHL_TX_DBG_INFO(hw_context, "MSC MSG: %02X %02X\n",
				hw_context->intr_info->msc_msg[0],
				hw_context->intr_info->msc_msg[1]);
	}

	/*
	 * don't do anything for a scratch pad write received interrupt.
	 * instead wait for the DSCR_CHG interrupt
	 */
	if(BIT_CBUS_MSC_MR_SET_INT & cbus_int) {
		MHL_TX_DBG_INFO(hw_context, "MHL INTR Received\n");
		/*
		 * Save received SET INT message info for later
		 * post interrupt processing
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_SET_INT;
		mhl_tx_read_reg_block(hw_context,
				REG_CBUS_SET_INT_0,
				ARRAY_SIZE(hw_context->intr_info->int_msg),
				hw_context->intr_info->int_msg);

		mhl_tx_write_reg_block(hw_context,
				REG_CBUS_SET_INT_0,
				ARRAY_SIZE(hw_context->intr_info->int_msg),
				hw_context->intr_info->int_msg);

		if (MHL_INT_EDID_CHG & hw_context->intr_info->int_msg[1]) {
			MHL_TX_DBG_INFO(hw_context, "\n\ngot EDID_CHG\n\n");
			if (get_config(hw_context, TRANSCODE_MODE)) {
				/*
				 * Force upstream source to read the EDID again
				 * by appropriate toggling of HDMI HPD
				 */
				drive_hpd_low(hw_context);
				msleep(110);
				drive_hpd_high(hw_context);

			} else {
				int reg_val;

				/* clear this bit so that BIT_TPI_INFO_EN will get cleared by the h/w */
				mhl_tx_modify_reg(hw_context,REG_TPI_INFO_FSEL
						,BIT_TPI_INFO_RPT
						,0
						);

				/* MHL module will re-read EDID */
				drive_hpd_low(hw_context);

				stop_video(hw_context);

				/* prevent HDCP interrupts from coming in */
				reg_val = mhl_tx_read_reg(hw_context, REG_TPI_SEL);
				MHL_TX_DBG_INFO(hw_context, "REG_TPI_SEL:%02x\n",
						reg_val);
				reg_val &= ~BIT_TPI_SEL_SW_TPI_EN_MASK;
				reg_val |= BIT_TPI_SEL_SW_TPI_EN_NON_HW_TPI;
				mhl_tx_write_reg(hw_context, REG_TPI_SEL, reg_val);

				/* SetTPIMode */
				MHL_TX_DBG_INFO(hw_context, "REG_TPI_SEL:%02x\n",
						reg_val);
				reg_val &= ~BIT_TPI_SEL_SW_TPI_EN_MASK;
				reg_val |= BIT_TPI_SEL_SW_TPI_EN_HW_TPI;
				mhl_tx_write_reg(hw_context, REG_TPI_SEL, reg_val);

				/*
				 * Clear HDCP interrupt - due to TPI enable, we may get one.
				 * TODO: Document this in the PR.
				 */
				mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page,
						g_intr_tbl[INTR_HDCP].stat_offset,
						0xff);
			}
		} else if (MHL_INT_DSCR_CHG & hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_INFO(hw_context, "got DSCR_CHG\n");
			if (hw_context->gen2_write_burst) {
				MHL_TX_DBG_INFO(hw_context,
						"Ignored DSCR_CHG since MDT is enabled\n");
			} else {
				mhl_tx_read_reg_block(hw_context, REG_CBUS_MHL_SCRPAD_0,
						ARRAY_SIZE(hw_context->write_burst_data),
						hw_context->write_burst_data);
			}
		} else if (MHL_INT_DCAP_CHG & hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_INFO(hw_context, "\n\ngot DCAP_CHG\n\n");
		}
	}
	return -1;
}

static int int_5_isr(struct drv_hw_context *hw_context, uint8_t int_5_status)
{
	uint8_t	new_ckdt;
	int	ret_val = 0;
	uint8_t temp;

	/* Look at the CKDT status if either is asserted */
	if( int_5_status & (BIT_INTR5_SCDT_CHANGE | BIT_INTR5_CKDT_CHANGE) ) {

		/* TODO: Document this in the PR */
		ret_val = int_5_status & (BIT_INTR5_SCDT_CHANGE | BIT_INTR5_CKDT_CHANGE);
		mhl_tx_write_reg(hw_context,REG_INTR5, ret_val);
		temp = mhl_tx_read_reg(hw_context, REG_TMDS_CSTAT_P3);
		MHL_TX_DBG_INFO(hw_context, "CSTAT_P3 = 0x%x\n", temp);

		if (BIT_TMDS_CSTAT_P3_PDO_CLOCK_DETECTED ==
					(temp & BIT_TMDS_CSTAT_P3_PDO_MASK)) {

			MHL_TX_DBG_INFO(hw_context, "\n\ngot CKDT HIGH\n");
			if (0 == hw_context->ckdt_done) {
				/* Re-read clock detect */
				new_ckdt = mhl_tx_read_reg(hw_context, REG_TMDS_CSTAT_P3);
				new_ckdt &= BIT_TMDS_CSTAT_P3_PDO_MASK;

				if (new_ckdt) {
					MHL_TX_DBG_INFO(hw_context, "Ensure AVIF\n");

					hw_context->saved_reg_mhltx_ctl2 =
							mhl_tx_read_reg(hw_context, REG_MHLTX_CTL2);

					/* Zero out bits [5:0] */
					mhl_tx_write_reg(hw_context, REG_MHLTX_CTL2,
							hw_context->saved_reg_mhltx_ctl2 &
								~BIT_MHL_TX_CTL2_TX_OE_MASK);
					/*
					 * Enable TMDS output power (keep muted) to work
					 * around infoframe interrupt issue.
					 */
					mhl_tx_modify_reg(hw_context, REG_TMDS_CCTRL,
								BIT_TMDS_CCTRL_TMDS_OE,
								BIT_TMDS_CCTRL_TMDS_OE);

					/* Do this again to be sure */
					mhl_tx_modify_reg(hw_context, REG_MHLTX_CTL2,
								BIT_MHL_TX_CTL2_TX_OE_MASK, 0);

					hw_context->ckdt_done   = 1;
				}
			}
		} else {
			MHL_TX_DBG_INFO(hw_context, "\n\ngot CKDT LOW\n");
		}
	}
	/* Check SCDT status now */
	if (int_5_status & BIT_INTR5_SCDT_CHANGE) {
		uint8_t temp;
		temp = mhl_tx_read_reg(hw_context, REG_TMDS_CSTAT_P3);

		if (BIT_TMDS_CSTAT_P3_SCDT & temp) {
			MHL_TX_DBG_ERR(hw_context, "got SCDT HIGH\n");

			if(si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
				/*
				 * This workaround ensures NEW_AVI interrupt when AVI does not change
				 * but due to downstream HDMI or MHL hot plug, we toggle upstream HPD
				 * and expect a new AVI.
				 */
				msleep(25);
				bch_force_avif(hw_context);
				{
					int temp;
					int temp2 = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);

					// TODO: Need to document this delay and workaround.
					msleep(900);
					temp2 = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
					temp = mhl_tx_read_reg(hw_context,REG_INTR8)
						& (BIT_INTR8_CEA_NEW_AVI | BIT_INTR8_CEA_NEW_VSI);
					if (0 == temp){
						int dummy;
						/*
						 *  simulate the interrupt that we MUST have
						 */
						temp =BIT_INTR8_CEA_NEW_AVI | BIT_INTR8_CEA_NEW_VSI;
						int_8_isr(hw_context,temp);
						/*
						 * always discard the real interrupt
						 */
						dummy = mhl_tx_read_reg(hw_context,REG_INTR8);
						mhl_tx_write_reg(hw_context,REG_INTR8,temp);
					}

				}
			} else {
				/*
				 * output video here for DVI or
				 * if a VIC is in hand for HDMI
				 */
				start_video(hw_context,hw_context->intr_info->edid_parser_context);
			}
		} else {
			MHL_TX_DBG_INFO(hw_context, "\n\ngot SCDT LOW\n");
			memset(&hw_context->current_vs_info_frame,0,sizeof(hw_context->current_vs_info_frame));
			memset(&hw_context->current_avi_info_frame,0,sizeof(hw_context->current_avi_info_frame));
			stop_video(hw_context);
			bch_setup(hw_context);
		}
	}
	return ret_val;
}

static void audio_disconnect(struct drv_hw_context *hw_context)
{
	if (hw_context->audio_poll_enabled == false) {
		MHL_TX_DBG_INFO(hw_context, "Turn off invalid\n");
		return;
	}

	/* Remove audio polling and disable deglitch */
	mhl_tx_write_reg(hw_context, REG_ASW_TEST_CTRL,
					 (VAL_ADC_MEAS_3_TIMES | VAL_ADC_RETRY_8_TIMES));
	hw_context->audio_poll_enabled 	= false;
	hw_context->accessory_type	= ACCESSORY_DISCONNECTED;
	mdt_destroy(container_of((void *)hw_context, struct mhl_dev_context, drv_context));

	/* Remove all bits */
	mhl_tx_write_reg(hw_context, REG_ASW_MANUAL_CTRL, 0x00);
	mhl_tx_write_reg(hw_context, REG_ASW_TEST_CTRL,(BIT_ID_DEGLITCH_PAUSE_ENABLE| asw_test_ctrl_default_bits));
	/*
	 * Necessary to avoid getting false interrupts
	 */
	mhl_tx_write_reg(hw_context , REG_ASW_INT1_MASK,
			BIT_VBUS_CHG_INT | BIT_ID_CHG_INT | BIT_WKUP_HIGH_INT);
}

static int manual_strap_measurement(struct drv_hw_context *hw_context,int pin)
{
	static uint8_t rid_manual_table[]=
	{
         0x00, 0x01, 0x02, 0x04, 0x06, 0x06, 0x08, 0x0A, 0x0C, 0x0E
        ,0x10      , 0x12, 0x14, 0x16, 0x16, 0x18, 0x1A, 0x1C, 0x1E
        ,0x20      , 0x22, 0x24, 0x26, 0x26, 0x28, 0x2A, 0x2C, 0x2E
        ,0x30      , 0x32, 0x34, 0x36, 0x36, 0x38, 0x3A, 0x3C, 0x3E
	};

	static uint8_t bc_manual_table[]=
	{
         0x00, 0x02, 0x04, 0x06, 0x06, 0x08, 0x0A, 0x0C, 0x0E
        ,0x10, 0x12, 0x14, 0x16, 0x16, 0x18, 0x1A, 0x1C, 0x1E
        ,0x20, 0x22, 0x24, 0x26, 0x26, 0x28, 0x2A, 0x2C, 0x2E
        ,0x30, 0x32, 0x34, 0x36, 0x36, 0x38, 0x3A, 0x3C, 0x3E
	};
	uint8_t *table;
	int status, ret_val;
	int high,low,mid;
	int num_entries;
	uint8_t sans_strobe = BIT_RID8_FW_ADC_OVR
				|BIT_RID8_FW_ADC_ENABLE
				|pin ;
	uint8_t with_strobe = sans_strobe | BIT_RID8_FW_ADC_STROBE;

	switch (pin) {
	case BIT_RID8_FW_ADC_SEL_BC_CTRL:
		table = bc_manual_table;
		num_entries = sizeof(bc_manual_table)/sizeof(bc_manual_table[0]);
		break;
	case BIT_RID8_FW_ADC_SEL_ID:
		table = rid_manual_table;
		num_entries = sizeof(rid_manual_table)/sizeof(rid_manual_table[0]);
		break;
	default:
		return -2;
	}
	/* execute a binary search, adding one to
		mid when the previous value of AON:EB[7] is 1
	*/
	high = num_entries-1;
	low = 0;
	status = 0;
	do {
		mid = (high+low)/2 + (status>>7);
		MHL_TX_DBG_INFO(hw_context,"h:0x%02x m:0x%02x l:0x%02x\n",high,mid,low);
		mhl_tx_write_reg(hw_context,REG_RID8,sans_strobe);
		ret_val = table[mid];
		mhl_tx_write_reg(hw_context,REG_RID9,ret_val);
		mhl_tx_write_reg(hw_context,REG_RID8,with_strobe);
		status = 0x80 & mhl_tx_read_reg(hw_context,REG_RIDB);
		if (status) {
			low=mid;
		} else {
			high=mid-1;
		}
	} while((high>low) || !status);
	/* clean up */
	mhl_tx_write_reg(hw_context,REG_RID8,0);

	if (0x80 & status) {
		switch (pin) {
		case BIT_RID8_FW_ADC_SEL_BC_CTRL:
			if (ret_val > BITS_RID9_RESERVED_THRESHOLD) {
				ret_val = BIT_RET_MON0_BC_CTRL_RESERVED;
			} else if (ret_val > BITS_RID9_BYPASS_THRESHOLD) {
				ret_val = BIT_RET_MON0_BC_CTRL_BYPASS;
			} else if (ret_val > BITS_RID9_FW_TRIG_THRESHOLD) {
				ret_val = BIT_RET_MON0_BC_CTRL_FW_TRIG;
			} else if (ret_val > BITS_RID9_VBAT_THRESHOLD) {
				ret_val = BIT_RET_MON0_BC_CTRL_VBAT;
			} else {
				ret_val = -1;
			}
			break;
		case BIT_RID8_FW_ADC_SEL_ID:
			// leave ret_val as is.
			if (1 == ret_val) {
				ret_val = BIT_ADC_VALUE_MHL;
			} else {
				ret_val >>=1;
			}
			break;
		}
	} else {
		ret_val = -1;
	}
	MHL_TX_DBG_INFO(hw_context,"manual strap measurement: 0x%x\n",ret_val);
	return ret_val;
}
#define ALL_THESE_BITS_SET(pattern,status) ((pattern) == ((pattern) & status))

/*
 * acc_switch_int_0
 *
 * Interrupt handler for Accessory Switch Interrupt 0.
 * This interrupt is not applicable to the Sil-8240.
 */
static int acc_switch_int_0(struct drv_hw_context *hw_context, uint8_t reg_asw_int0)
{
	if (reg_asw_int0) {
		mhl_tx_write_reg(hw_context, REG_ASW_INT0, reg_asw_int0);



		MHL_TX_DBG_INFO(hw_context, "ASW_INT0 = 0x%02X\n", reg_asw_int0);

		if( ALL_THESE_BITS_SET(BIT_RID_MSR_ERR_INT | BIT_RID_MSR_DONE_INT,reg_asw_int0)) {
			// if MSR_DONE happened together with an error, try it again
			MHL_TX_DBG_ERR(,"RID Measure Error -> Re-measure\n");
			mhl_tx_write_reg(hw_context,REG_ASW_INT0, BIT_RID_MSR_ERR_INT | BIT_RID_MSR_DONE_INT);
			mhl_tx_modify_reg(hw_context,REG_ASW_CTRL,
						BIT_ASW_CTRL_RID_MEASUREMENT_REQUEST_STROBE,
						BIT_ASW_CTRL_RID_MEASUREMENT_REQUEST_STROBE);
		} else if (BIT_RID_MSR_DONE_INT & reg_asw_int0) {
			uint8_t regAdcValue;
			uint8_t regAdcTable;

			/*
			 *	1.	Read 345 (tells entries) and 346 (tells table # initialized earlier)
			 *	2.	Write 341 based on entry
			 *	3.	POP control if AUDIO. Only when under FW controlled
			 *		switching. 341[7]
			 *	4.	Analyze Audio buttons if aud, and perform action.
			 *		API to AP.
			 *	5.	If audio accessory is detected, write 0x342[1:0] to start
			 *		audio switch polling.
			 */
			regAdcValue = mhl_tx_read_reg(hw_context, REG_ADC_R_VALUE);	/* 0x345 */
			regAdcTable = mhl_tx_read_reg(hw_context, REG_ADC_R_TABLE);	/* 0x346 */


			if(BIT_ADC_VALUE_VALID & regAdcValue) {
				int reg_pin_mon;
				reg_pin_mon = mhl_tx_read_reg(hw_context,REG_PIN_MON);
				if (regAdcValue & BIT_ADC_VALUE_MHL) {

					/* Power up the chip cores to access registers */
					power_up(hw_context);

					rid_index = MHL_INDEX;
					mhl_tx_write_reg(hw_context
								, REG_ASW_MANUAL_CTRL
								, rid_table[rid_index].asw_manual_ctrl_value
								);
					/* since BIT_PIN_STATUS_VBUS
						"will take around 546 ms to go high after VBUS is applied
						and around 0.5 ms to go low after VBUS is removed",
						we cannot rely on its value here.
						So, simply enable VBUS, and wait for DEVCAP results or
						NON_MHL_EST, or cbus disconnect to turn it off.
					*/
					/* Call platform function to turn the VBUS on for unpowered dongle */
					mhl_tx_vbus_control(VBUS_ON);
					msleep(100); /* see MHL spec for vbus out to stable */

					/*
					 * MHL impedance is measured (bit 5), no need to
					 * access table
					 */
					MHL_TX_DBG_INFO(hw_context, "INTR_RID_MSR_DONE. MHL "
									"Possible: 0x345 = %02X, 0x346 = %02X\n"
									,regAdcValue ,regAdcTable);

					/*
					 * enable remaining discovery interrupts
					 * ignore RGND and LKOUT for 8558
					 */
					enable_intr(hw_context, INTR_DISC,
						  BIT_INTR4_MHL_EST
						| BIT_INTR4_NON_MHL_EST
/*						| BIT_INTR4_CBUS_LKOUT		*/
						| BIT_INTR4_CBUS_DISCONNECT
/*						| BIT_INTR4_RGND_DETECTION	*/
						| BIT_INTR4_VBUS_CHG /* this is required */
						);
					/* Enable MSC interrupt to handle initial exchanges */
					enable_intr(hw_context, INTR_MERR,
							 ( BIT_CBUS_DDC_ABRT
							 | BIT_CBUS_MSC_ABORT_RCVD
							 | BIT_CBUS_CMD_ABORT
							 ));
					enable_intr(hw_context, INTR_MSC,
							(  BIT_CBUS_MSC_MT_DONE
							 | BIT_CBUS_HPD_RCVD
							 | BIT_CBUS_MSC_MR_WRITE_STAT
							 | BIT_CBUS_MSC_MR_MSC_MSG
							 | BIT_CBUS_MSC_MR_WRITE_BURST
							 | BIT_CBUS_MSC_MR_SET_INT
							 | BIT_CBUS_MSC_MT_DONE_NACK
							));
					set_pin(hw_context,MHL_LED,GPIO_LED_OFF);

					/*
						The following block ensures USB_LED is ON when strapped to BYPASS mode with override_strap=0.
					*/
					if (hw_context->strapping_value != BIT_RET_MON0_BC_CTRL_BYPASS ||
						hw_context->suspend_strapped_mode == true) {
						set_pin(hw_context,USB_LED, GPIO_LED_OFF);
					}

					set_pin(hw_context,AUDIO_LED, GPIO_LED_OFF);
					set_pin(hw_context,MIC_LED,  GPIO_LED_OFF);
					set_pin(hw_context,UART_LED, GPIO_LED_OFF);
					hw_context->accessory_type	= ACCESSORY_MHL;

					/* prevent RID measurement from happening again
						before we clear BIT_INTR4_NON_MHL_EST
					*/
					mhl_tx_write_reg(hw_context, REG_DISC_CTRL8,0x01);

					/* Enable MHL discovery so we are awakened by h/w on discovery result */
					mhl_tx_write_reg(hw_context, REG_DISC_CTRL1,
							VAL_DISC_CTRL1_DEFAULT |
							BIT_DISC_CTRL1_MHL_DISCOVERY_ENABLE);

					/*
					 * Nothing else to do here... the switch to D3.Hot
					 * above will allow handling of RGND
					 */
					return reg_asw_int0;
				}

				regAdcValue &= MSK_ADC_MEAS_VALUE;

				MHL_TX_DBG_ERR(hw_context, "INTR_RID_MSR_DONE."
								" 0x345 = %02X, 0x346 = %02X ==> %s\n"
								,regAdcValue, regAdcTable
								,rid_table[regAdcValue].description
								);

				if (NO_CONNECTION_INDEX == rid_index){
					if (OPEN_IMPEDANCE == regAdcValue){
						if (BIT_RET_MON0_BC_CTRL_BYPASS == hw_context->strapping_value){
							if (2 == override_strap){
								rid_index = regAdcValue;
								return reg_asw_int0;
							}
						}
					}
				}
				rid_index = regAdcValue;

				if (flag_boot & rid_table[rid_index].flags){
					/* Qualify the existing link as BOOT_DEVICE and do nothing */
					hw_context->accessory_type = ACCESSORY_BOOT_DEVICE;
					MHL_TX_DBG_ERR(, "ACCESSORY_BOOT_DEVICE\n");
					return reg_asw_int0;
				}
				if (rid_index == hw_context->boot_usb_impedance_reference) {
					/* Qualify the existing link as BOOT_DEVICE and do nothing */
					hw_context->accessory_type = ACCESSORY_BOOT_DEVICE;
					MHL_TX_DBG_ERR(, "ACCESSORY_BOOT_DEVICE\n");
					return reg_asw_int0;
				} else if (rid_index == hw_context->boot_uart_impedance_reference) {
					/* Qualify the existing link as BOOT_DEVICE and do nothing */
					hw_context->accessory_type = ACCESSORY_BOOT_DEVICE;
					MHL_TX_DBG_ERR(, "ACCESSORY_BOOT_DEVICE\n");
					return reg_asw_int0;
				} else if (rid_index == (1 + hw_context->boot_usb_impedance_reference)) {
					/* Qualify the existing link as BOOT_DEVICE and do nothing */
					hw_context->accessory_type = ACCESSORY_BOOT_DEVICE;
					MHL_TX_DBG_ERR(, "ACCESSORY_BOOT_DEVICE\n");
					return reg_asw_int0;
				} else if (rid_index == (1 + hw_context->boot_uart_impedance_reference)) {
					/* Qualify the existing link as BOOT_DEVICE and do nothing */
					hw_context->accessory_type = ACCESSORY_BOOT_DEVICE;
					MHL_TX_DBG_ERR(, "ACCESSORY_BOOT_DEVICE\n");
					return reg_asw_int0;
				}
				MHL_TX_DBG_ERR(, "usb imp:0x%02x uart imp: 0x%02x continuing...\n"
						,hw_context->boot_usb_impedance_reference
						,hw_context->boot_uart_impedance_reference
						);
/* begin bz 25172 ( */
				mhl_tx_write_reg(hw_context
					, REG_ASW_MANUAL_CTRL
					, rid_table[regAdcValue].asw_manual_ctrl_value
					);

				/*
				 *  If measured value is more than 0 less than 28K,
				 *  its an audio button
				 *
				 * Also check if specific Audio device impedances are there
				 */
				if (flag_uart & rid_table[regAdcValue].flags){
					set_pin(hw_context,UART_LED, GPIO_LED_ON);
				}
				if (flag_usb & rid_table[regAdcValue].flags){
					set_pin(hw_context,USB_LED, GPIO_LED_ON);
				}
				if (flag_mic & rid_table[regAdcValue].flags){
					set_pin(hw_context,MIC_LED, GPIO_LED_ON);
				}
				if (flag_audio& rid_table[regAdcValue].flags){
					set_pin(hw_context,AUDIO_LED,GPIO_LED_ON); /* active low */
				}
				if (( flag_audio_button
					| flag_no_buttons
					) & rid_table[regAdcValue].flags) {
					/* don't mess with anything
					 * to do with enabling or disabling audio polling
					 */
				} else if (flag_audio_poll & rid_table[regAdcValue].flags) {

					MHL_TX_DBG_ERR(hw_context, "Start/Enable Audio "\
									"Polling. Enable audio pop "\
									"suppression\n0x351 = 0x22 for "\
									"deglitch = 8ms and disconnect "\
									"= 400ms\n");

					#if 0	//MDT_HOT_PLUG
					mdt_init(
							container_of((void *)hw_context
										, struct mhl_dev_context
										, drv_context)
							);
					#endif

					/* check for values 0x18 and 0x19*/
					if (flag_mcpc_button & rid_table[regAdcValue].flags){
							hw_context->accessory_type = ACCESSORY_MCPC_SETUP;
					}else{
						hw_context->accessory_type = ACCESSORY_BOARD;
					}

					/* 500ms+ for LKP and 40ms to 500ms for SKP. */
					mhl_tx_write_reg(hw_context,REG_AUDIO_POLL_BLOCK,0x41);

					/* adc value is the value for the default */
					mhl_tx_write_reg(hw_context,REG_AUD_POLL_DEFAULT_VAL,regAdcValue<<1);

					/* Setting for 10 ms interval */
					mhl_tx_write_reg(hw_context, REG_AUDIO_POLL_CTRL,
									 BIT_AUDIO_POLL_START
									 | BIT_AUDIO_POLL_EN
									 | BIT_AUDIO_POLL_INT_MSB);

/* )end bz 25172 */
					/*
					 * After fixing the deglitch issue, no need to change
					 * the default deglitch value
					 */
					hw_context->audio_poll_enabled = true;

					// Handled by IsrRegAswInt1
					mhl_tx_write_reg(hw_context, REG_ASW_INT1_MASK,
									 BIT_VBUS_CHG_INT_MASK
									 | BIT_ID_CHG_INT_MASK
									 | BIT_ADC_CHG_INT_MASK
									 | BIT_WKUP_HIGH_INT_MASK
									 | BIT_OVP_INT_MASK
									 | BIT_SHORT_KEY_PRESSED_INT_MASK
									 | BIT_LONG_KEY_PRESSED_INT_MASK
									 | BIT_LONG_KEY_RELEASED_INT_MASK);
				} else {
					MHL_TX_DBG_ERR(hw_context, "Turn OFF Audio Polling "\
									"and pop suppression\n");
					audio_disconnect(hw_context);
					MHL_TX_DBG_ERR(hw_context, "INPUT_DEV_ACCESSORY - R_ID DONE\n");
					switch (regAdcValue) {
					case 0x1C:		// DEMO CUSTOM CABLE
						// Turn off for DEMO cable
						set_pin(hw_context, USB_LED, GPIO_LED_OFF);

						// Fall through intentionally.
					case 0x00:		// micro A plug detected <= 10 ohms
						// in case of micro A usb led stays on. Turn off for DEMO cable

						/*
						 * VBUS control and M2U_VBUS_CTRL_M decisions will be made again
						 * 	as part of SDP handling.
						 */
						// To make an informed decision about power requirements
						//      check if power is already provided.


						reg_pin_mon = mhl_tx_read_reg(hw_context,REG_PIN_MON);
						MHL_TX_DBG_ERR(hw_context,"VBUS_CHG detected."
							" VBUS pin is %s\n",
							(BIT_PIN_STATUS_VBUS & reg_pin_mon)?"HIGH":"LOW");

						if ((BIT_PIN_STATUS_VBUS & reg_pin_mon) != 0) {
							/*
							 * This is a device that is providing power.
							 */
							MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - USB HOST - VBUS PWR DETECTED; OFF\n");
							mhl_tx_vbus_control(VBUS_OFF);
							hw_context->accessory_type = ACCESSORY_OTG_HOST;

							// OTG needs VBUS to detect a host.
							MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - USB_VBUS <==> VBUS\n");
							set_pin(hw_context,M2U_VBUS_CTRL_M,1);
						} else {
							MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - USB DEV - USB_VBUS < != > VBUS\n");
							set_pin(hw_context,M2U_VBUS_CTRL_M,0);
							// This is a device that doesn't provide power or is a peripheral
							//     with an incorrect cable.
							// Provide this confused device with power.
							MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - USB DEV -VBUS PWR ON\n");
							mhl_tx_vbus_control(VBUS_ON);
							hw_context->accessory_type = ACCESSORY_USB_DEVICE;
						}
						mhl_tx_write_reg(hw_context,REG_ASW_INT0_MASK, 0);
						mhl_tx_write_reg(hw_context,REG_ASW_INT0, mhl_tx_read_reg(hw_context,REG_ASW_INT0));

						// Handled by IsrRegAswInt1
						mhl_tx_write_reg(hw_context,REG_ASW_INT1_MASK, BIT_ID_CHG_INT);
						mhl_tx_write_reg(hw_context,REG_ASW_INT1, mhl_tx_read_reg(hw_context,REG_ASW_INT1));

						// After power is considered, move the rest of the switch over.
						MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - USB SW ON - ACTIVATE SW\n");
						mhl_tx_modify_reg(hw_context,REG_ASW_CTRL
							,BIT_ASW_CTRL_MODE_MASK
							,BIT_ASW_CTRL_MODE_MANUAL);


						// Don't need to move the ID pin since currently don't support OTG.
						// If ID pin switch is moved over, ID related interrupts must
						// be disabled here.
						//
						// For OTG and host the ID pin impedance remains important but, not here.
						break;
					case 0x0F:	/* Terminal demo -- accessory board diode does not allow MHL TX power */
						MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - UART - USB_VBUS < != > VBUS\n");
						set_pin(hw_context,M2U_VBUS_CTRL_M,0);

						break;
					}

					return reg_asw_int0;
				}
			} else {
				MHL_TX_DBG_INFO(hw_context, "INTR_RID_MSR_DONE. "\
								"Invalid ADC Value: 0x345 = %02X,"\
								"0x346 = %02X\n",
								regAdcValue, regAdcTable);
			}
		}

		if (BIT_RSTRAP_MSR_DONE_INT & reg_asw_int0) {
			MHL_TX_DBG_ERR(hw_context, "INTR_RSTRAP_MSR_DONE detected. "\
						   "0x352 = %02X, 0x353 = %02X\n",
						   mhl_tx_read_reg(hw_context, REG_RET_MON0),
						   mhl_tx_read_reg(hw_context, REG_RET_MON1));
		}

		if (BIT_ASW_ATTACH_INT & reg_asw_int0) {
			MHL_TX_DBG_ERR(hw_context, "INTR_ASW_ATTACH detected\n");
		}

		if (BIT_ASW_DETACH_INT & reg_asw_int0) {
			firmware_triggered_bc_detection_state = waiting_for_bc_done_fw_trig;
			MHL_TX_DBG_ERR(hw_context, "INTR_ASW_DETACH detected\n");

			set_pin(hw_context,SDP_LED  ,GPIO_LED_OFF);
			set_pin(hw_context,CDP_LED  ,GPIO_LED_OFF);
			set_pin(hw_context,DCP_LED  ,GPIO_LED_OFF);
			set_pin(hw_context,PS2_LED  ,GPIO_LED_OFF);
			set_pin(hw_context,SPARE_LED1,GPIO_LED_OFF);
			set_pin(hw_context,SPARE_LED2,GPIO_LED_OFF);
			set_pin(hw_context,MHL_LED  ,GPIO_LED_OFF);
			set_pin(hw_context,USB_LED  ,GPIO_LED_OFF);
			set_pin(hw_context,AUDIO_LED,GPIO_LED_OFF);
			set_pin(hw_context,MIC_LED  ,GPIO_LED_OFF);
			set_pin(hw_context,UART_LED ,GPIO_LED_OFF);

			rid_index = NO_CONNECTION_INDEX;

			if(hw_context->accessory_type < ACCESSORY_OTG_HOST) {
				if (hw_context->audio_poll_enabled) {
					MHL_TX_DBG_ERR(hw_context, "Remove audio polling due to "\
								   "DETACH\n");
					audio_disconnect(hw_context);
				}

				/*
				 * This is a hack. When micro B plug has VBUS, ID pin impedance
				 *    measurement may not complete and keep BIT_ID_CHG_INT & regAswInt1
				 *     from firing.
				 */

				MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - USB_VBUS < != > VBUS\n");
				set_pin(hw_context,M2U_VBUS_CTRL_M,0);
				MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - VBUS PWR OFF\n");
				mhl_tx_vbus_control(VBUS_OFF);

				mhl_tx_write_reg(hw_context,REG_ASW_MANUAL_CTRL, 0);

				MHL_TX_DBG_INFO(hw_context,"INPUT_DEV_ACCESSORY - resetting...\n");
				board_reset(hw_context,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);

				mhl_tx_write_reg(hw_context,REG_POWER_CTRL,(BIT_MASTER_POWER_CTRL| BIT_ISO_EN));
				mhl_tx_write_reg(hw_context,REG_DISC_CTRL1,VAL_DISC_CTRL1_DEFAULT);
				mhl_tx_write_reg(hw_context,REG_DISC_CTRL5,0x03);
				mhl_tx_write_reg(hw_context,REG_DISC_CTRL9
						 , BIT_DC9_WAKE_DRVFLT
						 | BIT_DC9_DISC_PULSE_PROCEED
						);
				/* Enable interrupts for accessory switch */
				enable_intr(hw_context, INTR_ASW0,
						( BIT_RID_MSR_DONE_INT
						| BIT_RID_MSR_ERR_INT
						| BIT_RSTRAP_MSR_DONE_INT
						| BIT_ASW_ATTACH_INT
						| BIT_ASW_DETACH_INT
						| BIT_BC_DONE_INT)
						);
				enable_intr(hw_context, INTR_ASW1,
						( BIT_VBUS_CHG_INT
						| BIT_ID_CHG_INT
						| BIT_ADC_CHG_INT
						| BIT_WKUP_HIGH_INT
						| BIT_OVP_INT
						| BIT_SHORT_KEY_PRESSED_INT
						| BIT_LONG_KEY_PRESSED_INT
						| BIT_LONG_KEY_RELEASED_INT)
						);
				mhl_tx_write_reg(hw_context,REG_ASW_INT4_MASK,0xFF);
				mhl_tx_write_reg(hw_context,REG_8240_USB_CHARGE_PUMP,0x04);
				// todo, find charge pump register on 8558
				mhl_tx_write_reg(hw_context,REG_ASW_TEST_CTRL,asw_test_ctrl_default_bits |BIT_ID_DEGLITCH_PAUSE_ENABLE);
				switch_to_d3(hw_context,false);
				rid_index = NO_CONNECTION_INDEX;
			}
		}

		if (BIT_BC_DONE_INT & reg_asw_int0) {
			uint8_t regBCcore = 0;		// holds 0x30D
			extern bool special_charger;

			MHL_TX_DBG_ERR(hw_context, "INTR_BC_DONE detected\n");
			regBCcore = mhl_tx_read_reg(hw_context,REG_BC_CORE_RESULTS);
			MHL_TX_DBG_ERR(hw_context,"BC_RESULT - %d\n", (int)regBCcore);
			// Since ID measurement may not happen if BC_DONE fires
			//     do work here.
			switch(firmware_triggered_bc_detection_state){
			case waiting_for_bc_done_fw_trig:

				if (regBCcore & BIT_BC_CORE_RESULTS_STANDARD_DS_PORT_DETECTED) {
					firmware_triggered_bc_detection_state = charging_sdp;
					set_pin(hw_context,SDP_LED,GPIO_LED_ON);
					set_pin(hw_context,M2U_VBUS_CTRL_M,1);
					set_pin(hw_context,USB_LED, GPIO_LED_ON); // possible USB host is connected
					MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - SDP charger/USB\n");
				}else if (regBCcore & BIT_BC_CORE_RESULTS_CHARGING_DS_PORT_DETECTED){
					firmware_triggered_bc_detection_state = charging_cdp;
					set_pin(hw_context,CDP_LED,GPIO_LED_ON);
					set_pin(hw_context,M2U_VBUS_CTRL_M,1);
					set_pin(hw_context,USB_LED, GPIO_LED_ON); // possible USB host is connected
					// Since power is provided by a downstream
					MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - CDP charger\n");
				}else if (regBCcore & BIT_BC_CORE_RESULTS_DEDICATED_DS_PORT_DETECTED){
					firmware_triggered_bc_detection_state = charging_dcp;
					set_pin(hw_context,DCP_LED,GPIO_LED_ON);
					set_pin(hw_context,M2U_VBUS_CTRL_M,0);
					// Since power is provided by a downstream
					MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - DCP charger.\n");
				}else if (regBCcore & BIT_BC_CORE_RESULTS_PS2_PORT_DETECTED){
					if( special_charger ){
						firmware_triggered_bc_detection_state = waiting_for_bc_done_fw_trig_special;
						MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - special charger detection start.\n");
						/* turn off manual start so that turning it back on will have an effect */
						mhl_tx_modify_reg(hw_context,REG_BC_CTRL_1
							,BIT_BC_CTRL_1_MANUAL_START
							,0
							);
						mhl_tx_modify_reg(hw_context,REG_BC_CTRL_2
							,BIT_BC_CTRL_2_FORCE_SPECIAL_MODE
							,BIT_BC_CTRL_2_FORCE_SPECIAL_MODE
							);
						if (BIT_RET_MON0_BC_CTRL_FW_TRIG == hw_context->strapping_value){
							mhl_tx_modify_reg(hw_context,REG_BC_OVR_CTRL
								,BIT_BC_OVR_CTRL_BC_FW_RDY
								,BIT_BC_OVR_CTRL_BC_FW_RDY
								);
						}
						mhl_tx_modify_reg(hw_context,REG_BC_CTRL_1
							,BIT_BC_CTRL_1_MANUAL_START
							,BIT_BC_CTRL_1_MANUAL_START
							);
					}else{
						firmware_triggered_bc_detection_state =charging_ps2;
						set_pin(hw_context,PS2_LED,GPIO_LED_ON);
						set_pin(hw_context,M2U_VBUS_CTRL_M,0);
						MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - PS/2 port detected.\n");
					}
				}else if (regBCcore & BIT_BC_CORE_RESULTS_NO_RECOGNIZED_DS_PORT){
					firmware_triggered_bc_detection_state = no_downstream_charger;
					set_pin(hw_context,M2U_VBUS_CTRL_M,0);
					MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - no recognized downstream port.\n");
				}
				break;
			case waiting_for_bc_done_fw_trig_special:
				{
					int bc_core_res_2;
					bc_core_res_2 = mhl_tx_read_reg(hw_context,REG_BC_CORE_RESULTS_2);
					if (bc_core_res_2  & BIT_BC_CORE_RESULTS_2_SONY_DET){
						firmware_triggered_bc_detection_state =charging_sony;
						set_pin(hw_context,SPARE_LED1,GPIO_LED_ON);
						set_pin(hw_context,M2U_VBUS_CTRL_M,0);
						MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - Sony charger.\n");
					}else if (bc_core_res_2 & BIT_BC_CORE_RESULTS_2_APPLE_10_DET){
						firmware_triggered_bc_detection_state =charging_apple_1_0A;
						set_pin(hw_context,SPARE_LED2,GPIO_LED_ON);
						set_pin(hw_context,M2U_VBUS_CTRL_M,0);
						MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - Apple 1 Amp charger.\n");
					}else if (regBCcore & BIT_BC_CORE_RESULTS_PS2_PORT_DETECTED){
						firmware_triggered_bc_detection_state =charging_ps2;
						set_pin(hw_context,PS2_LED,GPIO_LED_ON);
						set_pin(hw_context,M2U_VBUS_CTRL_M,0);
						MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - PS/2 port detected.\n");
					}
				}
				break;
			case charging_dcp:
			case charging_cdp:
			case charging_sdp:
			case charging_ps2:
			case charging_sony:
			case charging_apple_1_0A:
			case no_downstream_charger:
				MHL_TX_DBG_ERR(NULL,"unexpected BC_DONE event. Remaining in state: %d\n"
					,firmware_triggered_bc_detection_state);
			}
			/* do some processing for the new state */
			switch(firmware_triggered_bc_detection_state){
			uint8_t	tmp;
			case no_downstream_charger:
				/* clear special charger register bit */
				mhl_tx_modify_reg(hw_context,REG_BC_CTRL_2
					,BIT_BC_CTRL_2_FORCE_SPECIAL_MODE
					,0
					);
				mhl_tx_modify_reg(hw_context,REG_BC_CTRL_1
					,BIT_BC_CTRL_1_MANUAL_START
					,0
					);
				break;
			case charging_dcp:
			case charging_cdp:
			case charging_sdp:
			case charging_ps2:
			case charging_sony:
			case charging_apple_1_0A:
				/* clear special charger register bit */
				mhl_tx_modify_reg(hw_context,REG_BC_CTRL_2
					,BIT_BC_CTRL_2_FORCE_SPECIAL_MODE
					,0
					);
				mhl_tx_modify_reg(hw_context,REG_BC_CTRL_1
					,BIT_BC_CTRL_1_MANUAL_START
					,0
					);
				// Since power is provided by a downstream
				MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - VBUS PWR DETECTED; OFF.\n");
				mhl_tx_vbus_control(VBUS_OFF);

				// After power is considered, move the rest of the switch over.
				tmp = mhl_tx_read_reg(hw_context,REG_ASW_CTRL);

				if ((tmp & BIT_ASW_CTRL_MODE_MANUAL) == 0) {
					MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - ACTIVATE SW\n");
					mhl_tx_write_reg(hw_context,REG_ASW_CTRL, tmp | BIT_ASW_CTRL_MODE_MANUAL);
				}

				/* todo: kiran says that this is unnecessary */
				mhl_tx_write_reg(hw_context,REG_ASW_MANUAL_CTRL
					,BIT_ASW_MAN_CTRL_USB_ID_ON
					|BIT_ASW_MAN_CTRL_USB_D_ON);
				break;
			case waiting_for_bc_done_fw_trig:
			case waiting_for_bc_done_fw_trig_special:
			default:
				;
			}
		}
	}
	return reg_asw_int0;
}
static void detach_on_id_change(struct drv_hw_context *hw_context)
{
	set_pin(hw_context,MHL_LED  , GPIO_LED_OFF);
	set_pin(hw_context,USB_LED  , GPIO_LED_OFF);
	set_pin(hw_context,AUDIO_LED, GPIO_LED_OFF);
	set_pin(hw_context,MIC_LED  , GPIO_LED_OFF);
	set_pin(hw_context,UART_LED , GPIO_LED_OFF);

	if ((hw_context->accessory_type < ACCESSORY_OTG_HOST) && (hw_context->audio_poll_enabled))
	{
		MHL_TX_DBG_ERR(hw_context,"Remove audio polling due to ID_CHG\n");
		audio_disconnect(hw_context);
	}

	mdt_destroy(
				container_of((void *)hw_context
							, struct mhl_dev_context
							, drv_context
							)
				);

	MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - USB_VBUS < != > VBUS\n");
	set_pin(hw_context,M2U_VBUS_CTRL_M,0);
	MHL_TX_DBG_ERR(hw_context,"INPUT_DEV_ACCESSORY - VBUS PWR OFF\n");
	mhl_tx_vbus_control(VBUS_OFF);

	mhl_tx_write_reg(hw_context,REG_ASW_MANUAL_CTRL, 0);

	MHL_TX_DBG_INFO(hw_context,"INPUT_DEV_ACCESSORY - resetting...\n");
	board_reset(hw_context,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);

	mhl_tx_write_reg(hw_context,REG_POWER_CTRL,(BIT_MASTER_POWER_CTRL| BIT_ISO_EN));
	mhl_tx_write_reg(hw_context,REG_DISC_CTRL1,VAL_DISC_CTRL1_DEFAULT);
	mhl_tx_write_reg(hw_context,REG_DISC_CTRL5,0x03);
	mhl_tx_write_reg(hw_context,REG_DISC_CTRL9
			 , BIT_DC9_WAKE_DRVFLT
			 | BIT_DC9_DISC_PULSE_PROCEED
			);
	/* Enable interrupts for accessory switch */
	enable_intr(hw_context, INTR_ASW0,
			( BIT_RID_MSR_DONE_INT
			| BIT_RID_MSR_ERR_INT
			| BIT_RSTRAP_MSR_DONE_INT
			| BIT_ASW_ATTACH_INT
			| BIT_ASW_DETACH_INT
			| BIT_BC_DONE_INT)
			);
	enable_intr(hw_context, INTR_ASW1,
			( BIT_VBUS_CHG_INT
			| BIT_ID_CHG_INT
			| BIT_ADC_CHG_INT
			| BIT_WKUP_HIGH_INT
			| BIT_OVP_INT
			| BIT_SHORT_KEY_PRESSED_INT
			| BIT_LONG_KEY_PRESSED_INT
			| BIT_LONG_KEY_RELEASED_INT)
			);
	mhl_tx_write_reg(hw_context,REG_ASW_INT4_MASK,0xFF);
	mhl_tx_write_reg(hw_context,REG_8240_USB_CHARGE_PUMP,0x04);
	// todo, find charge pump register on 8558
	mhl_tx_write_reg(hw_context,REG_ASW_TEST_CTRL,asw_test_ctrl_default_bits |BIT_ID_DEGLITCH_PAUSE_ENABLE);

	rid_index = NO_CONNECTION_INDEX;
}
/*
 * acc_switch_int_1
 *
 * Interrupt handler for Accessory Switch Interrupt 1.
 * This interrupt is not applicable to the Sil-8240.
 */
static int acc_switch_int_1(struct drv_hw_context *hw_context,
							uint8_t reg_asw_int1)
{
	uint8_t reg_pin_mon;	/* holds 0x34B (REG_PIN_MON) */
	int ret_val = 0;

	if (reg_asw_int1) {
		reg_pin_mon = mhl_tx_read_reg(hw_context, REG_PIN_MON);	/* 0x34B */

		MHL_TX_DBG_INFO(hw_context, "VBAT pin is %s. ID pin is %s IOVCC pin is %s\n"
	 					,(BIT_PIN_STATUS_VBAT & reg_pin_mon) ? "HIGH" : "LOW"
						,(BIT_PIN_STATUS_IOVCC & reg_pin_mon) ? "HIGH" : "LOW"
						,(BIT_PIN_STATUS_ID & reg_pin_mon)? "HIGH":"LOW"
						);

		if (BIT_VBUS_CHG_INT & reg_asw_int1){
			int consider_battery_charging=0;
			MHL_TX_DBG_ERR(hw_context, "VBUS_CHG detected. VBUS pin is %s"
							" rid_index:0x%02x \"%s\" AON:5B: 0x%02x\n",
						   (BIT_PIN_STATUS_VBUS & reg_pin_mon) ? "HIGH" : "LOW"
							,rid_index,rid_table[rid_index].description
							,mhl_tx_read_reg(hw_context,REG_ASW_DBG1)
							);

			switch(rid_index){
			case NO_CONNECTION_INDEX:
				if (BIT_PIN_STATUS_VBUS & reg_pin_mon){
					if (BIT_PIN_STATUS_ID & reg_pin_mon){
						int temp;
						temp = manual_strap_measurement(hw_context,BIT_RID8_FW_ADC_SEL_ID);
						if (OPEN_IMPEDANCE == temp){
							consider_battery_charging=1;
						}
					}
				}
				break;
			case OPEN_IMPEDANCE:
			case RID_INDEX_USER:
				consider_battery_charging=1;
				break;
			default:
				consider_battery_charging=0;
				break;
			}
			if(consider_battery_charging){
				if (BIT_PIN_STATUS_VBUS & reg_pin_mon){
					extern bool special_charger;
					if (BIT_PIN_STATUS_ID & reg_pin_mon){
						if (BIT_RET_MON0_BC_CTRL_FW_TRIG == hw_context->strapping_value){
							mhl_tx_modify_reg(hw_context,REG_BC_OVR_CTRL
								,BIT_BC_OVR_CTRL_BC_FW_RDY
								,BIT_BC_OVR_CTRL_BC_FW_RDY
								);
						}else{
							mhl_tx_modify_reg(hw_context,REG_BC_OVR_CTRL
								,BIT_BC_OVR_CTRL_BC_FW_RDY
								,0
								);
							if (BIT_RET_MON0_BC_CTRL_BYPASS == hw_context->strapping_value){
								if (2 == override_strap) {
								/* make sure that we can control the ID pullup
										as well as the ID switch
								*/
									mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL
												,BIT_ASW_TEST_CTRL_ASW_FORCE_MODE
												,BIT_ASW_TEST_CTRL_ASW_FORCE_MODE
												);
									asw_test_ctrl_default_bits |= BIT_ASW_TEST_CTRL_ASW_FORCE_MODE;
									/* ensure that the pull-up is enabled */
									mhl_tx_modify_reg(hw_context,REG_ASW_ANA_CTRL
										,BIT_ASW_ANA_CTRL_ID_PULLUP_ON_OFF_MASK
										,0
										);
									mhl_tx_write_reg(hw_context
										, REG_ASW_MANUAL_CTRL
										, BIT_ASW_MAN_CTRL_USB_D_ON
										);
								}
							}
						}
						set_pin(hw_context,M2U_VBUS_CTRL_M,1);
					}else{
						set_pin(hw_context,M2U_VBUS_CTRL_M,0);
					}
					if ((BIT_RET_MON0_BC_CTRL_FW_TRIG == hw_context->strapping_value)
						|| special_charger){
						mhl_tx_modify_reg(hw_context,REG_BC_CTRL_1
							,BIT_BC_CTRL_1_MANUAL_START
							,BIT_BC_CTRL_1_MANUAL_START
							);
					}
				}else{
					if (BIT_RET_MON0_BC_CTRL_BYPASS == hw_context->strapping_value){
						if (2 == override_strap){
							/* relinquish control of the ID pullup and ID switch */
							mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL
										,BIT_ASW_TEST_CTRL_ASW_FORCE_MODE
										,0
										);
							asw_test_ctrl_default_bits &= ~BIT_ASW_TEST_CTRL_ASW_FORCE_MODE;

						}
					}
				}
			}
		}

		if(BIT_ID_CHG_INT & reg_asw_int1) {
			MHL_TX_DBG_ERR(hw_context, "CBUS or ID_CHG detected. "
						   "ID_PIN is %s\n",
						   (BIT_PIN_STATUS_ID & reg_pin_mon) ? "HIGH" : "LOW");
			if (BIT_PIN_STATUS_ID & reg_pin_mon) {
				if (rid_index < OPEN_IMPEDANCE){
					detach_on_id_change(hw_context);
				}
			}
		}
		if(BIT_ADC_CHG_INT & reg_asw_int1)
			MHL_TX_DBG_ERR(hw_context, "ADC value has changed. "
						   "ADC_CHG detected\n");

		if (BIT_WKUP_HIGH_INT & reg_asw_int1)
			MHL_TX_DBG_ERR(hw_context, "WKUP_HIGH detected. "
						   "WAKE_UP Pin is %s\n",
						   (BIT_PIN_STATUS_WAKEUP & reg_pin_mon) ? "HIGH" : "LOW");

		if (BIT_OVP_INT & reg_asw_int1) {
			uint8_t regOvpMon;		/* holds 0x34C () */
			regOvpMon 	= mhl_tx_read_reg(hw_context, REG_OVP_MON);	/* 0x34C */
			MHL_TX_DBG_ERR(hw_context, "Over Voltage Protection Interrupt "
						   "(OVP). PINMON OVP = %02X\n", regOvpMon);

			MHL_TX_DBG_ERR(hw_context, "OVP on VBUS is %s. OVP on ID pin "
						   "is %s. OVP on USB_D pin is %s. OVP on USB_DB "
						   "pin is %s\n",
						   (BIT_OVP_DET_VBUS & regOvpMon)?"HIGH":"LOW",
						   (BIT_OVP_DET_CBUSID & regOvpMon)?"HIGH":"LOW",
						   (BIT_OVP_DET_MHLD_DET & regOvpMon)?"HIGH":"LOW",
						   (BIT_OVP_DET_MHLDB_DET & regOvpMon)?"HIGH":"LOW");

		}
		if(BIT_SHORT_KEY_PRESSED_INT & reg_asw_int1){
			uint8_t regAdcValue;
			regAdcValue = mhl_tx_read_reg(hw_context,REG_AUDIO_POLL_STAT0)
							& MSK_AUDIO_POLL_SKP_IMPED_VAL;
			regAdcValue >>= 1;

			MHL_TX_DBG_ERR(hw_context, "Short (SKP). SK "
						   "Z (0x349) = %02X\n",regAdcValue);

			if (hw_context->accessory_type == ACCESSORY_MCPC_SETUP)
				hw_context->accessory_type = ACCESSORY_MCPC_ACTIVE;
			else if (hw_context->accessory_type == ACCESSORY_MCPC_ACTIVE ) {		/* MCPC accessories tend to bounce */
				if (regAdcValue <= TABLE_MCPC_49900_OHM_47K_RCSW) {				/* RCSW 47K ohm */
					if (hw_context->mcpc_button_on == 0) {			/* Handle the first 0 ohm and ignore */
					    hw_context->mcpc_button_on = 1;			/* all others.*/
				   	    mdt_toggle_keyboard_keycode(
							 container_of((void *)hw_context
										, struct mhl_dev_context
										, drv_context
										)
							,0xA4);
					}
				} else
					hw_context->mcpc_button_on = 0;
			} else if (hw_context->accessory_type == ACCESSORY_BOARD){
				switch (regAdcValue) {
				case TABLE_ONE_2000_OHM_END_BUTTON:
						mdt_toggle_keyboard_keycode(
							container_of((void *)hw_context
										, struct mhl_dev_context
										, drv_context
										)
							,0xA4);
						break;//KEY_PLAYPAUSE
					case TABLE_ONE_2604_OHM_S1_BUTTON:
						mdt_toggle_keyboard_keycode(
							container_of((void *)hw_context
										, struct mhl_dev_context
										, drv_context
										)
							,0x73);
						break;//KEY_VOLUMEUP
					case TABLE_ONE_3208_OHM_S2_BUTTON:
						mdt_toggle_keyboard_keycode(
							container_of((void *)hw_context
										, struct mhl_dev_context
										, drv_context
										)
							,0x72);
						break;//KEY_VOLUMEDOWN
				};
			}
		}
		if (BIT_LONG_KEY_PRESSED_INT & reg_asw_int1){
			uint8_t regAdcValue;
			// NEED A DEBOUNCE HERE ?
			regAdcValue = mhl_tx_read_reg(hw_context,REG_AUDIO_POLL_STAT1) & MSK_AUDIO_POLL_LKP_IMPED_VAL;
			regAdcValue >>= 1;

			MHL_TX_DBG_ERR(hw_context, "Long (LKP). LK "\
						   "Z:(0x34A) = %02X\n",regAdcValue);
			if (OPEN_IMPEDANCE == regAdcValue){
				detach_on_id_change(hw_context);
			}
		}

		if(BIT_LONG_KEY_RELEASED_INT & reg_asw_int1)
			MHL_TX_DBG_ERR(hw_context, "Long Key Released (LKR)\n");


	}
	return ret_val;
}

/*
    get_device_id
    returns chip Id
 */
int get_device_id(/*struct drv_hw_context *hw_context*/)
{
	int ret_val;
	static int device_id = 0;
	uint16_t number;

	if (!device_id) {
		ret_val = mhl_tx_read_reg(NULL, REG_DEV_IDH_AON);
		if (ret_val < 0) {
			MHL_TX_DBG_ERR(NULL, "I2C error 0x%x\n", ret_val);
			return ret_val;
		}
		number = ret_val << 8;

		ret_val = mhl_tx_read_reg(NULL, REG_DEV_IDL_AON);
		if (ret_val < 0) {
			MHL_TX_DBG_ERR(NULL, "I2C error 0x%x\n", ret_val);
			return ret_val;
		}
		ret_val |= number;

		if(ret_val == DEVICE_ID_8558)
			device_id = DEVICE_ID_8558;
		else
			device_id = DEVICE_ID_8240;

		PR_DISP_INFO("MHL type:%x\n", device_id);
	}

	return device_id;
}
/*
    get_device_rev
    returns chip revision
 */
static int get_device_rev(struct drv_hw_context *hw_context)
{
	int ret_val;

	if(get_device_id() == DEVICE_ID_8558)
		ret_val = mhl_tx_read_reg(hw_context, REG_DEV_REV_AON);
	else
		ret_val = mhl_tx_read_reg(hw_context, REG_DEV_REV);

	if (ret_val < 0) {
		MHL_TX_DBG_ERR(hw_context, "I2C error\n");
		ret_val = -1;
	}

    return ret_val;
}

static void init_transcode_mode(struct drv_hw_context *hw_context)
{
	if (hw_context->chip_device_id == DEVICE_ID_8558) {
		 /* Power up CVCC 1.2V core */
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0x75);
		msleep(100);
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0x7D);

		/* Toggle SW Reset */
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0x6D);
		msleep(10);
		mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0x7D);

		/* Enable Tx PLL */
		mhl_tx_modify_reg(hw_context, REG_TMDS_CCTRL,
				BIT_TMDS_CCTRL_BGRCTL_MASK |
				BIT_TMDS_CCTRL_TMDS_OE_MASK |
				BIT_TMDS_CCTRL_CKDT_EN_MASK,
				BIT_TMDS_CCTRL_TMDS_OE |
				BIT_TMDS_CCTRL_CKDT_EN | 0x04);

		/* Enable Tx outputs */
		mhl_tx_write_reg(hw_context, REG_MHLTX_CFG_OE, 0x3C);

		/* Enable TxPLL clock */
		mhl_tx_write_reg(hw_context, REG_TMDS_CLK_EN, 0x01);

		/* Enable Tx clock path & Equalizer */
		mhl_tx_write_reg(hw_context, REG_TMDS_CH_EN, 0x11);

		/* Discovery Registers */

		/* 1.8V CBUS VTH */
		mhl_tx_write_reg(hw_context, REG_DISC_CTRL5, 0x03);

		/* Enable CBUS discovery */
		mhl_tx_write_reg(hw_context, REG_DISC_CTRL1, VAL_DISC_CTRL1_DEFAULT | BIT_DISC_CTRL1_STROBE_OFF | BIT_DISC_CTRL1_MHL_DISCOVERY_ENABLE);

		/* Enable HDMI Trans-code mode & DDC port */
		mhl_tx_write_reg(hw_context, REG_DCTL, 0x1A);

	} else {
		 // Enable TxPLL clock
		mhl_tx_write_reg(hw_context, REG_TMDS_CLK_EN, 0x01);

		 // Enable Tx clock path & Equalizer
		mhl_tx_write_reg(hw_context, REG_TMDS_CH_EN, 0x11);

		 // Enable Rx PLL clock
		mhl_tx_modify_reg(hw_context, REG_TMDS_CCTRL, BIT_TMDS_CCTRL_BGRCTL_MASK, 0x04);

		 // Enable CBUS discovery
		mhl_tx_write_reg(hw_context,REG_DISC_CTRL1, VAL_DISC_CTRL1_DEFAULT | BIT_DISC_CTRL1_STROBE_OFF | BIT_DISC_CTRL1_MHL_DISCOVERY_ENABLE);
	}
}

/*****************************************************************************/
/**
 * @brief Handler for MHL transmitter reset requests.
 *
 * This function is called by the MHL layer to request that the MHL transmitter
 * chip be reset.  Since the MHL layer is platform agnostic and therefore doesn't
 * know how to control the transmitter's reset pin each platform application is
 * required to implement this function to perform the requested reset operation.
 *
 * @param[in]	hwResetPeriod	Time in ms. that the reset pin is to be asserted.
 * @param[in]	hwResetDelay	Time in ms. to wait after reset pin is released.
 *
 *****************************************************************************/
static void board_reset(struct drv_hw_context *hw_context,
						uint16_t hwResetPeriod,
						uint16_t hwResetDelay)
{
	// Reset the TX chip,
	set_pin(hw_context,TX_HW_RESET, LOW);
	msleep(hwResetPeriod);
	set_pin(hw_context,TX_HW_RESET, HIGH);

	msleep(hwResetDelay);
}
/*
 * clear_and_disable_on_disconnect
 */
static void clear_and_disable_on_disconnect(struct drv_hw_context *hw_context)
{
	uint8_t	intr_num;
	/* clear and mask all interrupts */
	for(intr_num = 0; intr_num < MAX_INTR; intr_num++) {
		if(DEVICE_ID_8558 == hw_context->chip_device_id) {
			if(INTR_ASW0 != intr_num && INTR_ASW1 != intr_num) {
				/* Clear and disable all other interrupts */
				mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].stat_page, g_intr_tbl[intr_num].stat_offset, 0xff);
				enable_intr(hw_context, intr_num, 0x00);
			}
		} else {
			if(INTR_DISC == intr_num) {
				/* clear discovery interrupts except MHL_EST before going to sleep. */
				mhl_tx_write_reg(hw_context,
						g_intr_tbl[INTR_DISC].stat_page,
						g_intr_tbl[INTR_DISC].stat_offset,
						/*BIT_INTR4_MHL_EST*/0xFF);

				/* Keep only RGND interrupt enabled for 8240 and ASW interrupts for 8558 */
				enable_intr(hw_context,
					INTR_DISC,
					BIT_INTR4_RGND_DETECTION);
			} else {
				/* Clear and disable all other interrupts */
				mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].stat_page, g_intr_tbl[intr_num].stat_offset, 0xff);
				enable_intr(hw_context, intr_num, 0x00);
			}
		}
	}
}
/*
 * switch_to_d3
 * This function performs s/w as well as h/w state transitions.
 */
void switch_to_d3(struct drv_hw_context *hw_context,bool do_interrupt_clear)
{
	/*
	 * Check GPIO if chip can enter D3.
	 * to read/write register contents from SiIMon, user may prevent D3.
	 */
	if (get_config(hw_context, ALLOW_D3)) {

		/*
		 * Need to apply infoframe workaround at every power cycle
		 */
		hw_context->ckdt_done = 0;

		mhl_tx_vbus_control(VBUS_OFF);
		/* Meet an MHL CTS timing - Tsrc:cbus_float */
		msleep(50);

		if (do_interrupt_clear) {
			clear_and_disable_on_disconnect(hw_context);
		}
		/*
		 * Change state to D3 by clearing bit 0 power control reg
		 */
		if (hw_context->chip_device_id == DEVICE_ID_8558) {
			mhl_tx_write_reg(hw_context, REG_POWER_CTRL,0x00);
		} else {
			/*
			 * TODO: Check if this is mentioned in the PR?
			 * Power down must only touch bit 0.
			 * otherwise RGND interrupt would not be asserted
			 */
			mhl_tx_write_reg(hw_context, REG_POWER_CTRL, 0xfe);
		}
	} else if (do_interrupt_clear) {
		clear_and_disable_on_disconnect(hw_context);
	}
}

/*
 * disconnect_mhl
 * This function performs s/w as well as h/w state transitions.
 */
static void disconnect_mhl(struct drv_hw_context *hw_context,bool do_interrupt_clear, bool discovery_enable)
{
	/*
	 * Pull down the HPD line to upstream.
	 * set reg_hpd_out_ovr_en to control the hpd
	 * and clear reg_hpd_out_ovr_val.
	 * This is done to prevent upstream to perform EDID or HDCP until we are ready.
	 */
	drive_hpd_low(hw_context);

	/*
	 * Change TMDS termination to high impedance on disconnection
	 */
	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL1,
				BIT_MHLTX_CTL1_TX_TERM_MODE_OFF |
				BIT_MHLTX_CTL1_DISC_OVRIDE_ON);

	if(DEVICE_ID_8558 == hw_context->chip_device_id) {
		/* Disable MHL discovery until RID has been measured. */
		mhl_tx_write_reg(hw_context, REG_DISC_CTRL1, VAL_DISC_CTRL1_DEFAULT);
	} else if (DEVICE_ID_8240 == hw_context->chip_device_id) {
		/* Restore default for REG_DISC_CTRL2 */
		mhl_tx_write_reg(hw_context, REG_DISC_CTRL2, 0xAD);

		if (discovery_enable) {
			/* Enable MHL discovery so we are waken up by h/w on impedance measurement */
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL1,
					VAL_DISC_CTRL1_DEFAULT |
					BIT_DISC_CTRL1_STROBE_OFF |
					BIT_DISC_CTRL1_MHL_DISCOVERY_ENABLE);
		} else {
			/*
			*	Disable USB_ID swtich at the first time while mhl chip initialize.
			*	It can avoid system read adc value before usb driver ready.
			*	System would enable USB_ID switch later after related driver ready.
			*/
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL1,
					VAL_DISC_CTRL1_DEFAULT | BIT_DISC_CTRL1_STROBE_OFF);
		}
	}

	/* Disable hdmi and switch to USB D+/D-*/
	enable_hdmi(false);

	if (do_interrupt_clear) {
		clear_and_disable_on_disconnect(hw_context);
	}
	/* 11/23: clear this flag to fix DS hot plug issue */
	hw_context->cbus_status = 0;
}

/*
 * int_4_isr
 * MHL device discovery interrupt handler
 * 	1. When impedance is measured as 1k, RGND interrupt is asserted.
 * 	   (only for SiI8240. Impedance is measured via a table for SiI8558)
 * 	2. Chip sends out wake pulses and discovery pulses.
 * 	   Then asserts MHL_EST if CBUS stays high to meet MHL timings.
 * 	3. If discovery fails, NON_MHL_EST is asserted.
 *  4. If MHL cable is removed, CBUS_DIS is asserted.
 *     (Need to check this bit all the time)
 */
static int int_4_isr(struct drv_hw_context *hw_context, uint8_t int_4_status)
{
	int	ret_val = 0; /* Safe to clear interrupt from master handler */
	struct 	mhl_dev_context	*dev_context = get_mhl_device_context(hw_context);

	if ((BIT_INTR4_CBUS_DISCONNECT  & int_4_status) ||
		(BIT_INTR4_NON_MHL_EST & int_4_status)) {
		MHL_TX_DBG_ERR(hw_context, "got CBUS_DIS. MHL disconnection\n");

		set_pin(hw_context,MHL_LED, GPIO_LED_OFF);
		set_pin(hw_context,USB_LED, GPIO_LED_OFF);
		set_pin(hw_context,AUDIO_LED, GPIO_LED_OFF);
		set_pin(hw_context,MIC_LED,   GPIO_LED_OFF);
		/* Setup termination etc. */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_DISCONNECT;
		if (BIT_INTR4_CBUS_DISCONNECT & int_4_status) {
#ifdef DDC_BYPASS_API //(
			si_8240_8558_drv_ddc_bypass_control(hw_context,0);
#endif //)
			disconnect_mhl(hw_context, true, false);
			/* Inform MHL disconnect and cable type to USB driver */
			dev_context->isMHL = false;
			dev_context->statMHL = CONNECT_TYPE_UNKNOWN;
			queue_work(dev_context->wq, &dev_context->mhl_notifier_work);
			switch_to_d3(hw_context,false);
		} else { /* must be BIT_INTR4_NON_MHL_EST */
			disconnect_mhl(hw_context, false, false);
			/* Inform MHL disconnect and cable type to USB driver */
			dev_context->isMHL = false;
			dev_context->statMHL = CONNECT_TYPE_UNKNOWN;
			queue_work(dev_context->wq, &dev_context->mhl_notifier_work);
			switch_to_d3(hw_context,true);
		}

		dev_context->fake_cable_out = false;	/*For fake remove event used */
		set_pin(hw_context,SINK_VBUS_ON_LED,
				get_config(hw_context,TWELVE_VOLT_PS_SENSE) ?
				GPIO_LED_ON : GPIO_LED_OFF);
		rid_index = NO_CONNECTION_INDEX;
		ret_val = 0xFF;	/* interrupt has been cleared already in disconnect_mhl */

	} else if (int_4_status & BIT_INTR4_RGND_DETECTION) {
		/* For 8240 only. 8558 does not unmask RGND Interrupt */
		if (0x02 == (mhl_tx_read_reg(hw_context, REG_DISC_STAT2) & 0x03)) {
			MHL_TX_DBG_ERR(hw_context, "Cable impedance = 1k (MHL Device)\n");
			/* Enable HDMI and switch to MHL D+/D- */
			enable_hdmi(true);

			/* Power up the chip cores to access registers */
			power_up(hw_context);
			/* Ensure Wake up pulse is sent */
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL9
						, BIT_DC9_WAKE_DRVFLT
						| BIT_DC9_DISC_PULSE_PROCEED
						);

			/* Call platform function to turn the VBUS on for unpowered dongle */
			mhl_tx_vbus_control(VBUS_ON);
			msleep(100);
			/* Inform MHL connect and cable type to USB driver */
			dev_context->isMHL = true;
			dev_context->statMHL = CONNECT_TYPE_INTERNAL;
			queue_work(dev_context->wq, &dev_context->mhl_notifier_work);
		} else {
			MHL_TX_DBG_ERR(hw_context, "Skip cable impedance:%#x\n",
					mhl_tx_read_reg(hw_context, REG_DISC_STAT2));
		}

		/* enable remaining discovery interrupts */
		enable_intr(hw_context, INTR_DISC,
				  BIT_INTR4_MHL_EST
				| BIT_INTR4_NON_MHL_EST
				| BIT_INTR4_CBUS_LKOUT
				| BIT_INTR4_CBUS_DISCONNECT
				| BIT_INTR4_RGND_DETECTION
				| BIT_INTR4_VBUS_CHG // this is required
				);
		/* Enable MSC interrupt to handle initial exchanges */
		enable_intr(hw_context, INTR_MERR,
				 ( BIT_CBUS_DDC_ABRT
				 | BIT_CBUS_MSC_ABORT_RCVD
				 | BIT_CBUS_CMD_ABORT
				 ));
		enable_intr(hw_context, INTR_MSC,
				(  BIT_CBUS_MSC_MT_DONE
				 | BIT_CBUS_HPD_RCVD
				 | BIT_CBUS_MSC_MR_WRITE_STAT
				 | BIT_CBUS_MSC_MR_MSC_MSG
				 | BIT_CBUS_MSC_MR_WRITE_BURST
				 | BIT_CBUS_MSC_MR_SET_INT
				 | BIT_CBUS_MSC_MT_DONE_NACK
				));
	} else if (int_4_status & BIT_INTR4_MHL_EST) {
		/*
		 * ENABLE_DISCOVERY ensures it sends wake up and discovery pulse
		 * 	 and as result sink/dongle would respond CBUS high.
		 * 8240: ENABLE_DISCOVERY is always set. Therefore, on successful
		 * 		 discovery, 8240 will assert MHL_EST.
		 * 8588: ENABLE_DISCOVERY is performed after accessory switch interrupt
		 * 		 discovers 1K. Then MHL_EST will be asserted.
		 */
		MHL_TX_DBG_ERR(hw_context, "got MHL_EST.  MHL connection\n");

		/* turn on  MHL LED */
		set_pin(hw_context,MHL_LED,GPIO_LED_ON);
		set_pin(hw_context,USB_LED, GPIO_LED_OFF);
		set_pin(hw_context,SINK_VBUS_ON_LED,
				get_config(hw_context,TWELVE_VOLT_PS_SENSE) ?
				GPIO_LED_ON:GPIO_LED_OFF);

		/* Setup registers and enable interrupts for DCAP_RDY, SET_HPD etc. */
		init_regs(hw_context);

		/*
		 *  Setting this flag triggers sending DCAP_RDY.
		 *  Tested RK-9296 - it works fine.
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CONNECT;
	}
	return ret_val;
}

static int g2wb_isr(struct drv_hw_context *hw_context, uint8_t intr_stat)
{
	uint8_t	ret_val = 0;
	uint8_t	mdt_buffer[20] = {0};

	/* Read error register if there was any problem */
	ret_val  = mhl_tx_read_reg(hw_context, REG_CBUS_MDT_INT_1);

	if (ret_val) {
		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_INT_1, ret_val);
		MHL_TX_DBG_INFO(hw_context, "\n\ngot MDT Error = %02X\n", ret_val);
	} else {
		uint8_t	length;

		/* Read all bytes */
		mhl_tx_read_reg_block(hw_context,
				REG_CBUS_MDT_RCV_READ_PORT,
				16,
				mdt_buffer);

		/* first byte contains the length of data */
		length = mdt_buffer[0];
		/*
		 * There isn't any way to know how much of the scratch pad
		 * was written so we have to read it all.  The app. will have
		 * to parse the data to know how much of it is valid.
		 */
/*		mhl_tx_read_reg_block(hw_context, REG_CBUS_MDT_RCV_READ_PORT,
				ARRAY_SIZE(hw_context->write_burst_data),
				hw_context->write_burst_data);
*/
		memcpy(hw_context->write_burst_data, &mdt_buffer[1], 16);

		/* Signal upper layer of this arrival */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_WRITE_BURST;

		/*
		 * Clear current level in the FIFO.
		 * Moves pointer to the next keep RSM enabled
		 */
		mhl_tx_write_reg(hw_context,
				  REG_CBUS_MDT_RCV_CONTROL,
				  BIT_CBUS_MDT_RCV_CONTROL_RFIFO_CLR_CUR_CLEAR
				| BIT_CBUS_MDT_RCV_CONTROL_RCV_EN_ENABLE
				);
	}
	return 0;
}

static void enable_intr(struct drv_hw_context *hw_context, uint8_t intr_num, uint8_t intr_mask)
{
	g_intr_tbl[intr_num].mask = intr_mask;
	mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].mask_page,
				g_intr_tbl[intr_num].mask_offset, intr_mask);
}

void si_mhl_tx_drv_device_isr(struct drv_hw_context *hw_context,
				struct interrupt_info *intr_info)
{
	uint8_t	intr_num;

	hw_context->intr_info = intr_info;

	MHL_TX_DBG_INFO(hw_context, "\n\ngot INTR\n");

	/* Skip checking interrupts if GPIO pin is not asserted anymore */
	for(intr_num = 0; (intr_num < MAX_INTR) && (is_interrupt_asserted()); intr_num++) {
		if(g_intr_tbl[intr_num].mask) {
			int reg_value;
			uint8_t	intr_stat;

			reg_value = mhl_tx_read_reg(hw_context,
					g_intr_tbl[intr_num].stat_page,
					g_intr_tbl[intr_num].stat_offset);

			if (reg_value < 0) {
				return;
			}

			intr_stat = (uint8_t)reg_value;

			/* Process only specific interrupts we have enabled. Ignore others*/
			intr_stat = intr_stat & g_intr_tbl[intr_num].mask;
			if (intr_stat) {
				int already_cleared;

#ifdef	PRINT_ALL_INTR
				MHL_TX_DBG_ERR(hw_context, "INTR-%s = %02X\n",
						g_intr_tbl[intr_num].name, intr_stat);
#else	// PRINT_ALL_INTR
				MHL_TX_DBG_INFO(hw_context, "INTR-%s = %02X\n",
						g_intr_tbl[intr_num].name, intr_stat);
#endif	// PRINT_ALL_INTR

				if (intr_num == 0)	/*For Debug purpose to print INT_4*/
					MHL_TX_DBG_ERR(hw_context, "INTR-%s = %02X\n",
							g_intr_tbl[intr_num].name, intr_stat);

				already_cleared = g_intr_tbl[intr_num].isr(hw_context, intr_stat);
				if (already_cleared >= 0) {
					/*
					 * only clear the interrupts that were not cleared by the specific ISR.
					 */
					intr_stat &= ~already_cleared;
					if (intr_stat) {
						/* Clear interrupt since specific ISR did not. */
						mhl_tx_write_reg(hw_context,
								g_intr_tbl[intr_num].stat_page,
								g_intr_tbl[intr_num].stat_offset,
								intr_stat);
					}
				}
			}
		}
#ifdef	PRINT_ALL_INTR
		/* These lines print all interrupt status irrespective of mask */
		else {
			uint8_t	intr_stat;
			/* Only for debugging - Print other masked interrupts - do not clear */
			intr_stat = mhl_tx_read_reg(hw_context,
							g_intr_tbl[intr_num].stat_page,
							g_intr_tbl[intr_num].stat_offset);
			MHL_TX_DBG_ERR(hw_context, "INTN-%s = %02X\n",
					g_intr_tbl[intr_num].name, intr_stat);
		}
#endif	// PRINT_ALL_INTR
	}
}

static void trigger_retention(struct drv_hw_context *hw_context)
{
	int dummy;
	mhl_tx_write_reg(hw_context,REG_POWER_STAT ,BIT_POWER_STAT_I2C_RETAIN);
	dummy = mhl_tx_read_reg(hw_context,REG_POWER_STAT); /* commit USE_RET */
}


static void check_strapping_mode(struct drv_hw_context *hw_context)
{
	if (hw_context->suspend_strapped_mode) {
		int ret_data_0;
		int dummy;
		/* load up the retention registers with PON defaults */
		trigger_retention(hw_context);

		set_pin(hw_context,USB_LED     ,GPIO_LED_OFF);
		set_pin(hw_context,USB_ID_L_LED,GPIO_LED_OFF);

		MHL_TX_DBG_ERR(hw_context,"getting out of BYPASS mode\n");

		mhl_tx_write_reg(hw_context, REG_DISC_CTRL1,VAL_DISC_CTRL1_DEFAULT);
		mhl_tx_write_reg(hw_context,REG_INTR4,mhl_tx_read_reg(hw_context,REG_INTR4));

		mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL,BIT_RET_ADDR_CLR,BIT_RET_ADDR_CLR);
		ret_data_0 = mhl_tx_read_reg(hw_context,REG_RETENTION_READ);
		ret_data_0 &= 0xF3;
		if (BIT_RET_MON0_BC_CTRL_BYPASS== hw_context->strapping_value){
			if (2 == override_strap){
				/*
				* format the value from REG_RET_MON0 for retention register 0, bits 3:2
				*/
				ret_data_0 |= (BIT_RET_MON0_BC_CTRL_FW_TRIG >> 4);
				mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL
							,BIT_ASW_TEST_CTRL_ASW_FORCE_MODE
							,BIT_ASW_TEST_CTRL_ASW_FORCE_MODE
							);
			}
		}

		mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL,BIT_RET_ADDR_CLR,BIT_RET_ADDR_CLR);
		mhl_tx_write_reg(hw_context,REG_RETENTION_WRITE,ret_data_0);
		/* do dummy read to satisfy H/W requirements for REG_RETENTION_WRITE */
		dummy =  mhl_tx_read_reg(hw_context,REG_ASW_TEST_CTRL);

		board_reset(hw_context,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);

		MHL_TX_DBG_ERR(hw_context,"exited BYPASS mode ret_mon0:0x%x asw_mon:0x%x\n"
			,mhl_tx_read_reg(hw_context,REG_RET_MON0)
			,mhl_tx_read_reg(hw_context,REG_ASW_MON)
			);
	} else {
		trigger_retention(hw_context);
	}
}

/*
 * si_mhl_tx_chip_initialize
 *
 * Chip specific initialization.
 * This function resets and initializes the transmitter and puts chip into sleep.
 * MHL Detection interrupt setups up the chip for video.
 */
int si_mhl_tx_chip_initialize(struct drv_hw_context *hw_context, bool discovery_enable)
{
	int ret_val;
	int status = -1;
	hw_context->accessory_type =ACCESSORY_DISCONNECTED;
	hw_context->mcpc_button_on=0;
	hw_context->suspend_strapped_mode = false;
	/*
	 * Don't yet know which MHL transmitter we're controlling so we assume
	 * it's an 8558 since this part is pretty much a superset of the 8240.
	 * FW_WAKE pin is only available in 8558.
	 */
	set_pin(hw_context,TX_FW_WAKE, LOW);/* inverter on board */

	if (get_device_id() == DEVICE_ID_8558) {
		int ret_mon0;

		// Clear all mask. By default FW_WAKE mask is enabled.
		// disabling FW_WAKE ensures if we returned with error, no interrupt
		// shall be asserted.
		enable_intr(hw_context, INTR_ASW1, 0);

		ret_mon0 = mhl_tx_read_reg(hw_context,REG_RET_MON0);
		hw_context->boot_usb_impedance_reference = ret_mon0 & BIT_RET_MON0_BOOT_USB_CTRL_MASK;
		hw_context->boot_uart_impedance_reference= BIT_REG_MON2_BOOT_UART_CTRL_MASK & mhl_tx_read_reg(hw_context,REG_RET_MON1);
		ret_mon0 &= BIT_RET_MON0_MEAS_BC_CTRL_MASK;
		hw_context->strapping_value = ret_mon0;
		switch (ret_mon0) {
		case BIT_RET_MON0_BC_CTRL_VBAT:
			break;
		case BIT_RET_MON0_BC_CTRL_FW_TRIG:
			{
				if (override_strap) {
					int temp;
					temp = manual_strap_measurement(hw_context,BIT_RID8_FW_ADC_SEL_BC_CTRL);

					if (BIT_RET_MON0_BC_CTRL_BYPASS == temp) {
						hw_context->strapping_value       = temp;
						hw_context->suspend_strapped_mode = true;
					} else {
						mhl_tx_modify_reg(hw_context,REG_BC_OVR_CTRL
							,BIT_BC_OVR_CTRL_BC_FW_RDY
							,BIT_BC_OVR_CTRL_BC_FW_RDY);
					}
				} else {
					mhl_tx_modify_reg(hw_context,REG_BC_OVR_CTRL
						,BIT_BC_OVR_CTRL_BC_FW_RDY
						,BIT_BC_OVR_CTRL_BC_FW_RDY);
				}
			}
			break;
		case BIT_RET_MON0_BC_CTRL_BYPASS:
			set_pin(hw_context,USB_LED     ,GPIO_LED_ON);
			set_pin(hw_context,USB_ID_L_LED,GPIO_LED_ON);
			if (override_strap) {
				/* for all values other than zero */
				hw_context->suspend_strapped_mode = true;
			} else {
				MHL_TX_DBG_ERR(hw_context,"remaining in bypass mode\n");
				set_pin(hw_context,M2U_VBUS_CTRL_M,1);
				//return -1;

				//to continue init and report success, so the
				//driver could be properly removed by rmmod (to fix #27447)
			}
			break;
		case BIT_RET_MON0_BC_CTRL_RESERVED:
			return -1;
		}
	} else {
		/* Power up 8240 to read device ID */
		power_up(hw_context);
	}
	ret_val = get_device_rev(hw_context);

/*	if (ret_val > 0)		We should alert if someone is running older chip */
	{
		hw_context->chip_rev_id = (uint8_t)ret_val;

		ret_val = get_device_id(/*hw_context*/);
		if (ret_val > 0) {
			hw_context->chip_device_id = (uint16_t)ret_val;

			MHL_TX_DBG_ERR(hw_context, "Found SiI%04X rev: %01X.%01X\n",
					hw_context->chip_device_id,
					hw_context->chip_rev_id >> 4,
					(hw_context->chip_rev_id & 0x0f));

			if(DEVICE_ID_8558 == hw_context->chip_device_id) {
				int ret_mon0, ret_mon1, regAdcTable;
				uint8_t	reg_asw_mon;
				regAdcTable = mhl_tx_read_reg(hw_context, REG_ADC_R_TABLE);	/* 0x346 */
				regAdcTable &= MSK_ADC_R_TABLE;
				reg_asw_mon = mhl_tx_read_reg(hw_context, REG_ASW_MON);		/* 0x347 */
				MHL_TX_DBG_ERR(hw_context,"switch position:0x%02x,%s\n"
								,reg_asw_mon &MSK_ASW_MON_ASW
								,table_asw_mon[reg_asw_mon &MSK_ASW_MON_ASW]
								);
				if (get_config(hw_context,MCPC_MODE)){
					/*
					 * Choose loose or strict RID error checking based on MCPC mode
					 */

					mhl_tx_write_reg(hw_context
							, REG_RID2
							, (DEF_VAL_RID2 & ~BIT_RID2_STRICT_ERROR_MASK)
							  | BIT_RID2_STRICT_ERROR_LOOSE
							);
					rid_table = tableOneMCPC;
					MHL_TX_DBG_ERR(hw_context
						,"MCPC mode! Using tableOneMCPC"
						 " and loose R_ID checking\n");
				} else {
					mhl_tx_write_reg(hw_context
							, REG_RID2
							, (DEF_VAL_RID2 & ~BIT_RID2_STRICT_ERROR_MASK)
							  | BIT_RID2_STRICT_ERROR_STRICT
							);
					switch (regAdcTable) {
					case 0:
						rid_table = tableOne;
						MHL_TX_DBG_ERR(hw_context,"using table 1\n");
						break;
					case 1:
						rid_table = tableOne;
						MHL_TX_DBG_ERR(hw_context,"using t1p\n");
						break;
					case 2:
						rid_table = tableTwo;
						MHL_TX_DBG_ERR(hw_context,"using table 2\n");
						break;
					default:
						rid_table = tableOne;
						MHL_TX_DBG_ERR(hw_context,"Unknown! Defaulting to table 1\n");
					}
				}
				ret_mon0 = mhl_tx_read_reg(hw_context,REG_RET_MON0);
				ret_mon1 = mhl_tx_read_reg(hw_context,REG_RET_MON1);
				MHL_TX_DBG_ERR(hw_context,"\n"
						 "\tMeasured BC_CTRL  = 0x%02x\n"
						 "\tMeasured RID_CTRL = 0x%02x\n"
						 "\tMeasured BOOT_USB = 0x%02x\n"
						 "\tMeasured BOOT_UART= 0x%02x\n"
						, (ret_mon0 & BIT_RET_MON0_MEAS_BC_CTRL_MASK)>>6
						, (ret_mon1 & BIT_RET_MON1_MEAS_RID_CTRL_MASK)>>6
						, ret_mon0  & BIT_RET_MON0_MEAS_BOOT_USB_MASK
						, ret_mon1  & BIT_RET_MON1_MEAS_BOOT_UART_MASK
						);
				/* Enable interrupts for accessory switch */
				enable_intr(hw_context, INTR_ASW0,
						( BIT_RID_MSR_DONE_INT
						| BIT_RID_MSR_ERR_INT
						| BIT_RSTRAP_MSR_DONE_INT
						| BIT_ASW_ATTACH_INT
						| BIT_ASW_DETACH_INT
						| BIT_BC_DONE_INT)
						);
				enable_intr(hw_context, INTR_ASW1,
						( BIT_VBUS_CHG_INT
						| BIT_ID_CHG_INT
						| BIT_ADC_CHG_INT
//						| BIT_WKUP_HIGH_INT
						| BIT_OVP_INT
						| BIT_SHORT_KEY_PRESSED_INT
						| BIT_LONG_KEY_PRESSED_INT
						| BIT_LONG_KEY_RELEASED_INT)
						);
				/* Move to disconnected state. Let RGND/MHL connection event start the driver */
				disconnect_mhl(hw_context, true, discovery_enable);
				/* use retention registers */
				check_strapping_mode(hw_context);
			} else {
				/* Move to disconnected state. Let RGND/MHL connection event start the driver */
				disconnect_mhl(hw_context, true, discovery_enable);
			}
			switch_to_d3(hw_context,false);

			status = 0;
		}
	}

	set_pin(hw_context,TX_FW_WAKE, HIGH);
	return status;
}

int si_mhl_tx_get_accessory_switch_config(struct drv_hw_context *hw_context,char **string)
{
	*string = rid_table[rid_index].description;
		return mhl_tx_read_reg(hw_context,REG_ASW_MANUAL_CTRL);
}

void si_mhl_tx_set_accessory_switch_config(struct drv_hw_context *hw_context,int config)
{
	rid_index =RID_INDEX_USER;
	mhl_tx_write_reg(hw_context, REG_ASW_MANUAL_CTRL, (uint8_t)config);
}

void si_mhl_tx_id_impedance_measurement(struct drv_hw_context *hw_context)
{
	int dummy;
	bool fw_wake_state;
	/* save state of FW_WAKE pin */
	fw_wake_state = get_config(hw_context,TX_FW_WAKE);

	/* wake up the 8558 */
	set_pin(hw_context,TX_FW_WAKE, LOW);/* inverter on board */
	MHL_TX_DBG_INFO(hw_context,"REG_ASW_INT0_MASK: 0x%02x\n"
		,mhl_tx_read_reg(hw_context,REG_ASW_INT0_MASK));
	msleep(TX_HW_RESET_PERIOD);
	mhl_tx_modify_reg(hw_context
		, REG_ASW_CTRL
		, BIT_ASW_CTRL_RID_MEASUREMENT_REQUEST_STROBE
		, BIT_ASW_CTRL_RID_MEASUREMENT_REQUEST_STROBE
		);
	/*
		do an extra I2C transaction to satisfy H/W
		requirement for strobing this bit
	*/
	dummy = mhl_tx_read_reg(hw_context,REG_ASW_CTRL);

	/* restore state of FW_WAKE ping */
	set_pin(hw_context,TX_FW_WAKE, fw_wake_state);
}

int si_mhl_tx_get_retention_range(struct drv_hw_context *hw_context,retention_data_t values)
{
	int ret_val,dummy;
	mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL,BIT_RET_ADDR_CLR,BIT_RET_ADDR_CLR);
	ret_val = mhl_tx_read_reg_block(hw_context,REG_RETENTION_READ,NUM_RETENTION_REGS,values);

	/* do dummy read to satisfy H/W requirements for REG_RETENTION_READ */
	dummy =  mhl_tx_read_reg(hw_context,REG_ASW_TEST_CTRL);
	MHL_TX_DBG_ERR(hw_context,"ret-regs: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n"
		,values[0],values[1],values[2],values[3],values[4]
		,values[5],values[6],values[7],values[8],values[9]
		);

	return  ret_val;
}

int si_mhl_tx_set_retention_range(struct drv_hw_context *hw_context,retention_data_t values)
{
	int ret_val,dummy;
	mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL,BIT_RET_ADDR_CLR,BIT_RET_ADDR_CLR);
	ret_val = mhl_tx_write_reg_block(hw_context,REG_RETENTION_WRITE,NUM_RETENTION_REGS,values);

	/* do dummy read to satisfy H/W requirements for REG_RETENTION_WRITE */
	dummy =  mhl_tx_read_reg(hw_context,REG_ASW_TEST_CTRL);

	return ret_val;
}

void si_mhl_tx_drv_shutdown(struct drv_hw_context *hw_context)
{
	if(is_reset_on_exit_requested()) {
		if (hw_context->suspend_strapped_mode) {
			int dummy;
			int ret_data_0;
			set_pin(hw_context,TX_FW_WAKE, LOW);/* inverter on board */
			mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL,BIT_RET_ADDR_CLR,BIT_RET_ADDR_CLR);
			ret_data_0 = mhl_tx_read_reg(hw_context,REG_RETENTION_READ);
			ret_data_0 &= 0xF3;
			ret_data_0 |= (hw_context->strapping_value >> 4);
			mhl_tx_modify_reg(hw_context,REG_ASW_TEST_CTRL,BIT_RET_ADDR_CLR,BIT_RET_ADDR_CLR);
			mhl_tx_write_reg(hw_context,REG_RETENTION_WRITE,ret_data_0);
			dummy = mhl_tx_read_reg(hw_context,REG_RET_MON0);
			/* reset the board to let the chip reset itself into bypass mode
			*/
			board_reset(hw_context,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);
			set_pin(hw_context,TX_FW_WAKE, HIGH);/* inverter on board */
		} else {
			board_reset(hw_context,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);
		}
		MHL_TX_DBG_ERR(hw_context, "MHL hardware was reset\n");
	}
}

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
void Enable_Backdoor_Access( struct drv_hw_context *hw_context )
{
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x33);	/*enable backdoor access*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_1ST_TRANSMIT_DATA, 0x80);
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
					BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);
}

void Disable_Backdoor_Access( struct drv_hw_context *hw_context )
{
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x33);	/*disable backdoor access*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_1ST_TRANSMIT_DATA, 0x00);
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
					BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);
}

void Tri_state_dongle_GPIO0( struct drv_hw_context *hw_context )
{
	mhl_tx_write_reg(hw_context, REG_CBUS_WB_XMIT_DATA_0, 0xFF);    /* main page , FE for Cbus page*/
	mhl_tx_write_reg(hw_context, REG_CBUS_WB_XMIT_DATA_1, 0x7F);    /* offset*/
	mhl_tx_write_reg(hw_context, REG_CBUS_WB_XMIT_DATA_2, 0xFF);    /* data set GPIO=input*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_WRITE_BURST_DATA_LEN, 0x02);  /* burst length-1*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x48);         /* offset in scratch pad*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START, BIT_CBUS_MSC_WRITE_BURST);     /*trig this cmd*/
}

void Low_dongle_GPIO0(struct drv_hw_context *hw_context)
{
	mhl_tx_write_reg(hw_context, REG_CBUS_WB_XMIT_DATA_0, 0xFF);    /* main page , FE for Cbus page*/
	mhl_tx_write_reg(hw_context, REG_CBUS_WB_XMIT_DATA_1, 0x7F);    /* offset*/
	mhl_tx_write_reg(hw_context, REG_CBUS_WB_XMIT_DATA_2, 0xFC);    /* data set GPIO=input*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_WRITE_BURST_DATA_LEN, 0x02);  /* burst length-1*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x48);         /* offset in scratch pad*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START, BIT_CBUS_MSC_WRITE_BURST);     /* trig this cmd*/
}

void AppVbusControl(struct drv_hw_context *hw_context, uint8_t dev_cat)
{
	uint8_t devcap = dev_cat;

	if (!devcap) {	/*Not Sink, NOt Dongle, Not Source*/
		MHL_TX_DBG_ERR(NULL, "Invalid dev_cat:%#x\n", dev_cat);
		return;
	}

	MHL_TX_DBG_ERR(hw_context, "dev_cat:%#x\n", dev_cat);
	if ((devcap & (MHL_DEV_CATEGORY_POW_BIT | MHL_DEV_CAT_DONGLE)) ==
		(MHL_DEV_CATEGORY_POW_BIT | MHL_DEV_CAT_SINK)) {/*Sink can output power*/
		MHL_TX_DBG_ERR(hw_context, "==Rx_Sink\n");
		htc_charging_enable(dev_cat);
	} else if ((devcap & MHL_DEV_CAT_DONGLE) == MHL_DEV_CAT_DONGLE) { /*Dongle, due to POW=0 if USB attached, and GPIO0=low*/
		MHL_TX_DBG_ERR(hw_context, "==Rx_Dongle, 3 state GPIO0\n");
		Enable_Backdoor_Access(hw_context); // Karl 20131219 eliminate the times to en/disable backdoor
		Tri_state_dongle_GPIO0(hw_context);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x02);/* readDevCap02*/
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
						BIT_CBUS_MSC_READ_DEVCAP);	/*send cmd*/
		msleep(1);

		devcap = mhl_tx_read_reg(hw_context, REG_CBUS_PRI_RD_DATA_1ST);

		if (devcap & MHL_DEV_CATEGORY_POW_BIT) {
			MHL_TX_DBG_ERR(hw_context, "devcap=%02x ,POW=1=, low GPIO0\n",devcap);
			/* Turn off phone Vbus output*/
			Low_dongle_GPIO0(hw_context);
			mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x02);	/* readDevCap02*/
			mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
							BIT_CBUS_MSC_READ_DEVCAP);	/* send cmd*/
			msleep(1);

		} else{
			MHL_TX_DBG_ERR(hw_context, "Dongle has no power,enable 5V to Vbus\n");

			Disable_Backdoor_Access(hw_context);

			/*clear 0xc8:0xbc to 0x00*/
			mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x0F);	/* readDevCap0f*/
			mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
							BIT_CBUS_MSC_READ_DEVCAP);	/* send cmd*/
			msleep(1);

			devcap = mhl_tx_read_reg(hw_context, REG_CBUS_PRI_RD_DATA_1ST);
			return ;
		}

		devcap = mhl_tx_read_reg(hw_context, REG_CBUS_PRI_RD_DATA_1ST);

		if (devcap & MHL_DEV_CATEGORY_POW_BIT) {
			MHL_TX_DBG_ERR(hw_context, "AC attached\n");
			htc_charging_enable(dev_cat);
		} else {
			MHL_TX_DBG_ERR(hw_context, "USB attached, charger disabled, 3 state GPIO0\n");
			Tri_state_dongle_GPIO0(hw_context);/*avoid disconnect USB,
							    *no POW change interrupt happened situation*/
		}
		Disable_Backdoor_Access(hw_context);
	}
	/*clear 0xc8:0xbc to 0x00*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, 0x0F);	/*readDevCap0f*/
	mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START, BIT_CBUS_MSC_READ_DEVCAP);/* send cmd*/
	msleep(1);
	devcap = mhl_tx_read_reg(hw_context, REG_CBUS_PRI_RD_DATA_1ST);

	MHL_TX_DBG_ERR(hw_context, "Downstream DecCap0x0F = %#02x\n", devcap);
}
#endif
