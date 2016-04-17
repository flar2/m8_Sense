
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include "pn548_htc.h"

#if NFC_READ_RFSKUID
#define HAS_NFC_CHIP 0x7000000
#endif 


#if NFC_GET_BOOTMODE
#include <htc/devices_cmdline.h>
#endif 



#define D(x...)	\
	if (is_debug) \
		printk(KERN_DEBUG "[NFC] " x)
#define I(x...) printk(KERN_INFO "[NFC] " x)
#define E(x...) printk(KERN_ERR "[NFC] [Err] " x)


#if NFC_OFF_MODE_CHARGING_LOAD_SWITCH
static unsigned int   pvdd_gpio;
#endif 


int pn548_htc_check_rfskuid(int in_is_alive){
#if NFC_READ_RFSKUID
        int nfc_rfbandid_size = 0;
        int i;
        unsigned int *nfc_rfbandid_info;
        struct device_node *nfc_rfbandid;
        nfc_rfbandid = of_find_node_by_path("/chosen/mfg");
        if (nfc_rfbandid){
                nfc_rfbandid_info = (unsigned int *) of_get_property(nfc_rfbandid,"skuid.rf_id",&nfc_rfbandid_size);
                if (!nfc_rfbandid_info){
                        E("%s:Get null pointer of rfbandid\n",__func__);
                        return 1;
                }
        }else {
                E("%s:Get skuid.rf_id fail keep NFC by default\n",__func__);
                return 1;
        }
        if(nfc_rfbandid_size != 32) {  
                E("%s:Get skuid.rf_id size error keep NFC by default\n",__func__);
                return 1;
        }

        for ( i = 0; i < 8; i++) {
                if (nfc_rfbandid_info[i] == HAS_NFC_CHIP) {
                        I("%s: Check skuid.rf_id done device has NFC chip\n",__func__);
                        return 1;
                }
        }
        I("%s: Check skuid.rf_id done remove NFC\n",__func__);
        return 0;
#else 
        return in_is_alive;
#endif 

}


int pn548_htc_get_bootmode(void) {
	int bootmode = NFC_BOOT_MODE_NORMAL;
#if NFC_GET_BOOTMODE
	bootmode = board_mfg_mode();
	if (bootmode == MFG_MODE_OFFMODE_CHARGING) {
		I("%s: Check bootmode done NFC_BOOT_MODE_OFF_MODE_CHARGING\n",__func__);
		return NFC_BOOT_MODE_OFF_MODE_CHARGING;
	} else {
		I("%s: Check bootmode done NFC_BOOT_MODE_NORMAL mode = %d\n",__func__,bootmode);
		return NFC_BOOT_MODE_NORMAL;
	}
#else
	return bootmode;
#endif  
}


void pn548_htc_parse_dt(struct device *dev) {
#if NFC_OFF_MODE_CHARGING_LOAD_SWITCH
	struct device_node *dt = dev->of_node;
	pvdd_gpio = of_get_named_gpio_flags(dt, "nxp,pvdd-gpio",0, NULL);
	I("%s: pvdd_gpio:%d\n", __func__, pvdd_gpio);
#endif
}

void pn548_htc_off_mode_charging (void) {
#if NFC_OFF_MODE_CHARGING_LOAD_SWITCH
	I("%s: Turn off NFC_PVDD \n", __func__);
	gpio_set_value(pvdd_gpio, 0);
#endif
}

