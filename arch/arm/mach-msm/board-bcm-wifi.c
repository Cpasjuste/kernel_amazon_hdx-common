
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/if.h>
#include <linux/random.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/fs.h>
#include <asm/io.h>

#include <linux/of.h>

#define WLAN_STATIC_SCAN_BUF0           5
#define WLAN_STATIC_SCAN_BUF1           6
#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define DHD_SKB_HDRSIZE                 336
#define DHD_SKB_1PAGE_BUFSIZE   ((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE   ((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE   ((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM        17

#define WLAN_POWER          82
#define WLAN_HOSTWAKE       121

/*
static int gpio_power = WLAN_POWER;
static int gpio_hostwake = WLAN_HOSTWAKE;
*/

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

static void *wlan_static_scan_buf0 = NULL;
static void *wlan_static_scan_buf1 = NULL;

static void *bcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;

	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;

	if (wifi_mem_array[section].size < size)
		return NULL;

	return wifi_mem_array[section].mem_ptr;
}

static int bcm_wifi_mem_init(void)
{
	int i;

	for (i = 0; i < WLAN_SKB_BUF_NUM; i++) {
		wlan_static_skb[i] = NULL;
	}

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i]) {
			goto err_skb_alloc;
		}
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i]) {
			goto err_skb_alloc;
		}
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i]) {
		goto err_skb_alloc;
	}

	for (i = 0 ; i < PREALLOC_WLAN_NUMBER_OF_SECTIONS; i++) {
		wifi_mem_array[i].mem_ptr =
			kmalloc(wifi_mem_array[i].size, GFP_KERNEL);
		if (!wifi_mem_array[i].mem_ptr) {
			goto err_mem_alloc;
		}
	}

	wlan_static_scan_buf0 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf0) {
		goto err_mem_alloc;
	}

	wlan_static_scan_buf1 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf1) {
		goto err_static_scan_buf;
	}

	pr_info("%s: WIFI MEM Allocated\n", __func__);
	return 0;

err_static_scan_buf:
	pr_err("%s: failed to allocate scan_buf0\n", __func__);
	kfree(wlan_static_scan_buf0);
	wlan_static_scan_buf0 = NULL;

err_mem_alloc:
	pr_err("%s: failed to allocate mem_alloc\n", __func__);
	for (; i >= 0 ; i--) {
		kfree(wifi_mem_array[i].mem_ptr);
		wifi_mem_array[i].mem_ptr = NULL;
	}

	i = WLAN_SKB_BUF_NUM;
err_skb_alloc:
	pr_err("%s: failed to allocate skb_alloc\n", __func__);
	for (; i >= 0 ; i--) {
		dev_kfree_skb(wlan_static_skb[i]);
		wlan_static_skb[i] = NULL;
	}

	return -ENOMEM;
}


static int wifi_detect = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

int bcm_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	pr_info("%s\n", __func__);

	if (wifi_status_cb)
		return -EINVAL;

	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;

	return 0;
}

unsigned int bcm_wifi_status(struct device *dev)
{
	pr_info("%s: wifi_detect = %d\n", __func__, wifi_detect);
	return wifi_detect;
}

static int bcm_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	wifi_detect = val;

	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warn("%s: No callback set to notify\n", __func__);
	return 0;
}

static int bcm_wifi_set_power(int on)
{
	pr_info("------------------------------------------------");
	pr_info("------------------------------------------------\n");
	pr_info("%s Enter: power %s\n", __FUNCTION__, on ? "on" : "off");
	
	if (on) {
		 if (gpio_direction_output(WLAN_POWER, 1)) {
				 pr_err("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__);
				 return -EIO;
		 }
		 if (gpio_get_value(WLAN_POWER)) {
				 pr_info("WL_REG_ON on-step-2 : [%d]\n" , gpio_get_value(WLAN_POWER));
		 } else {
				 pr_info("[%s] gpio value is 0. We need reinit.\n", __FUNCTION__);
				 if (gpio_direction_output(WLAN_POWER, 1))
						 pr_err("%s: WL_REG_ON is "
									 "failed to pull up\n", __func__);
		 }
	} else {
		 if (gpio_direction_output(WLAN_POWER, 0)) {
				 pr_info("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__);
				 return -EIO;
		 }
		 if (gpio_get_value(WLAN_POWER)) {
				 pr_info("WL_REG_ON on-step-2 : [%d]\n" , gpio_get_value(WLAN_POWER));
		 }
	}
	return 0;
}

static int bcm_wifi_set_reset(int on)
{
	pr_info("%s: %d\n", __func__, on);
	return 0;
}

static int char2int(unsigned char c)
{
	if (('0' <= c) && (c <= '9')) {
		return c - '0';
	}
	else if (('A' <= c) && (c <= 'F')) {
		return c - 'A' + 10;
	}
	else if (('a' <= c) && (c <= 'f')) {
		return c - 'a' + 10;
	}

	return 0;
}

#define ETHER_ADDR_LEN    6
#define WIFI_MACADDR "/idme/mac_addr"

static int bcm_wifi_get_mac_addr(unsigned char *buf)
{
#ifdef CONFIG_IDME
	unsigned char mac_addr[6];
	int hexIdx,charIdx;

	struct device_node *ap;
	int len;
	const char *idme_mac;

	pr_info("%s\n", __func__);

	if (!buf)
		return -EFAULT;

	/* Get IDME mac address */
        ap = of_find_node_by_path(WIFI_MACADDR);
        if (!ap) {
                  pr_err("%s: Unable to get of node /idme/mac_addr\n", __func__);
                  return -EINVAL;
        }

        idme_mac = of_get_property(ap, "value", &len);
	pr_info("%s: IDME MAC_ADDR str %s\n", __func__, idme_mac);

	if (len != 13) {
                 pr_err("%s: IDME MAC_ADDR Invalid length %d != 13\n", __func__, len);
                 return -EINVAL;
        }

	/* Set mac address */
	/* Convert IDME mac address */
	for (charIdx = 0, hexIdx = 0; charIdx < 12; hexIdx++) {
		mac_addr[hexIdx] = char2int(idme_mac[charIdx++]);
		mac_addr[hexIdx] = mac_addr[hexIdx] << 4;
		mac_addr[hexIdx] = mac_addr[hexIdx] + char2int(idme_mac[charIdx++]);
	}
	memcpy(buf, mac_addr, 6);
	pr_err("%s: IDME MAC ADDR %02X:%02X:%02X:%02X:%02X:%02X\n",
		__func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
#else
	uint rand_mac;

	if (!buf)
		return -EFAULT;

	srandom32((uint)jiffies);
	rand_mac = random32();

	buf[0] = 0x00;
	buf[1] = 0x90;
	buf[2] = 0x4c;
	buf[3] = (unsigned char)rand_mac;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);

	pr_err("%s: Random MAC ADDR %02X:%02X:%02X:%02X:%02X:%02X\n",
		__func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

#endif /* CONFIG_IDME */

	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ	4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

/* Customized Locale table */
static struct cntry_locales_custom wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XV", 17},	/* Universal if Country code is unknown or empty */
	{"IR", "XV", 17},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XV", 17},	/* Universal if Country code is SUDAN */
	{"SY", "XV", 17},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XV", 17},	/* Universal if Country code is GREENLAND */
	{"PS", "XV", 17},	/* Universal if Country code is PALESTINE */
	{"TL", "XV", 17},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XV", 17},	/* Universal if Country code is MARSHALL ISLANDS */
	{"PK", "XV", 17},	/* Universal if Country code is PAKISTAN */
	{"CK", "XV", 17},	/* Universal if Country code is Cook Island (13.4.27)*/
	{"CU", "XV", 17},	/* Universal if Country code is Cuba (13.4.27)*/
	{"FK", "XV", 17},	/* Universal if Country code is Falkland Island (13.4.27)*/
	{"FO", "XV", 17},	/* Universal if Country code is Faroe Island (13.4.27)*/
	{"GI", "XV", 17},	/* Universal if Country code is Gibraltar (13.4.27)*/
	{"IM", "XV", 17},	/* Universal if Country code is Isle of Man (13.4.27)*/
	{"CI", "XV", 17},	/* Universal if Country code is Ivory Coast (13.4.27)*/
	{"JE", "XV", 17},	/* Universal if Country code is Jersey (13.4.27)*/
	{"KP", "XV", 17},	/* Universal if Country code is North Korea (13.4.27)*/
	{"FM", "XV", 17},	/* Universal if Country code is Micronesia (13.4.27)*/
	{"MM", "XV", 17},	/* Universal if Country code is Myanmar (13.4.27)*/
	{"NU", "XV", 17},	/* Universal if Country code is Niue (13.4.27)*/
	{"NF", "XV", 17},	/* Universal if Country code is Norfolk Island (13.4.27)*/
	{"PN", "XV", 17},	/* Universal if Country code is Pitcairn Islands (13.4.27)*/
	{"PM", "XV", 17},	/* Universal if Country code is Saint Pierre and Miquelon (13.4.27)*/
	{"SS", "XV", 17},	/* Universal if Country code is South_Sudan (13.4.27)*/
	{"AL", "AL", 2},
	{"DZ", "DZ", 1},
	{"AS", "AS", 12},	/* changed 2 -> 12*/
	{"AI", "AI", 1},
	{"AG", "AG", 2},
	{"AR", "AR", 21},
	{"AW", "AW", 2},
	{"AU", "AU", 6},
	{"AT", "AT", 4},
	{"AZ", "AZ", 2},
	{"BS", "BS", 2},
	{"BH", "BH", 4},	/* changed 24 -> 4*/
	{"BD", "BD", 2},
	{"BY", "BY", 3},
	{"BE", "BE", 4},
	{"BM", "BM", 12},
	{"BA", "BA", 2},
	{"BR", "BR", 4},
	{"VG", "VG", 2},
	{"BN", "BN", 4},
	{"BG", "BG", 4},
	{"KH", "KH", 2},
	{"CA", "CA", 31},
	{"KY", "KY", 3},
	{"CN", "CN", 24},
	{"CO", "CO", 17},
	{"CR", "CR", 17},
	{"HR", "HR", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ET", "ET", 2},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GF", "GF", 2},
	{"DE", "DE", 7},
	{"GR", "GR", 4},
	{"GD", "GD", 2},
	{"GP", "GP", 2},
	{"GU", "GU", 12},
	{"HK", "HK", 2},
	{"HU", "HU", 4},
	{"IS", "IS", 4},
	{"IN", "IN", 3},
	{"ID", "KR", 25},	/* ID/1 -> KR/24 */
	{"IE", "IE", 5},
	{"IL", "BO", 0},	/* IL/7 -> BO/0 */
	{"IT", "IT", 4},
	{"JP", "JP", 58},
	{"JO", "JO", 3},
	{"KW", "KW", 5},
	{"LA", "LA", 2},
	{"LV", "LV", 4},
	{"LB", "LB", 5},
	{"LS", "LS", 2},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"MO", "MO", 2},
	{"MK", "MK", 2},
	{"MW", "MW", 1},
	{"MY", "MY", 3},
	{"MV", "MV", 3},
	{"MT", "MT", 4},
	{"MQ", "MQ", 2},
	{"MR", "MR", 2},
	{"MU", "MU", 2},
	{"YT", "YT", 2},
	{"MX", "MX", 20},
	{"MD", "MD", 2},
	{"MC", "MC", 1},
	{"ME", "ME", 2},
	{"MA", "MA", 2},
	{"NP", "NP", 3},
	{"NL", "NL", 4},
	{"AN", "AN", 2},
	{"NZ", "NZ", 4},
	{"NO", "NO", 4},
	{"OM", "OM", 4},
	{"PA", "PA", 17},
	{"PG", "PG", 2},
	{"PY", "PY", 2},
	{"PE", "PE", 20},
	{"PH", "PH", 5},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PR", "PR", 20},
	{"RE", "RE", 2},
	{"RO", "RO", 4},
	{"SN", "SN", 2},
	{"RS", "RS", 2},
	{"SG", "SG", 4},
	{"SK", "SK", 4},
	{"SI", "SI", 4},
	{"ES", "ES", 4},
	{"LK", "LK", 1},
	{"SE", "SE", 4},
	{"CH", "CH", 4},
	{"TW", "TW", 1},
	{"TH", "TH", 5},
	{"TT", "TT", 3},
	{"TR", "TR", 7},
	{"AE", "AE", 6},
	{"UG", "UG", 2},
	{"GB", "GB", 6},
	{"UY", "UY", 1},
	{"VI", "VI", 13},
	{"VA", "VA", 12},	/* changed 2 -> 12 */
	{"VE", "VE", 3},
	{"VN", "VN", 4},
	{"MA", "MA", 1},
	{"ZM", "ZM", 2},
	{"EC", "EC", 21},
	{"SV", "SV", 19},
	{"KR", "KR", 57},
	{"RU", "RU", 13},
	{"UA", "UA", 8},
	{"GT", "GT", 1},
	{"MN", "MN", 1},
	{"NI", "NI", 2},
	{"US", "Q2", 57},
};

static void *bcm_wifi_get_country_code(char *ccode)
{
	int size, i;
	static struct cntry_locales_custom country_code;

	size = ARRAY_SIZE(wifi_translate_custom_table);

	if ((size == 0) || (ccode == NULL))
		return NULL;

	pr_info("%s: Country code: %s\n", __func__, ccode);

	for (i = 0; i < size; i++) {
		if (!strcmp(ccode, wifi_translate_custom_table[i].iso_abbrev))
			return &wifi_translate_custom_table[i];
	}

	memset(&country_code, 0, sizeof(struct cntry_locales_custom));
	strlcpy(country_code.custom_locale, ccode, WLC_CNTRY_BUF_SZ);

	return &country_code;
}

static int __init bcm_wifi_init_gpio_mem(struct platform_device *pdev)
{
	pr_info("%s: wifi gpio and mem initialized\n", __func__);
	return 0;
}

static struct wifi_platform_data bcm_wifi_control = {
	.mem_prealloc	= bcm_wifi_mem_prealloc,
	.set_power      = bcm_wifi_set_power,
	.set_reset      = bcm_wifi_set_reset,
	.set_carddetect = bcm_wifi_set_carddetect,
	.get_mac_addr	= bcm_wifi_get_mac_addr,
	.get_country_code = bcm_wifi_get_country_code,
};

static struct resource bcm_wifi_resource[] = {
	[0] = {
		.name = "bcmdhd_wlan_irq",
		.start = 0,  //assigned later
		.end   = 0,  //assigned later
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE, // for HW_OOB
	},
};

static struct platform_device bcm_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(bcm_wifi_resource),
	.resource       = bcm_wifi_resource,
	.dev            = {
		.platform_data = &bcm_wifi_control,
	},
};

static int __init init_bcm_wifi(void)
{
	bcm_wifi_init_gpio_mem(&bcm_wifi_device);
	if (bcm_wifi_mem_init() < 0) {
		pr_err("%s: %s - BCM memory allocation failed\n", __func__, bcm_wifi_device.name);
                return -EFAULT;
	}
	platform_device_register(&bcm_wifi_device);
        return 0;
}

late_initcall(init_bcm_wifi);
