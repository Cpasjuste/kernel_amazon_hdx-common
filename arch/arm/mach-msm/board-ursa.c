/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/memory.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/krait-regulator.h>
#include <linux/regulator/rpm-smd-regulator.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <asm/mach/map.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_smd.h>
#include <mach/restart.h>
#include <mach/rpm-smd.h>
#include <soc/qcom/smem.h>
#include <mach/socinfo.h>
#include "board-dt.h"
#include "clock.h"
#include "spm.h"
#include "pm.h"
#include "platsmp.h"
#include "mach/board-detect.h"

void __init ursa_reserve(void)
{
        of_scan_flat_dt(dt_scan_for_memory_reserve, NULL);
}

static void __init msm8974_early_memory(void)
{
	of_scan_flat_dt(dt_scan_for_memory_hole, NULL);
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init ursa_add_drivers(void)
{
	msm_smd_init();
	msm_rpm_driver_init();
	msm_pm_sleep_status_init();
	rpm_smd_regulator_driver_init();
	msm_spm_device_init();
	krait_power_init();
	tsens_tm_init_driver();
	msm_thermal_device_init();
}

static struct of_dev_auxdata msm_hsic_host_adata[] = {
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),
	{}
};

static struct of_dev_auxdata msm8974_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,hsusb-otg", 0xF9A55000, \
			"msm_otg", NULL),
	OF_DEV_AUXDATA("qcom,ehci-host", 0xF9A55000, \
			"msm_ehci_host", NULL),
	OF_DEV_AUXDATA("qcom,dwc-usb3-msm", 0xF9200000, \
			"msm_dwc3", NULL),
	OF_DEV_AUXDATA("qcom,usb-bam-msm", 0xF9304000, \
			"usb_bam", NULL),
	OF_DEV_AUXDATA("qcom,spi-qup-v2", 0xF9924000, \
			"spi_qsd.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98E4000, \
			"msm_sdcc.4", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98E4900, \
			"msm_sdcc.4", NULL),
	OF_DEV_AUXDATA("qcom,msm-rng", 0xF9BFF000, \
			"msm_rng", NULL),
	OF_DEV_AUXDATA("qcom,qseecom", 0xFE806000, \
			"qseecom", NULL),
	OF_DEV_AUXDATA("qcom,mdss_mdp", 0xFD900000, "mdp.0", NULL),
	OF_DEV_AUXDATA("qcom,msm-tsens", 0xFC4A8000, \
			"msm-tsens", NULL),
	OF_DEV_AUXDATA("qcom,qcedev", 0xFD440000, \
			"qcedev.0", NULL),
	OF_DEV_AUXDATA("qcom,qcrypto", 0xFD440000, \
			"qcrypto.0", NULL),
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, \
			"msm_hsic_host", NULL),
	OF_DEV_AUXDATA("qcom,hsic-smsc-hub", 0, "msm_smsc_hub",
			msm_hsic_host_adata),
	{}
};

static void __init ursa_map_io(void)
{
	msm_map_8974_io();
}

int wait = 1;

void ursa_init_gpiomux(void);

void __init ursa_init(void)
{
	struct of_dev_auxdata *adata = msm8974_auxdata_lookup;

	/*
	 * populate devices from DT first so smem probe will get called as part
	 * of msm_smem_init.  socinfo_init needs smem support so call
	 * msm_smem_init before it.  msm_8974_init_gpiomux needs socinfo so
	 * call socinfo_init before it.
	 */
	board_dt_populate(adata);
	pr_err("%s: board_dt_populate()\n", __func__);

	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	/* Populate system_rev with board revision */
	system_rev = ursa_board_revision();
	pr_info("%s: URSA board revision %d\n", __func__, system_rev);

	ursa_init_gpiomux();
	regulator_has_full_constraints();
	ursa_add_drivers();
}

void __init ursa_init_very_early(void)
{
	msm8974_early_memory();
}

static const char *ursa_dt_match[] __initconst = {
	"lab126,ursa",
	NULL
};

DT_MACHINE_START(MSM8974_DT, "Amazon/Lab126 URSA Board (Flattened Device Tree)")
	.map_io 		= ursa_map_io,
	.init_machine 		= ursa_init,
	.dt_compat 		= ursa_dt_match,
	.reserve 		= ursa_reserve,
	.init_very_early 	= ursa_init_very_early,
	.restart 		= msm_restart,
	.smp 			= &msm8974_smp_ops,
MACHINE_END
