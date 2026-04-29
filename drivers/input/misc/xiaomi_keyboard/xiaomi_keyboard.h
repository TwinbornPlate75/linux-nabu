#ifndef __XIAOMI_KEYBOARD_H
#define __XIAOMI_KEYBOARD_H

#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>

#define XIAOMI_KB_TAG "xiaomi-keyboard"
#define MI_KB_INFO(fmt, args...) \
	pr_info("[%s] %s %d: " fmt, XIAOMI_KB_TAG, __func__, __LINE__, ##args)
#define MI_KB_ERR(fmt, args...) \
	pr_err("[%s] %s %d: " fmt, XIAOMI_KB_TAG, __func__, __LINE__, ##args)

struct xiaomi_keyboard_platdata {
	u32 rst_gpio;
	u32 rst_flags;
	u32 in_irq_gpio;
	u32 in_irq_flags;
	u32 vdd_gpio;
};

struct xiaomi_keyboard_data {
	struct pinctrl *pinctrl;
	struct platform_device *pdev;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_suspend;
	struct notifier_block lid_notif;
	struct notifier_block drm_notif;
	struct workqueue_struct *event_wq;
	struct work_struct resume_work;
	struct work_struct suspend_work;
	struct delayed_work lid_work;
	struct xiaomi_keyboard_platdata *pdata;

	int irq;
	bool dev_pm_suspend;
	bool lid_is_closed;
	bool keyboard_is_enable;
	bool is_in_suspend;
	bool keyboard_switch;
};
#endif
