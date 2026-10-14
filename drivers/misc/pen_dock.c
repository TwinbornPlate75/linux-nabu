// SPDX-License-Identifier: GPL-2.0
/*
 * Pen dock detection for the Xiaomi Pad 5 (nabu).
 *
 * The stylus is held on the side of the tablet by magnets and charged there
 * by the wireless charger. Two hall sensors along the park position pull
 * their output low while the pen is attached; either one being low means the
 * pen is docked.
 *
 * Both the touchscreen (which has to keep its pen sensing off while the pen
 * is parked, as a docked pen disturbs the digitizer) and the wireless pen
 * charger (which has to start charging) need that state, so this driver owns
 * the two lines and exports the state instead of letting every consumer
 * request the same GPIOs.
 *
 * The lines are level held: a pen that is already docked when the driver
 * probes produces no edge at all, so the level is sampled once at probe and
 * consumers may query it at any time. Change notifications are delivered
 * from a workqueue, hence callbacks may sleep.
 */

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/pen_dock.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/workqueue.h>

#define PEN_DOCK_MAX_LINES	2

struct pen_dock {
	struct device *dev;
	struct gpio_desc *gpiod[PEN_DOCK_MAX_LINES];
	int irq[PEN_DOCK_MAX_LINES];
	int nlines;
	struct work_struct work;
	bool docked;
	bool wake;
};

static BLOCKING_NOTIFIER_HEAD(pen_dock_chain);
static struct pen_dock *pen_dock;

/*
 * The hall sensors are active low, so the DT describes the lines as
 * GPIO_ACTIVE_LOW and an asserted line (logical 1) means the pen is
 * attached. Being asserted on either sensor is enough.
 */
static bool pen_dock_read(struct pen_dock *pd)
{
	int i;

	for (i = 0; i < pd->nlines; i++) {
		if (gpiod_get_value_cansleep(pd->gpiod[i]))
			return true;
	}

	return false;
}

static void pen_dock_work(struct work_struct *work)
{
	struct pen_dock *pd = container_of(work, struct pen_dock, work);
	bool docked = pen_dock_read(pd);

	if (docked == pd->docked)
		goto out;

	pd->docked = docked;
	dev_info(pd->dev, "pen is %s the dock\n", docked ? "in" : "out of");
	blocking_notifier_call_chain(&pen_dock_chain, docked, NULL);
out:
	if (pd->wake)
		pm_relax(pd->dev);
}

static irqreturn_t pen_dock_irq_handler(int irq, void *data)
{
	struct pen_dock *pd = data;

	if (pd->wake)
		pm_stay_awake(pd->dev);
	schedule_work(&pd->work);

	return IRQ_HANDLED;
}

bool pen_dock_is_docked(void)
{
	if (!pen_dock)
		return false;

	return READ_ONCE(pen_dock->docked);
}
EXPORT_SYMBOL_GPL(pen_dock_is_docked);

int pen_dock_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pen_dock_chain, nb);
}
EXPORT_SYMBOL_GPL(pen_dock_register_notifier);

int pen_dock_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pen_dock_chain, nb);
}
EXPORT_SYMBOL_GPL(pen_dock_unregister_notifier);

static ssize_t docked_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct pen_dock *pd = dev_get_drvdata(dev);

	if (!pd)
		return -ENODEV;

	return sysfs_emit(buf, "%d\n", pd->docked);
}
static DEVICE_ATTR_RO(docked);

static struct attribute *pen_dock_attrs[] = {
	&dev_attr_docked.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pen_dock);

static int pen_dock_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pen_dock *pd;
	int i, ret;

	if (pen_dock)
		return dev_err_probe(dev, -EBUSY,
				     "another pen dock is already registered\n");

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->dev = dev;
	INIT_WORK(&pd->work, pen_dock_work);

	for (i = 0; i < PEN_DOCK_MAX_LINES; i++) {
		struct gpio_desc *gpiod;

		gpiod = devm_gpiod_get_index_optional(dev, "pen-dock", i,
						      GPIOD_IN);
		if (IS_ERR(gpiod))
			return dev_err_probe(dev, PTR_ERR(gpiod),
					     "failed to get pen dock gpio %d\n",
					     i);
		if (!gpiod)
			break;

		pd->gpiod[pd->nlines++] = gpiod;
	}

	if (!pd->nlines)
		return dev_err_probe(dev, -ENODEV, "no pen dock gpio\n");

	/* level held: no edge is coming for a pen that is already docked */
	pd->docked = pen_dock_read(pd);

	pd->wake = device_property_read_bool(dev, "wakeup-source");
	if (pd->wake)
		device_init_wakeup(dev, true);

	for (i = 0; i < pd->nlines; i++) {
		pd->irq[i] = gpiod_to_irq(pd->gpiod[i]);
		if (pd->irq[i] < 0)
			return dev_err_probe(dev, pd->irq[i],
					     "pen dock gpio %d has no irq\n", i);

		ret = devm_request_irq(dev, pd->irq[i], pen_dock_irq_handler,
				       IRQF_TRIGGER_RISING |
					       IRQF_TRIGGER_FALLING,
				       dev_name(dev), pd);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request pen dock irq %d\n",
					     i);

		if (pd->wake)
			enable_irq_wake(pd->irq[i]);
	}

	platform_set_drvdata(pdev, pd);
	pen_dock = pd;

	/*
	 * If the pen was attached or removed between sampling the level above
	 * and arming the irqs, that change produced no edge. Run the work once
	 * so the level sampled then is compared against the current one.
	 */
	schedule_work(&pd->work);

	dev_info(dev, "pen dock detection ready, pen is %s the dock\n",
		 pd->docked ? "in" : "out of");

	return 0;
}

static void pen_dock_remove(struct platform_device *pdev)
{
	struct pen_dock *pd = platform_get_drvdata(pdev);
	int i;

	/* drop the irqs first so that no new work can be queued */
	for (i = 0; i < pd->nlines; i++)
		devm_free_irq(pd->dev, pd->irq[i], pd);

	cancel_work_sync(&pd->work);
	pen_dock = NULL;
}

static const struct of_device_id pen_dock_of_match[] = {
	{ .compatible = "xiaomi,pen-dock" },
	{ }
};
MODULE_DEVICE_TABLE(of, pen_dock_of_match);

static struct platform_driver pen_dock_driver = {
	.probe = pen_dock_probe,
	.remove = pen_dock_remove,
	.driver = {
		.name = "pen-dock",
		.of_match_table = pen_dock_of_match,
		.dev_groups = pen_dock_groups,
	},
};
module_platform_driver(pen_dock_driver);

MODULE_DESCRIPTION("Pen dock (magnetic stylus attachment) detection");
MODULE_LICENSE("GPL");
