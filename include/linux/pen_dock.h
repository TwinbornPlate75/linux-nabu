/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Pen dock (magnetic stylus attachment) state.
 *
 * The stylus parks on the side of the tablet, held there by magnets, and is
 * charged while it sits there. Two hall sensors next to the park position
 * pull their output low while the pen is attached.
 *
 * That state is a board level fact several drivers need: the touchscreen has
 * to keep its pen sensing off while the pen is parked (a docked pen disturbs
 * the digitizer), and the wireless charger has to start charging it. Instead
 * of having every consumer request the same GPIOs, the pen dock driver owns
 * the lines and hands out the state, either on demand or through the
 * notifier below.
 */

#ifndef _LINUX_PEN_DOCK_H
#define _LINUX_PEN_DOCK_H

#include <linux/errno.h>
#include <linux/notifier.h>
#include <linux/types.h>

/* notifier event, also the argument to pen_dock_register_notifier() users */
enum pen_dock_event {
	PEN_DOCK_UNDOCKED = 0,
	PEN_DOCK_DOCKED = 1,
};

#if IS_ENABLED(CONFIG_PEN_DOCK)

bool pen_dock_is_docked(void);

int pen_dock_register_notifier(struct notifier_block *nb);
int pen_dock_unregister_notifier(struct notifier_block *nb);

#else /* !CONFIG_PEN_DOCK */

/*
 * Without the pen dock driver there is nothing to detect, so report the pen
 * as undocked: consumers then keep the behaviour they have on boards without
 * a pen dock at all.
 */
static inline bool pen_dock_is_docked(void)
{
	return false;
}

static inline int pen_dock_register_notifier(struct notifier_block *nb)
{
	return -ENODEV;
}

static inline int pen_dock_unregister_notifier(struct notifier_block *nb)
{
	return -ENODEV;
}

#endif /* CONFIG_PEN_DOCK */

#endif /* _LINUX_PEN_DOCK_H */
