/*
 *  pmtest.c - Driver power management tester
 *
 *  Copyright (C) 2000 Andrew Henroid
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/pm.h>
#include <asm/uaccess.h>

#define PMTEST_MAX_ENTRY 32

static struct proc_dir_entry *pmtest_proc = NULL;

/*
 * Handle read from /proc/drivers/pmtest/devices
 */
static int pmtest_devices(char *page,
			  char **start,
			  off_t offset,
			  int count,
			  int *eof,
			  void *data)
{
	struct pm_dev *pmdev = NULL;
	char *i = page;

	while ((i - page) < (PAGE_SIZE - PMTEST_MAX_ENTRY))
	{
		pmdev = pm_find(PM_UNKNOWN_DEV, pmdev);
		if (!pmdev)
			break;
		i += sprintf(i, "%d 0x%lx %d\n",
			     pmdev->type,
			     pmdev->id,
			     pmdev->state);
	}
	*start = NULL;
	*eof = 1;
	return (i - page);
}

/*
 * Send suspend/resume request to matching devices
 */
static int pmtest_send(int type,
		       unsigned long id,
		       pm_request_t rqst,
		       unsigned long state)
{
	struct pm_dev *pmdev = NULL;
	for (;;)
	{
		pmdev = pm_find(type, pmdev);
		if (!pmdev)
			break;
		if (!id || pmdev->id == id)
		{
			printk(KERN_INFO "pmtest: %s %d 0x%lx\n",
			       state ? "suspending":"resuming",
			       pmdev->type,
			       pmdev->id);
			pm_send(pmdev, rqst, (void*) state);
		}
	}
	return 0;
}

/*
 * Handle write to /proc/drivers/pmtest/control
 */
static int pmtest_control(struct file *file,
			  const char *buffer,
			  unsigned long count,
			  void *data)
{
	char info[PMTEST_MAX_ENTRY], *i;
	int size, type;
	unsigned long id, state;

	size = count;
	if (size >= sizeof(info))
		size = sizeof(info) - 1;
	copy_from_user(info, buffer, size);
	info[size] = '\0';

	i = info + strspn(info, " \t\n");
	type = (int) simple_strtoul(i, &i, 0);
	i += strspn(i, " \t\n");
	id = simple_strtoul(i, &i, 0);
	i += strspn(i, " \t\n");
	state = simple_strtoul(i, &i, 0);

	if(state < 4)
		pmtest_send(type, id, state ? PM_SUSPEND:PM_RESUME, state);

	return count;
}

/*
 * Setup /proc/drivers/pmtest entries
 */
static int pmtest_init(void)
{
	struct proc_dir_entry *entry;

	pmtest_proc = proc_mkdir("pmtest", proc_root_driver);
	if(pmtest_proc)
	{
		entry = create_proc_entry("devices", 0, pmtest_proc);
		if (entry)
			entry->read_proc = pmtest_devices;
		entry = create_proc_entry("control", 0, pmtest_proc);
		if (entry)
			entry->write_proc = pmtest_control;
	}
	return 0;
}

/*
 * Destroy /proc/drivers/pmtest entries
 */
static void pmtest_exit(void)
{
	remove_proc_entry("control", pmtest_proc);
	remove_proc_entry("devices", pmtest_proc);
	remove_proc_entry("pmtest", proc_root_driver);
}

module_init(pmtest_init);
module_exit(pmtest_exit);
