/*
 * (C) 2001 Clemson University and The University of Chicago
 *
 * Changes by Acxiom Corporation to add proc file handler for pvfs2 client
 * parameters, Copyright Acxiom Corporation, 2005.
 *
 * See COPYING in top-level directory.
 */

#include "protocol.h"
#include "pvfs2-kernel.h"
#include "pvfs2-debugfs.h"
#include "pvfs2-sysfs.h"

/* PVFS2_VERSION is a ./configure define */
#ifndef PVFS2_VERSION
#define PVFS2_VERSION "Unknown"
#endif

/*
 * global variables declared here
 */

/* array of client debug keyword/mask values */
struct client_debug_mask *cdm_array;
int cdm_element_count;

char kernel_debug_string[PVFS2_MAX_DEBUG_STRING_LEN] = "none";
char client_debug_string[PVFS2_MAX_DEBUG_STRING_LEN];
char client_debug_array_string[PVFS2_MAX_DEBUG_STRING_LEN];

char *debug_help_string;
int help_string_initialized;
struct dentry *help_file_dentry;
struct dentry *client_debug_dentry;
struct dentry *debug_dir;
int client_verbose_index;
int client_all_index;
struct pvfs2_stats g_pvfs2_stats;

/* the size of the hash tables for ops in progress */
int hash_table_size = 509;

static ulong module_parm_debug_mask;
__u64 gossip_debug_mask;
struct client_debug_mask client_debug_mask = { NULL, 0, 0 };
unsigned int kernel_mask_set_mod_init; /* implicitly false */
int op_timeout_secs = PVFS2_DEFAULT_OP_TIMEOUT_SECS;
int slot_timeout_secs = PVFS2_DEFAULT_SLOT_TIMEOUT_SECS;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("PVFS2 Development Team");
MODULE_DESCRIPTION("The Linux Kernel VFS interface to PVFS2");
MODULE_PARM_DESC(module_parm_debug_mask, "debugging level (see pvfs2-debug.h for values)");
MODULE_PARM_DESC(op_timeout_secs, "Operation timeout in seconds");
MODULE_PARM_DESC(slot_timeout_secs, "Slot timeout in seconds");
MODULE_PARM_DESC(hash_table_size,
		 "size of hash table for operations in progress");

static struct file_system_type pvfs2_fs_type = {
	.name = "pvfs2",
	.mount = pvfs2_mount,
	.kill_sb = pvfs2_kill_sb,
	.owner = THIS_MODULE,
};

module_param(hash_table_size, int, 0);
module_param(module_parm_debug_mask, ulong, 0644);
module_param(op_timeout_secs, int, 0);
module_param(slot_timeout_secs, int, 0);

/* synchronizes the request device file */
struct mutex devreq_mutex;

/*
  blocks non-priority requests from being queued for servicing.  this
  could be used for protecting the request list data structure, but
  for now it's only being used to stall the op addition to the request
  list
*/
struct mutex request_mutex;

/* hash table for storing operations waiting for matching downcall */
struct list_head *htable_ops_in_progress;
DEFINE_SPINLOCK(htable_ops_in_progress_lock);

/* list for queueing upcall operations */
LIST_HEAD(pvfs2_request_list);

/* used to protect the above pvfs2_request_list */
DEFINE_SPINLOCK(pvfs2_request_list_lock);

/* used for incoming request notification */
DECLARE_WAIT_QUEUE_HEAD(pvfs2_request_list_waitq);

static int __init pvfs2_init(void)
{
	int ret = -1;
	__u32 i = 0;

	/* convert input debug mask to a 64-bit unsigned integer */
	gossip_debug_mask = (unsigned long long) module_parm_debug_mask;

	/*
	 * set the kernel's gossip debug string; invalid mask values will
	 * be ignored.
	 */
	debug_mask_to_string(&gossip_debug_mask, 0);

	/* remove any invalid values from the mask */
	debug_string_to_mask(kernel_debug_string, &gossip_debug_mask, 0);

	/*
	 * if the mask has a non-zero value, then indicate that the mask
	 * was set when the kernel module was loaded.  The pvfs2 dev ioctl
	 * command will look at this boolean to determine if the kernel's
	 * debug mask should be overwritten when the client-core is started.
	 */
	if (gossip_debug_mask != 0)
		kernel_mask_set_mod_init = true;

	/* print information message to the system log */
	pr_info("pvfs2: pvfs2_init called with debug mask: :%s: :%llx:\n",
	       kernel_debug_string,
	       (unsigned long long)gossip_debug_mask);

	ret = bdi_init(&pvfs2_backing_dev_info);

	if (ret)
		return ret;

	if (op_timeout_secs < 0)
		op_timeout_secs = 0;

	if (slot_timeout_secs < 0)
		slot_timeout_secs = 0;

	/* initialize global book keeping data structures */
	ret = op_cache_initialize();
	if (ret < 0)
		goto err;

	ret = dev_req_cache_initialize();
	if (ret < 0)
		goto cleanup_op;

	ret = pvfs2_inode_cache_initialize();
	if (ret < 0)
		goto cleanup_req;

	ret = kiocb_cache_initialize();
	if (ret  < 0)
		goto cleanup_inode;

	/* Initialize the pvfsdev subsystem. */
	ret = pvfs2_dev_init();
	if (ret < 0) {
		gossip_err("pvfs2: could not initialize device subsystem %d!\n",
			   ret);
		goto cleanup_kiocb;
	}

	mutex_init(&devreq_mutex);
	mutex_init(&request_mutex);

	htable_ops_in_progress =
	    kcalloc(hash_table_size, sizeof(struct list_head), GFP_KERNEL);
	if (!htable_ops_in_progress) {
		gossip_err("Failed to initialize op hashtable");
		ret = -ENOMEM;
		goto cleanup_device;
	}

	/* initialize a doubly linked at each hash table index */
	for (i = 0; i < hash_table_size; i++)
		INIT_LIST_HEAD(&htable_ops_in_progress[i]);

	ret = fsid_key_table_initialize();
	if (ret < 0)
		goto cleanup_progress_table;

	/*
	 * Build the contents of /sys/kernel/debug/orangefs/debug-help
	 * from the keywords in the kernel keyword/mask array.
	 *
	 * The keywords in the client keyword/mask array are
	 * unknown at boot time.
	 *
	 * orangefs_prepare_debugfs_help_string will be used again
	 * later to rebuild the debug-help file after the client starts
	 * and passes along the needed info. The argument signifies
	 * which time orangefs_prepare_debugfs_help_string is being
	 * called.
	 *
	 */
	ret = orangefs_prepare_debugfs_help_string(1);
	if (ret)
		goto out;

	pvfs2_debugfs_init();
	pvfs2_kernel_debug_init();
	orangefs_sysfs_init();

	ret = register_filesystem(&pvfs2_fs_type);
	if (ret == 0) {
		pr_info("pvfs2: module version %s loaded\n", PVFS2_VERSION);
		return 0;
	}

	pvfs2_debugfs_cleanup();
	orangefs_sysfs_exit();
	fsid_key_table_finalize();

cleanup_progress_table:
	kfree(htable_ops_in_progress);

cleanup_device:
	pvfs2_dev_cleanup();

cleanup_kiocb:
	kiocb_cache_finalize();

cleanup_inode:
	pvfs2_inode_cache_finalize();

cleanup_req:
	dev_req_cache_finalize();

cleanup_op:
	op_cache_finalize();

err:
	bdi_destroy(&pvfs2_backing_dev_info);

out:
	return ret;
}

static void __exit pvfs2_exit(void)
{
	int i = 0;
	struct pvfs2_kernel_op_s *cur_op = NULL;

	gossip_debug(GOSSIP_INIT_DEBUG, "pvfs2: pvfs2_exit called\n");

	unregister_filesystem(&pvfs2_fs_type);
	pvfs2_debugfs_cleanup();
	orangefs_sysfs_exit();
	fsid_key_table_finalize();
	pvfs2_dev_cleanup();
	/* clear out all pending upcall op requests */
	spin_lock(&pvfs2_request_list_lock);
	while (!list_empty(&pvfs2_request_list)) {
		cur_op = list_entry(pvfs2_request_list.next,
				    struct pvfs2_kernel_op_s,
				    list);
		list_del(&cur_op->list);
		gossip_debug(GOSSIP_INIT_DEBUG,
			     "Freeing unhandled upcall request type %d\n",
			     cur_op->upcall.type);
		op_release(cur_op);
	}
	spin_unlock(&pvfs2_request_list_lock);

	for (i = 0; i < hash_table_size; i++)
		while (!list_empty(&htable_ops_in_progress[i])) {
			cur_op = list_entry(htable_ops_in_progress[i].next,
					    struct pvfs2_kernel_op_s,
					    list);
			op_release(cur_op);
		}

	kiocb_cache_finalize();
	pvfs2_inode_cache_finalize();
	dev_req_cache_finalize();
	op_cache_finalize();

	kfree(htable_ops_in_progress);

	bdi_destroy(&pvfs2_backing_dev_info);

	pr_info("pvfs2: module version %s unloaded\n", PVFS2_VERSION);
}

/*
 * What we do in this function is to walk the list of operations
 * that are in progress in the hash table and mark them as purged as well.
 */
void purge_inprogress_ops(void)
{
	int i;

	for (i = 0; i < hash_table_size; i++) {
		struct pvfs2_kernel_op_s *op;
		struct pvfs2_kernel_op_s *next;

		list_for_each_entry_safe(op,
					 next,
					 &htable_ops_in_progress[i],
					 list) {
			spin_lock(&op->lock);
			gossip_debug(GOSSIP_INIT_DEBUG,
				"pvfs2-client-core: purging in-progress op tag "
				"%llu %s\n",
				llu(op->tag),
				get_opname_string(op));
			set_op_state_purged(op);
			spin_unlock(&op->lock);
			wake_up_interruptible(&op->waitq);
		}
	}
}

module_init(pvfs2_init);
module_exit(pvfs2_exit);
