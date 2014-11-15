/*++
 * WonderMedia Memory Block driver
 *
 * Copyright c 2010  WonderMedia  Technologies, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * WonderMedia Technologies, Inc.
 * 10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C
--*/
/*----------------------------------------------------------------------------
 * MODULE       : driver/video/wmt/memblock.c
 * DATE         : 2008/12/08
 * DESCRIPTION  : Physical memory management driver
 * HISTORY      :
 * Version 1.0.0.2 , 2013/11/12
*----------------------------------------------------------------------------*/
#ifndef MEMBLOCK_C
#define MEMBLOCK_C
#endif

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/mman.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <mach/wmt_env.h>

#include "com-mb.h"
#include <linux/wmt-mb.h>

#define THE_MB_USER "WMT-MB"
#define MB_VERSION_MAJOR 1
#define MB_VERSION_MINOR 0
#define MB_VERSION_MICRO 0
#define MB_VERSION_BUILD 6

#define MB_DEBUG
#define MB_INFO(fmt, args...)  \
	printk(KERN_INFO "["THE_MB_USER"] " fmt , ## args)
#define MB_WARN(fmt, args...)  \
	printk(KERN_WARNING "["THE_MB_USER" *W*] " fmt, ## args)
#define MB_ERROR(fmt, args...) \
	printk(KERN_ERR "["THE_MB_USER" *E*] " fmt , ## args)

#ifdef MB_DEBUG
#define MB_OPDBG(fmt, args...) { /* FILE OPERATION DBG */ \
	if (MBMSG_LEVEL) { \
		printk("\t" KERN_DEBUG "["THE_MB_USER"] %s: " fmt, \
			__func__ , ## args); \
	} \
}
#define MB_DBG(fmt, args...) { \
	if (MBMSG_LEVEL > 1) { \
		printk("\t" KERN_DEBUG "["THE_MB_USER"] %s: " fmt, \
			__func__ , ## args); \
	} \
}
#define MB_WDBG(fmt, args...) { \
	if (MBMSG_LEVEL > 2) { \
		printk("\t" KERN_DEBUG "["THE_MB_USER"] %s: " fmt, \
			__func__ , ## args); \
	} \
}
#else
#define MB_OPDBG(fmt, args...) do {} while (0)
#define MB_DBG(fmt, args...) do {} while (0)
#define MB_WDBG(fmt, args...) do {} while (0)
#endif

#ifdef CONFIG_WMT_MB_RESERVE_FROM_IO
#include <linux/fb.h>
#define MBIO_FSCREENINFO   FBIOGET_FSCREENINFO
#define MBA_MAX_ORDER      20 /* 2^20 pages = 4 GB */
#else
#define MBA_MAX_ORDER      (MAX_ORDER - 1) /* 2^10 pages = 4 MB */
#endif
#define MBA_MIN_ORDER      (MAX_ORDER - 5) /* 2^6  pages = 256 KB */

#define MBAFLAG_STATIC     0x80000000 /* static, without release if empty */

#define MBFLAG_CACHED      0x00000001 /* MB mapping as cachable */

#define MBFIND_VIRT        0x00000000 /* search virt address */
#define MBFIND_PHYS        0x00000001 /* search phys address */

#define MBUFIND_USER       0x00000001 /* search user address */
#define MBUFIND_VIRT       0x00000002 /* search kernel virt address */
#define MBUFIND_PHYS       0x00000004 /* search kernel phys address */
#define MBUFIND_ALL        0x00000010 /* search all mbu */
#define MBUFIND_CREATOR    0x00000020 /* search creator */
#define MBUFIND_MMAP       0x00000040 /* search mbu which mapping from user */
#define MBUFIND_GETPUT     0x00000080 /* search mbu which came from get/put */
#define MBUFIND_ADDRMASK   (MBUFIND_USER|MBUFIND_VIRT|MBUFIND_PHYS)

#define MBPROC_BUFSIZE     (16*1024)
#define MBA_SHOW_BUFSIZE   (8*1024)
#define MB_SHOW_BUFSIZE    (4*1024)

#define MB_COUNT(mb)       atomic_read(&(mb)->count)
#define MB_IN_USE(mb)      (MB_COUNT(mb)) /* count != 0 */
#define PAGE_KB(a)         ((a)*(PAGE_SIZE/1024))
#define MB_COMBIND_MBA(h, t) { \
	(h)->pgi.pfn_end = (t)->pgi.pfn_end; \
	(h)->pages += (t)->pages; \
	(h)->tot_free_pages += (t)->tot_free_pages; \
	(h)->max_available_pages += (t)->max_available_pages; \
	list_del(&((t)->mba_list)); \
	wmt_mbah->nr_mba--; \
	kmem_cache_free(wmt_mbah->mba_cachep, t); \
}

#define list_loop list_for_each_entry
struct mb_user;

struct page_info {
	unsigned long          pfn_start; /* start pfn of this block */
	unsigned long          pfn_end; /* end pfn of this block */
};

struct mba_host_struct {
	/* MB area link list link all MBAs */
	struct list_head       mba_list;

	/* page information of this MBA */
	unsigned int           nr_mba;

	/* total pages of all manager MBAs */
	unsigned long          tot_pages;

	/* total static pages of all manager MBAs */
	unsigned long          tot_static_pages;

	/* total free pages of all manager MBAs */
	unsigned long          tot_free_pages;

	/* max free pages of all manager MBAs */
	unsigned long          max_available_pages;

	/* allocator of MBA,
	   use slab to prevent memory from fragment. */
	struct kmem_cache      *mba_cachep;

	/* allocator of MB,
	   use slab to prevent memory from fragment. */
	struct kmem_cache      *mb_cachep;

	/* allocator of mb_user,
	   use slab to prevent memory from fragment. */
	struct kmem_cache      *mbu_cachep;

	/* allocator of mb_task_info,
	   use slab to prevent memory from fragment. */
	struct kmem_cache      *mbti_cachep;
};

struct mb_area_struct {
	/* MB area link list link all MBAs */
	struct list_head       mba_list;

	/* link list link to all MBs
	   belong to this MBA */
	struct list_head       mb_list;

	/* pointer point to dedicated MBAH */
	struct mba_host_struct *mbah;

	/* flags of MBA */
	unsigned int           flags;

	/* prefech record task id */
	pid_t                  tgid;

	/* start physical address of this MBA */
	unsigned long          phys;

	/* start virtual address of this MBA */
	void                   *virt;

	/* size of this MB area. Normally,
	   MAX kernel permitted size */
	unsigned long          pages;

	/* page information of this MBA */
	struct page_info       pgi;

	/* page information of this MBA */
	unsigned int           nr_mb;

	/* cur total free pages of this MBA */
	unsigned long          tot_free_pages;

	/* cur max free pages of this MBA */
	unsigned long          max_available_pages;
};

struct mb_struct {
	/* MB link list link all MBs
	   in dedicated MBA */
	struct list_head       mb_list;

	/* pointer point to dedicated MBA */
	struct mb_area_struct  *mba;

	/* MB kernel page information */
	struct page_info       pgi;

	/* start physical address of this MB */
	unsigned long          phys;

	/* start virtual address of this MB */
	void                   *virt;

	/* allocate size */
	unsigned long          size;

	/* current MB use count,
	   release until zero */
	atomic_t               count;

	/* flags of MB */
	unsigned int           flags;

	/* point to owner created the mb.
	   this enlisted in mbu_list */
	struct mb_user         *creator;

	/* use for trace the user of mb */
	struct list_head       mbu_list;
};

struct mb_user {
	/* user link list link all users
	   (include creator) of belonged MB */
	struct list_head       mbu_list;

	/* the mb to which this user belong */
	struct mb_struct       *mb;

	/* task id to which user belonged,
	   user space user only */
	pid_t                  tgid;

	/* mb_mmap and MBIO_GET   : user address
	   mb_get and mb->creator : physical address */
	unsigned long          addr;

	/* kernel space user:  mb size
	 * user space user:    mmap size
	 * zero:               owner - user space allocate but not mapped yet
	 *                     not owner -come from get/put */
	unsigned long          size;

	/* user name for recoder */
	char                   the_user[TASK_COMM_LEN+1];
};

struct mb_task_info {
	/* task link list link all tasks which use MBDev */
	struct list_head       mbti_list;
	pid_t                  tgid; /* task pid */
	atomic_t               count; /* multi open record */
	char                   task_name[TASK_COMM_LEN+1];
	struct task_struct     *task;
};

#ifdef CONFIG_WMT_MB_SIZE
static int MB_TOTAL_SIZE = CONFIG_WMT_MB_SIZE * 1024;
#else
static int MB_TOTAL_SIZE = 32 * 1024;
#endif
static struct mba_host_struct *wmt_mbah;
static struct mb_task_info wmt_mbti;
const struct file_operations mb_fops;
static unsigned char MBMSG_LEVEL;
static unsigned char MBMAX_ORDER;
static unsigned char MBMIN_ORDER;
static unsigned char USR2PRDT_METHOD;

/* read/write spinlock for multientry protection. */
static spinlock_t mb_do_lock;
static spinlock_t mb_search_lock;
static spinlock_t mb_ioctl_lock;
static spinlock_t mb_task_mm_lock;
static spinlock_t mb_task_lock;
static char show_mb_buffer[MB_SHOW_BUFSIZE];
static char show_mba_buffer[MBA_SHOW_BUFSIZE];

static inline uint64_t checkSum(unsigned long phys, int size, int type)
{
	char *d = (type) ? __va(phys) : mb_phys_to_virt(phys);
	int i, pageSize, len, offset;
	uint64_t sum, cs = 0;

	len = size;
	offset = (unsigned long)d % PAGE_SIZE;
	while (len > 0) {
		sum = 0;
		pageSize = min((int)(PAGE_SIZE - offset), len);
		for (i = 0; i < pageSize; i++)
			sum += d[i];
		MB_WDBG("[CS] addr %#lx ~ %#lx size %#x cs %#llx\n",
			phys, phys + size, size, sum);
		MB_WDBG("[CS:DATA] %02x %02x %02x %02x %02x %02x %02x %02x\n",
			d[0] & 0xff, d[1] & 0xff, d[2] & 0xff, d[3] & 0xff,
			d[4] & 0xff, d[5] & 0xff, d[6] & 0xff, d[7] & 0xff);
		d += pageSize;
		len -= pageSize;
		cs += sum;
	}

	return cs;
}

unsigned long user_to_phys(unsigned long user)
{
	unsigned long par, phys, offset;
	offset = user % PAGE_SIZE;

	asm volatile(
		"mcr p15, 0, %[user], c7, c8, 0\n"
		"isb\n"
		"mrc p15, 0,  %[par], c7, c4, 0\n"
		: [par] "=r" (par)
		: [user] "r" (user));

	phys = (par & PAGE_MASK) + offset;
	MB_DBG("%s: user %lx par %lx phys %lx\n",
		__func__, user, par, phys);

	return (par & 0x1) ? 0 : phys;
}
EXPORT_SYMBOL(user_to_phys);

/* return address is guaranteeed only under page alignment */
void *user_to_virt(unsigned long user)
{
	unsigned long phys;

	phys = user_to_phys(user);
	if (!phys) {
		MB_WARN("user 0x%lx to virt fail.\n", user);
		return 0;
	}

	return __va(phys);
}
EXPORT_SYMBOL(user_to_virt);

struct prdt_info {
	int index;
	int items;
	int offset;
	int size;
	struct prdt_struct *prdt;
	struct prdt_struct *last;
};

static int prdt_proc(struct prdt_info *info, unsigned long phys)
{
	int len;

	if (!info) {
		MB_WARN("PRDT process fail. (NULL info)\n");
		return -1;
	}

	/* PRDT has done or no PRDT space */
	if (info->size <= 0 ||
		(info->last && info->last->EDT))
		return 0;

	len = PAGE_SIZE - info->offset;
	if (len > info->size) /* last page */
		len = info->size;

	/* Check it could combind with previous one */
	if (info->last &&
		/* prd size boundary check, MAX 60K */
		(info->last->size <= ((1 << 16) - (2 * PAGE_SIZE))) &&
		/* page continuity check */
		((info->last->addr + info->last->size) == phys))
		info->last->size += len;
	else { /* create new one */
		if (!info->items) {
			MB_WARN("PRD table full (ptr %p # %d left %d).\n",
				info->last+1, info->index+1, info->size);
			if (info->last)
				info->last->EDT = 1;
			return -1;
		}
		if (info->last == NULL)
			info->last = info->prdt;
		else
			info->last++;
		info->last->addr = phys + info->offset;
		info->last->size = len;
		info->index++;
		info->items--;
		info->offset = 0;
	}
	info->size -= len;
	info->last->reserve = 0;
	info->last->EDT = (info->size) ? 0 : 1;
	MB_WDBG("\t[PRD %3d] %p (0x%x ~ 0x%x, 0x%x, %x) phys 0x%lx offset 0x%x len 0x%x left %x\n",
		info->index, info->last, info->last->addr, info->last->addr + info->last->size,
		info->last->size, info->last->EDT, phys, info->offset, len, info->size);

	return 0;
}

static inline void show_prdt(struct prdt_struct *next)
{
	int idx = 1;

	while (!next->EDT) {
		MB_INFO("PRDT %d-th item: addr %x size %d EDT %d\n",
			idx, next->addr, next->size, next->EDT);
		idx++;
		next++;
	}
	MB_INFO("PRDT last(%d-th) item: addr %x size %d EDT %d\n",
		idx, next->addr, next->size, next->EDT);
}

int user_to_prdt(
	unsigned long user,
	unsigned int size,
	struct prdt_struct *prdt,
	unsigned int items)
{
	struct prdt_info info = {0};
	unsigned long addr;
	int idx;

	if (!prdt || ((size / PAGE_SIZE) + 2) > items) {
		MB_WARN("PRD table space not enough (ptr %p at least %lu but %d).\n",
			prdt, (size/PAGE_SIZE)+2, items);
		return -EINVAL;
	}

	MB_DBG("Memory(%#lx,%#x) PRDT(%p,%d)\n", user, size, prdt, items);
	info.items = items;
	info.offset = user % PAGE_SIZE;
	info.size = size;
	info.prdt = prdt;
	addr = user & PAGE_MASK;
	while (info.size > 0) {
		unsigned long phys = user_to_phys(addr);
		if (!phys) {
			MB_WARN("user to prdt fail: unknown addr %lx (%lx, %x)\n",
				addr, user, size);
			return -EINVAL;
		}
		prdt_proc(&info, phys);
		addr += PAGE_SIZE;
	}

	/* flush cache */
	MB_WDBG("flush PRDT %p, from %lx size %d\n", prdt, user, size);
	dmac_flush_range((const void *)user, (const void *)(user + size));
	for (idx = 0;; idx++) {
		MB_WDBG("PRDT[%d] %p addr %x( ~ %x) size %d EDT %d (%p ~ %p)\n",
			idx, &prdt[idx], prdt[idx].addr, prdt[idx].addr + prdt[idx].size, prdt[idx].size,
			prdt[idx].EDT, __va(prdt[idx].addr), __va(prdt[idx].addr + prdt[idx].size));
		outer_flush_range(prdt[idx].addr,
			prdt[idx].addr + prdt[idx].size);
		if (prdt[idx].EDT)
			break;
	}

	/* show_prdt(prdt); */
	return 0;
}
EXPORT_SYMBOL(user_to_prdt);

int mb_to_prdt(
	unsigned long phys,
	unsigned int size,
	struct prdt_struct *prdt,
	unsigned int items)
{
	struct prdt_info info = {0};
	unsigned long addr;
	void *virt = NULL;
	int idx;

	if (!prdt || ((size / PAGE_SIZE) + 2) > items) {
		MB_WARN("PRD table space not enough (ptr %p at least %lu but %d).\n",
			prdt, (size/PAGE_SIZE)+2, items);
		return -EINVAL;
	}

	MB_DBG("Memory(%#lx,%#x) PRDT(%p,%d)\n", phys, size, prdt, items);
	info.items = items;
	info.offset = phys % PAGE_SIZE;
	info.size = size;
	info.prdt = prdt;
	addr = phys & PAGE_MASK;
	while (info.size > 0) {
		prdt_proc(&info, addr);
		addr += PAGE_SIZE;
	}

	/* flush cache */
	MB_WDBG("PRDT %p, from %lx size %d\n", prdt, phys, size);
    virt = mb_phys_to_virt(phys);
	dmac_flush_range(virt, virt + size);
	for (idx = 0;; idx++) {
		MB_WDBG("PRDT[%d] addr %x size %d EDT %d\n",
			idx, prdt[idx].addr, prdt[idx].size, prdt[idx].EDT);
		outer_flush_range(prdt[idx].addr,
			prdt[idx].addr + prdt[idx].size);
		if (prdt[idx].EDT)
			break;
	}

	/* show_prdt(prdt); */
	return 0;
}
EXPORT_SYMBOL(mb_to_prdt);

unsigned int wmt_mmu_table_size(unsigned int size)
{
	unsigned int nPte, nPde, need;

	if (!size)
		return 0;

	nPte = (size / PAGE_SIZE) + 2; /* maybe cross page up and down boundary */
	nPde = PAGE_ALIGN(nPte * 4) / PAGE_SIZE;
	need = PAGE_ALIGN(nPde * 4);
	need += nPte * 4;

	MB_DBG("PDE %d PTE %d RequestSize %d\n", nPde, nPte, need);
	return need;
}
EXPORT_SYMBOL(wmt_mmu_table_size);

int wmt_mmu_table_check(
	unsigned int mmuPhys,
	unsigned int mmuSize,
	unsigned int size)
{
	int request;

	if (!size) {
		MB_WARN("mmu table failure. NULL size\n");
		return 0;
	}

	if (!mmuSize) {
		MB_WARN("mmu table failure. NULL mmu size\n");
		return 0;
	}

	if (mmuPhys % PAGE_SIZE) {
		MB_WARN("mmu table failure. PDE Table not align\n");
		return 0;
	}

	request = wmt_mmu_table_size(size);
	if (mmuSize < request) {
		MB_WARN("mmu table failure. size %d < %d (request)\n",
			mmuSize, request);
		return 0;
	}

	return 1;
}
EXPORT_SYMBOL(wmt_mmu_table_check);

void wmt_mmu_table_dump(struct mmu_table_info *info)
{
	unsigned int size, iPde, iPte, addrOffset;
	unsigned int *mmuVirt, *pte = NULL, *pde = NULL;

	if (info == 0) {
		MB_ERROR("[WMT_MMU_TABLE] Null input pointer\n");
		return;
	}

	mmuVirt = (unsigned int *)mb_phys_to_virt(info->addr);

	size = info->size;
	iPte = (info->offset >> 12) % 1024;
	iPde = (info->offset >> 22) % 1024;
	addrOffset = info->offset % PAGE_SIZE;

	MB_INFO("MMU (%x): offset pde %x pte %x addr %x\n",
		info->offset, iPde, iPte, addrOffset);

	pde = mmuVirt;
	pte = mb_phys_to_virt(pde[iPde]);
	while (size > 0) {
		int pageSize = min((unsigned int)(PAGE_SIZE - addrOffset), size);
		MB_INFO("PDE(%p/%#x @ %d) -> PTE(%p/%#x @ %d) -> %#x, s %#x # %#x\n",
			pde + iPde, info->addr + iPde * sizeof(pde), iPde,
			pte + iPte, pde[iPde] + iPte * sizeof(pte), iPte,
			pte[iPte] + addrOffset, pageSize, size);
		iPte++;
		addrOffset = 0;
		size -= pageSize;
		if (!(iPte % 1024)) {
			iPde++;
			iPte = 0;
			pte = mb_phys_to_virt(pde[iPde]);
		}
	}
}
EXPORT_SYMBOL(wmt_mmu_table_dump);

/* parameter:
   mmuPhys  [IN] mmu table phys address
   mmuSize  [IN] mmu table size
   addr     [IN] physical address of convert data
   size     [IN] convert size */
unsigned int wmt_mmu_table_from_phys(
	unsigned int mmuPhys,
	unsigned int mmuSize,
	unsigned int addr,
	unsigned int size)
{
	unsigned int iPde, iPte, nPte, nPde, virBufAddr;
	unsigned int *pte, *pde, *mmuVirt;
	int64_t len = size;
	void *virt = NULL;

	iPde = iPte = nPte = nPde = 0;
	if (!wmt_mmu_table_check(mmuPhys, mmuSize, size)) {
		MB_WARN("phys %x (size %d) to mmu fail\n", addr, size);
		return 0xFFFFFFFF;
	}

	virt = mb_phys_to_virt(addr);
	virBufAddr = addr % PAGE_SIZE;
	nPte = size / PAGE_SIZE + 2;
	nPde = PAGE_ALIGN(nPte * 4) / PAGE_SIZE;
	pde = mmuVirt = mb_phys_to_virt(mmuPhys);
	pte = pde + nPde * 1024;
	*pde = mmuPhys + nPde * PAGE_SIZE;
	MB_DBG("[%s] pde %p/%x # %d pte %p/%x # %d\n",
		__func__, pde, mmuPhys, nPde, pte, *pde, nPte);

	while (len > 0) {
		pte[iPte] = addr & PAGE_MASK;
		iPte++;
		if (!(iPte % 1024)) {
			pde[iPte + 1] = pde[iPte] + PAGE_SIZE;
			iPte++;
		}
		addr += PAGE_SIZE;
		len -= PAGE_SIZE;
	}

	dmac_flush_range(virt, virt + size);
	outer_flush_range(addr, addr + size);

	return virBufAddr;
}
EXPORT_SYMBOL(wmt_mmu_table_from_phys);

struct mmu_info {
	unsigned int size;
	unsigned int iPte;
	unsigned int *pte;
	unsigned int *pde;
};

static inline int wmt_mmu_table_proc(struct mmu_info *info, unsigned long phys)
{
	int pageSize, offset;
	if (!info) {
		MB_WARN("WMT MMU process fail. (NULL info)\n");
		return -1;
	}

	if (info->size <= 0) {
		MB_WARN("WMT MMU process fail. (size %d)\n", info->size);
		return -1;
	}

	offset = phys % PAGE_SIZE;
	pageSize = min((unsigned int)(PAGE_SIZE - offset), info->size);
	*info->pte = phys & PAGE_MASK;
	info->size -= pageSize;
	MB_WDBG("\t[PTE] PDE[%d](%p)=%x PTE[%d](%p)=%x size %#x left %#x\n",
		info->iPte/1024, info->pde, *info->pde, info->iPte,
		info->pte, *info->pte, pageSize, info->size);

	info->pte++;
	info->iPte++;
	if (!(info->iPte % 1024)) {
		*info->pde = mb_virt_to_phys(info->pte);
		info->pde++;
	}

	return 0;
}

/* parameter:
   mmuPhys  [IN] mmu table phys address
   mmuSize  [IN] mmu table size
   addr     [IN] physical address of convert data
   size     [IN] convert size */
unsigned int wmt_mmu_table_from_user(
	unsigned int mmuPhys,
	unsigned int mmuSize,
	unsigned int user,
	unsigned int size)
{
#ifndef __KERNEL__
	return wmt_mmu_table_from_phys(mmuPhys, mmuSize, user, size);
#else
	struct mmu_info info = {0};
	unsigned int nPte, nPde;
	unsigned long addr = user;
	int offset = user % PAGE_SIZE;

	if (!wmt_mmu_table_check(mmuPhys, mmuSize, size)) {
		printk(KERN_WARNING "user %x (size %d) to mmu fail\n",
			user, size);
		return 0xFFFFFFFF;
	}

	MB_DBG("[%s] Memory(%#x,%#x) MMU(%#x,%d)\n",
		__func__, user, size, mmuPhys, mmuSize);
	nPte = size / PAGE_SIZE + 2;
	nPde = PAGE_ALIGN(nPte * 4) / PAGE_SIZE;
	info.size = size;
	info.pde = mb_phys_to_virt(mmuPhys);
	info.pte = info.pde + nPde * 1024;
	*info.pde = mmuPhys + nPde * PAGE_SIZE;
	dmac_flush_range((const void *)user, (const void *)(user + size));
	while (info.size > 0) {
		unsigned long phys = user_to_phys(addr);
		int pageSize = min((unsigned int)(PAGE_SIZE - offset), info.size);
		if (!phys) {
			MB_WARN("user to mmu fail: unknown addr %lx (%x # %x)\n",
				addr, user, size);
			return -EINVAL;
		}
		outer_flush_range(phys, phys + pageSize);
		wmt_mmu_table_proc(&info, phys);
		addr += pageSize;
		offset = 0;
	}

	return user % PAGE_SIZE;
#endif
}
EXPORT_SYMBOL(wmt_mmu_table_from_user);

/* parameter:
   addr      [IN] start address (user or phys depends on addr_type)
   size      [IN] size of memory
   addrType  [IN] address type (0: phys 1: user)
   info      [out] pointer of mmu table info */
unsigned int wmt_mmu_table_create(
	unsigned int addr,
	unsigned int size,
	unsigned int addrType,
	struct mmu_table_info *info)
{
	int i, nPde, nPte, *pde, mmuSize;
	unsigned int *mmuAddr = NULL;

	if (!addr || !size || !info || addrType > 1) {
		MB_ERROR("[WMT_MMU_TABLE] invalid args:\n");
		MB_ERROR("\t addr %x # %x type %x info %p\n",
			addr, size, addrType, info);
		return -EINVAL;
	}

	mmuSize = wmt_mmu_table_size(size);
	info->addr = mb_alloc(mmuSize);
	info->size = size;
	MB_DBG("[%s] mmu table (%#x # %#x) addr %#x size %#x type %d\n",
		__func__, info->addr, mmuSize, addr, size, addrType);
	if (!info->addr) {
		MB_ERROR("[%s] fail. Out of MB (%d)\n", __func__, mmuSize);
		return -ENOMEM;
	}

	pde = mmuAddr = (unsigned int *)mb_phys_to_virt((unsigned long)info->addr);
	memset(mmuAddr, 0x0, mmuSize);
	nPte = (size / PAGE_SIZE) + 2; /* maybe cross page up and down boundary */
	nPde = PAGE_ALIGN(nPte * 4) / PAGE_SIZE;
	for (i = 1; i <= nPde; i++, pde++)
		*pde = info->addr + i * PAGE_SIZE;
	info->offset = (addrType) ?
		wmt_mmu_table_from_user(info->addr, mmuSize, addr, size) :
		wmt_mmu_table_from_phys(info->addr, mmuSize, addr, size);

	if (info->offset == 0xFFFFFFFF) {
		MB_ERROR("[WMT_MMU_TABLE] create fail:");
		MB_ERROR("\ttype %x addr %x # %x mmuAddr %p/%x # %d\n",
			addrType, addr, size, mmuAddr, info->addr, info->size);
		mb_free(info->addr);
		return -EFAULT;
	}

	MB_DBG("[%s] success! (addr %#x size %#x offset %#x)\n",
		__func__, info->addr, info->size, info->offset);
	return 0;
}
EXPORT_SYMBOL(wmt_mmu_table_create);

unsigned int wmt_mmu_table_destroy(struct mmu_table_info *info)
{
	if (!info) {
		MB_ERROR("[WMT_MMU_TABLE] destroy fail. NULL mmu table info");
		return -EINVAL;
	}
	MB_DBG("[%s] success! addr %#x\n", __func__, info->addr);
	mb_free(info->addr);
	return 0;
}
EXPORT_SYMBOL(wmt_mmu_table_destroy);

static int mb_show_mb(
	struct mb_struct *mb,
	char *msg,
	char *str,
	int size
)
{
	struct mb_user *mbu;
	char *p;

	if (!str && MBMSG_LEVEL <= 1)
		return 0;

	if (!mb)
		return 0;

	memset(show_mb_buffer, 0x0, MB_SHOW_BUFSIZE);
	p = show_mb_buffer;
	p += sprintf(p, "%s%s[MB,%p] %p %08lx %4ldKB [%4lx,%4lx] %3x\n",
		msg ? msg : "", msg ? " " : "", mb, mb->virt, mb->phys,
		mb->size/1024, mb->pgi.pfn_start, mb->pgi.pfn_end,
		MB_COUNT(mb));

	if (!list_empty(&mb->mbu_list)) {
		list_loop(mbu, &mb->mbu_list, mbu_list) {
			char tmp[256], *p1 = tmp;
			int expectLen = (int)(p - show_mb_buffer);
			p1 += sprintf(tmp, "            [%s%s,%p] %08lx ",
				(mbu == mb->creator) ? "O" : " ",
				(mbu->tgid == MB_DEF_TGID) ? "K" : "U",
				mbu, mbu->addr);
			p1 += sprintf(p1, "# %4ldKB By %s(%d)\n",
				mbu->size/1024, mbu->the_user, mbu->tgid);
			expectLen += strlen(tmp) + 1;
			if (expectLen > MB_SHOW_BUFSIZE) {
				p += sprintf(p, "\n\n               .......\n");
				break;
			}
			p += sprintf(p, "%s", tmp);
		}
	}
	if (str) {
		if (strlen(show_mb_buffer) < size)
			size = strlen(show_mb_buffer);
		strncpy(str, show_mb_buffer, size);
	} else {
		size = strlen(show_mb_buffer);
		MB_DBG("%s", show_mb_buffer);
	}

	return size;
}

static int mb_show_mba(
	struct mb_area_struct *mba,
	char *msg,
	char *str,
	int size,
	int follow
)
{
	struct mb_struct *mb;
	char *p;
	int idx = 1;

	if (!str && MBMSG_LEVEL <= 1)
		return 0;

	if (!mba)
		return 0;

	memset(show_mba_buffer, 0x0, MBA_SHOW_BUFSIZE);

	p = show_mba_buffer;
	if (msg)
		p += sprintf(p, "%s ", msg);
	p += sprintf(p, "[%p] %p %08lx %3ldMB [%5lx,%5lx] ",
		mba, mba->virt, mba->phys, PAGE_KB(mba->pages)/1024,
		mba->pgi.pfn_start, mba->pgi.pfn_end);
	p += sprintf(p, "%3x %6ldKB/%6ldKB   %s\n", mba->nr_mb,
		PAGE_KB(mba->max_available_pages),
		PAGE_KB(mba->tot_free_pages),
		(mba->flags | MBAFLAG_STATIC) ? "S" : "D");
	if (follow) {
		p += sprintf(p, "    index -    [MemBlock] VirtAddr PhysAddr");
		p += sprintf(p, "    size [  zs,  ze] cnt\n");
		list_loop(mb, &mba->mb_list, mb_list) {
			if ((MBA_SHOW_BUFSIZE - (p - show_mba_buffer)) < 22) {
				p += sprintf(p, "\n\n        more ...\n");
				break;
			}
			p += sprintf(p, "    %5d - ", idx++);
			/* -2 is for \n and zero */
			p += mb_show_mb(mb, NULL, p,
				MBA_SHOW_BUFSIZE - (p - show_mba_buffer) - 2);
		}
	}

	p += sprintf(p, "\n");

	if (str) {
		if (strlen(show_mba_buffer) < size)
			size = strlen(show_mba_buffer);
		strncpy(str, show_mba_buffer, size);
	} else {
		size = strlen(show_mba_buffer);
		MB_DBG("%s", show_mba_buffer);
	}

	return size;
}

/* return physical address */
#ifdef CONFIG_WMT_MB_RESERVE_FROM_IO
/* mb_do_lock locked */
static void *mb_alloc_pages(unsigned long *phys, unsigned long *nr_pages)
{
	unsigned int order = get_order(num_physpages << PAGE_SHIFT);
	unsigned int size = *nr_pages << PAGE_SHIFT;
	unsigned long addr = ((1 << order) - *nr_pages) << PAGE_SHIFT;
	void *virt = NULL;

	if (!nr_pages || !phys) {
		MB_WARN("mb_alloc_pages fail. unknown argument\n");
		return NULL;
	}

	*phys = 0;
	if (*nr_pages == 0) {
		MB_WARN("mb_alloc_pages zero size\n");
		return NULL;
	}

	if (!request_mem_region(addr, size, "memblock")) {
		MB_WARN("request memory region fail. addr %#lx size %d(KB).\n",
			addr, size/1024);
		return NULL;
	}

	virt = ioremap(addr, size);
	if (!virt) {
		MB_WARN("cannot ioremap memory. addr %#lx size %d(KB).\n",
			addr, size/1024);
		release_mem_region(addr, size);
		return NULL;
	}

	*phys = addr;
	MB_DBG("allocate mem region. addr V %p P %#lx size %d(KB)\n",
		virt, addr, size/1024);

	return virt;
}

/* mb_do_lock locked */
static void mb_free_pages(void *virt, unsigned long phys, unsigned int nr_pages)
{
	iounmap(virt);
	MB_DBG("release mem region. addr V %p P %#lx size %d(KB)\n",
		virt, phys, 4*nr_pages);
	return release_mem_region(phys, nr_pages << PAGE_SHIFT);
}

#else
/* mb_do_lock locked */
static void *mb_alloc_pages(unsigned long *phys, unsigned long *nr_pages)
{
	unsigned int i, order;
	unsigned long virt = 0;
	struct zone *zone;

	if (!nr_pages || !phys) {
		MB_WARN("mb_alloc_pages fail. unknown argument\n");
		return NULL;
	}

	*phys = 0;
	if (*nr_pages == 0) {
		MB_WARN("mb_alloc_pages zero size\n");
		return NULL;
	}

	order = get_order(*nr_pages * PAGE_SIZE);
	order = max(order, (unsigned int)(MBA_MIN_ORDER));
	MB_DBG(" %ld/%d pages, order %d\n", *nr_pages, 1 << order, order);
	if (order > MBMAX_ORDER) {
		MB_WARN("mb_alloc_mba fail. page out of size, %ld/%d/%d\n",
			*nr_pages, (1<<order), (1<<MBMAX_ORDER));
		return NULL;
	}

	zone = (first_online_pgdat())->node_zones;
	for (i = order; i < MAX_ORDER; i++) {
		if (zone->free_area[i].nr_free) {
			virt = __get_free_pages(GFP_ATOMIC | GFP_DMA, order);
			break;
		}
	}

	if (virt) {
		*nr_pages = 1 << order;
		*phys = __pa(virt);
		for (i = 0; i < *nr_pages; i++)
			SetPageReserved(virt_to_page(virt + i * PAGE_SIZE));
		MB_DBG("allocate mem region. addr V %#lx P %#lx size %ld(KB)\n",
			virt, *phys, PAGE_KB(*nr_pages));
	} else {
		struct free_area *fa;
		char msg[256], *p = msg;
		MB_WARN("__get_free_pages fail! (pages: %d free %lu)\n",
			1 << order, (unsigned long)nr_free_pages());
		zone = (first_online_pgdat())->node_zones;
		fa = zone->free_area;
		p += sprintf(msg, "DMA ZONE:");
		for (i = 0; i < MAX_ORDER; i++)
			p += sprintf(p, " %ld*%dKB", fa[i].nr_free, 4 << i);
		MB_WARN("%s= %ldkB\n", msg, PAGE_KB(nr_free_pages()));
	}

	return (void *)virt;
}

/* mb_do_lock locked */
static void mb_free_pages(void *virt, unsigned long phys, unsigned int nr_pages)
{
	unsigned int index;

	if (!phys || !nr_pages) {
		MB_WARN("mb_free_pages unknow addr V %p P %#lx size %d pages\n",
			virt, phys, nr_pages);
		return;
	}

	for (index = 0; index < nr_pages; index++) {
		unsigned long addr = (unsigned long)virt + index * PAGE_SIZE;
		ClearPageReserved(virt_to_page(addr));
	}

	free_pages((unsigned long)virt, get_order(nr_pages * PAGE_SIZE));
}
#endif

/* mb_do_lock locked */
static struct mb_area_struct *mb_alloc_mba(unsigned int pages)
{
	struct mba_host_struct *mbah = wmt_mbah;
	struct mb_area_struct *mba;

	if (!mbah || !pages) {
		MB_WARN("mb_alloc_mba fail. mbah %p, pages %d\n", mbah, pages);
		return NULL;
	}

	mba = kmem_cache_alloc(mbah->mba_cachep, GFP_ATOMIC);
	if (!mba) {
		MB_WARN("mb_alloc_mba fail. out of memory\n");
		return NULL;
	}

	memset(mba, 0x0, sizeof(struct mb_area_struct));
	mba->pages = pages;
	mba->virt = mb_alloc_pages(&mba->phys, &mba->pages);
	if (!mba->virt) {
		MB_WARN("mb_alloc_mba fail. no available space\n");
		kmem_cache_free(mbah->mba_cachep, mba);
		return NULL;
	}

	/* initialization */
	INIT_LIST_HEAD(&mba->mb_list);
	INIT_LIST_HEAD(&mba->mba_list);
	mba->mbah = mbah;
	mba->tot_free_pages = mba->max_available_pages = mba->pages;
	mba->pgi.pfn_start = mba->phys >> PAGE_SHIFT;
	mba->pgi.pfn_end = mba->pgi.pfn_start + mba->pages;
	list_add_tail(&mba->mba_list, &mbah->mba_list);

	/* update MBA host */
	mbah->nr_mba++;
	mbah->tot_pages += mba->pages;
	mbah->tot_free_pages += mba->tot_free_pages;
	if (mbah->max_available_pages < mba->max_available_pages)
		mbah->max_available_pages = mba->max_available_pages;
	mb_show_mba(mba, "alloc", NULL, 0, 0);

	return mba;
}

/* mb_do_lock locked */
static int mb_free_mba(struct mb_area_struct *mba)
{
	struct mba_host_struct *mbah;
	struct mb_area_struct *entry;

	if (!mba || !mba->mbah || mba->nr_mb) {
		MB_WARN("mb_free_mba fail. unknow arg.(%p,%p,%x)\n",
			mba, mba ? mba->mbah : NULL, mba ? mba->nr_mb : 0);
		return -EFAULT;
	}

	mbah = mba->mbah;
	list_loop(entry, &mbah->mba_list, mba_list) {
		if (entry == mba)
			break;
	}

	if (entry != mba) {
		MB_WARN("mb_free_mba fail. unknow MBA %p\n", mba);
		return -EFAULT;
	}
	if (mba->flags & MBAFLAG_STATIC)
		return 0;

	mb_show_mba(mba, "ReleaseMBA", NULL, 0, 0);

	/* free mba */
	list_del(&mba->mba_list);
	mb_free_pages(mba->virt, mba->phys, mba->pages);
	mbah->nr_mba--;
	mbah->tot_free_pages -= mba->tot_free_pages;
	mbah->tot_pages -= mba->pages;
	kmem_cache_free(mbah->mba_cachep, mba);
	mba = NULL;

	/* update max mb size */
	mbah->max_available_pages = 0;
	list_loop(entry, &mbah->mba_list, mba_list) {
		if (mbah->max_available_pages < entry->max_available_pages)
			mbah->max_available_pages = entry->max_available_pages;
	}

	return 0;
}

/* mb_do_lock locked */
static struct mb_struct *mb_alloc_mb(
	struct mb_area_struct *mba,
	unsigned long size)
{
	struct mba_host_struct *mbah;
	struct mb_struct *mb, *entry;
	struct list_head *next;
	unsigned long pages, zs, ze;

	if (!mba || !mba->mbah || !size) {
		MB_WARN("mb_alloc_mb fail. unknow arg.(%p,%p,%lx)\n",
			mba, mba ? mba->mbah : NULL, size);
		return NULL;
	}

	mbah = mba->mbah;
	size = PAGE_ALIGN(size);
	pages = size >> PAGE_SHIFT;

	/* mb free size is not enough */
	if (mba->max_available_pages < pages) {
		MB_WARN("mb_alloc_mb fail. no space in MBA (%lx<%lx)\n",
			pages, mba->max_available_pages);
		return NULL;
	}

	/* search available zone
	zone start from mba start */
	ze = zs = mba->pgi.pfn_start;
	next = &mba->mb_list;
	list_loop(entry, &mba->mb_list, mb_list) {
		next = &entry->mb_list;
		ze = entry->pgi.pfn_start;
		if ((ze - zs) >= pages)
			break;
		zs = entry->pgi.pfn_end;
	}

	if (zs >= ze) {
		next = &mba->mb_list;
		ze = mba->pgi.pfn_end;
	}

	/* impossible, something wrong */
	if ((ze - zs) < pages) {
		MB_WARN("something wrong in MBA %p when allocate MB.\n", mba);
		return NULL;
	}

	MB_DBG("Zone finding start %lx end %lx size %lx for size %lx\n",
		zs, ze, ze - zs, pages);

	mb = kmem_cache_alloc(mbah->mb_cachep, GFP_ATOMIC);
	if (!mb) {
		MB_WARN("mb_alloc_mb mb_cachep out of memory\n");
		return NULL;
	}

	memset(mb, 0x0, sizeof(struct mb_struct));
	INIT_LIST_HEAD(&mb->mb_list);
	INIT_LIST_HEAD(&mb->mbu_list);
	mb->mba = mba;
	mb->pgi.pfn_start = zs;
	mb->pgi.pfn_end = mb->pgi.pfn_start + pages;
	mb->size = size;
	mb->phys = mba->phys + ((zs - mba->pgi.pfn_start) << PAGE_SHIFT);
	mb->virt = mba->virt + (mb->phys - mba->phys);
	list_add_tail(&mb->mb_list, next);

	mba->nr_mb++;
	mba->tot_free_pages -= pages;
	mbah->tot_free_pages -= pages;

	/* update mba */
	zs = mba->pgi.pfn_start;
	mba->max_available_pages = 0;
	list_loop(entry, &mba->mb_list, mb_list) {
		mba->max_available_pages =
		max(entry->pgi.pfn_start - zs, mba->max_available_pages);
		zs = entry->pgi.pfn_end;
	}
	mba->max_available_pages =
	max(mba->pgi.pfn_end - zs, mba->max_available_pages);

	/* update mbah */
	mbah->max_available_pages = 0;
	list_loop(mba, &mbah->mba_list, mba_list) {
		if (mbah->max_available_pages < mba->max_available_pages)
			mbah->max_available_pages = mba->max_available_pages;
	}

	mb_show_mb(mb, "alloc", NULL, 0);

	return mb;
}

/* mb_do_lock locked */
static int mb_free_mb(struct mb_struct *mb)
{
	unsigned long zs, nr_pages;
	struct mba_host_struct *mbah;
	struct mb_area_struct *mba;
	struct mb_struct *entry;

	if (!mb) {
		MB_WARN("mb_free_mb fail. unknow MB %p.\n", mb);
		return -EFAULT;
	}

	if (MB_IN_USE(mb)) {
		MB_WARN("mb_free_mb fail. invalid arg.(%p,%x,%lx,%ldKB)\n",
			mb, MB_COUNT(mb), mb->phys, mb->size/1024);
		return -EINVAL;
	}

	mba = mb->mba;
	if (!mba || !mba->mbah) {
		MB_WARN("mb_free_mb fail. unknow para.(%p,%p)\n",
			mba, mba ? mba->mbah : NULL);
		return -EFAULT;
	}

	list_loop(entry, &mba->mb_list, mb_list) {
		if (entry == mb)
			break;
	}

	if (entry != mb) {
		MB_WARN("mb_free_mb fail. unknow MB %p\n", mb);
		return -EFAULT;
	}

	mbah = mba->mbah;

	mb_show_mb(mb, "Retrieve unused MB", NULL, 0);

	/* free mb */
	list_del(&mb->mb_list);
	kmem_cache_free(mbah->mb_cachep, mb);
	mba->nr_mb--;
	mb = NULL;

	/* unused mba, release it */
	if (!mba->nr_mb && !(mba->flags & MBAFLAG_STATIC))
		return mb_free_mba(mba);

	/* update max mb size and free mb size */
	/* reduce old one and then increase new one */
	mbah->tot_free_pages -= mba->tot_free_pages;
	zs = mba->pgi.pfn_start;
	mba->tot_free_pages = mba->max_available_pages = nr_pages = 0;
	list_loop(entry, &mba->mb_list, mb_list) {
		nr_pages = max(entry->pgi.pfn_start - zs, nr_pages);
		mba->tot_free_pages += entry->pgi.pfn_start - zs;
		zs = entry->pgi.pfn_end;
	}
	mba->tot_free_pages += mba->pgi.pfn_end - zs;
	mba->max_available_pages = max(mba->pgi.pfn_end - zs, nr_pages);
	mbah->tot_free_pages += mba->tot_free_pages; /* add new one */
	mbah->max_available_pages =
		max(mbah->max_available_pages, mba->max_available_pages);

	return 0;
}

/* type 0 - virtual address
 *      1 - physical address */
static struct mb_struct *mb_find_mb(void *addr, int type)
{
	struct mba_host_struct *mbah = wmt_mbah;
	struct mb_area_struct *mba;
	struct mb_struct *mb;
	unsigned long flags;

	if (!addr || (type != MBFIND_PHYS && type != MBFIND_VIRT)) {
		MB_WARN("mb_find_mb fail. unknow type %d addr %p\n",
			type, addr);
		return NULL;
	}

	MB_DBG("IN, type %x addr %p\n", type, addr);

	spin_lock_irqsave(&mb_search_lock, flags);

	list_loop(mba, &mbah->mba_list, mba_list) {
		void *eaddr, *saddr = (void *)(mba->phys);
		if (type != MBFIND_PHYS)
			saddr = mba->virt;
		eaddr = saddr + mba->pages*PAGE_SIZE;
		if (addr < saddr || addr > eaddr)
			continue; /* address out of mba range */
		list_loop(mb, &mba->mb_list, mb_list) {
			void *mbaddr = (void *)(mb->phys);
			if (type != MBFIND_PHYS)
				mbaddr = mb->virt;
			if (addr >= mbaddr && addr < (mbaddr + mb->size)) {
				spin_unlock_irqrestore(&mb_search_lock, flags);
				MB_DBG("OUT, va %p pa %lx s %ld(KB) cnt %d\n",
					mb->virt, mb->phys, mb->size/1024,
					MB_COUNT(mb));
				return mb;
			}
		}
	}

	spin_unlock_irqrestore(&mb_search_lock, flags);

	MB_DBG("OUT, NULL mb\n");

	return NULL;
}

/*
 * addr - search address
 * tgid - task pid
 * st   - search type (MBUFIND_XXX)
 *     MBUFIND_USER: user address
 *     MBUFIND_VIRT: kernel virt address
 *     MBUFIND_PHYS: kernel phys address
 *     MBUFIND_ALL:
 *         1. mbu->tgid == tgid
 *         2. tgid == MB_DEF_TGID, address and
 *                 one of MBUFIND_VIRT and MBUFIND_PHYS must be set
 *            tgid != MB_DEF_TGID, address and MBUFIND_USER must be set
 *     MBUFIND_CREATOR:
 *         1. mbu->tgid == tgid
 *         2. tgid == MB_DEF_TGID, address and
 *                 one of MBUFIND_VIRT and MBUFIND_PHYS must be set
 *            tgid != MB_DEF_TGID, address and MBUFIND_USER must be set
 *         3. mbu == mb->creator
 *         4. mbu->size == mb->size
 *     MBUFIND_GETPUT:
 *         1. mbu->tgid == tgid
 *         2. tgid == MB_DEF_TGID, address and
 *                 MBUFIND_PHYS must be set (mbu->addr == mb->phys)
 *            tgid != MB_DEF_TGID, address and
 *                 MBUFIND_USER must be set (mbu->addr == user address)
 *         3. mbu->size == 0
 *     MBUFIND_MMAP:
 *         1. mbu->tgid == tgid
 *         2. address and MBUFIND_USER must be set
 *         3. mbu->size != 0
 */
static struct mb_user *mb_find_mbu(void *addr, pid_t tgid, int st)
{
	struct mba_host_struct *mbah = wmt_mbah;
	struct mb_area_struct *mba = NULL;
	struct mb_struct *mb = NULL;
	struct mb_user *mbu = NULL;
	unsigned long flags;

	if (!addr || !(st & MBUFIND_ADDRMASK)) {
		MB_WARN("mb_find_mbu fail. unknow addr %p\n", addr);
		return NULL;
	}

	MB_DBG("IN, addr %p search type %x TGID %x CurTask %s\n",
		addr, st, tgid, current->comm);

	spin_lock_irqsave(&mb_search_lock, flags);
	list_loop(mba, &mbah->mba_list, mba_list) {
		if (st & (MBUFIND_VIRT | MBUFIND_PHYS)) {
			void *eaddr, *saddr = (void *)mba->phys;
			if (st & MBUFIND_VIRT)
				saddr = mba->virt;
			eaddr = saddr + mba->pages*PAGE_SIZE;
			if (addr < saddr || addr > eaddr)
				continue; /* address out of mba range */
		}
		list_loop(mb, &mba->mb_list, mb_list) {
			if ((st & MBUFIND_PHYS) && (addr != (void *)mb->phys))
				continue; /* physical address not match */
			if ((st & MBUFIND_VIRT) && (addr != mb->virt))
				continue; /* virtual address not match */
			list_loop(mbu, &mb->mbu_list, mbu_list) {
				if (mbu->tgid != tgid && tgid != MB_DEF_TGID)
					continue; /* tgid not match */
				if ((st & MBUFIND_USER) &&
				    (addr != (void *)mbu->addr))
					continue; /* user address not match */
				if (st & MBUFIND_ALL)
					goto leave; /* found */
				if (st & MBUFIND_CREATOR) {
					mbu = mb->creator;
					goto leave; /* found */
				}
				/* found (if mmap, mbu->size has map size) */
				if (st & MBUFIND_MMAP && mbu->size)
					goto leave;
				/* found (if get/put, mbu->size should be 0) */
				if (st & MBUFIND_GETPUT &&
				    mbu->addr && !mbu->size)
					goto leave;
			}
		}
	}
	mbu = NULL;

leave:
	if (mbu == NULL)
		MB_DBG("OUT, NULL mbu\n");

	if (mbu)
		MB_DBG("OUT, mbu %p (%p TGID %x a %#lx s %ld(KB) by %s)\n",
			mbu, mbu->mb, mbu->tgid, mbu->addr,
			mbu->size/1024, mbu->the_user);

	spin_unlock_irqrestore(&mb_search_lock, flags);

	return mbu;
}

static void mb_update_mbah(void)
{
	unsigned long mbah_free = 0, mbah_max = 0;
	struct mba_host_struct *mbah = wmt_mbah;
	struct mb_area_struct *mba = NULL;
	struct mb_struct *mb = NULL;

	if (!mbah) {
		MB_WARN("mb_update_mbah fail. unknow mbah\n");
		return;
	}

	list_loop(mba, &mbah->mba_list, mba_list) {
		unsigned long mba_free = 0, mba_max = 0, nr_pages = 0;
		unsigned long zs = mba->pgi.pfn_start; /* zone start */
		list_loop(mb, &mba->mb_list, mb_list) {
			nr_pages = max(mb->pgi.pfn_start - zs, nr_pages);
			mba_free += mb->pgi.pfn_start - zs;
			zs = mb->pgi.pfn_end;
		}
		mba_free += mba->pgi.pfn_end - zs;
		mbah_free += mba_free;
		mba_max = max(mba->pgi.pfn_end - zs, nr_pages);
		mbah_max = max(mbah_max, mba_max);
	}

	mbah->tot_free_pages = mbah_free;
	mbah->max_available_pages = mbah_max;

	return;
}

/* called from kernel only, mb_do_lock isn't needed */
void *mb_do_phys_to_virt(unsigned long phys, pid_t tgid, char *name)
{
	struct mb_struct *mb = NULL;
	void *virt = NULL;

	MB_DBG("IN, Phys %lx TGID %x NAME %s\n", phys, tgid, name);
	mb = mb_find_mb((void *)phys, MBFIND_PHYS);
	if (mb)
		virt = mb->virt + (phys - mb->phys);
	else {
		virt = __va(phys);
		MB_WARN("%s do phys to virt fail. addr %lx not found\n",
			name, phys);
	}

	MB_DBG("OUT, Virt %p\n", virt);

	return virt;
}
EXPORT_SYMBOL(mb_do_phys_to_virt);

/* called from kernel only, mb_do_lock isn't needed */
unsigned long mb_do_virt_to_phys(void *virt, pid_t tgid, char *name)
{
	struct mb_struct *mb = NULL;
	unsigned long phys = 0x0;

	MB_DBG("IN, Virt %p TGID %x NAME %s\n", virt, tgid, name);
	mb = mb_find_mb(virt, MBFIND_VIRT);
	if (mb)
		phys = mb->phys + (virt - mb->virt);
	else {
		phys = __pa(virt);
		MB_WARN("%s do virt to phys fail. addr %p not found\n",
			name, virt);
	}

	MB_DBG("OUT, Phys %lx\n", phys);

	return phys;
}
EXPORT_SYMBOL(mb_do_virt_to_phys);

unsigned long mb_do_user_to_phys(unsigned long user, pid_t tgid, char *name)
{
	unsigned long flags, phys = 0;
	struct mb_user *mbu = NULL;

	MB_DBG("IN, usr %lx TGID %x name %s\n", user, tgid, name);

	spin_lock_irqsave(&mb_do_lock, flags);
	mbu = mb_find_mbu((void *)user, tgid, MBUFIND_USER|MBUFIND_ALL);
	if (!mbu) {
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_WARN("%s do user to phys. unknow addr %lx\n", name, user);
		return 0;
	}

	if (!mbu->mb)
		MB_WARN("%s do user to phys fail. unknow block (addr %lx)\n",
			name, user);
	else
		phys = mbu->mb->phys;

	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("OUT, Phys %#lx\n", phys);

	return phys;
}
EXPORT_SYMBOL(mb_do_user_to_phys);

unsigned long mb_do_user_to_virt(unsigned long user, pid_t tgid, char *name)
{
	struct mb_user *mbu = NULL;
	unsigned long flags;
	void *virt = NULL;

	MB_DBG("IN, usr %lx TGID %x name %s\n", user, tgid, name);

	spin_lock_irqsave(&mb_do_lock, flags);
	mbu = mb_find_mbu((void *)user, tgid, MBUFIND_USER|MBUFIND_ALL);
	if (!mbu) {
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_WARN("%s user to virt fail. unknow addr %lx\n", name, user);
		return 0;
	}

	if (!mbu->mb)
		MB_WARN("%s user to virt fail. unknow block (addr %lx)\n",
			name, user);
	else
		virt = mbu->mb->virt;

	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("OUT, Virt %p\n", virt);

	return (unsigned long)virt;
}
EXPORT_SYMBOL(mb_do_user_to_virt);

/* physical address */
int mb_do_counter(unsigned long addr, char *name)
{
	struct mb_struct *mb;
	unsigned long flags;
	int counter = -1;

	MB_DBG("IN, addr %#lx name %s\n", addr, name);

	if (!addr || !wmt_mbah) {
		MB_WARN("%s do counter fail. invalid args %p/%lx\n",
			name, wmt_mbah, addr);
		return -EINVAL;
	}
	spin_lock_irqsave(&mb_do_lock, flags);
	mb = mb_find_mb((void *)addr, MBFIND_PHYS);
	counter = (mb) ? MB_COUNT(mb) : -1;
	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("OUT, addr %#lx count %d\n", addr, counter);

	return counter;
}
EXPORT_SYMBOL(mb_do_counter);

unsigned long mb_do_alloc(unsigned long size, pid_t tgid, char *name)
{
	struct mb_area_struct *entry, *mba = NULL;
	struct mb_struct *mb = NULL;
	struct mb_user *mbu = NULL;
	unsigned long flags, addr;
	unsigned int pages;
	size_t ns; /* name size */

	if (!name) {
		MB_WARN("mb_alloc fail. null user tgid %x\n", tgid);
		return 0;
	}

	size = PAGE_ALIGN(size);
	pages = size >> PAGE_SHIFT;

	if (!pages) {
		MB_WARN("%s alloc fail. unavailable size %x(KB)\n",
			name, 4*pages);
		return 0;
	}

	mbu = kmem_cache_alloc(wmt_mbah->mbu_cachep, GFP_ATOMIC);
	if (!mbu) {
		MB_WARN("%s alloc fail. mbu_cachep out of memory.\n", name);
		return 0;
	}
	memset(mbu, 0x0, sizeof(struct mb_user));

	MB_DBG("IN, TGID %x size %ld(KB) name %s\n", tgid, size/1024, name);

	spin_lock_irqsave(&mb_do_lock, flags);

	if (pages > wmt_mbah->max_available_pages) {
#ifdef CONFIG_WMT_MB_DYNAMIC_ALLOCATE_SUPPORT
		mba = mb_alloc_mba(pages);
		if (mba == NULL)
			MB_WARN("%s alloc %u page fail. [MBA %lu/%lu/%lu]\n",
				name, pages, wmt_mbah->max_available_pages,
				wmt_mbah->tot_free_pages, wmt_mbah->tot_pages);
#else
		MB_DBG("DYNAMIC ALLOCATED(%u) unsuported. [MBA %lu/%lu/%lu]\n",
			pages, wmt_mbah->max_available_pages,
			wmt_mbah->tot_free_pages, wmt_mbah->tot_pages);
		goto error;
#endif
	} else {
		list_loop(entry, &wmt_mbah->mba_list, mba_list) {
			if (entry->max_available_pages >= pages) {
				mba = entry;
				break;
			}
		}
	}

	if (!mba) {
		MB_WARN("%s alloc fail. dedicated MBA not found\n", name);
		goto error;
	}

	mb = mb_alloc_mb(mba, size);
	if (mb == NULL) {
		MB_WARN("%s alloc fail. create MB\n", name);
		goto error;
	}

	INIT_LIST_HEAD(&mbu->mbu_list);
	mbu->mb = mb;
	mbu->tgid = tgid;
	mbu->size = mb->size;
	/* mbu->addr = 0; // address of creator is 0 */
	ns = strlen(name);
	if (ns > TASK_COMM_LEN)
		ns = TASK_COMM_LEN;
	strncpy(mbu->the_user, name, ns);
	mbu->the_user[ns] = 0;
	list_add_tail(&mbu->mbu_list, &mb->mbu_list);
	atomic_inc(&mb->count);
	mb->creator = mbu;
	addr = mb->phys;
	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("OUT, Addr %lx TGID %x size %ld(KB) name %s\n",
		addr, tgid, size/1024, name);

	return addr;

error:

	kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
	spin_unlock_irqrestore(&mb_do_lock, flags);
	return 0;
}
EXPORT_SYMBOL(mb_do_alloc);

/* physical address */
int mb_do_free(unsigned long addr, pid_t tgid, char *name)
{
	struct mb_struct *mb;
	struct mb_user *mbu;
	unsigned long flags;
	size_t ns; /* name size */
	int ret;

	if (!addr || !name) {
		MB_WARN("mb_free fail. invalid args %lx/%s\n",
			addr, name);
		return -EINVAL;
	}

	MB_DBG("IN, TGID %x addr %#lx name %s\n", tgid, addr, name);

	spin_lock_irqsave(&mb_do_lock, flags);
	mbu = mb_find_mbu((void *)addr, tgid, MBUFIND_PHYS|MBUFIND_CREATOR);
	if (!mbu || !mbu->mb) {
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_WARN("%s free fail. unknow addr %lx\n", name, addr);
		return -EINVAL;
	}

	ns = strlen(name);
	if (ns > TASK_COMM_LEN)
		ns = TASK_COMM_LEN;
	if (strncmp(mbu->the_user, name, ns))
		MB_DBG("Owner no match. (%s/%s)\n", mbu->the_user, name);

	mb = mbu->mb;

	list_del(&mbu->mbu_list);
	kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
	mb->creator = NULL;
	atomic_dec(&mb->count);
	if (MB_IN_USE(mb)) {
		MB_DBG("<mb_free, MB %8lx still in use. Cnt %d>\n",
			mb->phys, MB_COUNT(mb));
		spin_unlock_irqrestore(&mb_do_lock, flags);
		return 0;
	}

	ret = mb_free_mb(mb);
	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("OUT\n");

	return ret;
}
EXPORT_SYMBOL(mb_do_free);

int mb_do_get(unsigned long addr, pid_t tgid, char *name)
{
	struct mb_user *mbu = NULL, *newmbu;
	struct mb_struct *mb = NULL;
	unsigned long flags;
	size_t ns; /* name size */
	void *virt = (void *)addr;

	if (!addr || !name) {
		MB_WARN("mb_do_get fail. invalid args %lx/%s\n",
			addr, name);
		return -EINVAL;
	}

	newmbu = kmem_cache_alloc(wmt_mbah->mbu_cachep, GFP_ATOMIC);
	if (!newmbu) {
		MB_DBG("<mbu_cachep out of memory.>\n");
		return -ENOMEM;
	}

	MB_DBG("IN, TGID %x addr %#lx name %s\n", tgid, addr, name);

	spin_lock_irqsave(&mb_do_lock, flags);
	if (tgid != MB_DEF_TGID) { /* find user address, only exist on MMAP */
		mbu = mb_find_mbu(virt, tgid, MBUFIND_USER|MBUFIND_MMAP);
		mb = (mbu) ? mbu->mb : NULL;
	} else
		mb = mb_find_mb(virt, MBFIND_PHYS);

	if (!mb) {
		spin_unlock_irqrestore(&mb_do_lock, flags);
		kmem_cache_free(wmt_mbah->mbu_cachep, newmbu);
		MB_WARN("%s mb_get fail. unknown addr %8lx\n", name, addr);
		return -EFAULT;
	}

	memset(newmbu, 0x0, sizeof(struct mb_user));
	INIT_LIST_HEAD(&newmbu->mbu_list);
	newmbu->addr = addr;
	newmbu->mb = mb;
	newmbu->tgid = tgid;
	ns = strlen(name);
	if (ns > TASK_COMM_LEN)
		ns = TASK_COMM_LEN;
	strncpy(newmbu->the_user, name, ns);
	newmbu->the_user[ns] = 0;
	atomic_inc(&mb->count);
	list_add_tail(&newmbu->mbu_list, &mb->mbu_list);

	spin_unlock_irqrestore(&mb_do_lock, flags);

	MB_DBG("out, [mbu] addr %lx %s [mb] addr %lx cnt %d\n",
		addr, newmbu->the_user, mb->phys, MB_COUNT(mb));

	return 0;
}
EXPORT_SYMBOL(mb_do_get);

int mb_do_put(unsigned long addr, pid_t tgid, char *name)
{
	struct mb_struct *mb;
	struct mb_user *mbu;
	unsigned long flags;
	void *virt = (void *)addr;

	if (!addr || !name) {
		MB_WARN("mb_do_put fail. invalid args %lx/%s\n", addr, name);
		return -EINVAL;
	}

	MB_DBG("IN, TGID %x addr %#lx name %s\n", tgid, addr, name);

	spin_lock_irqsave(&mb_do_lock, flags);
	if (tgid == MB_DEF_TGID)
		mbu = mb_find_mbu(virt, tgid, MBUFIND_PHYS|MBUFIND_GETPUT);
	else
		mbu = mb_find_mbu(virt, tgid, MBUFIND_USER|MBUFIND_GETPUT);

	if (!mbu) {
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_WARN("%s mb_put fail. unknow addr %8lx\n", name, addr);
		return -EINVAL;
	}

	mb = mbu->mb;
	if (!MB_IN_USE(mb)) {
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_WARN("%s mb_put fail. block %p unbalance.\n", name, mb);
		return -EPERM;
	}

	list_del_init(&mbu->mbu_list);
	atomic_dec(&mb->count);
	/* retrieve unused block */
	if (!MB_IN_USE(mb))
		mb_free_mb(mb);

	spin_unlock_irqrestore(&mb_do_lock, flags);
	kmem_cache_free(wmt_mbah->mbu_cachep, mbu);

	MB_DBG("out, [mbu] addr %lx %s [mb] addr %lx cnt %d\n",
		addr, mbu->the_user, mb->phys, MB_COUNT(mb));

	return 0;
}
EXPORT_SYMBOL(mb_do_put);

#define DEVICE_NAME "Memory Block"

static int mb_dev_major;
static int mb_dev_minor;
static int mb_dev_nr;
static struct cdev *mb_cdev;
static struct class *mb_dev_class;

static int mb_open(struct inode *inode, struct file *filp)
{
	struct mb_task_info *mbti = NULL;
	unsigned long flags;
	int task_exist = 0;
	size_t ns;

	if (filp->private_data) {
		MB_ERROR("none empty private data.\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&mb_task_lock, flags);
	list_loop(mbti, &wmt_mbti.mbti_list, mbti_list) {
		if (mbti->task && mbti->tgid == current->tgid) {
			task_exist = 1;
			break;
		}
	}
	if (!task_exist) {
		mbti = kmem_cache_alloc(wmt_mbah->mbti_cachep, GFP_ATOMIC);
		if (!mbti) {
			spin_unlock_irqrestore(&mb_task_lock, flags);
			MB_ERROR("out of memory (mb_task_info).\n");
			return -EFAULT;
		}
		memset(mbti, 0x0, sizeof(struct mb_task_info));
		INIT_LIST_HEAD(&mbti->mbti_list);
		mbti->task = current;
		mbti->tgid = current->tgid;
		ns = strlen(current->comm);
		if (ns > TASK_COMM_LEN)
			ns = TASK_COMM_LEN;
		strncpy(mbti->task_name, current->comm, ns);
		mbti->task_name[ns] = 0;
		atomic_set(&mbti->count, 0);
		list_add_tail(&mbti->mbti_list, &wmt_mbti.mbti_list);
	}
	atomic_inc(&mbti->count);
	MB_DBG("mb driver is opened by task(%p) %s TGID %x count %d.\n",
		current, current->comm, current->tgid,
		atomic_read(&(mbti->count)));
	filp->private_data = (void *)mbti;
	spin_unlock_irqrestore(&mb_task_lock, flags);

	return 0;
}

static int mb_release(struct inode *inode, struct file *filp)
{
	struct mb_area_struct *mba;
	struct mb_struct *mb;
	struct mb_user *mbu;
	struct mb_task_info *cmbti = NULL, *mbti = NULL;
	unsigned long flags, tflags;
	int task_exist = 0;

	cmbti = (struct mb_task_info *)(filp->private_data);
	if (!cmbti) {
		MB_ERROR("none empty private data.\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&mb_task_lock, tflags);
	list_loop(mbti, &wmt_mbti.mbti_list, mbti_list) {
		if (mbti->task && mbti->tgid == cmbti->tgid) {
			atomic_dec(&mbti->count);
			task_exist = 1;
			break;
		}
	}
	if (!task_exist) {
		spin_unlock_irqrestore(&mb_task_lock, tflags);
		MB_INFO("mb driver is closed by unknown task %s TGID %x.\n",
		current->comm, current->tgid);
		return 0;
	}

	MB_DBG("mb driver is closed by task %s TGID %x cnt %d.\n",
		mbti->task_name, mbti->tgid, atomic_read(&mbti->count));
	if (atomic_read(&mbti->count)) {
		spin_unlock_irqrestore(&mb_task_lock, tflags);
		return 0;
	}

	spin_lock_irqsave(&mb_ioctl_lock, flags);
RESCAN_MBA:
	/* munmap virtual memroy and retrieve unused MB */
	list_loop(mba, &wmt_mbah->mba_list, mba_list) {
#ifdef CONFIG_WMT_MB_DYNAMIC_ALLOCATE_SUPPORT
		if (mba->tgid == mbti->tgid) { /* free prefetched mba marker */
			wmt_mbah->tot_static_pages -= mba->pages;
			mba->flags &= ~(MBAFLAG_STATIC);
			mba->tgid = MB_DEF_TGID;
			if (!mba->nr_mb) {
				mb_free_mba(mba);
				goto RESCAN_MBA;
			}
		}
#endif
		list_loop(mb, &mba->mb_list, mb_list) {
RESCAN_MBU:
			list_loop(mbu, &mb->mbu_list, mbu_list) {
				if (mbu->tgid == mbti->tgid) {
					char msg[128], *p = msg;
					const char *opStr[3] = {
						"\"ALLOC\"",
						"\"MAP\"",
						"\"GET\""};
					int op = (mbu->size) ? 1 : 2;
					if (mb->creator == mbu) {
						op = 0;
						mb->creator = NULL;
					}
					p += sprintf(p, "Stop mb operation %s",
						opStr[op]);
					p += sprintf(p, "(addr %lx # %lu(KB) ",
						mb->phys, mbu->size/1024);
					p += sprintf(p, "by %s) (Cnt %d)\n",
						 mbu->the_user, MB_COUNT(mb)-1);
					MB_INFO("%s", msg);
					atomic_dec(&mb->count);
					list_del_init(&mbu->mbu_list);
					kmem_cache_free(
						wmt_mbah->mbu_cachep, mbu);
					if (MB_COUNT(mb))
						goto RESCAN_MBU;
					MB_INFO("Recycle mb addr %lx # %lu\n",
						mb->phys, mb->size);
					mb_free_mb(mb);
					goto RESCAN_MBA;
				}
			}
		}
	}

	mb_update_mbah();
	list_del(&mbti->mbti_list);
	kmem_cache_free(wmt_mbah->mbti_cachep, mbti);
	filp->private_data = NULL;
	spin_unlock_irqrestore(&mb_ioctl_lock, flags);
	spin_unlock_irqrestore(&mb_task_lock, tflags);

	return 0;
}

static long mb_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mb_task_info *mbti = NULL;
	struct mb_area_struct *mba = NULL;
	struct mb_struct *mb = NULL;
	struct mb_user *mbu = NULL;
	unsigned long value, flags, phys, virt, size;
	pid_t tgid = MB_DEF_TGID;
	int count, ret = 0, cmdSize;
	size_t len;

	/* check type and number, if fail return ENOTTY */
#ifdef CONFIG_WMT_MB_RESERVE_FROM_IO
	if (_IOC_TYPE(cmd) != MB_IOC_MAGIC && cmd != FBIOGET_FSCREENINFO)
		return -ENOTTY;
#else
	if (_IOC_TYPE(cmd) != MB_IOC_MAGIC)
		return -ENOTTY;
#endif

	/* check argument area */
	cmdSize = _IOC_SIZE(cmd);
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void __user *) arg, cmdSize);
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void __user *) arg, cmdSize);

	if (ret)
		return -EFAULT;

	/* check task validation */
	if (!filp || !filp->private_data) {
		MB_ERROR("mmap with null file info or task.\n");
		return -EFAULT;
	}

	mbti = (struct mb_task_info *)(filp->private_data);
	tgid = mbti->tgid;
	if (tgid != current->tgid) {
		MB_WARN("ioctl within diff task.\n");
		MB_WARN("Open: task %s TGID %x VS Curr: task %s TGID %x\n",
			mbti->task_name, tgid, current->comm, current->tgid);
	}

	switch (cmd) {
	/* _IOR (MB_IOC_MAGIC, 16, unsigned long) O: mb driver version */
	case MBIO_GET_VERSION:
		value = MB_VERSION_MAJOR << 24 |
			MB_VERSION_MINOR << 16 |
			MB_VERSION_MICRO << 8  |
			MB_VERSION_BUILD;
		put_user(value, (unsigned long *)arg);
		break;

	/* _IOWR(MB_IOC_MAGIC, 0, unsigned long)
	 *  I: size
	 *  O: physical address */
	case MBIO_MALLOC:
		get_user(value, (unsigned long *)arg);
		phys = mb_do_alloc(value, tgid, mbti->task_name);
		if (!phys) {
			MB_WARN("MALLOC: size %ld fail.\n\n", value);
			return -EFAULT;
		}
		MB_OPDBG("MALLOC: addr %lx size %ld\n\n", phys, value);
		put_user(phys, (unsigned long *)arg);
		break;

	/* _IOW (MB_IOC_MAGIC, 17, unsigned long) O: property of MB */
	case MBIO_SET_CACHABLE:
		get_user(value, (unsigned long *)arg);
		spin_lock_irqsave(&mb_do_lock, flags);
		mb = mb_find_mb((void *)value, MBFIND_PHYS);
		if (!mb) {
			spin_unlock_irqrestore(&mb_do_lock, flags);
			MB_WARN("SET_CACHABLE: phys %8lx (name %s) fail\n",
				value, mbti->task_name);
			return -EFAULT;
		}
		mb->flags |= MBFLAG_CACHED;
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_OPDBG("SET_CACHABLE: phys %8lx\n\n", value);
		break;

	/* _IOW (MB_IOC_MAGIC,18, unsigned long) O: property of MB */
	case MBIO_CLR_CACHABLE:
		get_user(value, (unsigned long *)arg);
		spin_lock_irqsave(&mb_do_lock, flags);
		mb = mb_find_mb((void *)value, MBFIND_PHYS);
		if (!mb) {
			spin_unlock_irqrestore(&mb_do_lock, flags);
			MB_WARN("CLR_CACHABLE: phys %8lx (name %s) fail\n",
				value, mbti->task_name);
			return -EFAULT;
		}
		mb->flags &= ~MBFLAG_CACHED;
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_OPDBG("CLR_CACHABLE: phys %8lx\n\n", value);
		break;

	/* _IOWR(MB_IOC_MAGIC, 1, unsigned long)
	 *  I: user address if map, physical address if not map
	 *  O: ummap size */
	case MBIO_FREE: /* if not unmap, unmap first */
		get_user(value, (unsigned long *)arg);
		spin_lock_irqsave(&mb_ioctl_lock, flags);
		size = value;
		mbu = mb_find_mbu((void *)value, tgid,
			MBUFIND_USER|MBUFIND_MMAP);
		if (mbu != NULL) {
			size = mbu->size;
			value = mbu->mb->phys;
			list_del(&mbu->mbu_list);
			atomic_dec(&mbu->mb->count);
			kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
		} else
			MB_DBG("FREE: unknown MMAP addr %lx\n", value);
		ret = mb_do_free(value, tgid, mbti->task_name);
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		if (ret < 0)
			MB_WARN("FREE: addr %lx fail\n", value);
		/* return mmap size for ummap */
		put_user(size, (unsigned long *)arg);
		MB_OPDBG("FREE: addr %lx out %ld\n\n", value, size);
		break;

	/* _IOWR(MB_IOC_MAGIC, 2, unsigned long)
	 *  I: user address
	 *  O: ummap size */
	case MBIO_UNMAP:
		get_user(value, (unsigned long *)arg);
		spin_lock_irqsave(&mb_ioctl_lock, flags);
		mbu = mb_find_mbu((void *)value, tgid,
			MBUFIND_USER|MBUFIND_MMAP);
		if (mbu == NULL) {
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			MB_WARN("UNMAP: addr %lx fail\n\n", value);
			return -EFAULT;
		}
		mb = mbu->mb;
		size = mbu->size;
		list_del(&mbu->mbu_list);
		kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
		atomic_dec(&mb->count);
		count = MB_COUNT(mb);
		if (count <= 0)
			MB_WARN("UNMAP: MB %8lx count %d weird\n\n",
				mb->phys, count);
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		 /* return mmap size for ummap */
		put_user(size, (unsigned long *)arg);
		return 0;

	/* _IOWR(MB_IOC_MAGIC, 3, unsigned long)
	 *  I: phys address
	 *  O: mb size */
	case MBIO_MBSIZE:
		get_user(value, (unsigned long *)arg);
		spin_lock_irqsave(&mb_ioctl_lock, flags);
		mb = mb_find_mb((void *)value, MBFIND_PHYS);
		if (mb == NULL) {
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			MB_WARN("MBSIZE: user %lx fail.\n\n", value);
			return -EFAULT;
		}
		size = mb->size;
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		MB_OPDBG("MBSIZE: addr %lx size %ld\n\n", value, size);
		put_user(size, (unsigned long *)arg);
		break;

	/* _IOR (MB_IOC_MAGIC, 4, unsigned long)
	 *  O: max free mba size */
	case MBIO_MAX_AVAILABLE_SIZE:
		put_user(wmt_mbah->max_available_pages*PAGE_SIZE,
			(unsigned long *)arg);
		MB_OPDBG("MAX_MB_SIZE: size %ld KB\n\n",
			PAGE_KB(wmt_mbah->max_available_pages));
		break;

	/* _IOW (MB_IOC_MAGIC, 5, unsigned long)
	 *  I: user address */
	case MBIO_GET:
		get_user(value, (unsigned long *)arg);
		ret = mb_do_get(value, tgid, mbti->task_name);
		if (MBMSG_LEVEL) {
			spin_lock_irqsave(&mb_ioctl_lock, flags);
			mbu = mb_find_mbu((void *)value, tgid,
				MBUFIND_USER|MBUFIND_GETPUT);
			count = mbu ? MB_COUNT(mbu->mb) : 0xffffffff;
			MB_OPDBG("GET: addr %lx cnt %x\n\n", value, count);
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		}
		break;

	/* _IOW (MB_IOC_MAGIC, 6, unsigned long)
	 *  I: user address */
	case MBIO_PUT:
		get_user(value, (unsigned long *)arg);
		if (MBMSG_LEVEL) {
			spin_lock_irqsave(&mb_ioctl_lock, flags);
			mbu = mb_find_mbu((void *)value, tgid,
				MBUFIND_USER|MBUFIND_GETPUT);
			count = mbu ? MB_COUNT(mbu->mb) - 1 : 0xffffffff;
			MB_OPDBG("PUT: addr %lx cnt %x\n\n", value, count);
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		}
		ret = mb_do_put(value, tgid, mbti->task_name);
		break;

	/* _IOWR(MB_IOC_MAGIC, 7, unsigned long)
	 *  I: user address
	 *  O: virt address */
	case MBIO_USER_TO_VIRT:
		get_user(value, (unsigned long *)arg);
		virt = mb_do_user_to_virt(value, tgid, mbti->task_name);
		put_user(virt, (unsigned long *)arg);
		MB_OPDBG("USER_TO_VERT: user %8lx virt %lx\n\n", value, virt);
		break;

	/* _IOWR(MB_IOC_MAGIC, 8, unsigned long)
	 *  I: user address
	 *  O: phys address */
	case MBIO_USER_TO_PHYS:
		get_user(value, (unsigned long *)arg);
		phys = mb_do_user_to_phys(value, tgid, mbti->task_name);
		put_user(phys, (unsigned long *)arg);
		MB_OPDBG("USER_TO_PHYS: user %8lx phys %lx\n\n", value, phys);
		break;

#ifdef CONFIG_WMT_MB_RESERVE_FROM_IO
	/* This IOCTRL is provide AP to recognize MB region as framebuffer,
	then can access mb memory directly. */
	case MBIO_FSCREENINFO:
	{
		struct fb_fix_screeninfo ffs;

		if (!wmt_mbah || list_empty(&wmt_mbah->mba_list)) {
			MB_WARN("MBIO_FSCREENINFO: MB driver does not init.\n");
			return -EFAULT;
		}

		mba = (struct mb_area_struct *)wmt_mbah->mba_list.next;
		ffs.smem_start = mba->phys;
		ffs.smem_len = wmt_mbah->tot_static_pages * PAGE_SIZE;
		if (copy_to_user((void __user *)arg, &ffs, sizeof(ffs)))
			ret = -EFAULT;
		MB_OPDBG("FSCREENINFO: phys %lx len %x\n\n",
			ffs.smem_start, ffs.smem_len);
		break;
	}
#endif

	/* _IOW (MB_IOC_MAGIC, 9, unsigned long)
	 *  I: size */
	case MBIO_PREFETCH:
	{
		unsigned long fetch_size = 0;
		get_user(value, (unsigned long *)arg);
#ifdef CONFIG_WMT_MB_DYNAMIC_ALLOCATE_SUPPORT
{
		unsigned long try_size = 0;

		MB_OPDBG("PREFETCH: size %ld KB\n\n", value/1024);
		spin_lock_irqsave(&mb_ioctl_lock, flags);
		while (try_size < value) {
			int order;
			order = get_order(value-fetch_size);
			if (order > MBMAX_ORDER)
				order = MBMAX_ORDER;
			try_size += (1 << order) * PAGE_SIZE;
			mba = mb_alloc_mba(1 << order);
			if (mba) {
				mba->flags |= MBAFLAG_STATIC;
				mba->tgid = current->tgid;
				wmt_mbah->tot_static_pages += mba->pages;
				fetch_size += mba->pages * PAGE_SIZE;
			}
		}
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		MB_INFO("PREFETCH: SIZE %ld / %ld kB by %s\n",
			fetch_size/1024, value/1024, current->comm);
}
#else
		MB_WARN("PREFETCH: SIZE %ld / %ld kB by %s fail\n",
			fetch_size/1024, value/1024, current->comm);
		MB_WARN("PREFETCH: Dynamic Allocated Not Support\n");
		ret = -ENOTTY;
#endif
		break;
	}

	/* _IOW (MB_IOC_MAGIC, 10, unsigned long)
	 *  O: static mba size */
	case MBIO_STATIC_SIZE:
		put_user(wmt_mbah->tot_static_pages * PAGE_SIZE,
			(unsigned long *)arg);
		MB_OPDBG("STATIC_SIZE: size %ld KB\n\n",
			PAGE_KB(wmt_mbah->tot_static_pages));
		break;

	/* _IOWR(MB_IOC_MAGIC,11, unsigned long)
	 *  I: physical address
	 *  O: use counter */
	case MBIO_MB_USER_COUNT:
		get_user(value, (unsigned long *)arg);

		spin_lock_irqsave(&mb_ioctl_lock, flags);
		mb = mb_find_mb((void *)value, MBFIND_PHYS);
		if (mb == NULL)
			size = 0;
		else
			size = MB_COUNT(mb);
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);

		put_user(size, (unsigned long *)arg);
		MB_OPDBG("USER_COUNT: addr %lx count %ld\n\n", value, size);
		break;

	/* _IO  (MB_IOC_MAGIC,12) */
	case MBIO_FORCE_RESET:
		spin_lock_irqsave(&mb_ioctl_lock, flags);
RESCAN_MBA:
		list_loop(mba, &wmt_mbah->mba_list, mba_list) {
			list_loop(mb, &mba->mb_list, mb_list) {
				list_loop(
					mbu, &mb->mbu_list, mbu_list) {
					list_del(&mbu->mbu_list);
					kmem_cache_free(
						wmt_mbah->mbu_cachep, mbu);
				}
				mb->creator = NULL;
				atomic_set(&mb->count, 0);
				mb_free_mb(mb);
				/* because mba link maybe breaken */
				goto RESCAN_MBA;
			}
		}
		mb_update_mbah();
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		MB_OPDBG("RESET: OK\n\n");
		break;

	/* _IOW (MB_IOC_MAGIC,13, unsigned long) // I: phys address */
	case MBIO_GET_BY_PHYS:
		get_user(value, (unsigned long *)arg);
		if (!value || !mbti->task_name || tgid == MB_DEF_TGID) {
			MB_WARN("GET(phys): invalid args %lx/%d/%s\n",
				value, tgid, mbti->task_name);
			return -EINVAL;
		}

		mbu = kmem_cache_alloc(wmt_mbah->mbu_cachep, GFP_ATOMIC);
		if (!mbu) {
			MB_WARN("GET(phys): mbu_cachep out of memory\n");
			return -ENOMEM;
		}

		spin_lock_irqsave(&mb_do_lock, flags);
		mb = mb_find_mb((void *)value, MBFIND_PHYS);
		if (!mb) {
			spin_unlock_irqrestore(&mb_do_lock, flags);
			kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
			MB_WARN("GET(phys): unknown phys %8lx name %s\n",
				value, mbti->task_name);
			return -EFAULT;
		}

		memset(mbu, 0x0, sizeof(struct mb_user));
		INIT_LIST_HEAD(&mbu->mbu_list);
		mbu->addr = value;
		mbu->mb = mb;
		mbu->tgid = tgid;
		len = strlen(mbti->task_name);
		if (len > TASK_COMM_LEN)
			len = TASK_COMM_LEN;
		strncpy(mbu->the_user, mbti->task_name, len);
		mbu->the_user[len] = 0;
		atomic_inc(&mb->count);
		list_add_tail(&mbu->mbu_list, &mb->mbu_list);
		spin_unlock_irqrestore(&mb_do_lock, flags);
		MB_OPDBG("GET(phys): phys %8lx cnt %x\n\n",
			value, MB_COUNT(mb));
		break;

	/* _IOW (MB_IOC_MAGIC,14, unsigned long) // I: phys address */
	case MBIO_PUT_BY_PHYS:
		get_user(value, (unsigned long *)arg);
		if (!value || !mbti->task_name || tgid == MB_DEF_TGID) {
			MB_WARN("PUT(phys): invalid args %lx/%d/%s\n",
				value, tgid, mbti->task_name);
			return -EINVAL;
		}

		spin_lock_irqsave(&mb_do_lock, flags);
		mbu = mb_find_mbu((void *)value, tgid,
			MBUFIND_PHYS|MBUFIND_GETPUT);
		if (!mbu) {
			spin_unlock_irqrestore(&mb_do_lock, flags);
			MB_WARN("PUT(phys): unknow phys %8lx name %s\n",
				value, mbti->task_name);
			return -EINVAL;
		}

		mb = mbu->mb;
		if (!MB_IN_USE(mb)) {
			spin_unlock_irqrestore(&mb_do_lock, flags);
			MB_WARN("PUT(phys): phys %8lx unbalance.\n", value);
			return -EPERM;
		}

		list_del_init(&mbu->mbu_list);
		atomic_dec(&mb->count);
		size = MB_COUNT(mb);
		/* retrieve unused block */
		if (!size)
			mb_free_mb(mb);

		spin_unlock_irqrestore(&mb_do_lock, flags);
		kmem_cache_free(wmt_mbah->mbu_cachep, mbu);

		MB_OPDBG("PUT(phys): phys %8lx cnt %lx\n\n", value, size);

		break;

	/* _IOW (MB_IOC_MAGIC,15, unsigned long) // I: user address */
	case MBIO_SYNC_CACHE:
		get_user(value, (unsigned long *)arg);
		spin_lock_irqsave(&mb_ioctl_lock, flags);
		mbu = mb_find_mbu((void *)value, tgid,
			MBUFIND_USER|MBUFIND_MMAP);
		if (mbu == NULL) {
			spin_unlock_irqrestore(&mb_ioctl_lock, flags);
			MB_WARN("SYNC_CACHE: invalid addr %lx\n\n", value);
			return -EFAULT;
		}
		mb = mbu->mb;
		size = mbu->size;
		dmac_flush_range(mb->virt, mb->virt + size);
		outer_flush_range(mb->phys, mb->phys + size);
		MB_OPDBG("SYNC_CACHE: (%lx # %lx) V %p ~ %p P %lx ~ %lx\n\n",
			value, size, mb->virt, mb->virt + size,
			mb->phys, mb->phys + size);
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		return 0;

	/* _IOW(MB_IOC_MAGIC, 19, unsigned long) // I: phys address */
	case MBIO_STATIC_GET:
		get_user(value, (unsigned long *)arg);
		if (!value || !mbti->task_name || tgid == MB_DEF_TGID) {
			MB_WARN("GET(STATIC): invalid args %lx/%d/%s\n",
				value, tgid, mbti->task_name);
			return -EINVAL;
		}

		ret = mb_do_get(value, MB_DEF_TGID, mbti->task_name);
		if (ret != 0)
			MB_WARN("GET(STATIC): fail (args %lx/%d/%s). ret %d\n",
				value, tgid, mbti->task_name, ret);
		MB_OPDBG("GET(STATIC): phys %8lx done\n\n", value);
		break;

	/* _IOW(MB_IOC_MAGIC, 20, unsigned long) // I: phys address */
	case MBIO_STATIC_PUT:
		get_user(value, (unsigned long *)arg);
		if (!value || !mbti->task_name || tgid == MB_DEF_TGID) {
			MB_WARN("PUT(STATIC): invalid args %lx/%d/%s\n",
				value, tgid, mbti->task_name);
			return -EINVAL;
		}

		ret = mb_do_put(value, MB_DEF_TGID, mbti->task_name);
		if (ret != 0)
			MB_WARN("PUT(STATIC): fail (args %lx/%d/%s). ret %d\n",
				value, tgid, mbti->task_name, ret);
		MB_OPDBG("PUT(STATIC): phys %8lx done\n\n", value);
		break;

	/* _IOWR(MB_IOC_MAGIC, 100, unsigned long)
	   I: user address O: physical address */
	case MBIO_TEST_USER2PHYS:
		get_user(value, (unsigned long *)arg);
		if (!value) {
			MB_WARN("USER2PHYS: invalid args %lx/%d/%s\n",
				value, tgid, mbti->task_name);
			return -EINVAL;
		}
		phys = user_to_phys(value);
		put_user(phys, (unsigned long *)arg);
		if (!phys)
			MB_WARN("USER2PHYS fail: value %#lx phys %#lx\n", value, phys);
		break;

	/* _IOWR(MB_IOC_MAGIC, 101, unsigned long)
	   I: user address, size, prdt addr, and prdt size */
	case MBIO_TEST_USER2PRDT:
	/* _IOW(MB_IOC_MAGIC, 103, unsigned long)
	*  I: user address, size, prdt addr, and prdt size */
	case MBIO_TEST_MB2PRDT: {
		int items, type = (cmd == MBIO_TEST_MB2PRDT) ? 0 : 1;
		unsigned long size,  data[4] = {0};
		struct prdt_struct *prdt;
		copy_from_user(data, (const void *)arg, 4 * sizeof(unsigned long));
		if (!data[0] || !data[1] || !data[2] || !data[3]) {
			MB_WARN("%s: invalid args [%#lx, %#lx, %#lx, %#lx]\n",
				type ? "MB2PRDT" : "USER2PRDT",
				data[0], data[1], data[2], data[3]);
			return -EINVAL;
		}
		items = data[1] / PAGE_SIZE + 2;
		size = items * sizeof(struct prdt_struct);
		if (data[3] < size) {
			MB_WARN("%s: invalid args. prdt size %#lx < %#lx\n",
				type ? "MB2PRDT" : "USER2PRDT", data[3], size);
			return -EINVAL;
		}
		prdt = kmalloc(size, GFP_ATOMIC);
		if (prdt == NULL) {
			MB_WARN("%s: fail, no prdt space %d for size %ld\n",
				type ? "MB2PRDT" : "USER2PRDT",
				items * sizeof(struct prdt_struct), data[1]);
			return -EINVAL;
		}
		MB_DBG("%s: IN %#lx, %#lx, %#lx, %#lx\n",
			type ? "MB2PRDT" : "USER2PRDT",
			data[0], data[1], data[2], data[3]);
		memset(prdt, 0x0, items * sizeof(struct prdt_struct));
		if ((!type && !mb_to_prdt(data[0], data[1], prdt, items)) ||
			(!user_to_prdt(data[0], data[1], prdt, items))) {
			if (MBMSG_LEVEL)
				show_prdt(prdt);
			copy_to_user((void __user *)data[2], prdt, size);
			MB_INFO("%s: Done %#lx, %#lx, %#lx, %#lx\n",
				type ? "MB2PRDT" : "USER2PRDT",
				data[0], data[1], data[2], data[3]);
		} else {
			MB_WARN("%s: fail.\n", type ? "MB2PRDT" : "USER2PRDT");
			ret = -EINVAL;
		}
		kfree(prdt);
		break;
	}

	/* _IOWR(MB_IOC_MAGIC, 102, unsigned long)
	   I: user address, size, mmu addr, and mmu size O: mmu physical addr */
	case MBIO_TEST_USER2MMUT:
	/* _IOWR(MB_IOC_MAGIC, 104, unsigned long)
	   I: user address, size, mmu addr, and mmu size O: mmu physical addr */
	case MBIO_TEST_MB2MMUT: {
		int type = (cmd == MBIO_TEST_MB2MMUT) ? 0 : 1;
		unsigned long mmuSize, data[4] = {0};
		struct mmu_table_info info = {0};
		copy_from_user(data, (const void *)arg, 4 * sizeof(unsigned long));
		if (!data[0] || !data[1] || !data[2] || !data[3]) {
			MB_WARN("%s: invalid args [%#lx, %#lx, %#lx, %#lx]\n",
				type ? "USER2MMUT" : "PHYS2MMUT",
				data[0], data[1], data[2], data[3]);
			return -EINVAL;
		}
		mmuSize = wmt_mmu_table_size(data[1]);
		if (data[3] < mmuSize) {
			MB_WARN("%s: invalid args. mmu size %#lx < %#lx\n",
				type ? "MB2PRDT" : "USER2PRDT", data[3], mmuSize);
			return -EINVAL;
		}
		MB_DBG("%s: IN %#lx, %#lx, %#lx, %#lx(req %#lx)\n",
			type ? "USER2MMUT" : "PHYS2MMUT",
			data[0], data[1], data[2], data[3], mmuSize);
		if (wmt_mmu_table_create(data[0], data[1], type, &info) == 0) {
			void *mmuAddr = mb_phys_to_virt((unsigned long)info.addr);
			if (MBMSG_LEVEL)
				wmt_mmu_table_dump(&info);
			copy_to_user((void __user *)data[2], mmuAddr, mmuSize);
			put_user(info.addr, (unsigned long *)arg);
			ret = mb_do_get(info.addr, MB_DEF_TGID, mbti->task_name);
			wmt_mmu_table_destroy(&info);
			MB_INFO("%s: Done %#lx, %#lx, %#lx, %#lx\n",
				type ? "USER2MMUT" : "PHYS2MMUT",
				data[0], data[1], data[2], data[3]);
		} else {
			MB_WARN("%s: fail. no mmu table space for size %ld\n",
				type ? "USER2MMUT" : "PHYS2MMUT", data[1]);
			return -EINVAL;
		}
		break;
	}

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static int mb_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct mb_user *mbu = NULL;
	struct mb_task_info *mbti;
	struct mb_struct *mb;
	unsigned long off, flags;
	unsigned int len;
	pid_t tgid;
	size_t ns;

	if (!filp || !filp->private_data) {
		MB_ERROR("mmap with null file info or task.\n");
		return -EFAULT;
	}

	mbti = (struct mb_task_info *)(filp->private_data);
	tgid = mbti->tgid;
	if (tgid != current->tgid) {
		MB_WARN("mmap within diff task.\n");
		MB_WARN("Open: task %s TGID %x VS Curr: task %s TGID %x\n",
			mbti->task_name, tgid, current->comm, current->tgid);
	}

	off = vma->vm_pgoff << PAGE_SHIFT;
	MB_DBG("IN, offset %lx TGID %x name %s\n", off, tgid, mbti->task_name);
#ifdef CONFIG_WMT_MB_RESERVE_FROM_IO
	/* if page offset under MB total size, it means page offset
	 * is not physical address from MB_alloc and then map function
	 * works like framebuffer.
	 * Suppose MB total size will not cross MBAH phys start address */
	if (vma->vm_pgoff <= wmt_mbah->tot_static_pages) {
		struct mb_area_struct *mba = NULL;
		unsigned long mbtotal =
			wmt_mbah->tot_static_pages << PAGE_SHIFT;
		if ((vma->vm_end - vma->vm_start + off) > mbtotal) {
			MB_ERROR("mmap io fail. range %lx @ [%lx, %lx] s %lx\n",
				off, vma->vm_start, vma->vm_end, mbtotal);
			return -EINVAL;
		}
		mba = (struct mb_area_struct *)wmt_mbah->mba_list.next;
		vma->vm_pgoff += mba->phys >> PAGE_SHIFT;
		vma->vm_flags |= VM_IO | VM_RESERVED;
		if (io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
			MB_ERROR("mmap io fail. (io_remap pfn %lx len %lx)\n",
				vma->vm_pgoff, vma->vm_end - vma->vm_start);
			return -EAGAIN;
		}
		MB_OPDBG("MAP: IO from %lx to %lx/%lx size %ld\n\n",
			vma->vm_pgoff << PAGE_SHIFT,
			vma->vm_start, vma->vm_end,
			vma->vm_end - vma->vm_start);
		return 0;
	}
#endif

	mbu = kmem_cache_alloc(wmt_mbah->mbu_cachep, GFP_ATOMIC);
	if (!mbu) {
		MB_ERROR("mbu_cachep out of memory.\n");
		return -ENOMEM;
	}

	spin_lock_irqsave(&mb_ioctl_lock, flags);
	mb = mb_find_mb((void *)off, MBFIND_PHYS);
	if (!mb) {
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
		MB_ERROR("map addr %lx not exist in MB.\n", off);
		return -EINVAL;
	}

	len = PAGE_ALIGN(mb->phys + mb->size);
	if ((vma->vm_end - vma->vm_start + off) > len) {
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
		return -EINVAL;
	}

	memset(mbu, 0x0, sizeof(struct mb_user));
	INIT_LIST_HEAD(&mbu->mbu_list);
	mbu->tgid = tgid;
	mbu->mb = mb;
	mbu->addr = vma->vm_start;
	mbu->size = vma->vm_end - vma->vm_start;
	ns = strlen(mbti->task_name);
	if (ns > TASK_COMM_LEN)
		ns = TASK_COMM_LEN;
	strncpy(mbu->the_user, mbti->task_name, ns);
	mbu->the_user[ns] = 0;
	list_add_tail(&mbu->mbu_list, &mb->mbu_list);
	atomic_inc(&mb->count);

	off = mb->phys;
	spin_unlock_irqrestore(&mb_ioctl_lock, flags);

	vma->vm_flags |= VM_IO | VM_RESERVED;
	if (!(mb->flags & MBFLAG_CACHED))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		spin_lock_irqsave(&mb_ioctl_lock, flags);
		list_del(&mbu->mbu_list);
		atomic_dec(&mb->count);
		kmem_cache_free(wmt_mbah->mbu_cachep, mbu);
		spin_unlock_irqrestore(&mb_ioctl_lock, flags);
		MB_ERROR("virtual address memory map fail.\n");
		return -EAGAIN;
	}

	MB_OPDBG("MAP: IO from %lx to %lx/%lx size %ld mbcnt %d\n\n",
		off, vma->vm_start, vma->vm_end,
		(vma->vm_end - vma->vm_start), MB_COUNT(mb));

	return 0;
}

/*!*************************************************************************
    driver file operations struct define
****************************************************************************/
const struct file_operations mb_fops = {
	.owner = THIS_MODULE,
	.open = mb_open,
	.release = mb_release,
	.unlocked_ioctl = mb_ioctl,
	.mmap = mb_mmap,
};

static int mb_probe(struct platform_device *dev)
{
	struct mb_area_struct *mba = NULL, *bind = NULL;
	unsigned long flags;
	int ret = 0;
	dev_t dev_no;

	dev_no = MKDEV(mb_dev_major, mb_dev_minor);

	if (wmt_mbah) {
		MB_ERROR("%s dirty.\n", DEVICE_NAME);
		return -EINVAL;
	}

	/* register char device */
	mb_cdev = cdev_alloc();
	if (!mb_cdev) {
		MB_ERROR("alloc dev error.\n");
		return -ENOMEM;
	}

	cdev_init(mb_cdev, &mb_fops);
	ret = cdev_add(mb_cdev, dev_no, 1);

	if (ret) {
		MB_ERROR("reg char dev error(%d).\n", ret);
		cdev_del(mb_cdev);
		return ret;
	}

	wmt_mbah = kmalloc(sizeof(struct mba_host_struct), GFP_ATOMIC);
	if (!wmt_mbah) {
		MB_ERROR("out of memory (mb_area_struct).\n");
		cdev_del(mb_cdev);
		return -ENOMEM;
	}
	memset(wmt_mbah, 0x0, sizeof(struct mba_host_struct));

	wmt_mbah->mba_cachep = kmem_cache_create("mb_area_struct",
		sizeof(struct mb_area_struct),
		0,
		SLAB_HWCACHE_ALIGN,
		NULL);
	if (!wmt_mbah->mba_cachep) {
		MB_ERROR("out of memory (mba_cachep).\n");
		kfree(wmt_mbah);
		cdev_del(mb_cdev);
		return -ENOMEM;
	}

	wmt_mbah->mb_cachep = kmem_cache_create("mb_struct",
		sizeof(struct mb_struct),
		0,
		SLAB_HWCACHE_ALIGN,
		NULL);
	if (!wmt_mbah->mb_cachep) {
		MB_ERROR("out of memory (mb_cachep).\n");
		kmem_cache_destroy(wmt_mbah->mba_cachep);
		kfree(wmt_mbah);
		cdev_del(mb_cdev);
		return -ENOMEM;
	}

	wmt_mbah->mbu_cachep = kmem_cache_create("mb_user",
		sizeof(struct mb_user),
		0,
		SLAB_HWCACHE_ALIGN,
		NULL);
	if (!wmt_mbah->mbu_cachep) {
		MB_ERROR("out of memory (mbu_cachep).\n");
		kmem_cache_destroy(wmt_mbah->mb_cachep);
		kmem_cache_destroy(wmt_mbah->mba_cachep);
		kfree(wmt_mbah);
		cdev_del(mb_cdev);
		return -ENOMEM;
	}

	wmt_mbah->mbti_cachep = kmem_cache_create("mb_task_info",
		sizeof(struct mb_task_info),
		0,
		SLAB_HWCACHE_ALIGN,
		NULL);
	if (!wmt_mbah->mbti_cachep) {
		MB_ERROR("out of memory (mbti_cachep).\n");
		kmem_cache_destroy(wmt_mbah->mbu_cachep);
		kmem_cache_destroy(wmt_mbah->mb_cachep);
		kmem_cache_destroy(wmt_mbah->mba_cachep);
		kfree(wmt_mbah);
		cdev_del(mb_cdev);
		return -ENOMEM;
	}

	MBMAX_ORDER = MBA_MAX_ORDER;
	MBMIN_ORDER = MBA_MIN_ORDER;
	INIT_LIST_HEAD(&wmt_mbah->mba_list);
	INIT_LIST_HEAD(&wmt_mbti.mbti_list);

	spin_lock_init(&mb_do_lock);
	spin_lock_init(&mb_search_lock);
	spin_lock_init(&mb_ioctl_lock);
	spin_lock_init(&mb_task_mm_lock);
	spin_lock_init(&mb_task_lock);

	MB_INFO("Preparing VIDEO BUFFER (SIZE %d kB) ...\n", MB_TOTAL_SIZE);
	spin_lock_irqsave(&mb_do_lock, flags);
	while (PAGE_KB(wmt_mbah->tot_static_pages) < MB_TOTAL_SIZE) {
		int pages = MB_TOTAL_SIZE*1024 -
			wmt_mbah->tot_static_pages * PAGE_SIZE;
		pages = min((pages >> PAGE_SHIFT), (int)(1 << MBMAX_ORDER));
		mba = mb_alloc_mba(pages);
		if (mba) {
			mba->flags |= MBAFLAG_STATIC;
			mba->tgid = MB_DEF_TGID;
			wmt_mbah->tot_static_pages += mba->pages;
			/* conbine to continue mba if possible */
			if (bind && bind->pgi.pfn_end == mba->pgi.pfn_start) {
				MB_COMBIND_MBA(bind, mba);
				continue;
			}
			if (bind && bind->pgi.pfn_start == mba->pgi.pfn_end)
				MB_COMBIND_MBA(mba, bind);
			bind = mba;
		}
	}
	spin_unlock_irqrestore(&mb_do_lock, flags);

	mb_update_mbah();
	MB_INFO("MB version: %d.%d.%d.%d\n",
		MB_VERSION_MAJOR, MB_VERSION_MINOR,
		MB_VERSION_MICRO, MB_VERSION_BUILD);
	MB_INFO("MAX MB Area size: Max %ld Kb Min %ld Kb\n",
		PAGE_KB(1 << MBMAX_ORDER), PAGE_KB(1 << MBMIN_ORDER));
	MB_INFO("prob /dev/%s major %d, minor %d\n",
		DEVICE_NAME, mb_dev_major, mb_dev_minor);

	return ret;
}

static int mb_remove(struct platform_device *dev)
{
	if (mb_cdev)
		cdev_del(mb_cdev);

	if (wmt_mbah) {
		if (wmt_mbah->mba_cachep)
			kmem_cache_destroy(wmt_mbah->mba_cachep);
		if (wmt_mbah->mb_cachep)
			kmem_cache_destroy(wmt_mbah->mb_cachep);
		if (wmt_mbah->mbu_cachep)
			kmem_cache_destroy(wmt_mbah->mbu_cachep);
		if (wmt_mbah->mbti_cachep)
			kmem_cache_destroy(wmt_mbah->mbti_cachep);
		kfree(wmt_mbah);
	}

	MB_INFO("MB dev remove\n");
	return 0;
}

static int mb_suspend(struct platform_device *dev, pm_message_t state)
{
	MB_DBG("MB get suspend, Event %d.\n", state.event);
	return 0;
}

static int mb_resume(struct platform_device *dev)
{
	MB_DBG("MB get resume.\n");
	return 0;
}

int mb_area_read_proc(
	char *buf, char **start, off_t off,
	int count, int *eof, void *data
)
{
	struct mb_area_struct *mba;
	struct free_area *fa;
	struct zone *zone;
	unsigned long flags;
	unsigned int idx = 1;
	char *p = buf, *base = (char *)data;
	int datalen = 0, len;

	if (!data || !count) {
		p += sprintf(p, "no resource for mb_area read proc. (%p,%d)\n",
			data, count);
		return p-buf;
	}

	if (!wmt_mbah) {
		p += sprintf(p, "no MB area host existed.\n");
		return p-buf;
	}

	spin_lock_irqsave(&mb_ioctl_lock, flags);
	if (!off) { /* re read mb area information */
		p = base;
		memset(p, 0x0, MBPROC_BUFSIZE);

		/* show block information */
		p += sprintf(p, "VERSION:                  %3d.%3d.%3d.%3d\n",
			MB_VERSION_MAJOR, MB_VERSION_MINOR,
			MB_VERSION_MICRO, MB_VERSION_BUILD);
		p += sprintf(p, "MESSAGE LEVEL:       %8d   /%8lx\n",
			MBMSG_LEVEL, __pa(&MBMSG_LEVEL));
		p += sprintf(p, "STATIC MB SIZE:   %8ld MB   /%5d MB\n",
			PAGE_KB(wmt_mbah->tot_static_pages)/1024,
			MB_TOTAL_SIZE/1024);
		p += sprintf(p, "MAX MBA ORDER:       %8d   /%8lx\n",
			MBMAX_ORDER, __pa(&MBMAX_ORDER));
		p += sprintf(p, "MIN MBA ORDER:       %8d   /%8lx\n\n",
			MBMIN_ORDER, __pa(&MBMIN_ORDER));
		p += sprintf(p, "USER TO PRDT METHOD: %8d   /%8lx\n\n",
			USR2PRDT_METHOD, __pa(&USR2PRDT_METHOD));
		p += sprintf(p, "total MB areas:      %8d\n",
			wmt_mbah->nr_mba);
		p += sprintf(p, "total size:          %8ld kB\n",
			PAGE_KB(wmt_mbah->tot_pages));
		p += sprintf(p, "total free size:     %8ld kB\n",
			PAGE_KB(wmt_mbah->tot_free_pages));
		p += sprintf(p, "max MB size:         %8ld kB\n\n",
			PAGE_KB(wmt_mbah->max_available_pages));

		list_loop(mba, &wmt_mbah->mba_list, mba_list) {
			p += sprintf(p, "(ID) [MB Area] VirtAddr PhysAddr  ");
			p += sprintf(p, "size [   zs,   ze] MBs      Max/");
			p += sprintf(p, "    Free Ste\n");
			len = (int)(p-base);
			if ((MBPROC_BUFSIZE - len) < 12) {
				p += sprintf(p, " more ...\n");
				break;
			}
			p += sprintf(p, "(%2d)", idx++);
			len = (int)(p-base);
			/* show all MBs */
			/* -2 is for \n and zero */
			p += mb_show_mba(mba, NULL, p,
				MBPROC_BUFSIZE - len - 2, 1);
			p += sprintf(p, "\n");
		}
		/* show memory fragment */
		zone = (first_online_pgdat())->node_zones;
		fa = zone->free_area;
		p += sprintf(p, "DMA ZONE:\n");
		for (idx = 0; idx < MAX_ORDER; idx++)
			p += sprintf(p, " %5ld * %5ldkB = %5ld kB\n",
				fa[idx].nr_free, PAGE_KB((1<<idx)),
				fa[idx].nr_free * PAGE_KB((1<<idx)));
		p += sprintf(p, " ------------------------------\n");
		p += sprintf(p, "               + = %ld kB\n\n",
			PAGE_KB((unsigned long)nr_free_pages()));
	}
	spin_unlock_irqrestore(&mb_ioctl_lock, flags);
	datalen = strlen(base);
	if (off >= datalen) {
		*eof = 1;
		return 0;
	}
	len = min((int)(datalen - off), count);
	memcpy(buf, &base[off], len);
	*start = (char *)len; /* for case1: *start < page, mass read data */

	return len;
}

static struct platform_driver mb_driver = {
	.driver.name    = "wmt_mb",
	.probe          = mb_probe,
	.remove         = mb_remove,
	.suspend        = mb_suspend,
	.resume         = mb_resume
};

static void mb_platform_release(struct device *device)
{
	return;
}

static struct platform_device mb_device = {
	.name          = "wmt_mb",
	.id            = 0,
	.dev           = { .release =  mb_platform_release, },
	.num_resources = 0,
	.resource      = NULL,
};

static int __init mb_init(void)
{
	int varlen = 32, ret = -1, mbtotal = 0;
	unsigned char buf[32];
	dev_t dev_no;

	mb_dev_major = MB_MAJOR;
	mb_dev_minor = 0;
	mb_dev_nr = 1;

	dev_no = MKDEV(mb_dev_major, mb_dev_minor);
	ret = register_chrdev_region(dev_no, mb_dev_nr, "wmt_mb");
	if (ret < 0) {
		MB_ERROR("can't get %s device major %d\n",
			DEVICE_NAME, mb_dev_major);
		return ret;
	}

	if (wmt_getsyspara("mbsize", buf, &varlen) == 0) {
		sscanf(buf, "%dM", &mbtotal);
		mbtotal *= 1024;
		MB_INFO("Set MB total size %d KB\n", mbtotal);
		if (mbtotal > 0)
			MB_TOTAL_SIZE = mbtotal;
	}

	wmt_mbah = NULL;
	MBMSG_LEVEL = MBMAX_ORDER = MBMIN_ORDER = USR2PRDT_METHOD = 0;
	mb_dev_class = class_create(THIS_MODULE, "mbdev");
	device_create(mb_dev_class, NULL, dev_no, NULL, "mbdev");

#ifdef CONFIG_PROC_FS
	{
		void *proc_buffer = kmalloc(MBPROC_BUFSIZE, GFP_KERNEL);
		struct proc_dir_entry *res = NULL;
		if (proc_buffer) {
			memset(proc_buffer, 0x0, MBPROC_BUFSIZE);
			res = create_proc_entry("mbinfo", 0, NULL);
			if (res) {
				res->read_proc = mb_area_read_proc;
				res->data = proc_buffer;
				MB_DBG("create MB proc\n");
			} else
				kfree(proc_buffer);
		}
	}
#endif

	ret = platform_driver_register(&mb_driver);
	if (!ret) {
		ret = platform_device_register(&mb_device);
		if (ret)
			platform_driver_unregister(&mb_driver);
	}

	return ret;
}

/* because mb will not exit from system, not need to release proc resource */
static void __exit mb_exit(void)
{
	dev_t dev_no;

	platform_driver_unregister(&mb_driver);
	platform_device_unregister(&mb_device);
	dev_no = MKDEV(mb_dev_major, mb_dev_minor);
	device_destroy(mb_dev_class, dev_no);
	class_destroy(mb_dev_class);
	unregister_chrdev_region(dev_no, mb_dev_nr);

	return;
}

fs_initcall(mb_init);
module_exit(mb_exit);

MODULE_AUTHOR("WonderMedia Technologies, Inc.");
MODULE_DESCRIPTION("memory block driver");
MODULE_LICENSE("GPL");
