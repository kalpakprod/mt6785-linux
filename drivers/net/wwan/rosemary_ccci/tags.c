// SPDX-License-Identifier: GPL-2.0-only
/*
 * Poco M5s (rosemary): the bootloader's CCCI boot information.
 *
 * The memory layout of the modem link is not compiled into the kernel: the
 * bootloader works it out at every boot, reserves the regions and writes a tag
 * list at 0xbff70000 (region ccci_tag_mem), pointed at by the kernel device
 * tree's /chosen/ccci,modem_info_v2. The list carries 23 named tags: the
 * modem's own DRAM window, the share-memory layout, the CCB logging buffers,
 * the modem's MPU windows, the image type and much more. Every one of them is
 * read here and nothing is written.
 *
 * Two rules this file exists to honour:
 *  - The reserved regions are found by NAME, never by address: the loader's
 *    node names carry a boot-dependent index (mblock-24-... in one boot,
 *    mblock-26-... in the next), and the modem's DRAM window moves between
 *    boots (0xac000000 in the stock dump, 0xa2000000 in the port boot).
 *  - A tag is refused when it is short, and the walk is bounded both by the
 *    tag count and by the size the loader recorded, so a damaged list cannot
 *    walk the driver off into unreadable memory.
 *
 * The tag record layout and the meaning of each tag come from the vendor
 * kernel's GPL source: drivers/misc/mediatek/ccci_util/ccci_util_lib_fo.c
 * (struct _ccci_lk_info_v2, struct _ccci_tag_v2, find_ccci_tag_inf,
 * share_memory_info_parsing, md_mem_info_parsing, md_chk_hdr_info_parse) in
 * LineageOS android_kernel_xiaomi_mt6785 30485853fa89.
 */
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "rosemary_ccci.h"

/* struct _ccci_lk_info_v2 (ccci_util_lib_fo.c:63) */
#define LK_INFO_BASE		0x00
#define LK_INFO_SIZE		0x08
#define LK_INFO_ERR_NO		0x0c
#define LK_INFO_VERSION		0x10
#define LK_INFO_TAG_NUM		0x14
#define LK_INFO_LD_FLAG		0x18
#define LK_INFO_LD_MD_ERRNO	0x1c

/* struct _ccci_tag_v2: char name[64]; u32 data_offset, data_size, next */
#define LK_TAG_V2_HDR_SIZE	76
#define LK_TAG_NAME_LEN_V2	64

/* the loader's own tag count and version on this phone */
#define LK_VERSION_V2		2
#define LK_MAX_TAGS		64
/* MAX_LK_INFO_SIZE in ccci_util_lib_fo.c: the loader's block is 64 KiB */
#define LK_MAX_INFO_SIZE	0x10000

/* image_type sits at this offset in every check-header version (v3..v6) */
#define MD_CHK_IMAGE_TYPE	20

/*
 * World-mode capability bitmap (ccci_util_md_rat.c:14-23): bit0 G, bit1 T,
 * bit2 C, bit3 W, bit4 Lt, bit5 Lf, bit6 N. The phone's own vendor config
 * asks for ro.vendor.mtk_md1_support=9, i.e. ulwg = Lf|Lt|W|G = 0x39.
 */
#define MD_CAP_GSM		BIT(0)
#define MD_CAP_CDMA2000		BIT(2)
#define MD_CAP_WCDMA		BIT(3)
#define MD_CAP_TDD_LTE		BIT(4)
#define MD_CAP_FDD_LTE		BIT(5)
#define MD_CAP_NR		BIT(6)

/* what this board can actually do: G|T|W|Lt|Lf -- no cdma2000, no NR */
#define MD_CAP_BOARD_MASK	(MD_CAP_GSM | BIT(1) | MD_CAP_WCDMA | MD_CAP_TDD_LTE | \
				 MD_CAP_FDD_LTE)
/* the world-mode bitmap the vendor config asks for: ulwg */
#define MD_CAP_ULWG		(MD_CAP_GSM | MD_CAP_WCDMA | MD_CAP_TDD_LTE | \
				 MD_CAP_FDD_LTE)

/* md_img_capability_map[] (ccci_util_md_rat.c:33-86), indexed by image_type */
static const u32 md_img_cap_map[] = {
	0,
	MD_CAP_GSM,
	MD_CAP_WCDMA,
	MD_CAP_WCDMA | MD_CAP_GSM,
	MD_CAP_TDD_LTE | MD_CAP_GSM,
	MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_WCDMA | MD_CAP_GSM,
	MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_GSM | MD_CAP_CDMA2000,
	MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_GSM | MD_CAP_CDMA2000,
	MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_GSM | MD_CAP_CDMA2000,
	MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_WCDMA | MD_CAP_GSM,
	MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_WCDMA | MD_CAP_GSM |
		MD_CAP_CDMA2000,
	MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_WCDMA |
		MD_CAP_CDMA2000 | MD_CAP_GSM,
	MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_WCDMA |
		MD_CAP_CDMA2000 | MD_CAP_GSM,
	MD_CAP_NR | MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_WCDMA |
		MD_CAP_GSM,
	MD_CAP_NR | MD_CAP_FDD_LTE | MD_CAP_TDD_LTE | MD_CAP_WCDMA |
		MD_CAP_CDMA2000 | MD_CAP_GSM,
};

/*
 * The loader names its reservations "mblock-<index>-<name>", and the index
 * changes between boots, so a region is matched by the part after the second
 * dash. Anything else keeps its whole node name.
 */
static const char *region_basename(const char *nodename)
{
	const char *p = nodename;
	int i;

	for (i = 0; i < 2; i++) {
		p = strchr(p, '-');
		if (!p)
			return nodename;
		p++;
	}
	return p;
}

/**
 * rosemary_ccci_find_region() - look a loader reservation up by its own name
 * @md: the modem
 * @name: the name the loader gave the region, without the "mblock-<n>-" index
 *
 * Returns the reserved-memory node, or an ERR_PTR.
 */
static struct device_node *rosemary_ccci_find_region(struct ccci_md *md,
						    const char *name)
{
	struct device_node *rmem, *child;

	rmem = of_find_node_by_path("/reserved-memory");
	if (!rmem)
		return ERR_PTR(-ENOENT);
	for_each_child_of_node(rmem, child)
		if (!strcmp(region_basename(child->name), name)) {
			of_node_put(rmem);
			return child;
		}
	of_node_put(rmem);
	return ERR_PTR(-ENOENT);
}

static int region_lookup(struct ccci_md *md, const char *name,
			 phys_addr_t *base, resource_size_t *size)
{
	struct device_node *np;
	struct resource res;
	int ret;

	np = rosemary_ccci_find_region(md, name);
	if (IS_ERR(np))
		return PTR_ERR(np);
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return ret;
	*base = res.start;
	*size = resource_size(&res);
	return 0;
}

/* map the loader's tag list; it is no-map, so a plain remap is what is left */
static void __iomem *tags_map(struct ccci_md *md)
{
	if (md->tags)
		return md->tags;
	if (!md->tag_base || md->tag_size < LK_INFO_LD_MD_ERRNO + 16)
		return NULL;
	md->tags = devm_ioremap(md->dev, md->tag_base, md->tag_size);
	return md->tags;
}

/* copy a tag's payload out of the loader's block; -ENOENT, -ENOSPC or length */
static int tag_read(struct ccci_md *md, const char *name, void *dst,
		    size_t size)
{
	u32 off = 0;
	int i;

	for (i = 0; i < md->lk_tag_num; i++) {
		char tag_name[LK_TAG_NAME_LEN_V2];
		u32 data_off, data_size, next_off;

		if (off + LK_TAG_V2_HDR_SIZE > md->lk_size)
			return -ENOENT;
		memcpy_fromio(tag_name, md->tags + off, LK_TAG_NAME_LEN_V2);
		tag_name[sizeof(tag_name) - 1] = '\0';
		data_off = readl(md->tags + off + 64);
		data_size = readl(md->tags + off + 68);
		next_off = readl(md->tags + off + 72);
		if (!tag_name[0])
			return -ENOENT;
		if (data_size > md->lk_size || data_off > md->lk_size - data_size ||
		    next_off > md->lk_size)
			return -ENOENT;
		if (!strcmp(tag_name, name)) {
			size_t n = min(size, (size_t)data_size);

			memcpy_fromio(dst, md->tags + data_off, n);
			return n;
		}
		if (!next_off)
			return -ENOENT;
		off = next_off;
	}
	return -ENOENT;
}

static int tag_u32(struct ccci_md *md, const char *name, u32 *val)
{
	int ret = tag_read(md, name, val, sizeof(*val));

	if (ret != sizeof(*val))
		return ret < 0 ? ret : -ENOSPC;
	*val = le32_to_cpu(*val);
	return 0;
}

/*
 * struct _modem_info, one per modem: the MD's DRAM window. Parsed field by
 * field through the raw bytes so a padding difference cannot shift the
 * addresses.
 */
static void tags_read_md_mem(struct ccci_md *md)
{
	u8 buf[32];
	int ret;

	ret = tag_read(md, "hdr_tbl_inf", buf, sizeof(buf));
	if (ret < 16) {
		dev_err(md->dev, "no hdr_tbl_inf tag (%d)\n", ret);
		return;
	}
	md->md_mem_base = le64_to_cpu(get_unaligned_le64(buf));
	md->md_mem_size = le32_to_cpu(get_unaligned_le32(buf + 8));
	md->md_img_type = buf[14];
	dev_info(md->dev, "hdr_tbl_inf: md DRAM %#llx + %#x, md_id %u, errno %u, type %u\n",
		 md->md_mem_base, md->md_mem_size, buf[12], buf[13], buf[14]);
	if (buf[13])
		dev_warn(md->dev, "the loader reported MD image error %u\n",
			 buf[13]);
}

/* struct _smem_layout: base plus the AP<->MD1, AP<->MD3 and MD1<->MD3 windows */
static void tags_read_smem_layout(struct ccci_md *md)
{
	u8 buf[36];
	u64 base;
	u32 ap_md1_off, ap_md1_size, ap_md3_off, ap_md3_size, md1_md3_off,
	    md1_md3_size, total;
	int ret;

	ret = tag_read(md, "smem_layout", buf, sizeof(buf));
	if (ret < sizeof(buf)) {
		dev_err(md->dev, "no usable smem_layout tag (%d)\n", ret);
		return;
	}
	base = le64_to_cpu(get_unaligned_le64(buf));
	ap_md1_off = le32_to_cpu(get_unaligned_le32(buf + 8));
	ap_md1_size = le32_to_cpu(get_unaligned_le32(buf + 12));
	ap_md3_off = le32_to_cpu(get_unaligned_le32(buf + 16));
	ap_md3_size = le32_to_cpu(get_unaligned_le32(buf + 20));
	md1_md3_off = le32_to_cpu(get_unaligned_le32(buf + 24));
	md1_md3_size = le32_to_cpu(get_unaligned_le32(buf + 28));
	total = le32_to_cpu(get_unaligned_le32(buf + 32));

	md->smem_base = base;
	md->smem_size = ap_md1_size;
	dev_info(md->dev, "smem_layout: base %#llx, ap<->md1 %#x+%#x, ap<->md3 %#x+%#x, md1<->md3 %#x+%#x, total %#x\n",
		 base, ap_md1_off, ap_md1_size, ap_md3_off, ap_md3_size,
		 md1_md3_off, md1_md3_size, total);
}

/* struct _ccb_layout: where the modem's logging buffers live */
static void tags_read_ccb(struct ccci_md *md)
{
	u8 buf[12];
	int ret;

	ret = tag_read(md, "ccb_info", buf, sizeof(buf));
	if (ret < sizeof(buf)) {
		dev_err(md->dev, "no usable ccb_info tag (%d)\n", ret);
		return;
	}
	md->ccb_data_base = le64_to_cpu(get_unaligned_le64(buf));
	md->ccb_data_size = le32_to_cpu(get_unaligned_le32(buf + 8));
	dev_info(md->dev, "ccb_info: %#llx + %#x\n", md->ccb_data_base,
		 md->ccb_data_size);
}

/*
 * One entry of the loader's cacheable layout, struct _csmem_item
 * (ccci_util_lib_fo.c:380-386). Spelled with fixed-width types and left
 * unpacked, exactly as the loader writes it: u64 address, then md_offset,
 * csmem_buffer_size and item_cnt (the SMEM_USER_ID). The struct is not packed,
 * so its size is 24 with four bytes of tail padding -- taking the stride from
 * sizeof(*item) rather than adding the fields is what keeps entry n at the
 * right place.
 */
struct csmem_item {
	__le64 addr;
	__le32 md_offset;
	__le32 size;
	__le32 id;
};

#define CSMEM_ITEM_MAX		32

/**
 * tags_read_cacheable() - take the real cacheable layout from the loader
 * @md: the modem
 *
 * The cacheable window's sub-regions are NOT at fixed offsets: the loader
 * describes them in md1_bank4_cache_info (how many) and
 * md1_bank4_cache_layout (the array of struct _csmem_item, one per
 * SMEM_USER_ID). The vendor kernel reads them exactly so
 * (ccci_util_lib_fo.c cshare_memory_info_parsing() and
 * get_md_cache_region_info()), and this phone's own trial printed a layout
 * that no constant table would have guessed: md1_bank4_cache_info says
 * "buffer 0x8c000000 + 0x1780000, md offset 0xc000000, 5 items".
 *
 * Reading them here is what keeps the driver from writing outside the mapped
 * window: a constant table copied from another phone put the CCB buffers at
 * 0x4900000, i.e. 64 MiB into a 23.5 MiB window, and the first boot out ran
 * the mapping and oopsed.
 */
static void tags_read_cacheable(struct ccci_md *md)
{
	struct csmem_item *raw;
	u8 buf[sizeof(struct csmem_item)];
	u64 window_base;
	u32 window_size, items, i;
	int ret;

	/*
	 * The info tag is one struct _csmem_item too: address and size of the
	 * whole cacheable window, and how many descriptors the layout tag
	 * holds. Its size is what the vendor's get_md_resv_csmem_info()
	 * returns for the window.
	 */
	ret = tag_read(md, "md1_bank4_cache_info", buf, sizeof(buf));
	if (ret < sizeof(buf)) {
		dev_warn(md->dev, "no md1_bank4_cache_info tag (%d)\n", ret);
		return;
	}
	window_base = le64_to_cpu(get_unaligned_le64(buf));
	window_size = le32_to_cpu(get_unaligned_le32(buf + 12));
	items = le32_to_cpu(get_unaligned_le32(buf + 16));
	dev_info(md->dev, "md1_bank4_cache_info: buffer %#llx + %#x, %u items\n",
		 window_base, window_size, items);
	/*
	 * This is the loader's own statement of the window's extent: if it
	 * disagrees with the reservation the driver mapped, the mapped size is
	 * the smaller of the two and every write is checked against that.
	 */
	if (window_size && window_base == md->c_phys && window_size < md->c_size) {
		dev_info(md->dev, "the loader's cacheable window is %#x, the reservation %#llx: using the smaller\n",
			 window_size, (u64)md->c_size);
		md->c_size = window_size;
	}
	if (!items || items > CSMEM_ITEM_MAX) {
		dev_warn(md->dev, "%u cacheable items is outside what the loader can describe\n",
			 items);
		return;
	}

	/*
	 * The whole descriptor array is one tag. tag_read() copies at most the
	 * size it is given, so the buffer is sized from the count the loader
	 * just declared, capped so a bad count cannot overrun anything.
	 */
	raw = kcalloc(items, sizeof(*raw), GFP_KERNEL);
	if (!raw)
		return;
	ret = tag_read(md, "md1_bank4_cache_layout", raw,
		       items * sizeof(*raw));
	if (ret != items * sizeof(*raw)) {
		dev_warn(md->dev, "no usable md1_bank4_cache_layout tag (%d of %zu)\n",
			 ret, items * sizeof(*raw));
		kfree(raw);
		return;
	}
	for (i = 0; i < items; i++) {
		md->c_item[i].addr = le64_to_cpu(raw[i].addr);
		md->c_item[i].md_off = le32_to_cpu(raw[i].md_offset);
		md->c_item[i].size = le32_to_cpu(raw[i].size);
		md->c_item[i].id = le32_to_cpu(raw[i].id);
		dev_info(md->dev, "cacheable[%u] id %u: %#llx + %#x, md offset %#x\n",
			 i, md->c_item[i].id, md->c_item[i].addr,
			 md->c_item[i].size, md->c_item[i].md_off);
	}
	md->c_item_num = items;
	kfree(raw);
}

/*
 * The modem image's own capability: the check header the loader copied into
 * its tag list carries the image type, and the image type indexes the vendor's
 * capability table. This is where the world-mode bitmap sent at handshake 1
 * will come from, instead of a constant.
 */
/**
 * tags_read_capability() - what RATs this phone may announce to the modem
 * @md: the modem
 *
 * The image's own capability says what the SOFTWARE can do; it is not a
 * statement about this board. The load here reports image type 12, which the
 * vendor's md_img_capability_map[] (ccci_util_md_rat.c:69-71) describes as
 * LWCTG -- LTE plus WCDMA plus cdma2000 plus GSM -- so the raw map gives
 * 0x3d, with cdma2000 set. This phone has no CDMA subsystem at all: no C2K
 * driver, ro.vendor.mtk_md3_support=0, and its own configuration asks for
 * ro.vendor.mtk_md1_support=9, ulwg = 0x39. Announcing a RAT the hardware
 * cannot work is exactly the class of mistake the study warns about, so the
 * image's set is narrowed to what the board can actually do.
 *
 * The mask is G|T|W|Lt|Lf: everything this SoC's modem path supports except
 * cdma2000 (no subsystem) and NR (this SKU is 4G). Narrowing type 12's 0x3d
 * by it gives 0x39 -- the configuration value, arrived at independently.
 */
static void tags_read_capability(struct ccci_md *md)
{
	u8 buf[24];
	u32 type, image_cap;
	int ret;

	ret = tag_read(md, "md1_chk", buf, sizeof(buf));
	if (ret < MD_CHK_IMAGE_TYPE + 4) {
		dev_warn(md->dev, "no usable md1_chk tag (%d)\n", ret);
		return;
	}
	type = le32_to_cpu(get_unaligned_le32(buf + MD_CHK_IMAGE_TYPE));
	if (type >= ARRAY_SIZE(md_img_cap_map)) {
		dev_warn(md->dev, "md image type %u outside the capability table\n",
			 type);
		return;
	}
	image_cap = md_img_cap_map[type];
	md->md_img_cap = image_cap & MD_CAP_BOARD_MASK;
	dev_info(md->dev, "md image type %u cap %#x, board cap %#x\n",
		 type, image_cap, md->md_img_cap);
	if (image_cap & ~MD_CAP_BOARD_MASK)
		dev_info(md->dev, "the image offers RATs this board has no hardware for: %#x dropped\n",
			 (u32)(image_cap & ~MD_CAP_BOARD_MASK));
	if (md->md_img_cap != MD_CAP_ULWG)
		dev_warn(md->dev, "expected ulwg (%#x) from ro.vendor.mtk_md1_support=9\n",
			 (u32)MD_CAP_ULWG);
}

/**
 * rosemary_ccci_tags_init() - read the loader's CCCI boot information
 * @md: the modem
 *
 * Read-only: it maps the loader's tag block and the share-memory reservations
 * and prints them. Nothing here writes to a modem register or to share
 * memory, so it is safe whatever state the modem is in.
 */
/* the ULONG-sized fields the loader writes as a struct, in its own byte order */
static u32 lk_u32(const u8 *p)
{
	return get_unaligned_le32(p);
}

static u64 lk_u64(const u8 *p)
{
	return get_unaligned_le64(p);
}

/**
 * lk_header_from_chosen() - take the tag list's header from /chosen
 * @md: the modem
 *
 * The bootloader does NOT put the header at the start of the block it reserves.
 * Base, size, version, tag count and the loader's error words arrive as one
 * property of /chosen, and only the tag records themselves live at the base it
 * names. This is how the vendor kernel finds them (ccci_util_lib_fo.c:1052-1090,
 * collect_lk_boot_arguments(): of_get_flat_dt_prop(chosen,
 * "ccci,modem_info_v2")), and it is what the first phone trial proved: reading
 * the header from offset 0 of the block printed 0x2e474ccc / 0x9c7d6010, which
 * is the first tag record's name bytes, not a header.
 *
 * The property holds the struct as raw bytes in the loader's own order, so the
 * fields are decoded from a byte buffer instead of from an aligned struct.
 */
static int lk_header_from_chosen(struct ccci_md *md)
{
	struct device_node *chosen;
	const u8 *p;
	int len;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen) {
		dev_err(md->dev, "no /chosen node: cannot find the CCCI boot information\n");
		return -ENOENT;
	}
	p = of_get_property(chosen, "ccci,modem_info_v2", &len);
	if (!p) {
		dev_err(md->dev, "/chosen has no ccci,modem_info_v2: this boot has no CCCI boot information\n");
		of_node_put(chosen);
		return -ENOENT;
	}
	if (len < LK_INFO_LD_MD_ERRNO + 16) {
		dev_err(md->dev, "ccci,modem_info_v2 is %d bytes, too short for a v2 header\n",
			len);
		of_node_put(chosen);
		return -EINVAL;
	}

	md->lk_base = lk_u64(p + LK_INFO_BASE);
	md->lk_size = lk_u32(p + LK_INFO_SIZE);
	md->lk_err_no = lk_u32(p + LK_INFO_ERR_NO);
	md->lk_version = lk_u32(p + LK_INFO_VERSION);
	md->lk_tag_num = lk_u32(p + LK_INFO_TAG_NUM);
	md->lk_ld_flag = lk_u32(p + LK_INFO_LD_FLAG);
	md->lk_ld_md_errno = lk_u32(p + LK_INFO_LD_MD_ERRNO);
	of_node_put(chosen);

	dev_info(md->dev,
		 "/chosen ccci,modem_info_v2: block %#llx + %#x, version %u, %u tags, ld_flag %u, err %u, ld_md_errno %u\n",
		 md->lk_base, md->lk_size, md->lk_version, md->lk_tag_num,
		 md->lk_ld_flag, md->lk_err_no, md->lk_ld_md_errno);
	return 0;
}

/**
 * rosemary_ccci_tags_init() - read the loader's CCCI boot information
 * @md: the modem
 *
 * Read-only: it takes the header from /chosen, maps the tag block it names and
 * reads the tags out of it. Nothing here writes to a modem register or to share
 * memory, so it is safe whatever state the modem is in.
 */
/**
 * ccci_smem_region() - one region of the loader's cacheable layout
 * @md: the modem
 * @id: the SMEM_USER_ID to look for
 *
 * Returns the descriptor, or NULL when the loader did not describe that
 * region (or the layout tag was unreadable). A NULL means "do not touch it":
 * every caller must check.
 */
const struct ccci_c_item *ccci_smem_region(struct ccci_md *md, u32 id)
{
	u32 i;

	for (i = 0; i < md->c_item_num; i++)
		if (md->c_item[i].id == id && md->c_item[i].size)
			return &md->c_item[i];
	return NULL;
}

int rosemary_ccci_tags_init(struct ccci_md *md)
{
	phys_addr_t base;
	resource_size_t size;
	int ret;

	/* the header first: it tells us whether there is a block at all */
	ret = lk_header_from_chosen(md);
	if (ret)
		return ret;

	if (!md->lk_base) {
		/* the loader ran no modem image this boot (charger, recovery) */
		if (!md->lk_err_no) {
			dev_warn(md->dev, "the loader left no CCCI boot information: no modem this boot\n");
			return -ENODEV;
		}
		dev_err(md->dev, "the loader reports no modem image (%u)\n",
			md->lk_err_no);
		return -ENODEV;
	}
	if (md->lk_ld_md_errno) {
		dev_warn(md->dev, "the loader failed to load the modem image (%u)\n",
			 md->lk_ld_md_errno);
		return -ENODEV;
	}
	if (md->lk_size > LK_MAX_INFO_SIZE) {
		dev_err(md->dev, "the loader's tag block claims %#x bytes, more than the reservation\n",
			md->lk_size);
		return -EINVAL;
	}
	if (md->lk_version < LK_VERSION_V2) {
		dev_err(md->dev, "tag version %u is older than the 64-byte-name format\n",
			md->lk_version);
		return -EINVAL;
	}
	if (!md->lk_tag_num || md->lk_tag_num > LK_MAX_TAGS) {
		dev_err(md->dev, "%u tags is outside what the loader can write\n",
			md->lk_tag_num);
		return -EINVAL;
	}

	/* the block itself: the loader's own reservation, or the board's address */
	md->tag_base = md->lk_base;
	md->tag_size = md->lk_size;
	ret = region_lookup(md, "ccci_tag_mem", &base, &size);
	if (!ret && base != md->lk_base)
		dev_warn(md->dev, "the ccci_tag_mem reservation %#llx is not where the header says (%#llx)\n",
			 base, md->lk_base);
	if (!tags_map(md)) {
		dev_err(md->dev, "cannot map the tag block at %#llx\n",
			md->tag_base);
		return -ENOMEM;
	}
	/*
	 * The block starts with the first tag record, not with a header: its name
	 * must be printable and its data must lie inside the block. A boot where
	 * the loader wrote nothing is refused here instead of at the first tag.
	 */
	{
		char first[LK_TAG_NAME_LEN_V2];
		u32 data_off;

		memcpy_fromio(first, md->tags, sizeof(first));
		first[sizeof(first) - 1] = '\0';
		data_off = readl(md->tags + LK_TAG_NAME_LEN_V2);
		if (!first[0] || !isprint(first[0]) ||
		    data_off > md->lk_size) {
			dev_err(md->dev, "the tag block at %#llx holds no tag list (first record %*phN)\n",
				md->tag_base, 16, md->tags);
			return -EINVAL;
		}
		dev_info(md->dev, "tag block at %#llx: first tag \"%s\"\n",
			 md->tag_base, first);
	}

	/* the reservations the driver will use, found by the loader's names */
	ret = region_lookup(md, "ap_md_nc_smem", &base, &size);
	if (ret)
		dev_err(md->dev, "no ap_md_nc_smem reservation (%d)\n", ret);
	else {
		md->nc_phys = base;
		md->nc_size = size;
		dev_info(md->dev, "ap_md_nc_smem %#llx + %#llx\n", base, (u64)size);
	}
	ret = region_lookup(md, "ap_md_c_smem", &base, &size);
	if (ret)
		dev_err(md->dev, "no ap_md_c_smem reservation (%d)\n", ret);
	else {
		md->c_phys = base;
		md->c_size = size;
		dev_info(md->dev, "ap_md_c_smem %#llx + %#llx\n", base, (u64)size);
	}
	ret = region_lookup(md, "md_mem_usage", &base, &size);
	if (ret)
		dev_err(md->dev, "no md_mem_usage reservation (%d)\n", ret);
	else
		dev_info(md->dev, "md_mem_usage %#llx + %#llx\n", base, (u64)size);

	tags_read_md_mem(md);
	tags_read_smem_layout(md);
	tags_read_ccb(md);
	tags_read_cacheable(md);
	tags_read_capability(md);

	/* the bookkeeping tags the next stages need */
	if (!tag_u32(md, "md1_smem_cahce_offset", &md->c_view_off)) {
		dev_info(md->dev, "md1_smem_cahce_offset %#x\n", md->c_view_off);
	} else {
		md->c_view_off = 0x8000000;	/* the vendor default */
		dev_info(md->dev, "md1_smem_cahce_offset absent, vendor default %#x\n",
			 md->c_view_off);
	}
	if (!tag_u32(md, "ccb_gear_id", &md->ccb_gear_id))
		dev_info(md->dev, "ccb_gear_id %u\n", md->ccb_gear_id);
	if (!tag_u32(md, "md1_ccb_cap_gear", &md->ccb_cap_gear))
		dev_info(md->dev, "md1_ccb_cap_gear %u\n", md->ccb_cap_gear);
	if (!tag_u32(md, "hdr_count", &md->hdr_count))
		dev_info(md->dev, "hdr_count %u\n", md->hdr_count);
	return 0;
}
