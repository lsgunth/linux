// SPDX-License-Identifier: GPL-2.0
/*
 * Microsemi Switchtec(tm) PCIe Management Driver
 * Copyright (c) 2019, Logan Gunthorpe <logang@deltatee.com>
 * Copyright (c) 2019, GigaIO Networks, Inc
 */

#include "dmaengine.h"

#include <linux/dmaengine.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pci.h>

MODULE_DESCRIPTION("PLX ExpressLane PEX PCI Switch DMA Engine");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Logan Gunthorpe");

#define PLX_REG_DEVICE_CAP			0x6C
#define PLX_REG_DEVICE_CTRL			0x70
#define PLX_REG_DESC_RING_ADDR			0x214
#define PLX_REG_DESC_RING_NEXT_ADDR		0x21C
#define PLX_REG_DESC_RING_COUNT			0x220
#define PLX_REG_PREF_LIMIT			0x234
#define PLX_REG_CTRL				0x238
#define PLX_REG_CTRL2				0x23A
#define PLX_REG_INTR_CTRL			0x23C
#define PLX_REG_INTR_STATUS			0x23E

#define PLX_REG_DEVICE_CAP_MAX_PAYLOAD_MASK	7
#define PLX_REG_DEVICE_CAP_MAX_PAYLOAD_128B	0
#define PLX_REG_DEVICE_CAP_MAX_PAYLOAD_256B	1
#define PLX_REG_DEVICE_CAP_MAX_PAYLOAD_512B	2
#define PLX_REG_DEVICE_CAP_MAX_PAYLOAD_1KB	3
#define PLX_REG_DEVICE_CAP_MAX_PAYLOAD_2KB	4

#define PLX_REG_DEVICE_CTRL_CORR_ERR_RPT	BIT(0)
#define PLX_REG_DEVICE_CTRL_NON_FATAL_ERR_RPT	BIT(1)
#define PLX_REG_DEVICE_CTRL_FATAL_ERR_RPT	BIT(2)
#define PLX_REG_DEVICE_CTRL_UNSUP_REQ_RPT	BIT(3)
#define PLX_REG_DEVICE_CTRL_RELAX_ORDERING	BIT(4)
#define PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_MASK	(7 << 5)
#define PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_128B	(0 << 5)
#define PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_256B	(1 << 5)
#define PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_512B	(2 << 5)
#define PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_1KB	(3 << 5)
#define PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_2KB	(4 << 5)
#define PLX_REG_DEVICE_CTRL_EXT_TAG_EN		BIT(8)
#define PLX_REG_DEVICE_CTRL_EN_NO_SNOOP		BIT(11)

#define PLX_REG_PREF_LIMIT_PREF_FOUR		8

#define PLX_REG_CTRL_GRACEFUL_PAUSE		BIT(0)
#define PLX_REG_CTRL_ABORT			BIT(1)
#define PLX_REG_CTRL_WRITE_BACK_EN		BIT(2)
#define PLX_REG_CTRL_START			BIT(3)
#define PLX_REG_CTRL_RING_STOP_MODE		BIT(4)
#define PLX_REG_CTRL_DESC_MODE_BLOCK		(0 << 5)
#define PLX_REG_CTRL_DESC_MODE_ON_CHIP		(1 << 5)
#define PLX_REG_CTRL_DESC_MODE_OFF_CHIP		(2 << 5)
#define PLX_REG_CTRL_DESC_INVALID		BIT(8)
#define PLX_REG_CTRL_DESC_GRACEFUL_PAUSE_DONE	BIT(9)
#define PLX_REG_CTRL_DESC_ABORT_DONE		BIT(10)
#define PLX_REG_CTRL_DESC_IMM_PAUSE_DONE	BIT(12)

#define PLX_REG_CTRL_START_VAL	(PLX_REG_CTRL_WRITE_BACK_EN | \
				 PLX_REG_CTRL_DESC_MODE_OFF_CHIP | \
				 PLX_REG_CTRL_START)

#define PLX_REG_CTRL2_MAX_TXFR_SIZE_64B		0
#define PLX_REG_CTRL2_MAX_TXFR_SIZE_128B	1
#define PLX_REG_CTRL2_MAX_TXFR_SIZE_256B	2
#define PLX_REG_CTRL2_MAX_TXFR_SIZE_512B	3
#define PLX_REG_CTRL2_MAX_TXFR_SIZE_1KB		4
#define PLX_REG_CTRL2_MAX_TXFR_SIZE_2KB		5
#define PLX_REG_CTRL2_MAX_TXFR_SIZE_4B		7

#define PLX_REG_INTR_CRTL_ERROR_EN		BIT(0)
#define PLX_REG_INTR_CRTL_INV_DESC_EN		BIT(1)
#define PLX_REG_INTR_CRTL_ABORT_DONE_EN		BIT(3)
#define PLX_REG_INTR_CRTL_PAUSE_DONE_EN		BIT(4)
#define PLX_REG_INTR_CRTL_IMM_PAUSE_DONE_EN	BIT(5)

#define PLX_REG_INTR_STATUS_ERROR		BIT(0)
#define PLX_REG_INTR_STATUS_INV_DESC		BIT(1)
#define PLX_REG_INTR_STATUS_DESC_DONE		BIT(2)
#define PLX_REG_INTR_CRTL_ABORT_DONE		BIT(3)

struct plx_dma_hw_std_desc {
	__le32 flags_and_size;
	__le16 dst_addr_hi;
	__le16 src_addr_hi;
	__le32 dst_addr_lo;
	__le32 src_addr_lo;
};

#define PLX_DESC_FLAG_VALID		BIT(31)
#define PLX_DESC_FLAG_INT_WHEN_DONE	BIT(30)

#define PLX_DESC_WB_SUCCESS		BIT(30)
#define PLX_DESC_WB_RD_SUCCESS		BIT(29)
#define PLX_DESC_WB_WR_SUCCESS		BIT(28)

#define PLX_DMA_RING_COUNT		2048

struct plx_dma_dev {
	struct dma_device dma_dev;
	struct dma_chan dma_chan;
	void __iomem *bar;

	struct kref ref;

	struct plx_dma_hw_std_desc *hw_ring;
	struct dma_async_tx_descriptor **desc_ring;
};

static struct plx_dma_dev *chan_to_plx_dma_dev(struct dma_chan *c)
{
	return container_of(c, struct plx_dma_dev, dma_chan);
}

static irqreturn_t plx_dma_isr(int irq, void *devid)
{
	struct plx_dma_dev *plxdev = devid;
	u32 status;

	status = readw(plxdev->bar + PLX_REG_INTR_STATUS);

	if (!status)
		return IRQ_NONE;

	writew(status, plxdev->bar + PLX_REG_INTR_STATUS);

	return IRQ_HANDLED;
}

static void plx_dma_release(struct kref *ref)
{
	struct plx_dma_dev *plxdev = container_of(ref, struct plx_dma_dev, ref);

	dma_async_device_unregister(&plxdev->dma_dev);
	kfree(plxdev);
}

static void plx_dma_put(struct plx_dma_dev *plxdev)
{
	kref_put(&plxdev->ref, plx_dma_release);
}

static int plx_dma_alloc_desc(struct plx_dma_dev *plxdev)
{
	struct dma_async_tx_descriptor *desc;
	int i;

	plxdev->desc_ring = kcalloc(PLX_DMA_RING_COUNT,
				    sizeof(*plxdev->desc_ring), GFP_KERNEL);
	if (!plxdev->desc_ring)
		return -ENOMEM;

	for (i = 0; i < PLX_DMA_RING_COUNT; i++) {
		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc)
			goto free_and_exit;

		dma_async_tx_descriptor_init(desc, &plxdev->dma_chan);
		plxdev->desc_ring[i] = desc;
	}

	return 0;

free_and_exit:
	for (i = 0; i < PLX_DMA_RING_COUNT; i++)
		kfree(plxdev->desc_ring[i]);
	kfree(plxdev->desc_ring);
	return -ENOMEM;
}

static int plx_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct plx_dma_dev *plxdev = chan_to_plx_dma_dev(chan);
	size_t ring_sz = PLX_DMA_RING_COUNT * sizeof(*plxdev->hw_ring);
	dma_addr_t dma_addr;
	int rc;

	kref_get(&plxdev->ref);

	plxdev->hw_ring = dmam_alloc_coherent(plxdev->dma_dev.dev, ring_sz,
					      &dma_addr, GFP_KERNEL);
	if (!plxdev->hw_ring)
		return -ENOMEM;

	rc = plx_dma_alloc_desc(plxdev);
	if (rc) {
		plx_dma_put(plxdev);
		return rc;
	}

	writeq(dma_addr, plxdev->bar + PLX_REG_DESC_RING_ADDR);
	writel(dma_addr, plxdev->bar + PLX_REG_DESC_RING_NEXT_ADDR);
	writel(PLX_DMA_RING_COUNT, plxdev->bar + PLX_REG_DESC_RING_COUNT);
	writel(PLX_REG_PREF_LIMIT_PREF_FOUR, plxdev->bar + PLX_REG_PREF_LIMIT);

	return PLX_DMA_RING_COUNT;
}

static void plx_dma_free_chan_resources(struct dma_chan *chan)
{
	struct plx_dma_dev *plxdev = chan_to_plx_dma_dev(chan);
	int i;

	for (i = 0; i < PLX_DMA_RING_COUNT; i++)
		kfree(plxdev->desc_ring[i]);

	kfree(plxdev->desc_ring);

	plx_dma_put(plxdev);
}

static void plx_dma_set_max_transfer(struct plx_dma_dev *plxdev)
{
	const char *max_txfr_str;
	u32 dev_ctrl_val;
	u32 ctrl_val;
	u32 cap;

	cap = readl(plxdev->bar + PLX_REG_DEVICE_CAP) &
		PLX_REG_DEVICE_CAP_MAX_PAYLOAD_MASK;

	dev_ctrl_val = readl(plxdev->bar + PLX_REG_DEVICE_CTRL);
	dev_ctrl_val &= ~PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_MASK;

	switch (cap) {
	case PLX_REG_DEVICE_CAP_MAX_PAYLOAD_256B:
		max_txfr_str = "256B";
		dev_ctrl_val |= PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_256B;
		ctrl_val = PLX_REG_CTRL2_MAX_TXFR_SIZE_256B;
		break;
	case PLX_REG_DEVICE_CAP_MAX_PAYLOAD_512B:
		max_txfr_str = "512B";
		dev_ctrl_val |= PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_512B;
		ctrl_val = PLX_REG_CTRL2_MAX_TXFR_SIZE_512B;
		break;
	case PLX_REG_DEVICE_CAP_MAX_PAYLOAD_1KB:
		max_txfr_str = "1KB";
		dev_ctrl_val |= PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_1KB;
		ctrl_val = PLX_REG_CTRL2_MAX_TXFR_SIZE_1KB;
		break;
	case PLX_REG_DEVICE_CAP_MAX_PAYLOAD_2KB:
		max_txfr_str = "2KB";
		dev_ctrl_val |= PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_2KB;
		ctrl_val = PLX_REG_CTRL2_MAX_TXFR_SIZE_2KB;
		break;
	default:
		max_txfr_str = "128B";
		dev_ctrl_val |= PLX_REG_DEVICE_CTRL_MAX_PAYLOAD_128B;
		ctrl_val = PLX_REG_CTRL2_MAX_TXFR_SIZE_128B;
		break;
	}

	writel(dev_ctrl_val, plxdev->bar + PLX_REG_DEVICE_CTRL);
	writel(ctrl_val, plxdev->bar + PLX_REG_CTRL2);
	dev_info(plxdev->dma_dev.dev, "Maximum Transfer Size set to %s\n",
		 max_txfr_str);
}

static int plx_dma_create(struct pci_dev *pdev)
{
	struct plx_dma_dev *plxdev;
	struct dma_device *dma;
	struct dma_chan *chan;
	int rc;

	plxdev = kzalloc(sizeof(*plxdev), GFP_KERNEL);
	if (!plxdev)
		return -ENOMEM;

	rc = request_irq(pci_irq_vector(pdev, 0), plx_dma_isr, 0,
			 KBUILD_MODNAME, plxdev);
	if (rc) {
		kfree(plxdev);
		return rc;
	}

	kref_init(&plxdev->ref);

	plxdev->bar = pcim_iomap_table(pdev)[0];

	dma = &plxdev->dma_dev;
	dma->chancnt = 1;
	INIT_LIST_HEAD(&dma->channels);
	dma_cap_set(DMA_MEMCPY, dma->cap_mask);
	dma->copy_align = DMAENGINE_ALIGN_1_BYTE;
	dma->dev = &pdev->dev;

	dma->device_alloc_chan_resources = plx_dma_alloc_chan_resources;
	dma->device_free_chan_resources = plx_dma_free_chan_resources;

	chan = &plxdev->dma_chan;
	chan->device = dma;
	dma_cookie_init(chan);
	list_add_tail(&chan->device_node, &dma->channels);

	plx_dma_set_max_transfer(plxdev);
	dma_async_device_register(dma);

	pci_set_drvdata(pdev, plxdev);

	return 0;
}

static int plx_dma_probe(struct pci_dev *pdev,
			 const struct pci_device_id *id)
{
	int rc;

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;

	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(48));
	if (rc)
		rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(48));
	if (rc)
		rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	rc = pcim_iomap_regions(pdev, 1, KBUILD_MODNAME);
	if (rc)
		return rc;

	rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	if (rc)
		return rc;

	pci_set_master(pdev);

	rc = plx_dma_create(pdev);
	if (rc)
		goto err_free_irq_vectors;

	pci_info(pdev, "PLX DMA Channel Registered\n");

	return 0;

err_free_irq_vectors:
	pci_free_irq_vectors(pdev);
	return rc;
}

static void plx_dma_remove(struct pci_dev *pdev)
{
	struct plx_dma_dev *plxdev = pci_get_drvdata(pdev);

	free_irq(pci_irq_vector(pdev, 0),  plxdev);

	plxdev->bar = NULL;
	plx_dma_put(plxdev);

	pci_free_irq_vectors(pdev);
}

static const struct pci_device_id plx_dma_pci_tbl[] = {
	{
		.vendor		= PCI_VENDOR_ID_PLX,
		.device		= 0x87D0,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.class		= PCI_CLASS_SYSTEM_OTHER,
		.class_mask	= 0xFFFFFFFF,
	},
	{0}
};
MODULE_DEVICE_TABLE(pci, plx_dma_pci_tbl);

static struct pci_driver plx_dma_pci_driver = {
	.name           = KBUILD_MODNAME,
	.id_table       = plx_dma_pci_tbl,
	.probe          = plx_dma_probe,
	.remove		= plx_dma_remove,
};
module_pci_driver(plx_dma_pci_driver);
