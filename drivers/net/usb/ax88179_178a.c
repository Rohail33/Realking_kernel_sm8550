// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 *     Copyright (c) 2022    ASIX Electronic Corporation    All rights reserved.
 *
 * Copyright (C) 2011-2013 ASIX
 */

#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/usb/usbnet.h>
#include <uapi/linux/mdio.h>
#include <linux/mdio.h>

#define AX88179_PHY_ID				0x03
#define AX_EEPROM_LEN				0x100
#define AX88179_EEPROM_MAGIC			0x17900b95
#define AX_MCAST_FLTSIZE			8
#define AX_MAX_MCAST				64
#define AX_INT_PPLS_LINK			((u32)BIT(16))
#define AX_RXHDR_L4_TYPE_MASK			0x1c
#define AX_RXHDR_L4_TYPE_UDP			4
#define AX_RXHDR_L4_TYPE_TCP			16
#define AX_RXHDR_L3CSUM_ERR			2
#define AX_RXHDR_L4CSUM_ERR			1
#define AX_RXHDR_CRC_ERR			((u32)BIT(29))
#define AX_RXHDR_DROP_ERR			((u32)BIT(31))
#define AX_ACCESS_MAC				0x01
#define AX_ACCESS_PHY				0x02
#define AX_ACCESS_EEPROM			0x04
#define AX_ACCESS_EFUS				0x05
#define AX_RELOAD_EEPROM_EFUSE			0x06
#define AX_PAUSE_WATERLVL_HIGH			0x54
#define AX_PAUSE_WATERLVL_LOW			0x55

#define PHYSICAL_LINK_STATUS			0x02
	#define	AX_USB_SS		0x04
	#define	AX_USB_HS		0x02

#define GENERAL_STATUS				0x03
/* Check AX88179 version. UA1:Bit2 = 0,  UA2:Bit2 = 1 */
	#define	AX_SECLD		0x04

#define AX_SROM_ADDR				0x07
#define AX_SROM_CMD				0x0a
	#define EEP_RD			0x04
	#define EEP_BUSY		0x10

#define AX_SROM_DATA_LOW			0x08
#define AX_SROM_DATA_HIGH			0x09

#define AX_RX_CTL				0x0b
	#define AX_RX_CTL_DROPCRCERR	0x0100
	#define AX_RX_CTL_IPE		0x0200
	#define AX_RX_CTL_START		0x0080
	#define AX_RX_CTL_AP		0x0020
	#define AX_RX_CTL_AM		0x0010
	#define AX_RX_CTL_AB		0x0008
	#define AX_RX_CTL_AMALL		0x0002
	#define AX_RX_CTL_PRO		0x0001
	#define AX_RX_CTL_STOP		0x0000

#define AX_NODE_ID				0x10
#define AX_MULFLTARY				0x16

#define AX_MEDIUM_STATUS_MODE			0x22
	#define AX_MEDIUM_GIGAMODE	0x01
	#define AX_MEDIUM_FULL_DUPLEX	0x02
	#define AX_MEDIUM_EN_125MHZ	0x08
	#define AX_MEDIUM_RXFLOW_CTRLEN	0x10
	#define AX_MEDIUM_TXFLOW_CTRLEN	0x20
	#define AX_MEDIUM_RECEIVE_EN	0x100
	#define AX_MEDIUM_PS		0x200
	#define AX_MEDIUM_JUMBO_EN	0x8040

#define AX_MONITOR_MOD				0x24
	#define AX_MONITOR_MODE_RWLC	0x02
	#define AX_MONITOR_MODE_RWMP	0x04
	#define AX_MONITOR_MODE_PMEPOL	0x20
	#define AX_MONITOR_MODE_PMETYPE	0x40

#define AX_GPIO_CTRL				0x25
	#define AX_GPIO_CTRL_GPIO3EN	0x80
	#define AX_GPIO_CTRL_GPIO2EN	0x40
	#define AX_GPIO_CTRL_GPIO1EN	0x20

#define AX_PHYPWR_RSTCTL			0x26
	#define AX_PHYPWR_RSTCTL_BZ	0x0010
	#define AX_PHYPWR_RSTCTL_IPRL	0x0020
	#define AX_PHYPWR_RSTCTL_AT	0x1000

#define AX_RX_BULKIN_QCTRL			0x2e
#define AX_CLK_SELECT				0x33
	#define AX_CLK_SELECT_BCS	0x01
	#define AX_CLK_SELECT_ACS	0x02
	#define AX_CLK_SELECT_ULR	0x08

#define AX_RXCOE_CTL				0x34
	#define AX_RXCOE_IP		0x01
	#define AX_RXCOE_TCP		0x02
	#define AX_RXCOE_UDP		0x04
	#define AX_RXCOE_TCPV6		0x20
	#define AX_RXCOE_UDPV6		0x40

#define AX_TXCOE_CTL				0x35
	#define AX_TXCOE_IP		0x01
	#define AX_TXCOE_TCP		0x02
	#define AX_TXCOE_UDP		0x04
	#define AX_TXCOE_TCPV6		0x20
	#define AX_TXCOE_UDPV6		0x40

#define AX_LEDCTRL				0x73

#define GMII_PHY_PHYSR				0x11
	#define GMII_PHY_PHYSR_SMASK	0xc000
	#define GMII_PHY_PHYSR_GIGA	0x8000
	#define GMII_PHY_PHYSR_100	0x4000
	#define GMII_PHY_PHYSR_FULL	0x2000
	#define GMII_PHY_PHYSR_LINK	0x400

#define GMII_LED_ACT				0x1a
	#define	GMII_LED_ACTIVE_MASK	0xff8f
	#define	GMII_LED0_ACTIVE	BIT(4)
	#define	GMII_LED1_ACTIVE	BIT(5)
	#define	GMII_LED2_ACTIVE	BIT(6)

#define GMII_LED_LINK				0x1c
	#define	GMII_LED_LINK_MASK	0xf888
	#define	GMII_LED0_LINK_10	BIT(0)
	#define	GMII_LED0_LINK_100	BIT(1)
	#define	GMII_LED0_LINK_1000	BIT(2)
	#define	GMII_LED1_LINK_10	BIT(4)
	#define	GMII_LED1_LINK_100	BIT(5)
	#define	GMII_LED1_LINK_1000	BIT(6)
	#define	GMII_LED2_LINK_10	BIT(8)
	#define	GMII_LED2_LINK_100	BIT(9)
	#define	GMII_LED2_LINK_1000	BIT(10)
	#define	LED0_ACTIVE		BIT(0)
	#define	LED0_LINK_10		BIT(1)
	#define	LED0_LINK_100		BIT(2)
	#define	LED0_LINK_1000		BIT(3)
	#define	LED0_FD			BIT(4)
	#define	LED0_USB3_MASK		0x001f
	#define	LED1_ACTIVE		BIT(5)
	#define	LED1_LINK_10		BIT(6)
	#define	LED1_LINK_100		BIT(7)
	#define	LED1_LINK_1000		BIT(8)
	#define	LED1_FD			BIT(9)
	#define	LED1_USB3_MASK		0x03e0
	#define	LED2_ACTIVE		BIT(10)
	#define	LED2_LINK_1000		BIT(13)
	#define	LED2_LINK_100		BIT(12)
	#define	LED2_LINK_10		BIT(11)
	#define	LED2_FD			BIT(14)
	#define	LED_VALID		BIT(15)
	#define	LED2_USB3_MASK		0x7c00

#define GMII_PHYPAGE				0x1e
#define GMII_PHY_PAGE_SELECT			0x1f
	#define GMII_PHY_PGSEL_EXT	0x0007
	#define GMII_PHY_PGSEL_PAGE0	0x0000
	#define GMII_PHY_PGSEL_PAGE3	0x0003
	#define GMII_PHY_PGSEL_PAGE5	0x0005

static int ax88179_reset(struct usbnet *dev);

struct ax88179_data {
	u8  eee_enabled;
	u8  eee_active;
	u16 rxctl;
	u8 in_pm;
	u32 wol_supported;
	u32 wolopts;
	u8 disconnecting;
};

struct ax88179_int_data {
	__le32 intdata1;
	__le32 intdata2;
};

static const struct {
	unsigned char ctrl, timer_l, timer_h, size, ifg;
} AX88179_BULKIN_SIZE[] =	{
	{7, 0x4f, 0,	0x12, 0xff},
	{7, 0x20, 3,	0x16, 0xff},
	{7, 0xae, 7,	0x18, 0xff},
};

static void ax88179_set_pm_mode(struct usbnet *dev, bool pm_mode)
{
	struct ax88179_data *ax179_data = dev->driver_priv;

	ax179_data->in_pm = pm_mode;
}

static int ax88179_in_pm(struct usbnet *dev)
{
	struct ax88179_data *ax179_data = dev->driver_priv;

	return ax179_data->in_pm;
}

static int __ax88179_read_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			      u16 size, void *data)
{
	int ret;
	int (*fn)(struct usbnet *, u8, u8, u16, u16, void *, u16);
	struct ax88179_data *ax179_data = dev->driver_priv;

	BUG_ON(!dev);

	if (!ax88179_in_pm(dev))
		fn = usbnet_read_cmd;
	else
		fn = usbnet_read_cmd_nopm;

	ret = fn(dev, cmd, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		 value, index, data, size);

	if (unlikely((ret < 0) && !(ret == -ENODEV && ax179_data->disconnecting)))
		netdev_warn(dev->net, "Failed to read reg index 0x%04x: %d\n",
			    index, ret);

	return ret;
}

static int __ax88179_write_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			       u16 size, const void *data)
{
	int ret;
	int (*fn)(struct usbnet *, u8, u8, u16, u16, const void *, u16);
	struct ax88179_data *ax179_data = dev->driver_priv;

	BUG_ON(!dev);

	if (!ax88179_in_pm(dev))
		fn = usbnet_write_cmd;
	else
		fn = usbnet_write_cmd_nopm;

	ret = fn(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		 value, index, data, size);

	if (unlikely((ret < 0) && !(ret == -ENODEV && ax179_data->disconnecting)))
		netdev_warn(dev->net, "Failed to write reg index 0x%04x: %d\n",
			    index, ret);

	return ret;
}

int ax88179_signature(struct ax_device *axdev, struct _ax_ioctl_command *info)
{
	strscpy(info->sig, AX88179_SIGNATURE, sizeof(info->sig));
	return 0;
}

static int ax88179_read_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			    u16 size, void *data)
{
	int ret;

	if (2 == size) {
		u16 buf = 0;
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, &buf);
		le16_to_cpus(&buf);
		*((u16 *)data) = buf;
	} else if (4 == size) {
		u32 buf = 0;
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, &buf);
		le32_to_cpus(&buf);
		*((u32 *)data) = buf;
	} else {
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, data);
	}

	return ret;
}

static int ax88179_write_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			     u16 size, const void *data)
{
	int ret;

	if (2 == size) {
		u16 buf;
		buf = *((u16 *)data);
		cpu_to_le16s(&buf);
		ret = __ax88179_write_cmd(dev, cmd, value, index,
					  size, &buf);
	} else {
		ret = __ax88179_write_cmd(dev, cmd, value, index,
					  size, data);
	}

	kfree(buf);

	return 0;
}

int ax88179_write_eeprom(struct ax_device *axdev,
			 struct _ax_ioctl_command *info)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	struct ax88179_data *priv = dev->driver_priv;
	u16 tmp16;
	u8 tmp8;

	ax88179_set_pm_mode(dev, true);

	usbnet_suspend(intf, message);

	/* Enable WoL */
	if (priv->wolopts) {
		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MOD,
				 1, 1, &tmp8);
		if (priv->wolopts & WAKE_PHY)
			tmp8 |= AX_MONITOR_MODE_RWLC;
		if (priv->wolopts & WAKE_MAGIC)
			tmp8 |= AX_MONITOR_MODE_RWMP;

		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MOD,
				  1, 1, &tmp8);
	}

	/* Disable RX path */
	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			 2, 2, &tmp16);
	tmp16 &= ~AX_MEDIUM_RECEIVE_EN;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			  2, 2, &tmp16);

	/* Force bulk-in zero length */
	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			 2, 2, &tmp16);

	tmp16 |= AX_PHYPWR_RSTCTL_BZ | AX_PHYPWR_RSTCTL_IPRL;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			  2, 2, &tmp16);

	/* change clock */
	tmp8 = 0;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp8);

	/* Configure RX control register => stop operation */
	tmp16 = AX_RX_CTL_STOP;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &tmp16);

	ax88179_set_pm_mode(dev, false);

	return 0;
}

/* This function is used to enable the autodetach function. */
/* This function is determined by offset 0x43 of EEPROM */
static int ax88179_auto_detach(struct usbnet *dev)
{
	u16 tmp16;
	u8 tmp8;

	if (ax88179_read_cmd(dev, AX_ACCESS_EEPROM, 0x43, 1, 2, &tmp16) < 0)
		return 0;

	if ((tmp16 == 0xFFFF) || (!(tmp16 & 0x0100)))
		return 0;

	/* Enable Auto Detach bit */
	tmp8 = 0;
	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp8);
	tmp8 |= AX_CLK_SELECT_ULR;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp8);

	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, &tmp16);
	tmp16 |= AX_PHYPWR_RSTCTL_AT;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, &tmp16);

	return 0;
}

static int ax88179_resume(struct usb_interface *intf)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	u16 tmp16;
	u8 tmp8;

	ax88179_set_pm_mode(dev, true);

	usbnet_link_change(dev, 0, 0);

	/* Power up ethernet PHY */
	tmp16 = 0;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			  2, 2, &tmp16);
	udelay(1000);

	tmp16 = AX_PHYPWR_RSTCTL_IPRL;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			  2, 2, &tmp16);
	msleep(200);

	/* Ethernet PHY Auto Detach*/
	ax88179_auto_detach(dev);

	/* Enable clock */
	ax88179_read_cmd(dev, AX_ACCESS_MAC,  AX_CLK_SELECT, 1, 1, &tmp8);
	tmp8 |= AX_CLK_SELECT_ACS | AX_CLK_SELECT_BCS;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp8);
	msleep(100);

	/* Configure RX control register => start operation */
	tmp16 = AX_RX_CTL_DROPCRCERR | AX_RX_CTL_IPE | AX_RX_CTL_START |
		AX_RX_CTL_AP | AX_RX_CTL_AMALL | AX_RX_CTL_AB;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &tmp16);

	ax88179_set_pm_mode(dev, false);

	return usbnet_resume(intf);
}

static void ax88179_disconnect(struct usb_interface *intf)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	struct ax88179_data *ax179_data;

	if (!dev)
		return;

	ax179_data = dev->driver_priv;
	ax179_data->disconnecting = 1;

	usbnet_disconnect(intf);
}

static void
ax88179_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	struct ax88179_data *priv = dev->driver_priv;

	wolinfo->supported = priv->wol_supported;
	wolinfo->wolopts = priv->wolopts;
}

static int
ax88179_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	struct ax88179_data *priv = dev->driver_priv;

	if (wolinfo->wolopts & ~(priv->wol_supported))
		return -EINVAL;

	priv->wolopts = wolinfo->wolopts;

	if (info->type == 0) {
		if ((*(buf) >> 8) & 0x01) {
			netdev_info(axdev->netdev, "Cannot be set to muliticast MAC address, ");
			netdev_info(axdev->netdev, "bit0 of Node ID-0 cannot be set to 1. \r\n");
			kfree(buf);
			return -EINVAL;
		}

		csum = (*(buf + 3) & 0xff) + ((*(buf + 3) >> 8) & 0xff) +
		       (*(buf + 4) & 0xff) + ((*(buf + 4) >> 8) & 0xff);
		csum = 0xff - ((csum >> 8) + (csum & 0xff));
		data = ((*(buf + 5)) & 0xff) | (csum << 8);
		*(buf + 5) = data;

		for (i = 0; i < info->size; i++) {
			data = cpu_to_be16(*(buf + i));
			if (ax_write_cmd(axdev, AX_ACCESS_EEPROM,
					 i, 1, 2, &data) < 0) {
				kfree(buf);
				return -EINVAL;
			}

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;
	eeprom_buff = kmalloc_array(last_word - first_word + 1, sizeof(u16),
				    GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	/* ax88179/178A returns 2 bytes from eeprom on read */
	for (i = first_word; i <= last_word; i++) {
		ret = __ax88179_read_cmd(dev, AX_ACCESS_EEPROM, i, 1, 2,
					 &eeprom_buff[i - first_word]);
		if (ret < 0) {
			kfree(eeprom_buff);
			return -EIO;
		}

		for (i = 0; i < info->size; i++)
			csum += (*(buf + i) & 0xff) + ((*(buf + i) >> 8) & 0xff);

		csum -= ((*(buf + 0x19) >> 8) & 0xff);
		while (csum > 255)
			csum = (csum & 0x00FF) + ((csum >> 8) & 0x00FF);
		csum = 0xFF - csum;

		data = ((*(buf + 0x19)) & 0xff) | (csum << 8);
		*(buf + 0x19) = data;

		if (ax_write_cmd(axdev, AX_WRITE_EFUSE_EN, 0, 0, 0, NULL) < 0) {
			kfree(buf);
			return -EINVAL;
		}

		msleep(info->delay);

		for (i = 0; i < info->size; i++) {
			data = cpu_to_be16(*(buf + i));
			if (ax_write_cmd(axdev, AX_ACCESS_EFUSE, i, 1, 2, &data) < 0) {
				kfree(buf);
				return -EINVAL;
			}

			msleep(info->delay);
		}

		if (ax_write_cmd(axdev, AX_WRITE_EFUSE_DIS, 0, 0, 0, NULL) < 0) {
			kfree(buf);
			return -EINVAL;
		}

		msleep(info->delay);
	} else if (info->type == 2) {
		if (ax_read_cmd(axdev, AX_ACCESS_EFUSE, 0, 1, 2, &data, 1) < 0) {
			kfree(buf);
			return -EINVAL;
		}

		if (data == 0xFFFF)
			info->type = 0;
		else
			info->type = 1;
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return 0;
}

IOCTRL_TABLE ax88179_tbl[] = {
	ax88179_signature,
	NULL,//ax_usb_command,
	ax88179_read_eeprom,
	ax88179_write_eeprom,
};

int ax88179_siocdevprivate(struct net_device *netdev, struct ifreq *rq,
			   void __user *udata, int cmd)
{
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };
	struct ax88179_data *priv = dev->driver_priv;

	mii_ethtool_gset(&dev->mii, &ecmd);

	if (ecmd.duplex & DUPLEX_FULL) {
		int eee_lp, eee_cap, eee_adv;
		u32 lp, cap, adv, supported = 0;

		eee_cap = ax88179_phy_read_mmd_indirect(dev,
							MDIO_PCS_EEE_ABLE,
							MDIO_MMD_PCS);
		if (eee_cap < 0) {
			priv->eee_active = 0;
			return false;
		}

		cap = mmd_eee_cap_to_ethtool_sup_t(eee_cap);
		if (!cap) {
			priv->eee_active = 0;
			return false;
		}

		if (copy_to_user(uptr, &info, sizeof(struct _ax_ioctl_command)))
			return -EFAULT;

		break;
	default:
		ret = -EOPNOTSUPP;
	}

	return ret;
}

int ax88179_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	struct ax_device *axdev = netdev_priv(netdev);

	return generic_mii_ioctl(&axdev->mii, if_mii(rq), cmd, NULL);
}

void ax88179_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct ax88179_data *priv = dev->driver_priv;

	edata->eee_enabled = priv->eee_enabled;
	edata->eee_active = priv->eee_active;

	return ax88179_ethtool_get_eee(dev, edata);
}

static int ax88179_set_eee(struct net_device *net, struct ethtool_eee *edata)
{
	struct usbnet *dev = netdev_priv(net);
	struct ax88179_data *priv = dev->driver_priv;
	int ret;

	priv->eee_enabled = edata->eee_enabled;
	if (!priv->eee_enabled) {
		ax88179_disable_eee(dev);
	} else {
		priv->eee_enabled = ax88179_chk_eee(dev);
		if (!priv->eee_enabled)
			return -EOPNOTSUPP;

		ax88179_enable_eee(dev);
	}

	ret = ax88179_ethtool_set_eee(dev, edata);
	if (ret)
		return ret;

	mii_nway_restart(&dev->mii);

	usbnet_link_change(dev, 0, 0);

	return ret;
}

static int ax88179_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);
	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

static const struct ethtool_ops ax88179_ethtool_ops = {
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= ax88179_get_wol,
	.set_wol		= ax88179_set_wol,
	.get_eeprom_len		= ax88179_get_eeprom_len,
	.get_eeprom		= ax88179_get_eeprom,
	.set_eeprom		= ax88179_set_eeprom,
	.get_eee		= ax88179_get_eee,
	.set_eee		= ax88179_set_eee,
	.nway_reset		= usbnet_nway_reset,
	.get_link_ksettings	= ax88179_get_link_ksettings,
	.set_link_ksettings	= ax88179_set_link_ksettings,
	.get_ts_info		= ethtool_op_get_ts_info,
};

static void ax88179_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct ax88179_data *data = dev->driver_priv;
	u8 *m_filter = ((u8 *)dev->data);

	axdev->rxctl = (AX_RX_CTL_START | AX_RX_CTL_AB);

	if (net->flags & IFF_PROMISC) {
		axdev->rxctl |= AX_RX_CTL_PRO;
	} else if (net->flags & IFF_ALLMULTI || mc_count > AX_MAX_MCAST) {
		axdev->rxctl |= AX_RX_CTL_AMALL;
	} else if (mc_count == 0) {
	} else {
		/* We use dev->data for our 8 byte filter buffer
		 * to avoid allocating memory that is tricky to free later
		 */
		u32 crc_bits;
		struct netdev_hw_addr *ha = NULL;

		memset(m_filter, 0, AX_MCAST_FILTER_SIZE);
		netdev_for_each_mc_addr(ha, net) {
			crc_bits = ether_crc(ETH_ALEN, ha->addr) >> 26;
			*(m_filter + (crc_bits >> 3)) |=
				1 << (crc_bits & 7);
		}
		ax_write_cmd_async(axdev, AX_ACCESS_MAC, AX_MULTI_FILTER_ARRY,
				   AX_MCAST_FILTER_SIZE, AX_MCAST_FILTER_SIZE, m_filter);
		axdev->rxctl |= AX_RX_CTL_AM;
	}
	ax_write_cmd_async(axdev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &axdev->rxctl);
}

int ax88179_set_mac_addr(struct net_device *netdev, void *p)
{
	struct ax_device *axdev = netdev_priv(netdev);
	struct sockaddr *addr = p;
	int ret;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	if (netif_running(netdev))
		return -EBUSY;

	memcpy(netdev->dev_addr, addr->sa_data, ETH_ALEN);

	ret = ax_write_cmd(axdev, AX_ACCESS_MAC, AX_NODE_ID, ETH_ALEN,
			   ETH_ALEN, netdev->dev_addr);
	if (ret < 0)
		return ret;

	return 0;
}

static int ax88179_check_eeprom(struct ax_device *axdev)
{
	u8 i = 0;
	u8 buf[2];
	u8 eeprom[20];
	u16 csum = 0, delay = HZ / 10;

	for (i = 0 ; i < 6; i++) {
		buf[0] = i;
		if (ax_write_cmd(axdev, AX_ACCESS_MAC, AX_SROM_ADDR, 1, 1, buf) < 0)
			return -EINVAL;

		buf[0] = EEP_RD;
		if (ax_write_cmd(axdev, AX_ACCESS_MAC, AX_SROM_CMD, 1, 1, buf) < 0)
			return -EINVAL;

		do {
			ax_read_cmd(axdev, AX_ACCESS_MAC, AX_SROM_CMD, 1, 1, buf, 0);

			if (time_after(jiffies, (jiffies + delay)))
				return -EINVAL;
		} while (buf[0] & EEP_BUSY);

		ax_read_cmd(axdev, AX_ACCESS_MAC, AX_SROM_DATA_LOW, 2, 2, &eeprom[i * 2], 0);

		__ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_SROM_DATA_LOW,
				   2, 2, &eeprom[i * 2]);

		if ((i == 0) && (eeprom[0] == 0xFF))
			return -EINVAL;
	}

	csum = eeprom[6] + eeprom[7] + eeprom[8] + eeprom[9];
	csum = (csum >> 8) + (csum & 0xff);

	if ((csum + eeprom[10]) == 0xff)
		return 0;
	else
		return -EINVAL;

	return 0;
}

static int ax88179_check_efuse(struct ax_device *axdev, void *ledmode)
{
	u8	i = 0;
	u16	csum = 0;
	u8	efuse[64];

	if (ax_read_cmd(axdev, AX_ACCESS_EFUSE, 0, 64, 64, efuse, 0) < 0)
		return -EINVAL;

	if (efuse[0] == 0xFF)
		return -EINVAL;

	for (i = 0; i < 64; i++)
		csum = csum + efuse[i];

	while (csum > 255)
		csum = (csum & 0x00FF) + ((csum >> 8) & 0x00FF);

	if (csum == 0xFF) {
		memcpy((u8 *)ledmode, &efuse[51], 2);
		return 0;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int ax88179_convert_old_led(struct ax_device *axdev, u8 efuse, void *ledvalue)
{
	u8 ledmode = 0;
	u16 reg16;
	u16 led = 0;

	/* loaded the old eFuse LED Mode */
	if (efuse) {
		if (ax_read_cmd(axdev, AX_ACCESS_EFUSE, 0x18, 1, 2, &reg16, 1) < 0)
			return -EINVAL;
		ledmode = (u8)(reg16 & 0xFF);
	} else { /* loaded the old EEprom LED Mode */
		if (ax_read_cmd(axdev, AX_ACCESS_EEPROM, 0x3C, 1, 2, &reg16, 1) < 0)
			return -EINVAL;
		ledmode = (u8)(reg16 >> 8);
	}
	netdev_dbg(axdev->netdev, "Old LED Mode = %02X\n", ledmode);

	switch (ledmode) {
	case 0xFF:
		led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 |
		      LED1_LINK_1000 | LED2_ACTIVE | LED2_LINK_10 |
		      LED2_LINK_100 | LED2_LINK_1000 | LED_VALID;
		break;
	case 0xFE:
		led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 | LED_VALID;
		break;
	case 0xFD:
		led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 |
		      LED2_LINK_10 | LED_VALID;
		break;
	case 0xFC:
		led = LED0_ACTIVE | LED1_ACTIVE | LED1_LINK_1000 | LED2_ACTIVE |
		      LED2_LINK_100 | LED2_LINK_10 | LED_VALID;
		break;
	default:
		led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 |
		      LED1_LINK_1000 | LED2_ACTIVE | LED2_LINK_10 |
		      LED2_LINK_100 | LED2_LINK_1000 | LED_VALID;
		break;
	}

	memcpy((u8 *)ledvalue, &led, 2);

	return 0;
}

static void ax88179_gether_setting(struct ax_device *axdev)
{
	u16 reg16;

	reg16 = 0x03;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID,
		     31, 2, &reg16);
	reg16 = 0x3246;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID,
		     25, 2, &reg16);
	reg16 = 0;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID,
		     31, 2, &reg16);
}

static int ax88179_LED_setting(struct ax_device *axdev)
{
	u16 ledvalue = 0, delay = HZ / 10;
	u16 ledact, ledlink;
	u16 reg16;
	u8 value;

	ax_read_cmd(axdev, AX_ACCESS_MAC, GENERAL_STATUS, 1, 1, &value, 0);

	if (!(value & AX_SECLD)) {
		value = AX_GPIO_CTRL_GPIO3EN | AX_GPIO_CTRL_GPIO2EN |
			AX_GPIO_CTRL_GPIO1EN;
		if (ax_write_cmd(axdev, AX_ACCESS_MAC, AX_GPIO_CTRL,
				 1, 1, &value) < 0)
			return -EINVAL;
	}

	if (!ax88179_check_eeprom(axdev)) {
		value = 0x42;
		if (ax_write_cmd(axdev, AX_ACCESS_MAC, AX_SROM_ADDR, 1, 1, &value) < 0)
			return -EINVAL;

		value = EEP_RD;
		if (ax_write_cmd(axdev, AX_ACCESS_MAC, AX_SROM_CMD, 1, 1, &value) < 0)
			return -EINVAL;

		do {
			ax_read_cmd(axdev, AX_ACCESS_MAC, AX_SROM_CMD, 1, 1, &value, 0);

			ax_read_cmd(axdev, AX_ACCESS_MAC, AX_SROM_CMD, 1, 1, &value, 0);

			if (time_after(jiffies, (jiffies + delay)))
				return -EINVAL;
		} while (value & EEP_BUSY);

		ax_read_cmd(axdev, AX_ACCESS_MAC, AX_SROM_DATA_HIGH, 1, 1, &value, 0);
		ledvalue = (value << 8);
		ax_read_cmd(axdev, AX_ACCESS_MAC, AX_SROM_DATA_LOW, 1, 1, &value, 0);
		ledvalue |= value;

		if (ledvalue == 0xFFFF || ((ledvalue & LED_VALID) == 0))
			ax88179_convert_old_led(axdev, 0, &ledvalue);

	} else if (!ax88179_check_efuse(axdev, &ledvalue)) {
		if (ledvalue == 0xFFFF || ((ledvalue & LED_VALID) == 0))
			ax88179_convert_old_led(axdev, 0, &ledvalue);
	} else {
		ax88179_convert_old_led(axdev, 0, &ledvalue);
	}

	reg16 = GMII_PHY_PAGE_SELECT_EXT;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_PHY_PAGE_SELECT, 2, &reg16);

	reg16 = 0x2c;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_PHYPAGE, 2, &reg16);

	ax_read_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_LED_ACTIVE, 2, &ledact, 1);

	ax_read_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_LED_LINK, 2, &ledlink, 1);

	ledact &= GMII_LED_ACTIVE_MASK;
	ledlink &= GMII_LED_LINK_MASK;

	if (ledvalue & LED0_ACTIVE)
		ledact |= GMII_LED0_ACTIVE;
	if (ledvalue & LED1_ACTIVE)
		ledact |= GMII_LED1_ACTIVE;
	if (ledvalue & LED2_ACTIVE)
		ledact |= GMII_LED2_ACTIVE;

	if (ledvalue & LED0_LINK_10)
		ledlink |= GMII_LED0_LINK_10;
	if (ledvalue & LED1_LINK_10)
		ledlink |= GMII_LED1_LINK_10;
	if (ledvalue & LED2_LINK_10)
		ledlink |= GMII_LED2_LINK_10;

	if (ledvalue & LED0_LINK_100)
		ledlink |= GMII_LED0_LINK_100;
	if (ledvalue & LED1_LINK_100)
		ledlink |= GMII_LED1_LINK_100;
	if (ledvalue & LED2_LINK_100)
		ledlink |= GMII_LED2_LINK_100;

	if (ledvalue & LED0_LINK_1000)
		ledlink |= GMII_LED0_LINK_1000;
	if (ledvalue & LED1_LINK_1000)
		ledlink |= GMII_LED1_LINK_1000;
	if (ledvalue & LED2_LINK_1000)
		ledlink |= GMII_LED2_LINK_1000;

	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_LED_ACTIVE, 2, &ledact);

	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_LED_LINK, 2, &ledlink);

	reg16 = GMII_PHY_PAGE_SELECT_PAGE0;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_PHY_PAGE_SELECT, 2, &reg16);

	/* LED full duplex setting */
	reg16 = 0;
	if (ledvalue & LED0_FD)
		reg16 |= 0x01;
	else if ((ledvalue & LED0_USB3_MASK) == 0)
		reg16 |= 0x02;

	if (ledvalue & LED1_FD)
		reg16 |= 0x04;
	else if ((ledvalue & LED1_USB3_MASK) == 0)
		reg16 |= 0x08;

	if (ledvalue & LED2_FD) /* LED2_FD */
		reg16 |= 0x10;
	else if ((ledvalue & LED2_USB3_MASK) == 0) /* LED2_USB3 */
		reg16 |= 0x20;

	ax_write_cmd(axdev, AX_ACCESS_MAC, 0x73, 1, 1, &reg16);

	return 0;
}

static void ax88179_EEE_setting(struct ax_device *axdev)
{
	u16 reg16;
	/* Disable */
	reg16 = 0x07;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_PHY_MACR, 2, &reg16);
	reg16 = 0x3c;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_PHY_MAADR, 2, &reg16);
	reg16 = 0x4007;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_PHY_MACR, 2, &reg16);
	reg16 = 0x00;
	ax_write_cmd(axdev, AX_ACCESS_PHY, AX88179_PHY_ID, GMII_PHY_MAADR, 2, &reg16);
}

static int ax88179_auto_detach(struct ax_device *axdev, int in_pm)
{
	u16 reg16;
	usb_read_function fnr;
	usb_write_function fnw;

	if (!in_pm) {
		fnr = ax_read_cmd;
		fnw = ax_write_cmd;
	} else {
		fnr = ax_read_cmd_nopm;
		fnw = ax_write_cmd_nopm;
	}

	if (fnr(axdev, AX_ACCESS_EEPROM, 0x43, 1, 2, &reg16, 1) < 0)
		return 0;

static int ax88179_bind(struct usbnet *dev, struct usb_interface *intf)
{
	struct ax88179_data *ax179_data;

	reg16 = 0;
	fnr(axdev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &reg16, 0);
	reg16 |= AX_CLK_SELECT_ULR;
	fnw(axdev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &reg16);

	ax179_data = kzalloc(sizeof(*ax179_data), GFP_KERNEL);
	if (!ax179_data)
		return -ENOMEM;

	dev->driver_priv = ax179_data;

	ax_write_cmd(axdev, 0x91, 0, 0, 0, NULL);

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = ax88179_mdio_read;
	dev->mii.mdio_write = ax88179_mdio_write;
	dev->mii.phy_id_mask = 0xff;
	dev->mii.reg_num_mask = 0xff;
	dev->mii.phy_id = 0x03;
	dev->mii.supports_gmii = 1;

	dev->net->features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
			      NETIF_F_RXCSUM;

	dev->net->hw_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
				 NETIF_F_RXCSUM;

	ax88179_reset(dev);

	return 0;
}

static int ax88179_bind(struct ax_device *axdev)
{
	struct ax88179_data *ax179_data = dev->driver_priv;
	u16 tmp16;

	/* Configure RX control register => stop operation */
	tmp16 = AX_RX_CTL_STOP;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &tmp16);

	tmp16 = 0;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp16);

	/* Power down ethernet PHY */
	tmp16 = 0;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, &tmp16);

	kfree(ax179_data);
}

static void ax88179_unbind(struct ax_device *axdev)
{
}

static int ax88179_stop(struct ax_device *axdev)
{
	u16 reg16;

	reg16 = AX_RX_CTL_STOP;
	ax_write_cmd(axdev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE, 2, 2, &reg16);

	reg16 = 0;
	ax_write_cmd(axdev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &reg16);

	reg16 = AX_PHYPWR_RSTCTL_BZ;
	ax_write_cmd(axdev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, &reg16);
	msleep(200);

	return 0;
}

static int ax88179_link_reset(struct ax_device *axdev)
{
	u32 tx_hdr1, tx_hdr2;
	int frame_size = dev->maxpacket;
	int mss = skb_shinfo(skb)->gso_size;
	int headroom;
	void *ptr;

	tx_hdr1 = skb->len;
	tx_hdr2 = mss;
	if (((skb->len + 8) % frame_size) == 0)
		tx_hdr2 |= 0x80008000;	/* Enable padding */

	headroom = skb_headroom(skb) - 8;

	if ((skb_header_cloned(skb) || headroom < 0) &&
	    pskb_expand_head(skb, headroom < 0 ? 8 : 0, 0, GFP_ATOMIC)) {
		dev_kfree_skb_any(skb);
		return NULL;
	}

	ptr = skb_push(skb, 8);
	put_unaligned_le32(tx_hdr1, ptr);
	put_unaligned_le32(tx_hdr2, ptr + 4);

	return skb;
}

static int ax88179_link_reset(struct usbnet *dev)
{
	struct ax88179_data *ax179_data = dev->driver_priv;
	u8 tmp[5], link_sts;
	u16 mode, tmp16, delay = HZ / 10;
	u32 tmp32 = 0x40000000;
	unsigned long jtimeout;

	jtimeout = jiffies + delay;
	while (tmp32 & 0x40000000) {
		mode = 0;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &mode);
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2,
				  &ax179_data->rxctl);

		/*link up, check the usb device control TX FIFO full or empty*/
		ax88179_read_cmd(dev, 0x81, 0x8c, 0, 4, &tmp32);

		if (time_after(jiffies, jtimeout))
			return 0;
	}

	mode = AX_MEDIUM_RECEIVE_EN | AX_MEDIUM_TXFLOW_CTRLEN |
	       AX_MEDIUM_RXFLOW_CTRLEN;

	ax88179_read_cmd(dev, AX_ACCESS_MAC, PHYSICAL_LINK_STATUS,
			 1, 1, &link_sts);

	ax88179_read_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			 GMII_PHY_PHYSR, 2, &tmp16);

	if (!(tmp16 & GMII_PHY_PHYSR_LINK)) {
		return 0;
	} else if (GMII_PHY_PHYSR_GIGA == (tmp16 & GMII_PHY_PHYSR_SMASK)) {
		mode |= AX_MEDIUM_GIGAMODE | AX_MEDIUM_EN_125MHZ;
		if (dev->net->mtu > 1500)
			mode |= AX_MEDIUM_JUMBO_EN;

		if (link_sts & AX_USB_SS)
			memcpy(reg8, &AX88179_BULKIN_SIZE[0], 5);
		else if (link_sts & AX_USB_HS)
			memcpy(reg8, &AX88179_BULKIN_SIZE[1], 5);
		else
			memcpy(reg8, &AX88179_BULKIN_SIZE[3], 5);
	} else if (GMII_PHY_PHYSR_100 == (reg16 & GMII_PHY_PHYSR_SMASK)) {
		mode |= AX_MEDIUM_PS;
		if (link_sts & (AX_USB_SS | AX_USB_HS))
			memcpy(reg8, &AX88179_BULKIN_SIZE[2], 5);
		else
			memcpy(reg8, &AX88179_BULKIN_SIZE[3], 5);
	} else {
		memcpy(reg8, &AX88179_BULKIN_SIZE[3], 5);
	}

	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_RX_BULKIN_QCTRL, 5, 5, reg8);

	if (reg16 & GMII_PHY_PHYSR_FULL)
		mode |= AX_MEDIUM_FULL_DUPLEX;

	ax_read_cmd_nopm(axdev, 0x81, 0x8c, 0, 4, &reg32, 1);
	delay = HZ / 2;
	if (reg32 & 0x40000000) {
		unsigned long jtimeout;
		u16 temp16 = 0;

		ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_RX_CTL,
				  2, 2, &temp16);
		ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
				  2, 2, &mode);

		jtimeout = jiffies + delay;
		while (time_before(jiffies, jtimeout)) {
			ax_read_cmd_nopm(axdev, 0x81, 0x8c, 0, 4, &reg32, 1);

			if (!(reg32 & 0x40000000))
				break;

			reg32 = 0x80000000;
			ax_write_cmd(axdev, 0x81, 0x8c, 0, 4, &reg32);
		}

		temp16 = AX_RX_CTL_DROPCRCERR | AX_RX_CTL_START |
			 AX_RX_CTL_AP | AX_RX_CTL_AMALL | AX_RX_CTL_AB;
		ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_RX_CTL,
				  2, 2, &temp16);
	}

	axdev->rxctl |= AX_RX_CTL_DROPCRCERR | AX_RX_CTL_START | AX_RX_CTL_AB;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_RX_CTL,
			  2, 2, &axdev->rxctl);

	mode |= AX_MEDIUM_RECEIVE_EN;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			  2, 2, &mode);

	return 0;
}

static int ax88179_tx_fixup(struct ax_device *axdev, struct tx_desc *desc)
{
	u8 buf[5];
	u16 *tmp16;
	u8 *tmp;
	struct ax88179_data *ax179_data = dev->driver_priv;
	struct ethtool_eee eee_data;

		memset(tx_data, 0, AX_TX_HEADER_LEN);
		tx_hdr1 = (u32 *)tx_data;
		tx_hdr2 = tx_hdr1 + 1;
		*tx_hdr1 = skb->len;
		*tx_hdr2 = skb_shinfo(skb)->gso_size;
		cpu_to_le32s(tx_hdr1);
		cpu_to_le32s(tx_hdr2);
		tx_data += 8;

		if (skb_copy_bits(skb, 0, tx_data, skb->len) < 0) {
			stats->tx_dropped++;
			dev_kfree_skb_any(skb);
			continue;
		}

		tx_data += skb->len;
		desc->skb_len += skb->len;
		desc->skb_num += skb_shinfo(skb)->gso_segs ?: 1;
		dev_kfree_skb_any(skb);

		tx_data = __tx_buf_align(tx_data, axdev->tx_align_len);
		if (*tx_hdr2 > 0)
			break;
		remain = axdev->tx_casecade_size -
			 (int)((void *)tx_data - desc->head);
	}

	if (!skb_queue_empty(&skb_head)) {
		spin_lock(&tx_queue->lock);
		skb_queue_splice(&skb_head, tx_queue);
		spin_unlock(&tx_queue->lock);
	}

	/* Ethernet PHY Auto Detach*/
	ax88179_auto_detach(dev);

	/* Read MAC address from DTB or asix chip */
	ax88179_get_mac_addr(dev);
	memcpy(dev->net->perm_addr, dev->net->dev_addr, ETH_ALEN);

	netif_tx_unlock(axdev->netdev);

	ret = usb_autopm_get_interface_async(axdev->intf);
	if (ret < 0)
		goto out_tx_fill;

	usb_fill_bulk_urb(desc->urb, axdev->udev,
			  usb_sndbulkpipe(axdev->udev, 3),
			  desc->head, (int)(tx_data - (u8 *)desc->head),
			  (usb_complete_t)ax_write_bulk_callback, desc);

	ret = usb_submit_urb(desc->urb, GFP_ATOMIC);
	if (ret < 0)
		usb_autopm_put_interface_async(axdev->intf);

	/* Enable checksum offload */
	*tmp = AX_RXCOE_IP | AX_RXCOE_TCP | AX_RXCOE_UDP |
	       AX_RXCOE_TCPV6 | AX_RXCOE_UDPV6;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RXCOE_CTL, 1, 1, tmp);

	if (((*pkt_hdr & AX_RXHDR_L4_TYPE_MASK) == AX_RXHDR_L4_TYPE_TCP) ||
	    ((*pkt_hdr & AX_RXHDR_L4_TYPE_MASK) == AX_RXHDR_L4_TYPE_UDP))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
}

static void ax88179_rx_fixup
(struct ax_device *axdev, struct rx_desc *desc, int *work_done, int budget)
{
	u8 *rx_data;
	u32 const actual_length = desc->urb->actual_length;
	u32 rx_hdr = 0, pkt_hdr = 0, pkt_hdr_curr = 0, hdr_off = 0;
	u32 aa = 0;
	int pkt_cnt = 0;
	struct net_device *netdev = axdev->netdev;
	struct net_device_stats *stats = ax_get_stats(netdev);

	memcpy(&rx_hdr, (((u8 *)desc->head) + actual_length - 4),
	       sizeof(rx_hdr));
	le32_to_cpus(&rx_hdr);

	pkt_cnt = rx_hdr & 0xFF;
	hdr_off = rx_hdr >> 16;
	pkt_hdr_curr = hdr_off;

	aa = (actual_length - (((pkt_cnt + 2) & 0xFE) * 4));
	if (aa != hdr_off ||
	    hdr_off >= desc->urb->actual_length ||
	    pkt_cnt == 0) {
		desc->urb->actual_length = 0;
		stats->rx_length_errors++;
		return;
	}

	rx_data = desc->head;
	while (pkt_cnt--) {
		u32 pkt_len;
		struct sk_buff *skb;

		memcpy(&pkt_hdr, (((u8 *)desc->head) + pkt_hdr_curr),
		       sizeof(pkt_hdr));
		pkt_hdr_curr += 4;

	/* Check if WoL is supported */
	ax179_data->wol_supported = 0;
	if (ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MOD,
			     1, 1, &tmp) > 0)
		ax179_data->wol_supported = WAKE_MAGIC | WAKE_PHY;

	ax88179_led_setting(dev);

		if (pkt_hdr & AX_RXHDR_CRC_ERR) {
			stats->rx_crc_errors++;
			goto find_next_rx;
		}
		if (pkt_hdr & AX_RXHDR_DROP_ERR) {
			stats->rx_dropped++;
			goto find_next_rx;
		}

		skb = napi_alloc_skb(napi, pkt_len);
		if (!skb) {
			stats->rx_dropped++;
			goto find_next_rx;
		}

		memcpy(skb->data, rx_data, pkt_len);
		skb_put(skb, pkt_len);

		ax88179_rx_checksum(skb, &pkt_hdr);

		skb->protocol = eth_type_trans(skb, netdev);

		if (*work_done < budget) {
			napi_gro_receive(&axdev->napi, skb);
			*work_done += 1;
			stats->rx_packets++;
			stats->rx_bytes += pkt_len;
		} else {
			__skb_queue_tail(&axdev->rx_queue, skb);
		}
find_next_rx:
		rx_data += (pkt_len + 7) & 0xFFF8;
	}
}

static int ax88179_system_suspend(struct ax_device *axdev)
{
	u16 reg16;

	ax_read_cmd_nopm(axdev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			 2, 2, &reg16, 1);
	reg16 &= ~AX_MEDIUM_RECEIVE_EN;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			  2, 2, &reg16);

	ax_read_cmd_nopm(axdev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			 2, 2, &reg16, 1);
	reg16 |= AX_PHYPWR_RSTCTL_IPRL;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			  2, 2, &reg16);

	reg16 = AX_RX_CTL_STOP;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &reg16);

	return 0;
}

static int ax88179_system_resume(struct ax_device *axdev)
{
	u16 reg16;
	u8 reg8;

	reg16 = 0;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, &reg16);
	usleep_range(1000, 2000);

	reg16 = AX_PHYPWR_RSTCTL_IPRL;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, &reg16);
	msleep(200);

	ax88179_auto_detach(axdev, 1);

	ax_read_cmd_nopm(axdev, AX_ACCESS_MAC,  AX_CLK_SELECT, 1, 1, &reg8, 0);
	reg8 |= AX_CLK_SELECT_ACS | AX_CLK_SELECT_BCS;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &reg8);
	msleep(100);

	reg16 = AX_RX_CTL_START | AX_RX_CTL_AP |
		AX_RX_CTL_AMALL | AX_RX_CTL_AB;
	ax_write_cmd_nopm(axdev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &reg16);

	return 0;
}

const struct driver_info ax88179_info = {
	.bind = ax88179_bind,
	.unbind = ax88179_unbind,
	.hw_init = ax88179_hw_init,
	.stop = ax88179_stop,
	.link_reset = ax88179_link_reset,
	.rx_fixup = ax88179_rx_fixup,
	.tx_fixup = ax88179_tx_fixup,
	.system_suspend = ax88179_system_suspend,
	.system_resume = ax88179_system_resume,
	.napi_weight = AX88179_NAPI_WEIGHT,
	.buf_rx_size = AX88179_BUF_RX_SIZE,
};

static const struct usb_device_id products[] = {
{
	/* ASIX AX88179 10/100/1000 */
	USB_DEVICE(0x0b95, 0x1790),
	.driver_info = (unsigned long)&ax88179_info,
}, {
	/* ASIX AX88178A 10/100/1000 */
	USB_DEVICE(0x0b95, 0x178a),
	.driver_info = (unsigned long)&ax88178a_info,
}, {
	/* Cypress GX3 SuperSpeed to Gigabit Ethernet Bridge Controller */
	USB_DEVICE(0x04b4, 0x3610),
	.driver_info = (unsigned long)&cypress_GX3_info,
}, {
	/* D-Link DUB-1312 USB 3.0 to Gigabit Ethernet Adapter */
	USB_DEVICE(0x2001, 0x4a00),
	.driver_info = (unsigned long)&dlink_dub1312_info,
}, {
	/* Sitecom USB 3.0 to Gigabit Adapter */
	USB_DEVICE(0x0df6, 0x0072),
	.driver_info = (unsigned long)&sitecom_info,
}, {
	/* Samsung USB Ethernet Adapter */
	USB_DEVICE(0x04e8, 0xa100),
	.driver_info = (unsigned long)&samsung_info,
}, {
	/* Lenovo OneLinkDock Gigabit LAN */
	USB_DEVICE(0x17ef, 0x304b),
	.driver_info = (unsigned long)&lenovo_info,
}, {
	/* Belkin B2B128 USB 3.0 Hub + Gigabit Ethernet Adapter */
	USB_DEVICE(0x050d, 0x0128),
	.driver_info = (unsigned long)&belkin_info,
}, {
	/* Toshiba USB 3.0 GBit Ethernet Adapter */
	USB_DEVICE(0x0930, 0x0a13),
	.driver_info = (unsigned long)&toshiba_info,
}, {
	/* Magic Control Technology U3-A9003 USB 3.0 Gigabit Ethernet Adapter */
	USB_DEVICE(0x0711, 0x0179),
	.driver_info = (unsigned long)&mct_info,
},
	{ },
};
MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver ax88179_178a_driver = {
	.name =		"ax88179_178a",
	.id_table =	products,
	.probe =	usbnet_probe,
	.suspend =	ax88179_suspend,
	.resume =	ax88179_resume,
	.reset_resume =	ax88179_resume,
	.disconnect =	ax88179_disconnect,
	.supports_autosuspend = 1,
	.disable_hub_initiated_lpm = 1,
};

module_usb_driver(ax88179_178a_driver);

MODULE_DESCRIPTION("ASIX AX88179/178A based USB 3.0/2.0 Gigabit Ethernet Devices");
MODULE_LICENSE("GPL");
