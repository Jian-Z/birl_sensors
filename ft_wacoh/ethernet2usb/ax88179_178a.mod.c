#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xc6c01fa, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x16abfc10, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0xe4873a5b, __VMLINUX_SYMBOL_STR(ethtool_op_get_link) },
	{ 0x53331fd3, __VMLINUX_SYMBOL_STR(usbnet_set_msglevel) },
	{ 0x8bcaca47, __VMLINUX_SYMBOL_STR(usbnet_get_msglevel) },
	{ 0x37fe39dd, __VMLINUX_SYMBOL_STR(usbnet_tx_timeout) },
	{ 0x8d97a66a, __VMLINUX_SYMBOL_STR(eth_validate_addr) },
	{ 0x702aa167, __VMLINUX_SYMBOL_STR(usbnet_start_xmit) },
	{ 0xc417a984, __VMLINUX_SYMBOL_STR(usbnet_stop) },
	{ 0x9c9d7fa9, __VMLINUX_SYMBOL_STR(usbnet_open) },
	{ 0xd94e3726, __VMLINUX_SYMBOL_STR(usbnet_disconnect) },
	{ 0xd506f96d, __VMLINUX_SYMBOL_STR(usbnet_probe) },
	{ 0x56adb695, __VMLINUX_SYMBOL_STR(usb_deregister) },
	{ 0x148229db, __VMLINUX_SYMBOL_STR(usb_register_driver) },
	{ 0x7147434e, __VMLINUX_SYMBOL_STR(usbnet_write_cmd_async) },
	{ 0x4c759827, __VMLINUX_SYMBOL_STR(byte_rev_table) },
	{ 0x393d4de9, __VMLINUX_SYMBOL_STR(crc32_le) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xef45c8e7, __VMLINUX_SYMBOL_STR(usbnet_get_endpoints) },
	{ 0xd08d0d14, __VMLINUX_SYMBOL_STR(netdev_err) },
	{ 0x720a88d5, __VMLINUX_SYMBOL_STR(mii_nway_restart) },
	{ 0xeb67d5b7, __VMLINUX_SYMBOL_STR(__dynamic_netdev_dbg) },
	{ 0x1f474eef, __VMLINUX_SYMBOL_STR(usbnet_defer_kevent) },
	{ 0x1846825f, __VMLINUX_SYMBOL_STR(netif_carrier_on) },
	{ 0x876c103c, __VMLINUX_SYMBOL_STR(usbnet_suspend) },
	{ 0x757f9d0a, __VMLINUX_SYMBOL_STR(usbnet_resume) },
	{ 0x12a38747, __VMLINUX_SYMBOL_STR(usleep_range) },
	{ 0xb7eb0c3d, __VMLINUX_SYMBOL_STR(netif_carrier_off) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xf244a845, __VMLINUX_SYMBOL_STR(mii_check_media) },
	{ 0xf22a36db, __VMLINUX_SYMBOL_STR(netdev_info) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x920c2877, __VMLINUX_SYMBOL_STR(usbnet_write_cmd_nopm) },
	{ 0x562b80e8, __VMLINUX_SYMBOL_STR(usbnet_write_cmd) },
	{ 0xb7afaf64, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0xb4606292, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xc21cd52a, __VMLINUX_SYMBOL_STR(netdev_warn) },
	{ 0xf5c334d, __VMLINUX_SYMBOL_STR(usbnet_read_cmd_nopm) },
	{ 0xd1cc32e, __VMLINUX_SYMBOL_STR(usbnet_read_cmd) },
	{ 0xfdd4b459, __VMLINUX_SYMBOL_STR(generic_mii_ioctl) },
	{ 0x7fbe6dff, __VMLINUX_SYMBOL_STR(mii_ethtool_gset) },
	{ 0x5e1c3d0f, __VMLINUX_SYMBOL_STR(mii_ethtool_sset) },
	{ 0xa12fa6c7, __VMLINUX_SYMBOL_STR(usbnet_get_drvinfo) },
	{ 0x22743f5b, __VMLINUX_SYMBOL_STR(usbnet_skb_return) },
	{ 0x3f51808, __VMLINUX_SYMBOL_STR(skb_pull) },
	{ 0xb2fc44b3, __VMLINUX_SYMBOL_STR(skb_clone) },
	{ 0xfff0707c, __VMLINUX_SYMBOL_STR(skb_trim) },
	{ 0x8cca5c63, __VMLINUX_SYMBOL_STR(__pskb_pull_tail) },
	{ 0xb0e602eb, __VMLINUX_SYMBOL_STR(memmove) },
	{ 0x9b18c7b6, __VMLINUX_SYMBOL_STR(__dev_kfree_skb_any) },
	{ 0x1c67f0b5, __VMLINUX_SYMBOL_STR(skb_copy_expand) },
	{ 0xdca08f9b, __VMLINUX_SYMBOL_STR(skb_push) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=usbnet,mii";

MODULE_ALIAS("usb:v0B95p1790d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0B95p178Ad*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0DF6p0072d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v17EFp304Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0930p0A13d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E8pA100d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "ED1D3D33ED9142B3388F964");
