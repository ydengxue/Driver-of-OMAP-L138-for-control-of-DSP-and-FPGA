#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
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
	{ 0xcf817b95, "module_layout" },
	{ 0x5f403789, "misc_deregister" },
	{ 0x5f754e5a, "memset" },
	{ 0x7569558f, "misc_register" },
	{ 0xfa2a45e, "__memzero" },
	{ 0x17a142df, "__copy_from_user" },
	{ 0xea147363, "printk" },
	{ 0x448212fa, "down_trylock" },
	{ 0x8cf51d15, "up" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

