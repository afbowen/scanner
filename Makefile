CSRC = $(shell find src -name "*.c")
INCDIR = ./include

MODULE_SEARCH_DIRS = modules

MODULES_ENABLED = \
chibios_sys_init \
chibios_hal_init \
app_descriptor \
boot_msg \
timing \
system \
pubsub \
worker_thread \
can_driver_stm32 \
can \
can_autobaud \
uavcan \
uavcan_nodestatus_publisher \
uavcan_getnodeinfo_server \
uavcan_beginfirmwareupdate_server \
uavcan_allocatee \
uavcan_restart \
uavcan_debug \
freemem_check \
nanotec_controller

MESSAGES_ENABLED = \
uavcan.protocol.debug.LogMessage \
com.matternet.equipment.scanner.BarcodeContent

VENDOR_DSDL_NAMESPACE_DIRS = ./dsdl/com

include framework/include.mk
