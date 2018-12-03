ccflags-y += -I$(srctree)/drivers/media/platform/msm/vidc/
ccflags-y += -I$(srctree)/drivers/media/platform/msm/vidc/vmem/

msm-vidc-vmem-objs := vmem.o

ifdef CONFIG_DEBUG_FS
msm-vidc-vmem-objs += vmem_debugfs.o
endif

obj-$(CONFIG_MSM_VIDC_VMEM) := msm-vidc-vmem.o
