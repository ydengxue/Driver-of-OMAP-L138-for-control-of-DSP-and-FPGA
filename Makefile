ifneq ($(KERNELRELEASE),)
  obj-m := PeripheralDownload.o
else
  KERNELDIR=/home/yandx/work/linux-03.21.00.04
  PWD := $(shell pwd)

  default :
	$(MAKE) ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- -C $(KERNELDIR) M=$(PWD) modules 

  clean :
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers *.order

endif