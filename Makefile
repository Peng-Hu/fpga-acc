OVERLAY_DIR=../buildroot_overlay
DRIVER = fpga-acc.ko

obj-m += fpga-acc.o

PWD := $(shell pwd)
KDIR := $(PWD)/../linux

all:
	$(MAKE) -C $(KDIR) ARCH=riscv SUBDIRS=$(PWD) modules

copy:
	cp $(DRIVER) $(OVERLAY_DIR)


clean:
	rm -rvf *.o *.ko *.order *.symvers *.mod.c .tmp_versions .*o.cmd
