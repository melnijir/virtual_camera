target=vcmod
vcmod-objs  = vcmodule.o vccontrol.o vcdevice.o vcioctl.o vcvideobuf.o vcfb.o
obj-m = $(target).o
#ccflags-y += -DDEBUG

.PHONY: all
all:
	+$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

.PHONY: clean
clean:
	+$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

$(target).o: $(vcmod-objs)
