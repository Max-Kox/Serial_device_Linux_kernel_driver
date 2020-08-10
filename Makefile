ifneq ($(KERNELRELEASE),)

obj-m += serial.o

else

KDIR = /home/max/Embedded/Repos/Linux_Kernel_stable

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

endif
