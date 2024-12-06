.PHONY: all
ARCH=arm64
KERNEL=kernel8
CROSS_COMPILE=aarch64-linux-gnu-
KDIR=${HOME}/sources/rasp/linux # adapt

build:
	$(MAKE) ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -C ${KDIR} M=$(PWD) KDIR=${KDIR}

clean:
	$(MAKE) ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -C ${KDIR} M=$(PWD) clean
