
all: ubin

menuconfig:
	make -C ANDROID_3.4.5 menuconfig
	
ubin:
	make -C ANDROID_3.4.5 ubin -j12

Android_defconfig:
	make -C ANDROID_3.4.5 Android_defconfig
	
modules:
	make -C ANDROID_3.4.5 modules
	#mkdir modules
	#find ANDROID_3.4.5 -name "*.ko" |  xargs -i cp  {} `pwd`/modules/
	
modules_install:
	make -C ANDROID_3.4.5 modules_install
	
clean:	
	make -C ANDROID_3.4.5 clean
	#rm -rf modules
	cp ANDROID_3.4.5_Driver_Obj/* ANDROID_3.4.5/. -arf
	find ANDROID_3.4.5 -name "built-in.o" -exec rm -rf {} \;
	find ANDROID_3.4.5 -name ".*.o.cmd" -exec rm -rf {} \;


