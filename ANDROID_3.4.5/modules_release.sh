#!/bin/sh

export TMP=./output
export MODULES_CMN=${TMP}/module_cmn/system/modules/3.4.5-default
export MODULES_OPT=${TMP}/module_opt/system/modules/3.4.5-default
export PACKAGE_CMN=module_cmn.tgz
export PACKAGE_OPT=module_opt.tgz
export WIFI_PATH=./drivers/net/wireless
mkdir -p ${MODULES_CMN}
mkdir -p ${MODULES_OPT}

rm -rf ${PACKAGE_CMN} ${PACKAGE_OPT}

rm ./drivers/cpufreq/cpufreq.c
rm ./arch/arm/kernel/setup.c
rm ./arch/arm/kernel/smp.c
rm ./init/calibrate.c

make distclean
find ./drivers/media/dvb/siano -name "*.[hc]" | xargs rm -rf
cp ../ANDROID_3.4.5_Driver_Obj/ ../obj_tmp -arf
find ../obj_tmp -name ".svn" | xargs rm -rf 
cp ../obj_tmp/* ./ -arf

make Android_defconfig
## make uzImage
make ubin -j6
if [ $? -ne 0 ] ; then
    echo "  *E* Failed to compile kernel, exit!!"
    exit 1
fi

## make .ko
make modules -j6
if [ $? -ne 0 ] ; then
    echo "  *E* Failed to compile modules, exit!!"
    exit 1
fi

## make special WiFi driver
#make -C drivers/net/wireless/sci_wifi_921x KERNEL_DIR=${PWD}

#echo "Do sth for $android_version"
#make -C ${WIFI_PATH}/DPA_MT7601U_ANDROID_JB_SDK42_20130329 LINUX_SRC=${PWD} clean
#make -C ${WIFI_PATH}/DPA_MT7601U_ANDROID_JB_3.0.0.6_20130830 LINUX_SRC=${PWD} -j4
#if [ $? -ne 0 ] ; then
#   echo "  *E* Failed to compile DPA_MT7601U for android4.2 on kernel3.4.5, exit!!"
#   exit 1
#fi
#echo "build 7601 wifi drivers on android4.2 is ok!!!"

#make -C ${WIFI_PATH}/DPO_GPL_MT7601UAP_JB_SDK42_20130329 LINUX_SRC=${PWD} clean
#make -C ${WIFI_PATH}/DPO_MT7601U_ANDROID_AP_3.0.0.6_20130830 LINUX_SRC=${PWD} -j4
#if [ $? -ne 0 ] ; then
#  echo "  *E* Failed to compile DPO_GPL_MT7601UAP_JB_SDK42_20130329 for android4.2 on kernel3.4.5, exit!!"
#  exit 1
#fi

find  . -name "*.ko"  | xargs -i cp  {} ${MODULES_CMN}
mv ${MODULES_CMN}/s_wmt_batt*.ko ${MODULES_OPT}/
mv ${MODULES_CMN}/s_wmt_gsensor*.ko ${MODULES_OPT}/
mv ${MODULES_CMN}/s_wmt_lsensor*.ko ${MODULES_OPT}/
mv ${MODULES_CMN}/s_wmt_ts*.ko ${MODULES_OPT}/
mv ${MODULES_CMN}/mali.ko ${TMP}/module_cmn/system/modules/
mv ${MODULES_CMN}/ump.ko ${TMP}/module_cmn/system/modules/

cd ${TMP}/module_cmn && tar czf ${PACKAGE_CMN} system && cd ${OLDPWD}
cd ${TMP}/module_opt && tar czf ${PACKAGE_OPT} system && cd ${OLDPWD}

mv ${TMP}/module_cmn/${PACKAGE_CMN} ./
mv ${TMP}/module_opt/${PACKAGE_OPT} ./

rm -rf ${TMP}
