#!/bin/sh

SWU_TOOL=/sbin/swupdate_cmd.sh
OTA_FILE=$1

[ -e $OTA_FILE ] && {
    swu_current_system=$(fw_printenv -n boot_partition 2>/dev/null)

    if [ "x$swu_current_system" = "xbootA" ]; then
        $SWU_TOOL -i $OTA_FILE -e stable,now_A_next_B
    elif [ "x$swu_current_system" = "xbootB" ]; then
        $SWU_TOOL -i $OTA_FILE -e stable,now_B_next_A
    elif [ "x$swu_current_system" = "xboot" ]; then
        SDCARD_PATH=/mnt/SDCARD/OTA
        if mkdir -p $SDCARD_PATH; then
            rm -rf $SDCARD_PATH/*.img
            img_name=$(basename "$OTA_FILE")
            cp -rf $OTA_FILE $SDCARD_PATH/$img_name && sync
            OTA_FILE=$SDCARD_PATH/$img_name
        fi
        $SWU_TOOL -i $OTA_FILE -e stable,upgrade_recovery
    else
        echo "unknown current system!"
        exit 1
    fi
}

