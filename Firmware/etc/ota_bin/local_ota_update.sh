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
        APP_UPDATE_TOOL=/usr/bin/app_ota_cfg_update.sh
        if [ -f "$APP_UPDATE_TOOL" ]; then
            cp -rf $APP_UPDATE_TOOL /mnt/UDISK/
        fi
        SDCARD_PATH=/mnt/SDCARD/OTA
        if mkdir -p $SDCARD_PATH; then
            rm -rf $SDCARD_PATH/*.img
            img_name=$(basename "$OTA_FILE")
            if cp -rf $OTA_FILE $SDCARD_PATH/$img_name; then
                if [ -f "$SDCARD_PATH/$img_name" ]; then
                    OTA_FILE=$SDCARD_PATH/$img_name
                fi
            fi
        fi
        sync
        $SWU_TOOL -i $OTA_FILE -e stable,upgrade_recovery
    else
        echo "unknown current system!"
        exit 1
    fi
}

