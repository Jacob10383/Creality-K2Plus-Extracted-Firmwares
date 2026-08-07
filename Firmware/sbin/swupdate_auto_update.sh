#!/bin/sh

APP_UPDATE_TOOL=/mnt/UDISK/app_ota_cfg_update.sh
SWU_TOOL=/sbin/swupdate_cmd.sh
LOG_FILE=/mnt/UDISK/swupdate.log
STATUS_FILE=/mnt/UDISK/creality/upgrade/upgrade_script_status.json

OTA_UPDATE_FILE=""

log()
{
    echo "$(date '+%Y-%m-%d %H:%M:%S') [recovery] $1" | tee -a $LOG_FILE
}


update_status()
{
    local status=$1

    rm -rf "$STATUS_FILE"
    mkdir -p $(dirname "$STATUS_FILE")

    if [ $status = 1 ]; then
        result_msg="ota update ok"
    else
        result_msg="ota update failed"
    fi

    old_version=$(fw_printenv -n old_version 2>/dev/null)
    target_version=$(fw_printenv -n version 2>/dev/null)
    board=$(fw_printenv -n board 2>/dev/null)
    image_path=$(fw_printenv -n swu_param 2>/dev/null | sed 's/^-i *//')

    log "status: $status"
    log "board: $board"
    log "target_sys_version: $target_version"
    log "old_sys_version: $old_version"
    log "image_path: $image_path"

    tee $STATUS_FILE << EOF
{
  "version": 1,
  "type": "linux_local_ota",
  "status": $status,
  "board": "$board",
  "target_sys_version": "$target_version",
  "old_sys_version": "$old_version",
  "image_path": "$image_path",
  "result_msg": "$result_msg"
}
EOF

    sync
}


app_update_status()
{
    if [ ! -f "$APP_UPDATE_TOOL" ]; then
        log "not found: $APP_UPDATE_TOOL,status: $1"
        return 1
    fi

    $APP_UPDATE_TOOL $1
    rm -rf "$APP_UPDATE_TOOL"
    sync
    return 0
}


check_ota_necessity()
{
    local swu_mode=$(fw_printenv -n swu_mode 2>/dev/null)
    if [ -z "$swu_mode" ]; then
        log "not update mode"
        return 1
    fi

    local ota_file=$(fw_printenv -n swu_param 2>/dev/null | sed 's/^-i *//')
    if [ -z "$ota_file" ]; then
        log "No OTA file found in swu_param"
        return 1
    fi

    OTA_UPDATE_FILE="$ota_file"
    log "$OTA_UPDATE_FILE"
    return 0
}


check_img_status()
{
    local max_retries=10
    local retry=0
    while [ $retry -lt $max_retries ]; do
        if [ -f "$OTA_UPDATE_FILE" ]; then
            log "found: $OTA_UPDATE_FILE"
            return 0
        fi
        
        retry=$((retry + 1))
        if [ $retry -lt $max_retries ]; then
            log "wait file (attempt $retry/$max_retries), retrying in 1 second..."
            sleep 1
        fi
    done
    
    log "OTA file not found after $max_retries attempts: $OTA_UPDATE_FILE"
    return 1
}


update_system()
{
    if [ ! -f "$SWU_TOOL" ]; then
        log "SWU tool not found: $SWU_TOOL"
        return 1
    fi

    local swu_mode=$(fw_printenv -n swu_mode 2>/dev/null)
    log "Current swu_mode: $swu_mode"

    if [ "x$swu_mode" = "xupgrade_system" ]; then
        log "Continuing upgrade_system stage..."
        $SWU_TOOL -i "$OTA_UPDATE_FILE" -e stable,upgrade_system
    else
        log "Starting upgrade_recovery stage..."
        $SWU_TOOL -i "$OTA_UPDATE_FILE" -e stable,upgrade_recovery
    fi

    if [ $? -eq 0 ]; then
        log "OTA update completed successfully"
        return 0
    else
        log "OTA update failed"
        return 1
    fi
}


main_update()
{
    check_ota_necessity
    if [ $? -ne 0 ]; then
        log "not need update, skipping update"
        return 0
    fi

    check_img_status
    if [ $? -ne 0 ]; then
        log "OTA file not found, skipping update"
        update_status 2
        fw_setenv boot_partition boot
        fw_setenv old_version
        app_update_status "failed"
        reboot -f
        return 0
    fi

    update_system
    if [ $? -ne 0 ]; then
        log "update failed"
        update_status 2
        app_update_status "failed"
        fw_setenv boot_partition boot
    else
        log "update ok"
        update_status 1
        app_update_status "success"
    fi

    fw_setenv old_version
    reboot -f
}


main_update

