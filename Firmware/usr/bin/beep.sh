#!/bin/sh

# PWM6
# PWM_INDEX=6

# pwm_setting()
# {
#     if [ -e /sys/class/pwm/pwmchip0/export ]; then
#         [ -d /sys/class/pwm/pwmchip0/pwm${PWM_INDEX} ] || {
#             echo $PWM_INDEX > /sys/class/pwm/pwmchip0/export
#         }

#         # PWM 4KHz, 50%
#         echo 250000 > /sys/class/pwm/pwmchip0/pwm${PWM_INDEX}/period
#         echo 125000 > /sys/class/pwm/pwmchip0/pwm${PWM_INDEX}/duty_cycle
#     fi
# }

# beep()
# {
#     echo 1 > /sys/class/pwm/pwmchip0/pwm${PWM_INDEX}/enable
#     sleep $1
#     echo 0 > /sys/class/pwm/pwmchip0/pwm${PWM_INDEX}/enable
# }

gpio_setting()
{
    if [ ! -e /sys/class/gpio/gpio164 ]; then
        echo 164 > /sys/class/gpio/export
    fi

    echo out > /sys/class/gpio/gpio164/direction
}

beep()
{
    echo 1 > /sys/class/gpio/gpio164/value 
    sleep $1
    echo 0 > /sys/class/gpio/gpio164/value
}

if [ $# -eq 1 ]; then

    # pwm_setting
    gpio_setting

    if [  `expr $1 \> 0.0` -eq 1 -a `expr $1 \< 1.0` -eq 1 ]; then
        seconds=$1
    elif [ `expr $1 \> 30` -eq 1 ]; then
        seconds=30
    elif [ `expr $1 \<= 0` -eq 1 ]; then
        seconds=0.1
    else
        seconds=$1
    fi

    beep $seconds
else
    echo "  Usage: beep.sh <seconds>"
    exit 1
fi
