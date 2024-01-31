
#################################
# param 1: Device name
# param 2: HEX file path
################################

JLINK_PATH="C:/Program Files/SEGGER/JLink/JLink.exe"

MCU_FLY=$1
HEX_FILE=$2

case $MCU_FLY in
    at32f421?8*)
    DEVICE=-AT32F421G8U7
    ;;
    *)
    echo "error: unkown MCU family: $MCU_FLY"
    exit -1
    ;;
esac

########## J-Link command line script ###########
# select interface sw
# set interface rate to 4000kHz
# set mcu version
# reset mcu
# pause mcu
# erase flash
# download
# run
# exit jlink cmd line
#################################################
echo "
si 1
speed 4000
device $DEVICE
r 
h
erase
loadfile \"$HEX_FILE\"
r
go
q
" > ./jlink-script.txt

"${JLINK_PATH}" ./jlink-script.txt
rm -f ./jlink-script.txt