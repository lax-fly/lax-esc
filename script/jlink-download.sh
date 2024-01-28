
#################################
# param 1: Device name
# param 2: HEX file path
################################

JLINK_PATH="C:/Program Files/SEGGER/JLink/JLink.exe"

MCU_FLY=$1
HEX_FILE=$2

case $MCU_FLY in
    AT32F421?8*)
    DEVICE=-AT32F421G8U7
    ;;
    *)
    echo "error: unkown MCU family: $MCU_FLY"
    exit -1
    ;;
esac

########## J-Link command line script ###########
# 选择SW接口
# 设置接口速率4000kHz
# 设置芯片型号
# 复位芯片
# 暂停芯片
# 擦除
# 下载程序
# 运行
# 退出jlink命令行
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