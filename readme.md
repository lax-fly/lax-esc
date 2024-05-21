# features

1. support DSHOT ONESHOT MULTISHOT PWM and BRUSH protocols, and 3D mode.
2. support protocol switching automatically at runtime
3. support PWM frequency up to 50kHz
4. support dynamic PWM frequence
5. support debug mode by serial port
6. support motor stalling prevention
7. throttle response rate up to 50/ms, which means 40ms for full throttle(2000)

# requirements

1. Linux shell or Cygwin for windows.
2. arm-none-eabi-gcc 10.3.
3. J-Link adapter.

# how to compile

compiling steps like this:

```bash
make -j # if gcc is not in your environment, use GCC_PATH=xxxxxx to specify it
make download

# or
make BOARD=at32f421_v1_1 -j # the target board can be listed by 'make list', the board name should be the same as the suffix of file in the 'board' directory with prefix name 'board_'
```

watch this [video]() for more details.

# debug mode

```bash
make clean
make -j DEBUG=1
make download
```

the serial print is only available in debug mode, the baudrate is fixed to 256000
to shake hands with esc using SERIAL protocol, the host must use baudrate 256000 to send at least 48 continuous 0xff to the esc.
once using SERIAL protocol, the esc will never switch to other protocol automatically except reset.

Warning: debug mode is costly, which increse the risk of stalling at fast rpm.

# SERIAL protocol

1. shake hands

send more than 48 hex values of 0xff `0xff 0xff ...` to the esc.

2. arm the esc

send string `arm` to the esc

3. spin the motor

send string `throttle -2000~2000` to esc, e.g. `throttle -500`

# hardware designation

[click here](https://oshwhub.com/lax-fly/lax-esc-dev)

# to be done