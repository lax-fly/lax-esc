# requirements

1. Linux shell or Cygwin for windows.
2. arm-none-eabi-gcc.
3. J-Link adapter.

# how to compile

compiling steps like this:

```bash
cd makefile
make -j # if gcc is not in your environment, use GCC_PATH=xxxxxx to specify it
make download

# or
make BOARD=at32f421_v1_1 -j # the target board can be listed by 'make list', the board name should be the same as the suffix of file in the 'board' directory with prefix name 'board_'
```

# to be done