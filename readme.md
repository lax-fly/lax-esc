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
```

# to be done