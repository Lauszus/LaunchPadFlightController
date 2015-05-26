# OpenOCD
```bash
$ openocd --file /usr/local/share/openocd/scripts/board/ek-tm4c123gxl.cfg

$ arm-none-eabi-gdb gcc/LaunchPadFlightController.axf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) monitor reset init
```

# LMICDI

```bash
$ lmicdi

$ arm-none-eabi-gdb gcc/LaunchPadFlightController.axf
(gdb) target remote :7777
(gdb) monitor reset halt
(gdb) load
(gdb) monitor reset init
```