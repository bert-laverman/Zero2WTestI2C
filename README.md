# Zero2WTestI2C
Let a Raspberry Pi Zero 2W manage Picos using I2C

## Usage

```bash
./test-i2c [seconds [address ...]]
```

The program is the bus controller: it says `Hello` once per second, hands out addresses to the boards that ask for one,
and keeps them in `i2c-state.ini`, so a board gets the same address again after a restart. It runs for `seconds` seconds
(default 30).

If you also give the address of a board with a MAX7219 8-digit display (see `PicoTestI2C`), for example
`./test-i2c 30 0x61`, the display counts the seconds, using `RemoteMAX7219`. With more addresses (`./test-i2c 30 0x61 0x62`) the second display counts down, the third counts up again, and so on.

To compare displays side by side, start it with `MAX_DEMO=brightness`. Every 4 seconds they then go through: all segments
on at full strength (the test mode of the chip, whatever the brightness), and `88888888` at brightness 0, 3, 7 and 15.
