# Temperature probe

This project reads the temperature from a DHT11 device and displays is on an SSD1306 screen, using a Raspberry Pi Pico as microcontroller.

## Usage guide

```bash
cargo build --releaase
elf2uf2 target/thumbv6m-none-eabi/release/rp-temp rp-temp.uf2
# Then boot your Pico in bootsel mode and copy the uf2 file to the drive.
```