# comet
Rust on PiZero

# Versions
deadbeef-wsl = cargo 1.69.0 (6e9a83356 2023-04-12)
astro-linux  = cargo 1.69.0 (6e9a83356 2023-04-12)
# Linux setup
rustup target add arm-unknown-linux-gnueabihf
# Add the rpi tools to use the rpi linker for the broadcom chip used on the Pi Zero
git clone https://github.com/raspberrypi/tools $HOME/rpi_tools
Then, update .cargo/config to include:
[target.arm-unknown-linux-gnueabihf]
linker = "/home/{USER}/rpi_tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc"

## Must ensure libudev is not a dependency
libudev is removed by removing default features in Cargo.toml: serialport = {version = "4.2.2", default-features = false } 

# Always Clean and Build
cargo clean
cargo build --release --target=arm-unknown-linux-gnueabihf