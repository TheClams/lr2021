# LR2021 Driver

[![Crates.io](https://img.shields.io/crates/v/lr2021.svg)](https://crates.io/crates/lr2021)
[![Documentation](https://docs.rs/lr2021/badge.svg)](https://docs.rs/lr2021)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/TheClams/lr2021)

An async, no_std Rust driver for the Semtech LR2021 dual-band transceiver, supporting many different radio protocols including LoRa, BLE, ZigBee, Z-Wave, and more.

## Quick Start

Add this to your `Cargo.toml`:

```toml
[dependencies]
lr2021 = "0.5.0"
embassy-time = "0.3"
```

Basic usage:

```rust
use lr2021_driver::Lr2021;

let mut radio = Lr2021::new(reset_pin, busy_pin, spi_device, nss_pin);
radio.reset().await?;
// Configure and use your preferred protocol
```

## Hardware Requirements

- Semtech LR2021 transceiver module
- SPI-capable microcontroller
- 3 GPIO pins: Reset (output), Busy (input), NSS/CS (output) (not counting SPI SCK/MISO/MOSI)
- Embassy-compatible async runtime

## Documentation & Examples

- **[API Documentation](https://docs.rs/lr2021-driver)** - Complete API reference
- **[Example Applications](https://github.com/TheClams/lr2021-apps)** - Real-world usage examples on Nucleo boards

## Protocol Test Status

| Protocol | Status | Notes |
|----------|--------|-------|
| LoRa |**Partial** | Basic communication between two LR2021 devices: Smallest SF, highest bandwidth |
| BLE | **Partial** | 1MB/s mode, compatible with other BLE devices. TODO: 2Mb/s, Coded |
| FLRC | **Tested** | Basic communication between two LR2021 devices, max rate only |
| FSK | **Tested** | Generic FSK communication verified |
| Z-Wave | **Tested** | Scan mode tested with ZStick S2, R1-R3 reception |
| OOK | **Partial** | ADSB reception validated |
| ZigBee | **Planned** |  |
| WiSUN | **Planned** |  |
| WMBus | **Planned** |  |
| LR-FHSS | **Unplanned** | TX only (require gateway for test) |
| Sigfox (BPSK) | **Unplanned** | TX only (require gateway for test) |
