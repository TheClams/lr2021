# Driver for Semtech LR2021 transceiver

## General setup

---
## Radio protocols

### LoRa
Support all previous LoRa chips (dedicated legacy mode for SX127x syncword and SF6) and includes high bandwidth (1Mb/s) and better FEC.

*Status:* Basic test between two LR2021

### BLE
Bluetooth Low-Energy fully suppported, including BLE Coded.

*Status:* Tested only for BLE 1MB/s, both between two LR2021 and with other chips.

### FLRC (Fast Long Range Communication)
Semtech proprietary protocol (G)MSK modulation with high sensitivity.

*Status:* Basic test between two LR2021

### Generic FSK (Frequency Shift Keying)

*Status:* Tested between two LR2021

### WiSUN (Wireless Smart Utility Network)

### WMBus (Wireless MBus)

### OOK (On-Off Keying)

*Status:* Only ADSB reception validated

### LR-FHSS
Semtech proprietary protocol using ultra-narrow-band modulation with frequency hopping. Only transmission is supported.

### BPSK
Sigfox up-link modulation, TX only.

---
## Examples
A few simple applications using this driver are available
in the Github repository [lr2021-apps](https://github.com/TheClams/lr2021-apps)
