# Change Log

## [0.8.0] - 2025-09-07

### Added
  - API for Raw IQ TX/capture
  - Add method to patch RF setting for ranging as this needs to be done after set_packet_type
  - Add LoRa API for hidden features: set_lora_freq_range and set_sidedet_halfbin

### Changed
  - WMbus PktFormat renamed to WmbusFormat
  - LoRa:
    * use struct to define modulation and packet params
    * SF and LoRaBw implements Ord (i.e. can use comparison operators)

## [0.7.0] - 2025-09-07

### Added
  - LoRa: add API to retrieve FEI
  - Ranging:
    * add method to get the base TXRX Delay depending on bandwidth and SF
    * add set_rf_ranging to ensure constant delay whatever the RF selected
    * add method to retrieve the ranging RSSI correction offset
  - Ranging:

### Fixed
  - Fix command value of SetRxDutyCycle
  - Ranging: rssi2 has been removed (always null)
  - Patch RAM is now properly enabled after being loaded (to be tested)

## [0.6.0] - 2025-09-03

### Changed
  - Improved documentation (doc(alias), Summary of methods in each protocol file, ...)
  - Use enum for RxBoost in `set_rx_path`
  - Use enum for DIO in `set_dio_function_cmd`, `set_dio_irq_config_cmd`, `set_dio_rf_switch_config_cmd` and `set_lora_timing_sync`

### Added
  - Add API to control DIO functions (`set_dio_function` and `set_dio_rf_switch`)
  - Add API related to packet timestamp (`set_timestamp_source` and `get_timestamp`)
  - Add API related to TX/RS (`set_tx_test_mode`, `set_auto_txrx` and `set_rx_duty_cycle`)

## [0.5.0] - 2025-08-31

### Changed
  - FSK: dc_free parameter is now a bool in set_fsk_packet
  - ZWave:
    * timeout now uses the recommended value based on number of channel used
    * set_packet now uses a struct which can be build based on the Zwave mode

### Added
  - LoRa: add methods related to ranging, timing synch, preamble modulation
  - WM-Bus: API
  - Wisun: API
  - BPSK: API (TX only)
  - System:
    * `rd_mem` API allows reading a chunk of the chip memory (mainly for debug or multi-register dump)
    * `ErrorsRsp` now has a defmt::Format implementation as-well as a direct access to the 32b value

## [0.4.0] - 2025-08-23

### Changed
  - Core: the internal buffer was extended to support 256 data bytes and made externally accessible
    through `buffer()` and `buffer_mut()`.
  - Read/Write FIFO where changed and split in two:
    * `wr_tx_fifo`/`rd_rx_fifo` changed to take only a length argument and use the internal buffer
    * `wr_tx_fifo_to`/`wr_rx_fifo_from` corresponds to the old API when a buffer is provided to the command
  - FLRC set packet now uses a structures as input parameter (`FlrcPacket_Params::new(...)`)

### Added
  - PatchRam: `get_pram_info` allows to check if a patch RAM has been set and which version number it contains.
  - API for Zigbee: `set_zigbee_packet`, `set_zigbee_packet_len`, `set_zigbee_address`, `get_zigbee_packet_status`, `get_zigbee_rx_stats`
  - API for ZWave: `set_zwave_packet`, `set_zwave_home_id`, `set_zwave_beam_filt`, `set_zwave_scan_config`, `start_zwave_scan`, `get_zwave_packet_status`, `get_zwave_rx_stats`
  - API for LR-FHSS: `lrfhss_build_packet`, `set_lrfhss_syncword` and `set_lrfhss_hopping`.
  - All enums from command now derive defmt::Format if option is enabled


## [0.3.0] - 2025-08-18

### Added
  - API for legacy FSK modulation (using same packet format as previous Semtech chips)
  - API for FSK CAD/CCA (set_cad_params, set_cad, set_cca, get_cca_result)
  - API for first RSSI measurements (get_rssi_avg/inst)


## [0.2.0] - 2025-08-17

### Added
  - OOK: add API methods to configure the OOK receiver including configuration for ADS-B
  - Radio: `set_rx_gain` for manual gain setting and `force_crc_out` to always output CRC on the RX FIFO
  - System: add function to read and write registers
