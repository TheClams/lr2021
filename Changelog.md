# Change Log

## [0.5.0] - Unreleased

### Changed
  - FSK: dc_free parameter is now a bool in set_fsk_packet
  - ZWave: timeout now uses the recommended value based on number of channel used

### Added
  - LoRa: add methods related to ranging
  - WM-Bus: API
  - Wisun: API
  - BPSK: API (TX only)

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
