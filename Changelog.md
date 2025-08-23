# Change Log

## [0.4.0] - 2025-08-23

### Changed
  - Core: the internal buffer was extended to support 256 data bytes and made externally accessible
    through buffer() and buffer_mut().
  - Read/Write FIFO where changed and split in two:
    * wr_tx_fifo/rd_rx_fifo changed to take only a length argument and use the internal buffer
    * wr_tx_fifo_to/wr_rx_fifo_from corresponds to the old API when a buffer is provided to the command

### Added
  - PatchRam: get_pram_info allows to check if a patch RAM has been set and which version number it contains.
  - API for Zigbee: set_zigbee_packet, set_zigbee_packet_len, set_zigbee_address, get_zigbee_packet_status, get_zigbee_rx_stats
  - API for LR-FHSS: lrfhss_build_packet, set_lrfhss_syncword and set_lrfhss_hopping.


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
