

#pragma once

void rxNRF24Init(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
 bool rxNRF24ReceivePacket();