#include <chdebug.h>

namespace amiro {

template <bool ignore_deselect>
PCA9544<ignore_deselect>::
PCA9544(I2CDriver *driver, const uint8_t master_id) :
	I2CMultiplexer(driver),
	master_id(master_id),
	selected(-1) {

	chDbgCheck(master_id <= 0x07u, "PCA9544 ctor master_id");
	this->tx_params.addr = (PCA9544::SLA | this->master_id);
	this->rx_params.addr = (PCA9544::SLA | this->master_id);

}

template <bool ignore_deselect>
msg_t
PCA9544<ignore_deselect>::
select(const uint8_t bus_id) {

	chDbgCheck(bus_id <= 0x03u, "PCA9544 select bus_id");

	uint8_t tmp = PCA9544::BUS_ON | bus_id;

	if (bus_id == this->selected)
		return RDY_OK;

	this->selected = bus_id;

	this->tx_params.txbuf = &tmp;
	this->tx_params.txbytes = 1;
	this->tx_params.rxbytes = 0;
	return this->masterTransmit(&this->tx_params); // TODO select timeout

}

template <>
inline msg_t
PCA9544<true>::
deselect() {

	return RDY_OK;

}

template <>
inline msg_t
PCA9544<false>::
deselect() {

	uint8_t tmp = PCA9544::BUS_OFF;

	this->selected = -1;
	this->tx_params.txbuf = &tmp;
	this->tx_params.txbytes = 1;
	this->tx_params.rxbytes = 0;
	return this->masterTransmit(&this->tx_params); // TODO select timeout

}

template <bool ignore_deselect>
uint8_t
PCA9544<ignore_deselect>::
getInterruptSource() {

	uint8_t tmp;
	this->rx_params.rxbuf = &tmp;
	this->rx_params.rxbytes = 1;
	this->masterReceive(&this->rx_params); // TODO select timeout
	return tmp;

}

template <bool ignore_deselect>
uint8_t
PCA9544<ignore_deselect>::
getCheck(void) {

	const int8_t rxBufferMask = 0x07; /** We are only intrested in the first three bits*/
	int8_t txBuffer, resTx, resRx;
	uint8_t rxBuffer[2];

	// Copy the data, because some virtual driver can write into it
	I2CTxParams tx_params = this->tx_params;
	I2CRxParams rx_params = this->rx_params;

	tx_params.txbuf = (uint8_t*) &txBuffer;
	tx_params.txbytes = 1;
	tx_params.rxbytes = 0;
	rx_params.rxbuf = rxBuffer;
	rx_params.rxbytes = 2; /** This is a bug workaround: 1 byte is impossible to read*/

	for (txBuffer = rxBufferMask; txBuffer >= 0x00; txBuffer--) {
		this->acquireBus();
		resTx = this->masterTransmit(&tx_params);
		resRx = this->masterReceive(&rx_params);
		this->releaseBus();
		// Failed to transmit
		if (resTx != RDY_OK || resRx != RDY_OK) {
			break;
		}
		// Check data written to MUX register
		if (txBuffer != (rxBuffer[0] & rxBufferMask)) {
			break;
		}
	}

	// Data analysis
	if (txBuffer != -1) {
		return PCA9544::CHECK_FAIL;
	} else {
		return PCA9544::CHECK_OK;
	}
}

} /* amiro */
