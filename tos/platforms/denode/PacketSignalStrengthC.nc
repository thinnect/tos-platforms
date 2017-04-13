/**
 * RSSI calculation for Atmel atmega RFA1 / RFR2.
 * @author Raido Pahtma
 * @license MIT
*/
module PacketSignalStrengthC {
	provides interface PacketField<int8_t> as PacketSignalStrength;
	uses interface PacketField<uint8_t> as PacketRSSI;
}
implementation {

	async command bool PacketSignalStrength.isSet(message_t* msg) {
		return call PacketRSSI.isSet(msg);
	}

	async command int8_t PacketSignalStrength.get(message_t* msg) {
		return 3*(call PacketRSSI.get(msg)-1) - 90;  // P_RF = RSSI_BASE_VAL + 3 * (RSSI - 1) [dBm]
	}

	async command void PacketSignalStrength.clear(message_t* msg) {
		call PacketRSSI.clear(msg);
	}

	async command void PacketSignalStrength.set(message_t* msg, int8_t value) {
		uint8_t v = 0;
		if(value >= -90) {
			v = (90 + value)/3 + 1;
		}
		call PacketRSSI.set(msg, v);
	}

}
