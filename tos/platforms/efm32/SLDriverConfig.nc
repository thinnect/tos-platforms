interface SLDriverConfig
{
	/**
	 * Returns the length of a dummy header to align the payload properly.
	 */
	async command uint8_t headerLength(message_t* msg);

	/**
	 * Returns the maximum length of the PHY payload including the 
	 * length field but not counting the FCF field.
	 */
	async command uint8_t maxPayloadLength();

	/**
	 * Returns the length of a dummy metadata section to align the
	 * metadata section properly.
	 */
	async command uint8_t metadataLength(message_t* msg);

	/**
	 * Returns TRUE if before sending this message we should make sure that
	 * the channel is clear via a very basic (and quick) RSSI check.
	 */
	async command bool requiresRssiCca(message_t* msg);
}