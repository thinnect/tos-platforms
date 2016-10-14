/**
 * Dummy PacketLinkC
 *
 * @author Raido Pahtma
 * @license MIT
 */
#warning "*** DUMMY PACKET LINK LAYER"
configuration PacketLinkC {
	provides {
		interface Send;
		interface PacketLink;
	}
	uses {
		interface Send as SubSend;
	}
}
implementation {

	Send = SubSend;

}
