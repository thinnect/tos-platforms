/**
 * @author Raido Pahtma
 * @license Thinnect
 */
#include "IeeeEui64.h"
configuration LocalIeeeEui64C {
	provides interface LocalIeeeEui64;
}
implementation {

	components UserSignatureAreaC;
	LocalIeeeEui64 = UserSignatureAreaC.LocalIeeeEui64;

}
