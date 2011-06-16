configuration RFAPowerOverrideC {

}
implementation {

	components RFAPowerOverrideP;

	components McuSleepC;
	McuSleepC.McuPowerOverride -> RFAPowerOverrideP;

}
