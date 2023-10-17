/* Generated: Wed Oct 04 2023 12:01:34 GMT-0600 (Mountain Daylight Time) */

#include "device/hubdevice.h"
static void CCONV PhidgetHub_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetHub_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetHub_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetHub_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetHub_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetHub_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetHub_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetHub_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetHub_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetHub {
	struct _PhidgetChannel phid;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	int version;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 2) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 2 - functionality may be limited.", phid, version);
	}


	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {

	return (createBridgePacket(bp, BP_SETSTATUS, 1, "_class_version_=%u"
	  ,2 /* class version */
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETCALIBRATIONVALUES:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETCALIBRATIONVALUES2:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETFIRMWAREUPGRADEFLAG:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETPORTAUTOSETSPEED:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETPORTMODE:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETPORTPOWER:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_HUB0000_HUB_100:
		break;
	case PHIDCHUID_HUB0000_HUB_300:
		break;
	case PHIDCHUID_HUB0000_HUB_400:
		break;
	case PHIDCHUID_HUB0001_HUB_100:
		break;
	case PHIDCHUID_HUB0001_HUB_115:
		break;
	case PHIDCHUID_HUB0002_HUB_100:
		break;
	case PHIDCHUID_HUB0007_HUB_100:
		break;
	case PHIDCHUID_HUB0004_HUB_100:
		break;
	case PHIDCHUID_HUB0004_HUB_200:
		break;
	case PHIDCHUID_HUB5000_HUB_100:
		break;
	case PHIDCHUID_HUB5000_HUB_200:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_HUB0000_HUB_100:
		break;
	case PHIDCHUID_HUB0000_HUB_300:
		break;
	case PHIDCHUID_HUB0000_HUB_400:
		break;
	case PHIDCHUID_HUB0001_HUB_100:
		break;
	case PHIDCHUID_HUB0001_HUB_115:
		break;
	case PHIDCHUID_HUB0002_HUB_100:
		break;
	case PHIDCHUID_HUB0007_HUB_100:
		break;
	case PHIDCHUID_HUB0004_HUB_100:
		break;
	case PHIDCHUID_HUB0004_HUB_200:
		break;
	case PHIDCHUID_HUB5000_HUB_100:
		break;
	case PHIDCHUID_HUB5000_HUB_200:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetHub));
}

static PhidgetReturnCode CCONV
_create(PhidgetHubHandle *phidp) {

	CHANNELCREATE_BODY(Hub, PHIDCHCLASS_HUB);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHub_delete(PhidgetHubHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetHub_setADCCalibrationValues(PhidgetHubHandle ch, double voltageInputGain[6],
  double voltageRatioGain[6]) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUB);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCALIBRATIONVALUES, NULL, NULL, 2, "%6G%6G",
	  voltageInputGain, voltageRatioGain); 
}

API_PRETURN
PhidgetHub_setADCCalibrationValues2(PhidgetHubHandle ch, const double *voltageInputOffset,
  size_t voltageInputOffsetLen, const double *voltageInputGain, size_t voltageInputGainLen, const double *voltageRatioOffset, size_t voltageRatioOffsetLen, const double *voltageRatioGain, size_t voltageRatioGainLen) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUB);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCALIBRATIONVALUES2, NULL, NULL, 8,
	  "%*G%*G%*G%*G", (int)voltageInputOffsetLen, voltageInputOffset, (int)voltageInputGainLen, voltageInputGain, (int)voltageRatioOffsetLen, voltageRatioOffset, (int)voltageRatioGainLen, voltageRatioGain); 
}

API_PRETURN
PhidgetHub_setFirmwareUpgradeFlag(PhidgetHubHandle ch, int port, uint32_t timeout) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUB);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFIRMWAREUPGRADEFLAG, NULL, NULL, 2, "%d%u",
	  port, timeout); 
}

API_PRETURN
PhidgetHub_setPortAutoSetSpeed(PhidgetHubHandle ch, int port, int state) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUB);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPORTAUTOSETSPEED, NULL, NULL, 2, "%d%d",
	  port, state); 
}

API_PRETURN
PhidgetHub_setPortMode(PhidgetHubHandle ch, int port, PhidgetHub_PortMode mode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUB);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPORTMODE, NULL, NULL, 2, "%d%d", port,
	  mode); 
}

API_PRETURN
PhidgetHub_setPortPower(PhidgetHubHandle ch, int port, int state) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUB);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPORTPOWER, NULL, NULL, 2, "%d%d", port,
	  state); 
}
