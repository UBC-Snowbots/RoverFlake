#include "phidgetbase.h"
#include "class/motorvelocitycontroller.gen.h"
#include "class/motorvelocitycontroller.gen.c"

static void CCONV
PhidgetMotorVelocityController_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetMotorVelocityControllerHandle ch;

	ch = (PhidgetMotorVelocityControllerHandle)phid;

	switch (code) {
	case EEPHIDGET_MOTORSTALL:
		ch->engaged = 0;
		break;
	default:
		break;
	}
}

static void CCONV
PhidgetMotorVelocityController_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetMotorVelocityController_create(PhidgetMotorVelocityControllerHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetMotorVelocityController_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetMotorVelocityController_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetMotorVelocityController_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetMotorVelocityController_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;
	
	ret = _setDefaults(phid);

	switch (phid->UCD->uid) {
#if PHIDUID_DCC1020_SUPPORTED
	case PHIDCHUID_DCC1020_MOTORVELOCITYCONTROLLER_100:
#if PHIDUID_DCC1010_SUPPORTED
	case PHIDCHUID_DCC1010_MOTORVELOCITYCONTROLLER_100:
#endif
#if PHIDUID_DCC1030_SUPPORTED
	case PHIDCHUID_DCC1030_MOTORVELOCITYCONTROLLER_100:
#endif
#if PHIDUID_DCC1110_SUPPORTED
	case PHIDCHUID_DCC1110_MOTORVELOCITYCONTROLLER_100:
#endif
#if PHIDUID_DCC1120_SUPPORTED
	case PHIDCHUID_DCC1120_MOTORVELOCITYCONTROLLER_100:
#endif
#if PHIDUID_DCC1130_SUPPORTED
	case PHIDCHUID_DCC1130_MOTORVELOCITYCONTROLLER_100:
#endif
		switch (ret) {
		case EPHIDGET_FAILSAFE:
			FIRE_ERROR(phid, EEPHIDGET_ESTOP, "External stop procedure initiated.");
			if (phid->iop != NULL) {
				mos_iop_addnotev(phid->iop, "ESTOP Button Pressed.");
			}
			break;
		case EPHIDGET_BADPOWER:
			FIRE_ERROR(phid, EEPHIDGET_BADPOWER, "Your power supply voltage is too high for the motor controller to begin operation.");
			if (phid->iop != NULL) {
				mos_iop_addnotev(phid->iop, "Bad Power Supply");
			}
			break;
		case EPHIDGET_POWERCYCLE:
			FIRE_ERROR(phid, EEPHIDGET_BADPOWER, "An overvoltage fault has triggered. Power your device off and on to resume operation. "
				"We recommend a PowerGuard Phidget to prevent this in future. See _____ for details.");
			if (phid->iop != NULL) {
				mos_iop_addnotev(phid->iop, "Overvoltage Fault. Power cycle required.");
			}
			break;
		case EPHIDGET_HALLSENSOR:
			FIRE_ERROR(phid, EEPHIDGET_BADCONNECTION, "The hall sensor of your BLDC motor is not reporting a valid input. "
				"The cable is likely damaged or unplugged.");
			if (phid->iop != NULL) {
				mos_iop_addnotev(phid->iop, "Hall Sensor Error. Likely damaged or unplugged.");
			}
			break;
		default:
			break;
		}
		break;
#endif //PHIDUID_DCC1020_SUPPORTED
	default:
		break;
	}
	return ret;
}

static PhidgetReturnCode
PhidgetMotorVelocityController_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetMotorVelocityControllerHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetMotorVelocityControllerHandle)phid;

	switch (bp->vpkt) {
	case BP_VELOCITYCHANGE:
		ch->velocity = getBridgePacketDouble(bp, 0);
		FIRECH(ch, VelocityChange, (ch->velocity *ch->rescaleFactor));
		res = EPHIDGET_OK;
		break;
	case BP_SETFAILSAFETIME:
		TESTRANGE_IOP(bp->iop, "%u", getBridgePacketUInt32(bp, 0), ch->minFailsafeTime, ch->maxFailsafeTime);
		res = _bridgeInput(phid, bp);
		break;
#if (PHIDUID_DCC1020_SUPPORTED || PHIDUID_DCC1120_SUPPORTED)
	case BP_INDUCTANCECHANGE:
		ch->inductance = getBridgePacketDouble(bp, 0);
		res = EPHIDGET_OK;
		break;
	case BP_EXPECTEDVELOCITYCHANGE:
		ch->expectedVelocity = getBridgePacketDouble(bp, 0);
		FIRECH(ch, ExpectedVelocityChange, (ch->expectedVelocity *ch->rescaleFactor));
		res = EPHIDGET_OK;
		break;
#endif
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void CCONV
PhidgetMotorVelocityController_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetMotorVelocityController_hasInitialState(PhidgetChannelHandle phid) {
#if (PHIDUID_DCC1020_SUPPORTED || PHIDUID_DCC1120_SUPPORTED)
	if (((PhidgetMotorVelocityControllerHandle)phid)->inductance == PUNK_DBL)
		return (PFALSE);
#endif
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetMotorVelocityController_setAcceleration(PhidgetMotorVelocityControllerHandle ch,
	double acceleration) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETACCELERATION, NULL, NULL, 1, "%g",
		acceleration / fabs(ch->rescaleFactor)));
}

API_PRETURN
PhidgetMotorVelocityController_getAcceleration(PhidgetMotorVelocityControllerHandle ch,
	double *acceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(acceleration);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*acceleration = ch->acceleration * fabs(ch->rescaleFactor);
	if (ch->acceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_getMinAcceleration(PhidgetMotorVelocityControllerHandle ch,
	double *minAcceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minAcceleration);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*minAcceleration = ch->minAcceleration * fabs(ch->rescaleFactor);
	if (ch->minAcceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_getMaxAcceleration(PhidgetMotorVelocityControllerHandle ch,
	double *maxAcceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxAcceleration);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*maxAcceleration = ch->maxAcceleration * fabs(ch->rescaleFactor);
	if (ch->maxAcceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_getMaxTargetVelocity(PhidgetMotorVelocityControllerHandle ch,
	double *maxTargetVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxTargetVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*maxTargetVelocity = ch->maxTargetVelocity * fabs(ch->rescaleFactor);
	if (ch->maxTargetVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_setRescaleFactor(PhidgetMotorVelocityControllerHandle ch,
	double rescaleFactor) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	if (rescaleFactor == 0)
		return EPHIDGET_INVALIDARG;

	ch->rescaleFactor = rescaleFactor;
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_getMinTargetVelocity(PhidgetMotorVelocityControllerHandle ch,
	double *minTargetVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minTargetVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*minTargetVelocity = ch->minTargetVelocity;
	if (ch->minTargetVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_setTargetVelocity(PhidgetMotorVelocityControllerHandle ch, double TargetVelocity) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g",
		TargetVelocity / (ch->rescaleFactor)));
}

API_PRETURN
PhidgetMotorVelocityController_getTargetVelocity(PhidgetMotorVelocityControllerHandle ch, double *TargetVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(TargetVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*TargetVelocity = ch->targetVelocity * (ch->rescaleFactor);
	if (ch->targetVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_setDeadBand(PhidgetMotorVelocityControllerHandle ch, double deadBand) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDEADBAND, NULL, NULL, 1, "%g",
		((deadBand / fabs(ch->rescaleFactor)))));
}

API_PRETURN
PhidgetMotorVelocityController_getDeadBand(PhidgetMotorVelocityControllerHandle ch, double *deadBand) {

	TESTPTR_PR(ch);
	TESTPTR_PR(deadBand);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*deadBand = ch->deadBand * fabs(ch->rescaleFactor);
	if (ch->deadBand == PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

#if (PHIDUID_DCC1020_SUPPORTED || PHIDUID_DCC1120_SUPPORTED)
API_PRETURN
PhidgetMotorVelocityController_getExpectedVelocity(PhidgetMotorVelocityControllerHandle ch, double *ExpectedVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(ExpectedVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*ExpectedVelocity = ch->expectedVelocity * (ch->rescaleFactor);
	if (ch->expectedVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}
#endif

API_PRETURN
PhidgetMotorVelocityController_setStallVelocity(PhidgetMotorVelocityControllerHandle ch,
	double stallVelocity) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSTALLVELOCITY, NULL, NULL, 1, "%g",
		stallVelocity / fabs(ch->rescaleFactor)));
}

API_PRETURN
PhidgetMotorVelocityController_getStallVelocity(PhidgetMotorVelocityControllerHandle ch, double *stallVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(stallVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*stallVelocity = ch->stallVelocity * fabs(ch->rescaleFactor);
	if (ch->stallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_getMinStallVelocity(PhidgetMotorVelocityControllerHandle ch, double *minStallVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minStallVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*minStallVelocity = ch->minStallVelocity * fabs(ch->rescaleFactor);
	if (ch->minStallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_getMaxStallVelocity(PhidgetMotorVelocityControllerHandle ch, double *maxStallVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxStallVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*maxStallVelocity = ch->maxStallVelocity * fabs(ch->rescaleFactor);
	if (ch->maxStallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_setKp(PhidgetMotorVelocityControllerHandle ch,
	double Kp) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETKP, NULL, NULL, 1, "%g",
		Kp * fabs(ch->rescaleFactor)));
}

API_PRETURN
PhidgetMotorVelocityController_getKp(PhidgetMotorVelocityControllerHandle ch,
	double *Kp) {

	TESTPTR_PR(ch);
	TESTPTR_PR(Kp);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*Kp = ch->kp / fabs(ch->rescaleFactor);

	if (ch->kp == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_setKi(PhidgetMotorVelocityControllerHandle ch,
	double Ki) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETKI, NULL, NULL, 1, "%g",
		Ki * fabs(ch->rescaleFactor)));
}

API_PRETURN
PhidgetMotorVelocityController_getKi(PhidgetMotorVelocityControllerHandle ch,
	double *Ki) {

	TESTPTR_PR(ch);
	TESTPTR_PR(Ki);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*Ki = ch->ki / fabs(ch->rescaleFactor);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_setKd(PhidgetMotorVelocityControllerHandle ch,
	double Kd) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETKD, NULL, NULL, 1, "%g",
		Kd * fabs(ch->rescaleFactor)));
}

API_PRETURN
PhidgetMotorVelocityController_getKd(PhidgetMotorVelocityControllerHandle ch,
	double *Kd) {

	TESTPTR_PR(ch);
	TESTPTR_PR(Kd);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*Kd = ch->kd / fabs(ch->rescaleFactor);

	if (ch->kd == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_getVelocity(PhidgetMotorVelocityControllerHandle ch, double* velocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(velocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	*velocity = ch->velocity * fabs(ch->rescaleFactor);
	if (ch->velocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorVelocityController_setPositionType(PhidgetMotorVelocityControllerHandle ch,
	Phidget_PositionType positionType) {
	PhidgetReturnCode ret;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	ret = (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_POSITIONTYPE, NULL, NULL, 1, "%d",
		positionType));

	if (ret != EPHIDGET_OK)
		return ret;

	switch (ch->phid.UCD->uid) {
#if (PHIDUID_DCC1120_SUPPORTED)
	case PHIDCHUID_DCC1120_MOTORVELOCITYCONTROLLER_100:
#if PHIDUID_DCC1110_SUPPORTED
	case PHIDCHUID_DCC1110_MOTORVELOCITYCONTROLLER_100:
#endif
#if PHIDUID_DCC1130_SUPPORTED
	case PHIDCHUID_DCC1130_MOTORVELOCITYCONTROLLER_100:
#endif
		switch (positionType) {
		case POSITION_TYPE_ENCODER:
			ch->maxTargetVelocity = 500000;
			ch->minTargetVelocity = -500000;
			break;
		case POSITION_TYPE_HALL_SENSOR:
			ch->maxTargetVelocity = 2000;
			ch->minTargetVelocity = -2000;
			break;
		default:
			break;
		}
#endif //PHIDUID_DCC1120_SUPPORTED
	default:
		break;
	}

	return ret;
}

API_PRETURN
PhidgetMotorVelocityController_getPositionType(PhidgetMotorVelocityControllerHandle ch,
	Phidget_PositionType *positionType) {

	TESTPTR_PR(ch);
	TESTPTR_PR(positionType);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORVELOCITYCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
#if (PHIDUID_DCC1120_SUPPORTED)
	case PHIDCHUID_DCC1120_MOTORVELOCITYCONTROLLER_100:
#if PHIDUID_DCC1110_SUPPORTED
	case PHIDCHUID_DCC1110_MOTORVELOCITYCONTROLLER_100:
#endif
#if PHIDUID_DCC1130_SUPPORTED
	case PHIDCHUID_DCC1130_MOTORVELOCITYCONTROLLER_100:
#endif
		break;
#endif //PHIDUID_DCC1120_SUPPORTED
	default:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	}

	*positionType = ch->positionType;
	if (ch->positionType == (Phidget_PositionType)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}



