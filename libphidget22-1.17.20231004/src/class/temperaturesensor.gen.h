#ifndef _TEMPERATURESENSOR_H_
#define _TEMPERATURESENSOR_H_

/* Generated by WriteClassHeaderVisitor: Wed Oct 04 2023 12:01:33 GMT-0600 (Mountain Daylight Time) */

typedef struct _PhidgetTemperatureSensor *PhidgetTemperatureSensorHandle;

/* Methods */
API_PRETURN_HDR PhidgetTemperatureSensor_create(PhidgetTemperatureSensorHandle *ch);
API_PRETURN_HDR PhidgetTemperatureSensor_delete(PhidgetTemperatureSensorHandle *ch);

/* Properties */
API_PRETURN_HDR PhidgetTemperatureSensor_setDataInterval(PhidgetTemperatureSensorHandle ch,
  uint32_t dataInterval);
API_PRETURN_HDR PhidgetTemperatureSensor_getDataInterval(PhidgetTemperatureSensorHandle ch,
  uint32_t *dataInterval);
API_PRETURN_HDR PhidgetTemperatureSensor_getMinDataInterval(PhidgetTemperatureSensorHandle ch,
  uint32_t *minDataInterval);
API_PRETURN_HDR PhidgetTemperatureSensor_getMaxDataInterval(PhidgetTemperatureSensorHandle ch,
  uint32_t *maxDataInterval);
API_PRETURN_HDR PhidgetTemperatureSensor_setDataRate(PhidgetTemperatureSensorHandle ch,
  double dataRate);
API_PRETURN_HDR PhidgetTemperatureSensor_getDataRate(PhidgetTemperatureSensorHandle ch,
  double *dataRate);
API_PRETURN_HDR PhidgetTemperatureSensor_getMinDataRate(PhidgetTemperatureSensorHandle ch,
  double *minDataRate);
API_PRETURN_HDR PhidgetTemperatureSensor_getMaxDataRate(PhidgetTemperatureSensorHandle ch,
  double *maxDataRate);
API_PRETURN_HDR PhidgetTemperatureSensor_setRTDType(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_RTDType RTDType);
API_PRETURN_HDR PhidgetTemperatureSensor_getRTDType(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_RTDType *RTDType);
API_PRETURN_HDR PhidgetTemperatureSensor_setRTDWireSetup(PhidgetTemperatureSensorHandle ch,
  Phidget_RTDWireSetup RTDWireSetup);
API_PRETURN_HDR PhidgetTemperatureSensor_getRTDWireSetup(PhidgetTemperatureSensorHandle ch,
  Phidget_RTDWireSetup *RTDWireSetup);
API_PRETURN_HDR PhidgetTemperatureSensor_getTemperature(PhidgetTemperatureSensorHandle ch,
  double *temperature);
API_PRETURN_HDR PhidgetTemperatureSensor_getMinTemperature(PhidgetTemperatureSensorHandle ch,
  double *minTemperature);
API_PRETURN_HDR PhidgetTemperatureSensor_getMaxTemperature(PhidgetTemperatureSensorHandle ch,
  double *maxTemperature);
API_PRETURN_HDR PhidgetTemperatureSensor_setTemperatureChangeTrigger(PhidgetTemperatureSensorHandle ch,
  double temperatureChangeTrigger);
API_PRETURN_HDR PhidgetTemperatureSensor_getTemperatureChangeTrigger(PhidgetTemperatureSensorHandle ch,
  double *temperatureChangeTrigger);
API_PRETURN_HDR PhidgetTemperatureSensor_getMinTemperatureChangeTrigger(PhidgetTemperatureSensorHandle ch,
  double *minTemperatureChangeTrigger);
API_PRETURN_HDR PhidgetTemperatureSensor_getMaxTemperatureChangeTrigger(PhidgetTemperatureSensorHandle ch,
  double *maxTemperatureChangeTrigger);
API_PRETURN_HDR PhidgetTemperatureSensor_setThermocoupleType(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_ThermocoupleType thermocoupleType);
API_PRETURN_HDR PhidgetTemperatureSensor_getThermocoupleType(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_ThermocoupleType *thermocoupleType);

/* Events */
typedef void (CCONV *PhidgetTemperatureSensor_OnTemperatureChangeCallback)(PhidgetTemperatureSensorHandle ch, void *ctx, double temperature);

API_PRETURN_HDR PhidgetTemperatureSensor_setOnTemperatureChangeHandler(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_OnTemperatureChangeCallback fptr, void *ctx);

#endif /* _TEMPERATURESENSOR_H_ */
