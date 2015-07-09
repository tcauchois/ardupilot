/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

/*
  optionally turn down optimisation for debugging
 */
// #pragma GCC optimize("O0")

#include "AP_NavEKF.h"
#include <AP_AHRS.h>
#include <AP_Param.h>
#include <AP_Vehicle.h>

#include <stdio.h>

/*
  parameter defaults for different types of vehicle. The
  APM_BUILD_DIRECTORY is taken from the main vehicle directory name
  where the code is built. Note that this trick won't work for arduino
  builds on APM2, but NavEKF doesn't run on APM2, so that's OK
 */
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
// copter defaults
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       1.0f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    1E-05f
#define ABIAS_PNOISE_DEFAULT    0.00005f
#define MAG_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        5
#define POS_GATE_DEFAULT        10
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         3
#define GLITCH_ACCEL_DEFAULT    100
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_NOISE_DEFAULT      0.25f
#define FLOW_GATE_DEFAULT       3

#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
// rover defaults
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       1.0f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    8E-06f
#define ABIAS_PNOISE_DEFAULT    0.00005f
#define MAG_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        5
#define POS_GATE_DEFAULT        10
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         2
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   15
#define FLOW_MEAS_DELAY         25
#define FLOW_NOISE_DEFAULT      0.15f
#define FLOW_GATE_DEFAULT       5

#else
// generic defaults (and for plane)
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       0.5f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.5f
#define GBIAS_PNOISE_DEFAULT    8E-06f
#define ABIAS_PNOISE_DEFAULT    0.00005f
#define MAG_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        30
#define HGT_GATE_DEFAULT        20
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         0
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   20
#define FLOW_MEAS_DELAY         25
#define FLOW_NOISE_DEFAULT      0.3f
#define FLOW_GATE_DEFAULT       3

#endif // APM_BUILD_DIRECTORY


extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// when the wind estimation first starts with no airspeed sensor,
// assume 3m/s to start
#define STARTUP_WIND_SPEED 3.0f

// initial imu bias uncertainty (deg/sec)
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.3f

// Define tuning parameters
const AP_Param::GroupInfo NavEKF::var_info[] PROGMEM = {

    // @Param: VELNE_NOISE
    // @DisplayName: GPS horizontal velocity measurement noise scaler
    // @Description: This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("VELNE_NOISE",    0, NavEKF, _gpsHorizVelNoise, VELNE_NOISE_DEFAULT),

    // @Param: VELD_NOISE
    // @DisplayName: GPS vertical velocity measurement noise scaler
    // @Description: This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the weighting on this measurement.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("VELD_NOISE",    1, NavEKF, _gpsVertVelNoise, VELD_NOISE_DEFAULT),

    // @Param: POSNE_NOISE
    // @DisplayName: GPS horizontal position measurement noise (m)
    // @Description: This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("POSNE_NOISE",    2, NavEKF, _gpsHorizPosNoise, POSNE_NOISE_DEFAULT),

    // @Param: ALT_NOISE
    // @DisplayName: Altitude measurement noise (m)
    // @Description: This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("ALT_NOISE",    3, NavEKF, _baroAltNoise, ALT_NOISE_DEFAULT),

    // @Param: MAG_NOISE
    // @DisplayName: Magnetometer measurement noise (Gauss)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MAG_NOISE",    4, NavEKF, _magNoise, MAG_NOISE_DEFAULT),

    // @Param: EAS_NOISE
    // @DisplayName: Equivalent airspeed measurement noise (m/s)
    // @Description: This is the RMS value of noise in equivalent airspeed measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("EAS_NOISE",    5, NavEKF, _easNoise, 1.4f),

    // @Param: WIND_PNOISE
    // @DisplayName: Wind velocity process noise (m/s^2)
    // @Description: This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
    // @Range: 0.01 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PNOISE",    6, NavEKF, _windVelProcessNoise, 0.1f),

    // @Param: WIND_PSCALE
    // @DisplayName: Height rate to wind procss noise scaler
    // @Description: Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
    // @Range: 0.0 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PSCALE",    7, NavEKF, _wndVarHgtRateScale, 0.5f),

    // @Param: GYRO_PNOISE
    // @DisplayName: Rate gyro noise (rad/s)
    // @Description: This noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
    // @Range: 0.001 0.05
    // @Increment: 0.001
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("GYRO_PNOISE",    8, NavEKF, _gyrNoise, GYRO_PNOISE_DEFAULT),

    // @Param: ACC_PNOISE
    // @DisplayName: Accelerometer noise (m/s^2)
    // @Description: This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
    // @Range: 0.05 1.0
    // @Increment: 0.01
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ACC_PNOISE",    9, NavEKF, _accNoise, ACC_PNOISE_DEFAULT),

    // @Param: GBIAS_PNOISE
    // @DisplayName: Rate gyro bias process noise (rad/s)
    // @Description: This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier.
    // @Range: 0.0000001 0.00001
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("GBIAS_PNOISE",    10, NavEKF, _gyroBiasProcessNoise, GBIAS_PNOISE_DEFAULT),

    // @Param: ABIAS_PNOISE
    // @DisplayName: Accelerometer bias process noise (m/s^2)
    // @Description: This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
    // @Range: 0.00001 0.001
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ABIAS_PNOISE",    11, NavEKF, _accelBiasProcessNoise, ABIAS_PNOISE_DEFAULT),

    // @Param: MAG_PNOISE
    // @DisplayName: Magnetic field process noise (gauss/s)
    // @Description: This noise controls the growth of magnetic field state error estimates. Increasing it makes magnetic field bias estimation faster and noisier.
    // @Range: 0.0001 0.01
    // @User: Advanced
    // @Units: gauss/s
    AP_GROUPINFO("MAG_PNOISE",    12, NavEKF, _magProcessNoise, MAG_PNOISE_DEFAULT),

    // @Param: GSCL_PNOISE
    // @DisplayName: Rate gyro scale factor process noise (1/s)
    // @Description: This noise controls the rate of gyro scale factor learning. Increasing it makes rate gyro scale factor estimation faster and noisier.
    // @Range: 0.0000001 0.00001
    // @User: Advanced
    // @Units: 1/s
   AP_GROUPINFO("GSCL_PNOISE",    13, NavEKF, _gyroScaleProcessNoise, 1e-6f),

    // @Param: GPS_DELAY
    // @DisplayName: GPS measurement delay (msec)
    // @Description: This is the number of msec that the GPS measurements lag behind the inertial measurements.
    // @Range: 0 500
    // @Increment: 10
    // @User: Advanced
    // @Units: milliseconds
    AP_GROUPINFO("VEL_DELAY",    14, NavEKF, _msecGpsDelay, 220),

    // this slot has been deprecated and reserved for later use

    // @Param: GPS_TYPE
    // @DisplayName: GPS mode control
    // @Description: This parameter controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
    // @Values: 0:GPS 3D Vel and 2D Pos, 1:GPS 2D vel and 2D pos, 2:GPS 2D pos, 3:No GPS use optical flow
    // @User: Advanced
    AP_GROUPINFO("GPS_TYPE",    16, NavEKF, _fusionModeGPS, 0),

    // @Param: VEL_GATE
    // @DisplayName: GPS velocity measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VEL_GATE",    17, NavEKF, _gpsVelInnovGate, VEL_GATE_DEFAULT),

    // @Param: POS_GATE
    // @DisplayName: GPS position measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POS_GATE",    18, NavEKF, _gpsPosInnovGate, POS_GATE_DEFAULT),

    // @Param: HGT_GATE
    // @DisplayName: Height measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("HGT_GATE",    19, NavEKF, _hgtInnovGate, HGT_GATE_DEFAULT),

    // @Param: MAG_GATE
    // @DisplayName: Magnetometer measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAG_GATE",    20, NavEKF, _magInnovGate, MAG_GATE_DEFAULT),

    // @Param: EAS_GATE
    // @DisplayName: Airspeed measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EAS_GATE",    21, NavEKF, _tasInnovGate, 10),

    // @Param: MAG_CAL
    // @DisplayName: Magnetometer calibration mode
    // @Description: EKF_MAG_CAL = 0 enables calibration based on flying speed and altitude and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition and is recommended if in-flight magnetometer calibration is unreliable.
    // @Values: 0:Speed and Height,1:Acceleration,2:Never,3:Always
    // @User: Advanced
    AP_GROUPINFO("MAG_CAL",    22, NavEKF, _magCal, MAG_CAL_DEFAULT),

    // @Param: GLITCH_ACCEL
    // @DisplayName: GPS glitch accel gate size (cm/s^2)
    // @Description: This parameter controls the maximum amount of difference in horizontal acceleration between the value predicted by the filter and the value measured by the GPS before the GPS position data is rejected. If this value is set too low, then valid GPS data will be regularly discarded, and the position accuracy will degrade. If this parameter is set too high, then large GPS glitches will cause large rapid changes in position.
    // @Range: 100 500
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("GLITCH_ACCEL",    23, NavEKF, _gpsGlitchAccelMax, GLITCH_ACCEL_DEFAULT),

    // @Param: GLITCH_RAD
    // @DisplayName: GPS glitch radius gate size (m)
    // @Description: This parameter controls the maximum amount of difference in horizontal position (in m) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and an offset is applied to the GPS measurement to compensate. Position steps smaller than this value will be temporarily ignored, but will then be accepted and the filter will move to the new position. Position steps larger than this value will be ignored initially, but the filter will then apply an offset to the GPS position measurement.
    // @Range: 10 50
    // @Increment: 5
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("GLITCH_RAD",    24, NavEKF, _gpsGlitchRadiusMax, GLITCH_RADIUS_DEFAULT),

    // @Param: GND_GRADIENT
    // @DisplayName: Terrain Gradient % RMS
    // @Description: This parameter sets the RMS terrain gradient percentage assumed by the terrain height estimation. Terrain height can be estimated using optical flow and/or range finder sensor data if fitted. Smaller values cause the terrain height estimate to be slower to respond to changes in measurement. Larger values casue the terrain height estimate to be faster to respond, but also more noisy. Generally this value can be reduced if operating over very flat terrain and increased if operating over uneven terrain.
    // @Range: 1 - 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GND_GRADIENT",    25, NavEKF, _gndGradientSigma, 2),

    // @Param: FLOW_NOISE
    // @DisplayName: Optical flow measurement noise (rad/s)
    // @Description: This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 - 1.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("FLOW_NOISE",    26, NavEKF, _flowNoise, FLOW_NOISE_DEFAULT),

    // @Param: FLOW_GATE
    // @DisplayName: Optical Flow measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLOW_GATE",    27, NavEKF, _flowInnovGate, FLOW_GATE_DEFAULT),

    // @Param: FLOW_DELAY
    // @DisplayName: Optical Flow measurement delay (msec)
    // @Description: This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
    // @Range: 0 - 500
    // @Increment: 10
    // @User: Advanced
    // @Units: milliseconds
    AP_GROUPINFO("FLOW_DELAY",    28, NavEKF, _msecFLowDelay, FLOW_MEAS_DELAY),

    // @Param: RNG_GATE
    // @DisplayName: Range finder measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RNG_GATE",    29, NavEKF, _rngInnovGate, 5),

    // @Param: MAX_FLOW
    // @DisplayName: Maximum valid optical flow rate
    // @Description: This parameter sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
    // @Range: 1.0 - 4.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("MAX_FLOW",    30, NavEKF, _maxFlowRate, 2.5f),

    // @Param: FALLBACK
    // @DisplayName: Fallback strictness
    // @Description: This parameter controls the conditions necessary to trigger a fallback to DCM and INAV. A value of 1 will cause fallbacks to occur on loss of GPS and other conditions. A value of 0 will trust the EKF more.
    // @Values: 0:Trust EKF more, 1:Trust DCM more
    // @User: Advanced
    AP_GROUPINFO("FALLBACK",    31, NavEKF, _fallback, 1),

    // @Param: ALT_SOURCE
    // @DisplayName: Primary height source
    // @Description: This parameter controls which height sensor is used by the EKF during optical flow navigation (when EKF_GPS_TYPE = 3). A value of will 0 cause it to always use baro altitude. A value of 1 will casue it to use range finder if available.
    // @Values: 0:Use Baro, 1:Use Range Finder
    // @User: Advanced
    AP_GROUPINFO("ALT_SOURCE",    32, NavEKF, _altSource, 1),


    AP_GROUPEND
};

// constructor
NavEKF::NavEKF(const AP_AHRS *ahrs, AP_Baro &baro, const RangeFinder &rng) :
    _ahrs(ahrs),
    _baro(baro),
    _rng(rng),
    stateStruct(*reinterpret_cast<struct state_elements *>(&statesArray)),
    gpsNEVelVarAccScale(0.05f),     // Scale factor applied to horizontal velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsDVelVarAccScale(0.07f),      // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsPosVarAccScale(0.05f),       // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    msecHgtDelay(60),               // Height measurement delay (msec)
    msecMagDelay(60),               // Magnetometer measurement delay (msec)
    msecTasDelay(240),              // Airspeed measurement delay (msec)
    gpsRetryTimeUseTAS(10000),      // GPS retry time with airspeed measurements (msec)
    gpsRetryTimeNoTAS(7000),        // GPS retry time without airspeed measurements (msec)
    gpsFailTimeWithFlow(5000),      // If we have no GPS for longer than this and we have optical flow, then we will switch across to using optical flow (msec)
    hgtRetryTimeMode0(10000),       // Height retry time with vertical velocity measurement (msec)
    hgtRetryTimeMode12(5000),       // Height retry time without vertical velocity measurement (msec)
    tasRetryTime(5000),             // True airspeed timeout and retry interval (msec)
    magFailTimeLimit_ms(10000),     // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    magVarRateScale(0.05f),         // scale factor applied to magnetometer variance due to angular rate
    gyroBiasNoiseScaler(2.0f),      // scale factor applied to imu gyro bias learning before the vehicle is armed
    accelBiasNoiseScaler(1.0f),     // scale factor applied to imu accel bias learning before the vehicle is armed
    msecGpsAvg(200),                // average number of msec between GPS measurements
    msecHgtAvg(100),                // average number of msec between height measurements
    msecMagAvg(100),                // average number of msec between magnetometer measurements
    msecBetaAvg(100),               // average number of msec between synthetic sideslip measurements
    msecBetaMax(200),               // maximum number of msec between synthetic sideslip measurements
    msecFlowAvg(100),               // average number of msec between optical flow measurements
    dtVelPos(0.2f),                 // number of seconds between position and velocity corrections. This should be a multiple of the imu update interval.
    covTimeStepMax(0.07f),          // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),            // maximum delta angle between covariance prediction updates
    TASmsecMax(200),                // maximum allowed interval between airspeed measurement updates
    DCM33FlowMin(0.71f),            // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    fScaleFactorPnoise(1e-10f),     // Process noise added to focal length scale factor state variance at each time step
    flowTimeDeltaAvg_ms(100),       // average interval between optical flow measurements (msec)
    flowIntervalMax_ms(100),        // maximum allowable time between flow fusion events
    gndEffectTimeout_ms(1000),      // time in msec that baro ground effect compensation will timeout after initiation
    gndEffectBaroScaler(4.0f)      // scaler applied to the barometer observation variance when operating in ground effect

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ,_perf_UpdateFilter(perf_alloc(PC_ELAPSED, "EKF_UpdateFilter")),
    _perf_CovariancePrediction(perf_alloc(PC_ELAPSED, "EKF_CovariancePrediction")),
    _perf_FuseVelPosNED(perf_alloc(PC_ELAPSED, "EKF_FuseVelPosNED")),
    _perf_FuseMagnetometer(perf_alloc(PC_ELAPSED, "EKF_FuseMagnetometer")),
    _perf_FuseAirspeed(perf_alloc(PC_ELAPSED, "EKF_FuseAirspeed")),
    _perf_FuseSideslip(perf_alloc(PC_ELAPSED, "EKF_FuseSideslip"))
#endif
{
    AP_Param::setup_object_defaults(this, var_info);

}

// Check basic filter health metrics and return a consolidated health status
bool NavEKF::healthy(void) const
{
    uint8_t faultInt;
    getFilterFaults(faultInt);
    if (faultInt > 0) {
        return false;
    }
    if (_fallback && velTestRatio > 1 && posTestRatio > 1 && hgtTestRatio > 1) {
        // all three metrics being above 1 means the filter is
        // extremely unhealthy.
        return false;
    }
    // Give the filter a second to settle before use
    if ((imuSampleTime_ms - ekfStartTime_ms) < 1000 ) {
        return false;
    }
    // barometer and position innovations must be within limits when on-ground
    float horizErrSq = sq(innovVelPos[3]) + sq(innovVelPos[4]);
    if (!filterArmed && (fabsf(innovVelPos[5]) > 1.0f || horizErrSq > 1.0f)) {
        return false;
    }

    // all OK
    return true;
}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF::ResetPosition(void)
{
    if (constPosMode || (PV_AidingMode != AID_ABSOLUTE)) {
        stateStruct.position.x = 0;
        stateStruct.position.y = 0;
    } else if (!gpsNotAvailable) {
        // write to state vector and compensate for offset  between last GPs measurement and the EKF time horizon
        stateStruct.position.x = gpsDataNew.pos.x + gpsPosGlitchOffsetNE.x + 0.001f*gpsDataNew.vel.x*(float(imuDataDelayed.time_ms) - float(lastTimeGpsReceived_ms));
        stateStruct.position.y = gpsDataNew.pos.y + gpsPosGlitchOffsetNE.y + 0.001f*gpsDataNew.vel.y*(float(imuDataDelayed.time_ms) - float(lastTimeGpsReceived_ms));
    }
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i].position.x = stateStruct.position.x;
        storedOutput[i].position.y = stateStruct.position.y;
    }
    outputDataNew.position.x = stateStruct.position.x;
    outputDataNew.position.y = stateStruct.position.y;
    outputDataDelayed.position.x = stateStruct.position.x;
    outputDataDelayed.position.y = stateStruct.position.y;
}

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF::ResetVelocity(void)
{
    if (constPosMode || PV_AidingMode != AID_ABSOLUTE) {
         stateStruct.velocity.zero();
    } else if (!gpsNotAvailable) {
        // reset horizontal velocity states, applying an offset to the GPS velocity to prevent the GPS position being rejected when the GPS position offset is being decayed to zero.
        stateStruct.velocity.x  = gpsDataNew.vel.x + gpsVelGlitchOffset.x; // north velocity from blended accel data
        stateStruct.velocity.y  = gpsDataNew.vel.y + gpsVelGlitchOffset.y; // east velocity from blended accel data
    }
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i].velocity.x = stateStruct.velocity.x;
        storedOutput[i].velocity.y = stateStruct.velocity.y;
    }
    outputDataNew.velocity.x = stateStruct.velocity.x;
    outputDataNew.velocity.y = stateStruct.velocity.y;
    outputDataDelayed.velocity.x = stateStruct.velocity.x;
    outputDataDelayed.velocity.y = stateStruct.velocity.y;
}

// reset the vertical position state using the last height measurement
void NavEKF::ResetHeight(void)
{
    // read the altimeter
    readHgtData();
    // write to the state vector
    stateStruct.position.z = -baroDataNew.hgt; // down position from blended accel data
    terrainState = stateStruct.position.z + rngOnGnd;
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i].position.z = stateStruct.position.z;
    }
    outputDataNew.position.z = stateStruct.position.z;
    outputDataDelayed.position.z = stateStruct.position.z;
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
bool NavEKF::InitialiseFilterBootstrap(void)
{
    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && _ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        statesInitialised = false;
        return false;
    }

    // set re-used variables to zero
    InitialiseVariables();

    // Initialise IMU data
    dtIMUavg = 1.0f/_ahrs->get_ins().get_sample_rate();
    readIMUData();
    StoreIMU_reset();

    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f initAccVec;

    // TODO we should average accel readings over several cycles
    initAccVec = _ahrs->get_ins().get_accel();

    // read the magnetometer data
    readMagData();

    // normalise the acceleration vector
    float pitch=0, roll=0;
    if (initAccVec.length() > 0.001f) {
        initAccVec.normalize();

        // calculate initial pitch angle
        pitch = asinf(initAccVec.x);

        // calculate initial roll angle
        roll = -asinf(initAccVec.y / cosf(pitch));
    }

    // calculate initial roll and pitch orientation
    stateStruct.quat.from_euler(roll, pitch, 0.0f);

    // initialise static process state model states
    stateStruct.gyro_bias.zero();
    stateStruct.gyro_scale.x = 1.0f;
    stateStruct.gyro_scale.y = 1.0f;
    stateStruct.gyro_scale.z = 1.0f;
    stateStruct.accel_zbias = 0.0f;
    stateStruct.wind_vel.zero();

    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state
    readHgtData();
    ResetHeight();

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise the covariance matrix
    CovarianceInit();

    // reset output states
    StoreOutputReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    return true;
}

// Update Filter States - this should be called whenever new IMU data is available
void NavEKF::UpdateFilter()
{
    // zero the delta quaternion used by the strapdown navigation because it is published
    // and we need to return a zero rotation of the INS fails to update it
    correctedDelAngQuat.initialise();

    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    // start the timer used for load measurement
    perf_begin(_perf_UpdateFilter);

    //get starting time for update step
    imuSampleTime_ms = hal.scheduler->millis();

    // read IMU data and convert to delta angles and velocities
    readIMUData();

    // detect if the filter update has been delayed for too long
    if (imuDataDelayed.delAngDT > 0.2f) {
        // we have stalled for too long - reset states
        ResetVelocity();
        ResetPosition();
        ResetHeight();
        //Initialise IMU pre-processing states
        readIMUData();
        // stop the timer used for load measurement
        perf_end(_perf_UpdateFilter);
        return;
    }

    // check if on ground
    SetFlightAndFusionModes();

    // Check arm status and perform required checks and mode changes
    performArmingChecks();

    // run the strapdown INS equations every IMU update
    UpdateStrapdownEquationsNED();

    // sum delta angles and time used by covariance prediction
    summedDelAng = summedDelAng + correctedDelAng;
    summedDelVel = summedDelVel + correctedDelVel;
    dt += imuDataDelayed.delAngDT;

    // perform a covariance prediction if the total delta angle has exceeded the limit
    // or the time limit will be exceeded at the next IMU update
    if (((dt >= (covTimeStepMax - dtIMUavg)) || (summedDelAng.length() > covDelAngMax))) {
        CovariancePrediction();
    } else {
        covPredStep = false;
    }

    // Read range finder data which is used by both position and optical flow fusion
    readRangeFinder();

    // Update states using GPS and altimeter data
    SelectVelPosFusion();

    // Check for tilt convergence
    float alpha = 1.0f*dtIMUavg;
    float temp=tiltErrVec.length();
    tiltErrFilt = alpha*temp + (1.0f-alpha)*tiltErrFilt;
    if (tiltErrFilt < 0.005f && !tiltAlignComplete) {
        tiltAlignComplete = true;
        hal.console->printf("EKF tilt alignment complete\n");
    }

    // once tilt has converged, align yaw using magnetic field measurements
    if (tiltAlignComplete && !yawAlignComplete) {
        Vector3f eulerAngles;
        getEulerAngles(eulerAngles);
        stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        StoreQuatReset();
        yawAlignComplete = true;
        hal.console->printf("EKF yaw alignment complete\n");
    }

    // Update states using  magnetometer data
    SelectMagFusion();

    // Wind output forward from the fusion to output time horizon
    calcOutputStatesFast();

    // stop the timer used for load measurement
    perf_end(_perf_UpdateFilter);
}

// select fusion of velocity, position and height measurements
void NavEKF::SelectVelPosFusion()
{
    // check for and read new GPS data
    readGpsData();

         if (RecallGPS() && (imuDataDelayed.time_ms > gpsFixTime_ms)) {
            // use both if GPS use is enabled
            fuseVelData = true;
            fusePosData = true;
            // If a long time since last GPS update, then reset position and velocity and reset stored state history
            if (imuSampleTime_ms - secondLastGpsTime_ms > 5000) {
                // Apply an offset to the GPS position so that the position can be corrected gradually
                gpsPosGlitchOffsetNE.x = stateStruct.position.x - gpsDataDelayed.pos.x;
                gpsPosGlitchOffsetNE.y = stateStruct.position.y - gpsDataDelayed.pos.y;
                // limit the radius of the offset to 100m and decay the offset to zero radially
                decayGpsOffset();
                ResetPosition();
                ResetVelocity();
                // record the fail time
                lastPosFailTime = imuSampleTime_ms;
                // Reset the normalised innovation to avoid false failing the bad position fusion test
                posTestRatio = 0.0f;
            }
        } else {
            fuseVelData = false;
            fusePosData = false;
        }

    // check for and read new height data
    readHgtData();

    // If we haven't received height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime = (useGpsVertVel && !velTimeout) ? hgtRetryTimeMode0 : hgtRetryTimeMode12;
    if (imuSampleTime_ms - lastHgtReceived_ms > hgtRetryTime) {
        hgtTimeout = true;
    }

    // command fusion of height data
    // wait until the EKF time horizon catches up with the measurement
    if (RecallBaro()) {
        // enable fusion
        fuseHgtData = true;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        FuseVelPosNED();
    }

    // Detect and declare bad GPS aiding status for minimum 10 seconds if a GPS rejection occurs after
    // rejection of GPS and reset to GPS position. This addresses failure case where errors cause ongoing rejection
    // of GPS and severe loss of position accuracy.
    uint32_t gpsRetryTime;
    if (useAirspeed()) {
        gpsRetryTime = gpsRetryTimeUseTAS;
    } else {
        gpsRetryTime = gpsRetryTimeNoTAS;
    }
    if ((posTestRatio > 2.0f) && ((imuSampleTime_ms - lastPosFailTime) < gpsRetryTime) && ((imuSampleTime_ms - lastPosFailTime) > gpsRetryTime/2) && fusePosData) {
        lastGpsAidBadTime_ms = imuSampleTime_ms;
        gpsAidingBad = true;
    }
    gpsAidingBad = gpsAidingBad && ((imuSampleTime_ms - lastGpsAidBadTime_ms) < 10000);
}

// select fusion of magnetometer data
void NavEKF::SelectMagFusion()
{
    // start performance timer
    perf_begin(_perf_FuseMagnetometer);

    // check for and read new magnetometer measurements
    readMagData();

    // If we are using the compass and the magnetometer has been unhealthy for too long we declare a timeout
    if (magHealth) {
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

    bool temp = RecallMag();

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    bool dataReady = (temp && statesInitialised && use_compass() && yawAlignComplete);
    if (dataReady) {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        // If we haven't performed the first airborne magnetic field update or have inhibited magnetic field learning, then we use the simple method of declination to maintain heading
        if(inhibitMagStates) {
            fuseCompass();
        } else {
        // fuse the three magnetometer componenents sequentially
            for (mag_state.obsIndex = 0; mag_state.obsIndex <= 2; mag_state.obsIndex++) FuseMagnetometer();
        }
    }

    // stop performance timer
    perf_end(_perf_FuseMagnetometer);
}

// update the quaternion, velocity and position states using delayed IMU measurements
// because the EKF is running on a delayed time horizon
void NavEKF::UpdateStrapdownEquationsNED()
{
    Vector3f delVelNav;  // delta velocity vector

    // remove gyro scale factor errors
    correctedDelAng.x = imuDataDelayed.delAng.x * stateStruct.gyro_scale.x;
    correctedDelAng.y = imuDataDelayed.delAng.y * stateStruct.gyro_scale.y;
    correctedDelAng.z = imuDataDelayed.delAng.z * stateStruct.gyro_scale.z;

    // remove sensor bias errors
    correctedDelAng -= stateStruct.gyro_bias;
    correctedDelVel = imuDataDelayed.delVel;
    correctedDelVel.z -= stateStruct.accel_zbias;

    // apply correction for earths rotation rate
    // % * - and + operators have been overloaded
    correctedDelAng   = correctedDelAng - prevTnb * earthRateNED*imuDataDelayed.delAngDT;

    // convert the rotation vector to its equivalent quaternion
    correctedDelAngQuat.from_axis_angle(correctedDelAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    stateStruct.quat *= correctedDelAngQuat;
    stateStruct.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3f Tbn_temp;
    stateStruct.quat.rotation_matrix(Tbn_temp);
    prevTnb = Tbn_temp.transposed();

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    delVelNav  = Tbn_temp*correctedDelVel;
    delVelNav.z += GRAVITY_MSS*imuDataDelayed.delVelDT;

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED = delVelNav / imuDataDelayed.delVelDT;

    // apply a first order lowpass filter
    velDotNEDfilt = velDotNED * 0.05f + velDotNEDfilt * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = velDotNEDfilt.length();
    accNavMagHoriz = pythagorous2(velDotNEDfilt.x , velDotNEDfilt.y);

    // save velocity for use in trapezoidal intergration for position calcuation
    Vector3f lastVelocity = stateStruct.velocity;

    // sum delta velocities to get velocity
    stateStruct.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    stateStruct.position += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);

    // capture current angular rate to augmented state vector for use by optical flow fusion
    stateStruct.omega = correctedDelAng / imuDataDelayed.delAngDT;

    // limit states to protect against divergence
    ConstrainStates();
}

// Propagate PVA solution forward from the fusion time horizon to the current time horizon
// using buffered IMU data
void  NavEKF::calcOutputStates() {

    // initialise the store access at the fusion time horizon (it will be advanced later)
    uint8_t imuStoreAccessIndex = fifoIndexDelayed;
    imu_elements imuData;

    // Counter used to ensure the while loop always exits
    uint8_t watchdog = 0;

    // initialise to the solution at the fusion time horizon
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;

    // Iterate through the buffered IMU data, using the strapdown equations to wind forward from the fusion time horizon to current time
    // we stop iterating when we have reached the current imu Data
    do {

        // If the loop cannot exit, force exit
        if (watchdog > IMU_BUFFER_LENGTH+2) {
            return;
        }
        watchdog++;

        // advance to the next index
        imuStoreAccessIndex++;

        // if we have got to the end of the array, return to the start
        if (imuStoreAccessIndex >= IMU_BUFFER_LENGTH) {
            imuStoreAccessIndex = 0;
        }
        imuData = storedIMU[imuStoreAccessIndex];

        // remove gyro bias errors
        Vector3f delAng = imuData.delAng - stateStruct.gyro_bias;

        // remove Z accel bias error
        Vector3f delVel = imuData.delVel;
        delVel.z -= stateStruct.accel_zbias;

        // convert the rotation vector to its equivalent quaternion
        Quaternion deltaQuat;
        deltaQuat.from_axis_angle_fast(delAng);

        // update the quaternion states by rotating from the previous attitude through
        // the delta angle rotation quaternion and normalise
        outputDataNew.quat *= deltaQuat;
        outputDataNew.quat.normalize();

        // calculate the body to nav cosine matrix
        Matrix3f Tbn_temp;
        outputDataNew.quat.rotation_matrix(Tbn_temp);

        // transform body delta velocities to delta velocities in the nav frame
        // * and + operators have been overloaded
        Vector3f delVelNav  = Tbn_temp*delVel;
        delVelNav.z += GRAVITY_MSS*imuData.delVelDT;

        // use a simple Euler integration
        outputDataNew.position += outputDataNew.velocity*imuData.delVelDT;
    }
    while (imuStoreAccessIndex != fifoIndexNow);
}

// Propagate PVA solution forward from the fusion time horizon to the current time horizon
// using simple observer. This also applies an LPF to fusion corrections

void  NavEKF::calcOutputStatesFast() {

    // Calculate strapdown solution at the current time horizon

    // remove gyro scale factor errors
    Vector3f delAng;
    delAng.x = imuDataNew.delAng.x * stateStruct.gyro_scale.x;
    delAng.y = imuDataNew.delAng.y * stateStruct.gyro_scale.y;
    delAng.z = imuDataNew.delAng.z * stateStruct.gyro_scale.z;

    // remove sensor bias errors
    delAng -= stateStruct.gyro_bias;
    Vector3f delVel;
    delVel = imuDataNew.delVel;
    delVel.z -= stateStruct.accel_zbias;

    // apply corections to track EKF solution
    delAng += delAngCorrection;

    // convert the rotation vector to its equivalent quaternion
    Quaternion deltaQuat;
    deltaQuat.from_axis_angle(delAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    outputDataNew.quat *= deltaQuat;
    outputDataNew.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3f Tbn_temp;
    outputDataNew.quat.rotation_matrix(Tbn_temp);

    // transform body delta velocities to delta velocities in the nav frame
    // Add the earth frame correction required to track the EKF states
    // * and + operators have been overloaded
    Vector3f delVelNav  = Tbn_temp*delVel + delVelCorrection;
    delVelNav.z += GRAVITY_MSS*imuDataNew.delVelDT;

    // save velocity for use in trapezoidal intergration for position calcuation
    Vector3f lastVelocity = outputDataNew.velocity;

    // sum delta velocities to get velocity
    outputDataNew.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position, applying correction required to track EKF solution
    outputDataNew.position += (outputDataNew.velocity + lastVelocity) * (imuDataNew.delVelDT*0.5f) + velCorrection * imuDataNew.delVelDT;

    // store the output in the FIFO buffer
    StoreOutput();

    // extract data at the fusion time horizon from the FIFO buffer
    RecallOutput();

    // compare quaternion data with EKF quaternion at the fusion time horizon and calculate correction

    // divide the demanded quaternion by the estimated to get the error
    Quaternion quatErr = stateStruct.quat / outputDataDelayed.quat;

    // Convert to a delta rotation using a small angle approximation
    quatErr.normalize();
    Vector3f deltaAngErr;
    float scaler;
    if (quatErr[0] >= 0.0f) {
        scaler = 2.0f;
    } else {
        scaler = -2.0f;
    }
    deltaAngErr.x = scaler * quatErr[1];
    deltaAngErr.y = scaler * quatErr[2];
    deltaAngErr.z = scaler * quatErr[3];

    // multiply the angle error vector by a gain to calculate the delta angle correction required to track the EKF solution
    const float Kang = 1.0f;
    delAngCorrection = deltaAngErr * imuDataNew.delAngDT * Kang;

    // multiply velocity error by a gain to calculate the delta velocity correction required to track the EKF solution
    const float Kvel = 1.0f;
    delVelCorrection = (stateStruct.velocity - outputDataDelayed.velocity) * imuDataNew.delVelDT * Kvel;

    // multiply position error by a gain to calculate the velocity correction required to track the EKF solution
    const float Kpos = 1.0f;
    velCorrection = (stateStruct.velocity - outputDataDelayed.velocity) * Kpos;

}

// calculate the predicted state covariance matrix
void NavEKF::CovariancePrediction()
{
    perf_begin(_perf_CovariancePrediction);
    float windVelSigma; // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma;// delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma;// delta velocity bias 1-sigma process noise - m/s
    float dAngScaleSigma;// delta angle scale factor 1-Sigma process noise
    float magEarthSigma;// earth magnetic field 1-sigma process noise
    float magBodySigma; // body magnetic field 1-sigma process noise
    float daxNoise;     // X axis delta angle noise (rad)
    float dayNoise;     // Y axis delta angle noise (rad)
    float dazNoise;     // Z axis delta angle noise (rad)
    float dvxNoise;     // X axis delta velocity noise (m/s)
    float dvyNoise;     // Y axis delta velocity noise (m/s)
    float dvzNoise;     // Z axis delta velocity noise (m/s)
    float dvx;          // X axis delta velocity (m/s)
    float dvy;          // Y axis delta velocity (m/s)
    float dvz;          // Z axis delta velocity (m/s)
    float dax;          // X axis delta angle (rad)
    float day;          // Y axis delta angle (rad)
    float daz;          // Z axis delta angle (rad)
    float q0;           // attitude quaternion
    float q1;           // attitude quaternion
    float q2;           // attitude quaternion
    float q3;           // attitude quaternion
    float dax_b;        // X axis delta angle measurement bias (rad)
    float day_b;        // Y axis delta angle measurement bias (rad)
    float daz_b;        // Z axis delta angle measurement bias (rad)
    float dax_s;        // X axis delta angle measurement scale factor
    float day_s;        // Y axis delta angle measurement scale factor
    float daz_s;        // Z axis delta angle measurement scale factor
    float dvz_b;        // Z axis delta velocity measurement bias (rad)

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    // filter height rate using a 10 second time constant filter
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - stateStruct.velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    windVelSigma  = dt * constrain_float(_windVelProcessNoise, 0.01f, 1.0f) * (1.0f + constrain_float(_wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    dAngBiasSigma = dt * constrain_float(_gyroBiasProcessNoise, 1e-8f, 1e-4f);
    dVelBiasSigma = dt * constrain_float(_accelBiasProcessNoise, 1e-6f, 1e-2f);
    dAngScaleSigma = dt * constrain_float(_gyroScaleProcessNoise,1e-8f,1e-5f);
    magEarthSigma = dt * constrain_float(_magProcessNoise, 1e-4f, 1e-2f);
    magBodySigma  = dt * constrain_float(_magProcessNoise, 1e-4f, 1e-2f);
    for (uint8_t i= 0; i<=8;  i++) processNoise[i] = 1.0e-9f;
    for (uint8_t i=9; i<=11; i++) processNoise[i] = dAngBiasSigma;
    for (uint8_t i=10; i<=12; i++) processNoise[i] = dAngBiasSigma;
    for (uint8_t i=12; i<=14; i++) processNoise[i] = dAngScaleSigma;
    if (expectGndEffectTakeoff) {
        processNoise[15] = 0.0f;
    } else if (!filterArmed) {
        processNoise[15] = dVelBiasSigma * accelBiasNoiseScaler;
    } else {
        processNoise[15] = dVelBiasSigma;
    }
    for (uint8_t i=16; i<=18; i++) processNoise[i] = magEarthSigma;
    for (uint8_t i=19; i<=21; i++) processNoise[i] = magBodySigma;
    for (uint8_t i=22; i<=23; i++) processNoise[i] = windVelSigma;

    for (uint8_t i= 0; i<=stateIndexLim; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
    q0 = stateStruct.quat[0];
    q1 = stateStruct.quat[1];
    q2 = stateStruct.quat[2];
    q3 = stateStruct.quat[3];
    dax_b = stateStruct.gyro_bias.x;
    day_b = stateStruct.gyro_bias.y;
    daz_b = stateStruct.gyro_bias.z;
    dax_s = stateStruct.gyro_scale.x;
    day_s = stateStruct.gyro_scale.y;
    daz_s = stateStruct.gyro_scale.z;
    dvz_b = stateStruct.accel_zbias;
    _gyrNoise = constrain_float(_gyrNoise, 1e-3f, 5e-2f);
    daxNoise = dt*_gyrNoise;
    dayNoise = dt*_gyrNoise;
    // Account for 3% scale factor error on Z angular rate. This reduces chance of continuous fast rotations causing loss of yaw reference.
    dazNoise = dt*(pythagorous2(_gyrNoise,0.03f*yawRateFilt));
    _accNoise = constrain_float(_accNoise, 5e-2f, 1.0f);
    dvxNoise = dt*_accNoise;
    dvyNoise = dt*_accNoise;
    dvzNoise = dt*_accNoise;

    // calculate the predicted covariance due to inertial sensor error propagation
    // we calculate the upper diagonal and copy to take advantage of symmetry
    SF[0] = daz_b/2 + dazNoise/2 - (daz*daz_s)/2;
    SF[1] = day_b/2 + dayNoise/2 - (day*day_s)/2;
    SF[2] = dax_b/2 + daxNoise/2 - (dax*dax_s)/2;
    SF[3] = q3/2 - (q0*SF[0])/2 + (q1*SF[1])/2 - (q2*SF[2])/2;
    SF[4] = q0/2 - (q1*SF[2])/2 - (q2*SF[1])/2 + (q3*SF[0])/2;
    SF[5] = q1/2 + (q0*SF[2])/2 - (q2*SF[0])/2 - (q3*SF[1])/2;
    SF[6] = q3/2 + (q0*SF[0])/2 - (q1*SF[1])/2 - (q2*SF[2])/2;
    SF[7] = q0/2 - (q1*SF[2])/2 + (q2*SF[1])/2 - (q3*SF[0])/2;
    SF[8] = q0/2 + (q1*SF[2])/2 - (q2*SF[1])/2 - (q3*SF[0])/2;
    SF[9] = q2/2 + (q0*SF[1])/2 + (q1*SF[0])/2 + (q3*SF[2])/2;
    SF[10] = q2/2 - (q0*SF[1])/2 - (q1*SF[0])/2 + (q3*SF[2])/2;
    SF[11] = q2/2 + (q0*SF[1])/2 - (q1*SF[0])/2 - (q3*SF[2])/2;
    SF[12] = q1/2 + (q0*SF[2])/2 + (q2*SF[0])/2 + (q3*SF[1])/2;
    SF[13] = q1/2 - (q0*SF[2])/2 + (q2*SF[0])/2 - (q3*SF[1])/2;
    SF[14] = q3/2 + (q0*SF[0])/2 + (q1*SF[1])/2 + (q2*SF[2])/2;
    SF[15] = - sq(q0) - sq(q1) - sq(q2) - sq(q3);
    SF[16] = dvz_b - dvz + dvzNoise;
    SF[17] = dvx - dvxNoise;
    SF[18] = dvy - dvyNoise;
    SF[19] = sq(q2);
    SF[20] = SF[19] - sq(q0) + sq(q1) - sq(q3);
    SF[21] = SF[19] + sq(q0) - sq(q1) - sq(q3);
    SF[22] = 2*q0*q1 - 2*q2*q3;
    SF[23] = SF[19] - sq(q0) - sq(q1) + sq(q3);
    SF[24] = 2*q1*q2;

    SG[0] = - sq(q0) - sq(q1) - sq(q2) - sq(q3);
    SG[1] = sq(q3);
    SG[2] = sq(q2);
    SG[3] = sq(q1);
    SG[4] = sq(q0);

    SQ[0] = - dvyNoise*(2*q0*q1 + 2*q2*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvzNoise*(2*q0*q1 - 2*q2*q3)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxNoise*(2*q0*q2 - 2*q1*q3)*(2*q0*q3 + 2*q1*q2);
    SQ[1] = dvxNoise*(2*q0*q2 - 2*q1*q3)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvzNoise*(2*q0*q2 + 2*q1*q3)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyNoise*(2*q0*q1 + 2*q2*q3)*(2*q0*q3 - 2*q1*q2);
    SQ[2] = dvyNoise*(2*q0*q3 - 2*q1*q2)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxNoise*(2*q0*q3 + 2*q1*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) - dvzNoise*(2*q0*q1 - 2*q2*q3)*(2*q0*q2 + 2*q1*q3);
    SQ[3] = sq(SG[0]);
    SQ[4] = 2*q2*q3;
    SQ[5] = 2*q1*q3;
    SQ[6] = 2*q1*q2;
    SQ[7] = SG[4];

    SPP[0] = SF[17]*(2*q0*q1 + 2*q2*q3) + SF[18]*(2*q0*q2 - 2*q1*q3);
    SPP[1] = SF[18]*(2*q0*q2 + 2*q1*q3) + SF[16]*(SF[24] - 2*q0*q3);
    SPP[2] = 2*q3*SF[8] + 2*q1*SF[11] - 2*q0*SF[14] - 2*q2*SF[13];
    SPP[3] = 2*q1*SF[7] + 2*q2*SF[6] - 2*q0*SF[12] - 2*q3*SF[10];
    SPP[4] = 2*q0*SF[6] - 2*q3*SF[7] - 2*q1*SF[10] + 2*q2*SF[12];
    SPP[5] = 2*q0*SF[8] + 2*q2*SF[11] + 2*q1*SF[13] + 2*q3*SF[14];
    SPP[6] = 2*q0*SF[7] + 2*q3*SF[6] + 2*q2*SF[10] + 2*q1*SF[12];
    SPP[7] = SF[18]*SF[20] - SF[16]*(2*q0*q1 + 2*q2*q3);
    SPP[8] = 2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9];
    SPP[9] = 2*q0*SF[5] - 2*q1*SF[4] - 2*q2*SF[3] + 2*q3*SF[9];
    SPP[10] = SF[17]*SF[20] + SF[16]*(2*q0*q2 - 2*q1*q3);
    SPP[11] = SF[17]*SF[21] - SF[18]*(SF[24] + 2*q0*q3);
    SPP[12] = SF[17]*SF[22] - SF[16]*(SF[24] + 2*q0*q3);
    SPP[13] = 2*q0*SF[4] + 2*q1*SF[5] + 2*q3*SF[3] + 2*q2*SF[9];
    SPP[14] = 2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13];
    SPP[15] = SF[18]*SF[23] + SF[17]*(SF[24] - 2*q0*q3);
    SPP[16] = daz*SF[19] + daz*sq(q0) + daz*sq(q1) + daz*sq(q3);
    SPP[17] = day*SF[19] + day*sq(q0) + day*sq(q1) + day*sq(q3);
    SPP[18] = dax*SF[19] + dax*sq(q0) + dax*sq(q1) + dax*sq(q3);
    SPP[19] = SF[16]*SF[23] - SF[17]*(2*q0*q2 + 2*q1*q3);
    SPP[20] = SF[16]*SF[21] - SF[18]*SF[22];
    SPP[21] = 2*q0*q2 + 2*q1*q3;
    SPP[22] = SF[15];

    if (inhibitMagStates) {
        zeroRows(P,16,23);
        zeroCols(P,16,23);
    } else if (inhibitWindStates) {
        zeroRows(P,22,23);
        zeroCols(P,22,23);
    }

    nextP[0][0] = daxNoise*SQ[3] + SPP[5]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[9][0]*SPP[22] + P[12][0]*SPP[18] + P[2][0]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) - SPP[4]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[9][1]*SPP[22] + P[12][1]*SPP[18] + P[2][1]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) + SPP[8]*(P[0][2]*SPP[5] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18] - P[1][2]*(2*q0*SF[6] - 2*q3*SF[7] - 2*q1*SF[10] + 2*q2*SF[12])) + SPP[22]*(P[0][9]*SPP[5] - P[1][9]*SPP[4] + P[9][9]*SPP[22] + P[12][9]*SPP[18] + P[2][9]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) + SPP[18]*(P[0][12]*SPP[5] - P[1][12]*SPP[4] + P[9][12]*SPP[22] + P[12][12]*SPP[18] + P[2][12]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9]));
    nextP[0][1] = SPP[6]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) - SPP[2]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[22]*(P[0][10]*SPP[5] - P[1][10]*SPP[4] + P[2][10]*SPP[8] + P[9][10]*SPP[22] + P[12][10]*SPP[18]) + SPP[17]*(P[0][13]*SPP[5] - P[1][13]*SPP[4] + P[2][13]*SPP[8] + P[9][13]*SPP[22] + P[12][13]*SPP[18]) - (2*q0*SF[5] - 2*q1*SF[4] - 2*q2*SF[3] + 2*q3*SF[9])*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]);
    nextP[1][1] = dayNoise*SQ[3] - SPP[2]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[6]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) - SPP[9]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) + SPP[22]*(P[1][10]*SPP[6] - P[0][10]*SPP[2] - P[2][10]*SPP[9] + P[10][10]*SPP[22] + P[13][10]*SPP[17]) + SPP[17]*(P[1][13]*SPP[6] - P[0][13]*SPP[2] - P[2][13]*SPP[9] + P[10][13]*SPP[22] + P[13][13]*SPP[17]);
    nextP[0][2] = SPP[13]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) - SPP[3]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) + SPP[22]*(P[0][11]*SPP[5] - P[1][11]*SPP[4] + P[2][11]*SPP[8] + P[9][11]*SPP[22] + P[12][11]*SPP[18]) + SPP[16]*(P[0][14]*SPP[5] - P[1][14]*SPP[4] + P[2][14]*SPP[8] + P[9][14]*SPP[22] + P[12][14]*SPP[18]) + (2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13])*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]);
    nextP[1][2] = SPP[13]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) - SPP[3]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) + SPP[22]*(P[1][11]*SPP[6] - P[0][11]*SPP[2] - P[2][11]*SPP[9] + P[10][11]*SPP[22] + P[13][11]*SPP[17]) + SPP[16]*(P[1][14]*SPP[6] - P[0][14]*SPP[2] - P[2][14]*SPP[9] + P[10][14]*SPP[22] + P[13][14]*SPP[17]) + (2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13])*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]);
    nextP[2][2] = dazNoise*SQ[3] - SPP[3]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]) + SPP[14]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[13]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) + SPP[22]*(P[0][11]*SPP[14] - P[1][11]*SPP[3] + P[2][11]*SPP[13] + P[11][11]*SPP[22] + P[14][11]*SPP[16]) + SPP[16]*(P[0][14]*SPP[14] - P[1][14]*SPP[3] + P[2][14]*SPP[13] + P[11][14]*SPP[22] + P[14][14]*SPP[16]);
    nextP[0][3] = P[0][3]*SPP[5] - P[1][3]*SPP[4] + P[2][3]*SPP[8] + P[9][3]*SPP[22] + P[12][3]*SPP[18] + SPP[1]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[15]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) - SPP[21]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]);
    nextP[1][3] = P[1][3]*SPP[6] - P[0][3]*SPP[2] - P[2][3]*SPP[9] + P[10][3]*SPP[22] + P[13][3]*SPP[17] + SPP[1]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[15]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) - SPP[21]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]);
    nextP[2][3] = P[0][3]*SPP[14] - P[1][3]*SPP[3] + P[2][3]*SPP[13] + P[11][3]*SPP[22] + P[14][3]*SPP[16] + SPP[1]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[15]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) - SPP[21]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]);
    nextP[3][3] = P[3][3] + P[0][3]*SPP[1] + P[1][3]*SPP[19] + P[2][3]*SPP[15] - P[15][3]*SPP[21] + dvyNoise*sq(SQ[6] - 2*q0*q3) + dvzNoise*sq(SQ[5] + 2*q0*q2) + SPP[1]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[19]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]) + SPP[15]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]) - SPP[21]*(P[3][15] + P[0][15]*SPP[1] + P[2][15]*SPP[15] - P[15][15]*SPP[21] + P[1][15]*(SF[16]*SF[23] - SF[17]*SPP[21])) + dvxNoise*sq(SG[1] + SG[2] - SG[3] - SQ[7]);
    nextP[0][4] = P[0][4]*SPP[5] - P[1][4]*SPP[4] + P[2][4]*SPP[8] + P[9][4]*SPP[22] + P[12][4]*SPP[18] + SF[22]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) + SPP[12]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) + SPP[20]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[11]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]);
    nextP[1][4] = P[1][4]*SPP[6] - P[0][4]*SPP[2] - P[2][4]*SPP[9] + P[10][4]*SPP[22] + P[13][4]*SPP[17] + SF[22]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) + SPP[12]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) + SPP[20]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[11]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]);
    nextP[2][4] = P[0][4]*SPP[14] - P[1][4]*SPP[3] + P[2][4]*SPP[13] + P[11][4]*SPP[22] + P[14][4]*SPP[16] + SF[22]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) + SPP[12]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]) + SPP[20]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[11]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]);
    nextP[3][4] = P[3][4] + SQ[2] + P[0][4]*SPP[1] + P[1][4]*SPP[19] + P[2][4]*SPP[15] - P[15][4]*SPP[21] + SF[22]*(P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21]) + SPP[12]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]) + SPP[20]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[11]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]);
    nextP[4][4] = P[4][4] + P[15][4]*SF[22] + P[0][4]*SPP[20] + P[1][4]*SPP[12] + P[2][4]*SPP[11] + dvxNoise*sq(SQ[6] + 2*q0*q3) + dvzNoise*sq(SQ[4] - 2*q0*q1) + SF[22]*(P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11]) + SPP[12]*(P[4][1] + P[15][1]*SF[22] + P[0][1]*SPP[20] + P[1][1]*SPP[12] + P[2][1]*SPP[11]) + SPP[20]*(P[4][0] + P[15][0]*SF[22] + P[0][0]*SPP[20] + P[1][0]*SPP[12] + P[2][0]*SPP[11]) + SPP[11]*(P[4][2] + P[15][2]*SF[22] + P[0][2]*SPP[20] + P[1][2]*SPP[12] + P[2][2]*SPP[11]) + dvyNoise*sq(SG[1] - SG[2] + SG[3] - SQ[7]);
    nextP[0][5] = P[0][5]*SPP[5] - P[1][5]*SPP[4] + P[2][5]*SPP[8] + P[9][5]*SPP[22] + P[12][5]*SPP[18] + SF[20]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) - SPP[7]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[0]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) + SPP[10]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]);
    nextP[1][5] = P[1][5]*SPP[6] - P[0][5]*SPP[2] - P[2][5]*SPP[9] + P[10][5]*SPP[22] + P[13][5]*SPP[17] + SF[20]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) - SPP[7]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[0]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) + SPP[10]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]);
    nextP[2][5] = P[0][5]*SPP[14] - P[1][5]*SPP[3] + P[2][5]*SPP[13] + P[11][5]*SPP[22] + P[14][5]*SPP[16] + SF[20]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) - SPP[7]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[0]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) + SPP[10]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]);
    nextP[3][5] = P[3][5] + SQ[1] + P[0][5]*SPP[1] + P[1][5]*SPP[19] + P[2][5]*SPP[15] - P[15][5]*SPP[21] + SF[20]*(P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21]) - SPP[7]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[0]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]) + SPP[10]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]);
    nextP[4][5] = P[4][5] + SQ[0] + P[15][5]*SF[22] + P[0][5]*SPP[20] + P[1][5]*SPP[12] + P[2][5]*SPP[11] + SF[20]*(P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11]) - SPP[7]*(P[4][0] + P[15][0]*SF[22] + P[0][0]*SPP[20] + P[1][0]*SPP[12] + P[2][0]*SPP[11]) + SPP[0]*(P[4][2] + P[15][2]*SF[22] + P[0][2]*SPP[20] + P[1][2]*SPP[12] + P[2][2]*SPP[11]) + SPP[10]*(P[4][1] + P[15][1]*SF[22] + P[0][1]*SPP[20] + P[1][1]*SPP[12] + P[2][1]*SPP[11]);
    nextP[5][5] = P[5][5] + P[15][5]*SF[20] - P[0][5]*SPP[7] + P[1][5]*SPP[10] + P[2][5]*SPP[0] + dvxNoise*sq(SQ[5] - 2*q0*q2) + dvyNoise*sq(SQ[4] + 2*q0*q1) + SF[20]*(P[5][15] + P[15][15]*SF[20] - P[0][15]*SPP[7] + P[1][15]*SPP[10] + P[2][15]*SPP[0]) - SPP[7]*(P[5][0] + P[15][0]*SF[20] - P[0][0]*SPP[7] + P[1][0]*SPP[10] + P[2][0]*SPP[0]) + SPP[0]*(P[5][2] + P[15][2]*SF[20] - P[0][2]*SPP[7] + P[1][2]*SPP[10] + P[2][2]*SPP[0]) + SPP[10]*(P[5][1] + P[15][1]*SF[20] - P[0][1]*SPP[7] + P[1][1]*SPP[10] + P[2][1]*SPP[0]) + dvzNoise*sq(SG[1] - SG[2] - SG[3] + SQ[7]);
    nextP[0][6] = P[0][6]*SPP[5] - P[1][6]*SPP[4] + P[2][6]*SPP[8] + P[9][6]*SPP[22] + P[12][6]*SPP[18] + dt*(P[0][3]*SPP[5] - P[1][3]*SPP[4] + P[2][3]*SPP[8] + P[9][3]*SPP[22] + P[12][3]*SPP[18]);
    nextP[1][6] = P[1][6]*SPP[6] - P[0][6]*SPP[2] - P[2][6]*SPP[9] + P[10][6]*SPP[22] + P[13][6]*SPP[17] + dt*(P[1][3]*SPP[6] - P[0][3]*SPP[2] - P[2][3]*SPP[9] + P[10][3]*SPP[22] + P[13][3]*SPP[17]);
    nextP[2][6] = P[0][6]*SPP[14] - P[1][6]*SPP[3] + P[2][6]*SPP[13] + P[11][6]*SPP[22] + P[14][6]*SPP[16] + dt*(P[0][3]*SPP[14] - P[1][3]*SPP[3] + P[2][3]*SPP[13] + P[11][3]*SPP[22] + P[14][3]*SPP[16]);
    nextP[3][6] = P[3][6] + P[0][6]*SPP[1] + P[1][6]*SPP[19] + P[2][6]*SPP[15] - P[15][6]*SPP[21] + dt*(P[3][3] + P[0][3]*SPP[1] + P[1][3]*SPP[19] + P[2][3]*SPP[15] - P[15][3]*SPP[21]);
    nextP[4][6] = P[4][6] + P[15][6]*SF[22] + P[0][6]*SPP[20] + P[1][6]*SPP[12] + P[2][6]*SPP[11] + dt*(P[4][3] + P[15][3]*SF[22] + P[0][3]*SPP[20] + P[1][3]*SPP[12] + P[2][3]*SPP[11]);
    nextP[5][6] = P[5][6] + P[15][6]*SF[20] - P[0][6]*SPP[7] + P[1][6]*SPP[10] + P[2][6]*SPP[0] + dt*(P[5][3] + P[15][3]*SF[20] - P[0][3]*SPP[7] + P[1][3]*SPP[10] + P[2][3]*SPP[0]);
    nextP[6][6] = P[6][6] + P[3][6]*dt + dt*(P[6][3] + P[3][3]*dt);
    nextP[0][7] = P[0][7]*SPP[5] - P[1][7]*SPP[4] + P[2][7]*SPP[8] + P[9][7]*SPP[22] + P[12][7]*SPP[18] + dt*(P[0][4]*SPP[5] - P[1][4]*SPP[4] + P[2][4]*SPP[8] + P[9][4]*SPP[22] + P[12][4]*SPP[18]);
    nextP[1][7] = P[1][7]*SPP[6] - P[0][7]*SPP[2] - P[2][7]*SPP[9] + P[10][7]*SPP[22] + P[13][7]*SPP[17] + dt*(P[1][4]*SPP[6] - P[0][4]*SPP[2] - P[2][4]*SPP[9] + P[10][4]*SPP[22] + P[13][4]*SPP[17]);
    nextP[2][7] = P[0][7]*SPP[14] - P[1][7]*SPP[3] + P[2][7]*SPP[13] + P[11][7]*SPP[22] + P[14][7]*SPP[16] + dt*(P[0][4]*SPP[14] - P[1][4]*SPP[3] + P[2][4]*SPP[13] + P[11][4]*SPP[22] + P[14][4]*SPP[16]);
    nextP[3][7] = P[3][7] + P[0][7]*SPP[1] + P[1][7]*SPP[19] + P[2][7]*SPP[15] - P[15][7]*SPP[21] + dt*(P[3][4] + P[0][4]*SPP[1] + P[1][4]*SPP[19] + P[2][4]*SPP[15] - P[15][4]*SPP[21]);
    nextP[4][7] = P[4][7] + P[15][7]*SF[22] + P[0][7]*SPP[20] + P[1][7]*SPP[12] + P[2][7]*SPP[11] + dt*(P[4][4] + P[15][4]*SF[22] + P[0][4]*SPP[20] + P[1][4]*SPP[12] + P[2][4]*SPP[11]);
    nextP[5][7] = P[5][7] + P[15][7]*SF[20] - P[0][7]*SPP[7] + P[1][7]*SPP[10] + P[2][7]*SPP[0] + dt*(P[5][4] + P[15][4]*SF[20] - P[0][4]*SPP[7] + P[1][4]*SPP[10] + P[2][4]*SPP[0]);
    nextP[6][7] = P[6][7] + P[3][7]*dt + dt*(P[6][4] + P[3][4]*dt);
    nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
    nextP[0][8] = P[0][8]*SPP[5] - P[1][8]*SPP[4] + P[2][8]*SPP[8] + P[9][8]*SPP[22] + P[12][8]*SPP[18] + dt*(P[0][5]*SPP[5] - P[1][5]*SPP[4] + P[2][5]*SPP[8] + P[9][5]*SPP[22] + P[12][5]*SPP[18]);
    nextP[1][8] = P[1][8]*SPP[6] - P[0][8]*SPP[2] - P[2][8]*SPP[9] + P[10][8]*SPP[22] + P[13][8]*SPP[17] + dt*(P[1][5]*SPP[6] - P[0][5]*SPP[2] - P[2][5]*SPP[9] + P[10][5]*SPP[22] + P[13][5]*SPP[17]);
    nextP[2][8] = P[0][8]*SPP[14] - P[1][8]*SPP[3] + P[2][8]*SPP[13] + P[11][8]*SPP[22] + P[14][8]*SPP[16] + dt*(P[0][5]*SPP[14] - P[1][5]*SPP[3] + P[2][5]*SPP[13] + P[11][5]*SPP[22] + P[14][5]*SPP[16]);
    nextP[3][8] = P[3][8] + P[0][8]*SPP[1] + P[1][8]*SPP[19] + P[2][8]*SPP[15] - P[15][8]*SPP[21] + dt*(P[3][5] + P[0][5]*SPP[1] + P[1][5]*SPP[19] + P[2][5]*SPP[15] - P[15][5]*SPP[21]);
    nextP[4][8] = P[4][8] + P[15][8]*SF[22] + P[0][8]*SPP[20] + P[1][8]*SPP[12] + P[2][8]*SPP[11] + dt*(P[4][5] + P[15][5]*SF[22] + P[0][5]*SPP[20] + P[1][5]*SPP[12] + P[2][5]*SPP[11]);
    nextP[5][8] = P[5][8] + P[15][8]*SF[20] - P[0][8]*SPP[7] + P[1][8]*SPP[10] + P[2][8]*SPP[0] + dt*(P[5][5] + P[15][5]*SF[20] - P[0][5]*SPP[7] + P[1][5]*SPP[10] + P[2][5]*SPP[0]);
    nextP[6][8] = P[6][8] + P[3][8]*dt + dt*(P[6][5] + P[3][5]*dt);
    nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
    nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
    nextP[0][9] = P[0][9]*SPP[5] - P[1][9]*SPP[4] + P[2][9]*SPP[8] + P[9][9]*SPP[22] + P[12][9]*SPP[18];
    nextP[1][9] = P[1][9]*SPP[6] - P[0][9]*SPP[2] - P[2][9]*SPP[9] + P[10][9]*SPP[22] + P[13][9]*SPP[17];
    nextP[2][9] = P[0][9]*SPP[14] - P[1][9]*SPP[3] + P[2][9]*SPP[13] + P[11][9]*SPP[22] + P[14][9]*SPP[16];
    nextP[3][9] = P[3][9] + P[0][9]*SPP[1] + P[1][9]*SPP[19] + P[2][9]*SPP[15] - P[15][9]*SPP[21];
    nextP[4][9] = P[4][9] + P[15][9]*SF[22] + P[0][9]*SPP[20] + P[1][9]*SPP[12] + P[2][9]*SPP[11];
    nextP[5][9] = P[5][9] + P[15][9]*SF[20] - P[0][9]*SPP[7] + P[1][9]*SPP[10] + P[2][9]*SPP[0];
    nextP[6][9] = P[6][9] + P[3][9]*dt;
    nextP[7][9] = P[7][9] + P[4][9]*dt;
    nextP[8][9] = P[8][9] + P[5][9]*dt;
    nextP[9][9] = P[9][9];
    nextP[0][10] = P[0][10]*SPP[5] - P[1][10]*SPP[4] + P[2][10]*SPP[8] + P[9][10]*SPP[22] + P[12][10]*SPP[18];
    nextP[1][10] = P[1][10]*SPP[6] - P[0][10]*SPP[2] - P[2][10]*SPP[9] + P[10][10]*SPP[22] + P[13][10]*SPP[17];
    nextP[2][10] = P[0][10]*SPP[14] - P[1][10]*SPP[3] + P[2][10]*SPP[13] + P[11][10]*SPP[22] + P[14][10]*SPP[16];
    nextP[3][10] = P[3][10] + P[0][10]*SPP[1] + P[1][10]*SPP[19] + P[2][10]*SPP[15] - P[15][10]*SPP[21];
    nextP[4][10] = P[4][10] + P[15][10]*SF[22] + P[0][10]*SPP[20] + P[1][10]*SPP[12] + P[2][10]*SPP[11];
    nextP[5][10] = P[5][10] + P[15][10]*SF[20] - P[0][10]*SPP[7] + P[1][10]*SPP[10] + P[2][10]*SPP[0];
    nextP[6][10] = P[6][10] + P[3][10]*dt;
    nextP[7][10] = P[7][10] + P[4][10]*dt;
    nextP[8][10] = P[8][10] + P[5][10]*dt;
    nextP[9][10] = P[9][10];
    nextP[10][10] = P[10][10];
    nextP[0][11] = P[0][11]*SPP[5] - P[1][11]*SPP[4] + P[2][11]*SPP[8] + P[9][11]*SPP[22] + P[12][11]*SPP[18];
    nextP[1][11] = P[1][11]*SPP[6] - P[0][11]*SPP[2] - P[2][11]*SPP[9] + P[10][11]*SPP[22] + P[13][11]*SPP[17];
    nextP[2][11] = P[0][11]*SPP[14] - P[1][11]*SPP[3] + P[2][11]*SPP[13] + P[11][11]*SPP[22] + P[14][11]*SPP[16];
    nextP[3][11] = P[3][11] + P[0][11]*SPP[1] + P[1][11]*SPP[19] + P[2][11]*SPP[15] - P[15][11]*SPP[21];
    nextP[4][11] = P[4][11] + P[15][11]*SF[22] + P[0][11]*SPP[20] + P[1][11]*SPP[12] + P[2][11]*SPP[11];
    nextP[5][11] = P[5][11] + P[15][11]*SF[20] - P[0][11]*SPP[7] + P[1][11]*SPP[10] + P[2][11]*SPP[0];
    nextP[6][11] = P[6][11] + P[3][11]*dt;
    nextP[7][11] = P[7][11] + P[4][11]*dt;
    nextP[8][11] = P[8][11] + P[5][11]*dt;
    nextP[9][11] = P[9][11];
    nextP[10][11] = P[10][11];
    nextP[11][11] = P[11][11];
    nextP[0][12] = P[0][12]*SPP[5] - P[1][12]*SPP[4] + P[2][12]*SPP[8] + P[9][12]*SPP[22] + P[12][12]*SPP[18];
    nextP[1][12] = P[1][12]*SPP[6] - P[0][12]*SPP[2] - P[2][12]*SPP[9] + P[10][12]*SPP[22] + P[13][12]*SPP[17];
    nextP[2][12] = P[0][12]*SPP[14] - P[1][12]*SPP[3] + P[2][12]*SPP[13] + P[11][12]*SPP[22] + P[14][12]*SPP[16];
    nextP[3][12] = P[3][12] + P[0][12]*SPP[1] + P[1][12]*SPP[19] + P[2][12]*SPP[15] - P[15][12]*SPP[21];
    nextP[4][12] = P[4][12] + P[15][12]*SF[22] + P[0][12]*SPP[20] + P[1][12]*SPP[12] + P[2][12]*SPP[11];
    nextP[5][12] = P[5][12] + P[15][12]*SF[20] - P[0][12]*SPP[7] + P[1][12]*SPP[10] + P[2][12]*SPP[0];
    nextP[6][12] = P[6][12] + P[3][12]*dt;
    nextP[7][12] = P[7][12] + P[4][12]*dt;
    nextP[8][12] = P[8][12] + P[5][12]*dt;
    nextP[9][12] = P[9][12];
    nextP[10][12] = P[10][12];
    nextP[11][12] = P[11][12];
    nextP[12][12] = P[12][12];
    nextP[0][13] = P[0][13]*SPP[5] - P[1][13]*SPP[4] + P[2][13]*SPP[8] + P[9][13]*SPP[22] + P[12][13]*SPP[18];
    nextP[1][13] = P[1][13]*SPP[6] - P[0][13]*SPP[2] - P[2][13]*SPP[9] + P[10][13]*SPP[22] + P[13][13]*SPP[17];
    nextP[2][13] = P[0][13]*SPP[14] - P[1][13]*SPP[3] + P[2][13]*SPP[13] + P[11][13]*SPP[22] + P[14][13]*SPP[16];
    nextP[3][13] = P[3][13] + P[0][13]*SPP[1] + P[1][13]*SPP[19] + P[2][13]*SPP[15] - P[15][13]*SPP[21];
    nextP[4][13] = P[4][13] + P[15][13]*SF[22] + P[0][13]*SPP[20] + P[1][13]*SPP[12] + P[2][13]*SPP[11];
    nextP[5][13] = P[5][13] + P[15][13]*SF[20] - P[0][13]*SPP[7] + P[1][13]*SPP[10] + P[2][13]*SPP[0];
    nextP[6][13] = P[6][13] + P[3][13]*dt;
    nextP[7][13] = P[7][13] + P[4][13]*dt;
    nextP[8][13] = P[8][13] + P[5][13]*dt;
    nextP[9][13] = P[9][13];
    nextP[10][13] = P[10][13];
    nextP[11][13] = P[11][13];
    nextP[12][13] = P[12][13];
    nextP[13][13] = P[13][13];
    nextP[0][14] = P[0][14]*SPP[5] - P[1][14]*SPP[4] + P[2][14]*SPP[8] + P[9][14]*SPP[22] + P[12][14]*SPP[18];
    nextP[1][14] = P[1][14]*SPP[6] - P[0][14]*SPP[2] - P[2][14]*SPP[9] + P[10][14]*SPP[22] + P[13][14]*SPP[17];
    nextP[2][14] = P[0][14]*SPP[14] - P[1][14]*SPP[3] + P[2][14]*SPP[13] + P[11][14]*SPP[22] + P[14][14]*SPP[16];
    nextP[3][14] = P[3][14] + P[0][14]*SPP[1] + P[1][14]*SPP[19] + P[2][14]*SPP[15] - P[15][14]*SPP[21];
    nextP[4][14] = P[4][14] + P[15][14]*SF[22] + P[0][14]*SPP[20] + P[1][14]*SPP[12] + P[2][14]*SPP[11];
    nextP[5][14] = P[5][14] + P[15][14]*SF[20] - P[0][14]*SPP[7] + P[1][14]*SPP[10] + P[2][14]*SPP[0];
    nextP[6][14] = P[6][14] + P[3][14]*dt;
    nextP[7][14] = P[7][14] + P[4][14]*dt;
    nextP[8][14] = P[8][14] + P[5][14]*dt;
    nextP[9][14] = P[9][14];
    nextP[10][14] = P[10][14];
    nextP[11][14] = P[11][14];
    nextP[12][14] = P[12][14];
    nextP[13][14] = P[13][14];
    nextP[14][14] = P[14][14];
    nextP[0][15] = P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18];
    nextP[1][15] = P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17];
    nextP[2][15] = P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16];
    nextP[3][15] = P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21];
    nextP[4][15] = P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11];
    nextP[5][15] = P[5][15] + P[15][15]*SF[20] - P[0][15]*SPP[7] + P[1][15]*SPP[10] + P[2][15]*SPP[0];
    nextP[6][15] = P[6][15] + P[3][15]*dt;
    nextP[7][15] = P[7][15] + P[4][15]*dt;
    nextP[8][15] = P[8][15] + P[5][15]*dt;
    nextP[9][15] = P[9][15];
    nextP[10][15] = P[10][15];
    nextP[11][15] = P[11][15];
    nextP[12][15] = P[12][15];
    nextP[13][15] = P[13][15];
    nextP[14][15] = P[14][15];
    nextP[15][15] = P[15][15];

    if (stateIndexLim > 15) {
        nextP[0][16] = P[0][16]*SPP[5] - P[1][16]*SPP[4] + P[2][16]*SPP[8] + P[9][16]*SPP[22] + P[12][16]*SPP[18];
        nextP[1][16] = P[1][16]*SPP[6] - P[0][16]*SPP[2] - P[2][16]*SPP[9] + P[10][16]*SPP[22] + P[13][16]*SPP[17];
        nextP[2][16] = P[0][16]*SPP[14] - P[1][16]*SPP[3] + P[2][16]*SPP[13] + P[11][16]*SPP[22] + P[14][16]*SPP[16];
        nextP[3][16] = P[3][16] + P[0][16]*SPP[1] + P[1][16]*SPP[19] + P[2][16]*SPP[15] - P[15][16]*SPP[21];
        nextP[4][16] = P[4][16] + P[15][16]*SF[22] + P[0][16]*SPP[20] + P[1][16]*SPP[12] + P[2][16]*SPP[11];
        nextP[5][16] = P[5][16] + P[15][16]*SF[20] - P[0][16]*SPP[7] + P[1][16]*SPP[10] + P[2][16]*SPP[0];
        nextP[6][16] = P[6][16] + P[3][16]*dt;
        nextP[7][16] = P[7][16] + P[4][16]*dt;
        nextP[8][16] = P[8][16] + P[5][16]*dt;
        nextP[9][16] = P[9][16];
        nextP[10][16] = P[10][16];
        nextP[11][16] = P[11][16];
        nextP[12][16] = P[12][16];
        nextP[13][16] = P[13][16];
        nextP[14][16] = P[14][16];
        nextP[15][16] = P[15][16];
        nextP[16][16] = P[16][16];
        nextP[0][17] = P[0][17]*SPP[5] - P[1][17]*SPP[4] + P[2][17]*SPP[8] + P[9][17]*SPP[22] + P[12][17]*SPP[18];
        nextP[1][17] = P[1][17]*SPP[6] - P[0][17]*SPP[2] - P[2][17]*SPP[9] + P[10][17]*SPP[22] + P[13][17]*SPP[17];
        nextP[2][17] = P[0][17]*SPP[14] - P[1][17]*SPP[3] + P[2][17]*SPP[13] + P[11][17]*SPP[22] + P[14][17]*SPP[16];
        nextP[3][17] = P[3][17] + P[0][17]*SPP[1] + P[1][17]*SPP[19] + P[2][17]*SPP[15] - P[15][17]*SPP[21];
        nextP[4][17] = P[4][17] + P[15][17]*SF[22] + P[0][17]*SPP[20] + P[1][17]*SPP[12] + P[2][17]*SPP[11];
        nextP[5][17] = P[5][17] + P[15][17]*SF[20] - P[0][17]*SPP[7] + P[1][17]*SPP[10] + P[2][17]*SPP[0];
        nextP[6][17] = P[6][17] + P[3][17]*dt;
        nextP[7][17] = P[7][17] + P[4][17]*dt;
        nextP[8][17] = P[8][17] + P[5][17]*dt;
        nextP[9][17] = P[9][17];
        nextP[10][17] = P[10][17];
        nextP[11][17] = P[11][17];
        nextP[12][17] = P[12][17];
        nextP[13][17] = P[13][17];
        nextP[14][17] = P[14][17];
        nextP[15][17] = P[15][17];
        nextP[16][17] = P[16][17];
        nextP[17][17] = P[17][17];
        nextP[0][18] = P[0][18]*SPP[5] - P[1][18]*SPP[4] + P[2][18]*SPP[8] + P[9][18]*SPP[22] + P[12][18]*SPP[18];
        nextP[1][18] = P[1][18]*SPP[6] - P[0][18]*SPP[2] - P[2][18]*SPP[9] + P[10][18]*SPP[22] + P[13][18]*SPP[17];
        nextP[2][18] = P[0][18]*SPP[14] - P[1][18]*SPP[3] + P[2][18]*SPP[13] + P[11][18]*SPP[22] + P[14][18]*SPP[16];
        nextP[3][18] = P[3][18] + P[0][18]*SPP[1] + P[1][18]*SPP[19] + P[2][18]*SPP[15] - P[15][18]*SPP[21];
        nextP[4][18] = P[4][18] + P[15][18]*SF[22] + P[0][18]*SPP[20] + P[1][18]*SPP[12] + P[2][18]*SPP[11];
        nextP[5][18] = P[5][18] + P[15][18]*SF[20] - P[0][18]*SPP[7] + P[1][18]*SPP[10] + P[2][18]*SPP[0];
        nextP[6][18] = P[6][18] + P[3][18]*dt;
        nextP[7][18] = P[7][18] + P[4][18]*dt;
        nextP[8][18] = P[8][18] + P[5][18]*dt;
        nextP[9][18] = P[9][18];
        nextP[10][18] = P[10][18];
        nextP[11][18] = P[11][18];
        nextP[12][18] = P[12][18];
        nextP[13][18] = P[13][18];
        nextP[14][18] = P[14][18];
        nextP[15][18] = P[15][18];
        nextP[16][18] = P[16][18];
        nextP[17][18] = P[17][18];
        nextP[18][18] = P[18][18];
        nextP[0][19] = P[0][19]*SPP[5] - P[1][19]*SPP[4] + P[2][19]*SPP[8] + P[9][19]*SPP[22] + P[12][19]*SPP[18];
        nextP[1][19] = P[1][19]*SPP[6] - P[0][19]*SPP[2] - P[2][19]*SPP[9] + P[10][19]*SPP[22] + P[13][19]*SPP[17];
        nextP[2][19] = P[0][19]*SPP[14] - P[1][19]*SPP[3] + P[2][19]*SPP[13] + P[11][19]*SPP[22] + P[14][19]*SPP[16];
        nextP[3][19] = P[3][19] + P[0][19]*SPP[1] + P[1][19]*SPP[19] + P[2][19]*SPP[15] - P[15][19]*SPP[21];
        nextP[4][19] = P[4][19] + P[15][19]*SF[22] + P[0][19]*SPP[20] + P[1][19]*SPP[12] + P[2][19]*SPP[11];
        nextP[5][19] = P[5][19] + P[15][19]*SF[20] - P[0][19]*SPP[7] + P[1][19]*SPP[10] + P[2][19]*SPP[0];
        nextP[6][19] = P[6][19] + P[3][19]*dt;
        nextP[7][19] = P[7][19] + P[4][19]*dt;
        nextP[8][19] = P[8][19] + P[5][19]*dt;
        nextP[9][19] = P[9][19];
        nextP[10][19] = P[10][19];
        nextP[11][19] = P[11][19];
        nextP[12][19] = P[12][19];
        nextP[13][19] = P[13][19];
        nextP[14][19] = P[14][19];
        nextP[15][19] = P[15][19];
        nextP[16][19] = P[16][19];
        nextP[17][19] = P[17][19];
        nextP[18][19] = P[18][19];
        nextP[19][19] = P[19][19];
        nextP[0][20] = P[0][20]*SPP[5] - P[1][20]*SPP[4] + P[2][20]*SPP[8] + P[9][20]*SPP[22] + P[12][20]*SPP[18];
        nextP[1][20] = P[1][20]*SPP[6] - P[0][20]*SPP[2] - P[2][20]*SPP[9] + P[10][20]*SPP[22] + P[13][20]*SPP[17];
        nextP[2][20] = P[0][20]*SPP[14] - P[1][20]*SPP[3] + P[2][20]*SPP[13] + P[11][20]*SPP[22] + P[14][20]*SPP[16];
        nextP[3][20] = P[3][20] + P[0][20]*SPP[1] + P[1][20]*SPP[19] + P[2][20]*SPP[15] - P[15][20]*SPP[21];
        nextP[4][20] = P[4][20] + P[15][20]*SF[22] + P[0][20]*SPP[20] + P[1][20]*SPP[12] + P[2][20]*SPP[11];
        nextP[5][20] = P[5][20] + P[15][20]*SF[20] - P[0][20]*SPP[7] + P[1][20]*SPP[10] + P[2][20]*SPP[0];
        nextP[6][20] = P[6][20] + P[3][20]*dt;
        nextP[7][20] = P[7][20] + P[4][20]*dt;
        nextP[8][20] = P[8][20] + P[5][20]*dt;
        nextP[9][20] = P[9][20];
        nextP[10][20] = P[10][20];
        nextP[11][20] = P[11][20];
        nextP[12][20] = P[12][20];
        nextP[13][20] = P[13][20];
        nextP[14][20] = P[14][20];
        nextP[15][20] = P[15][20];
        nextP[16][20] = P[16][20];
        nextP[17][20] = P[17][20];
        nextP[18][20] = P[18][20];
        nextP[19][20] = P[19][20];
        nextP[20][20] = P[20][20];
        nextP[0][21] = P[0][21]*SPP[5] - P[1][21]*SPP[4] + P[2][21]*SPP[8] + P[9][21]*SPP[22] + P[12][21]*SPP[18];
        nextP[1][21] = P[1][21]*SPP[6] - P[0][21]*SPP[2] - P[2][21]*SPP[9] + P[10][21]*SPP[22] + P[13][21]*SPP[17];
        nextP[2][21] = P[0][21]*SPP[14] - P[1][21]*SPP[3] + P[2][21]*SPP[13] + P[11][21]*SPP[22] + P[14][21]*SPP[16];
        nextP[3][21] = P[3][21] + P[0][21]*SPP[1] + P[1][21]*SPP[19] + P[2][21]*SPP[15] - P[15][21]*SPP[21];
        nextP[4][21] = P[4][21] + P[15][21]*SF[22] + P[0][21]*SPP[20] + P[1][21]*SPP[12] + P[2][21]*SPP[11];
        nextP[5][21] = P[5][21] + P[15][21]*SF[20] - P[0][21]*SPP[7] + P[1][21]*SPP[10] + P[2][21]*SPP[0];
        nextP[6][21] = P[6][21] + P[3][21]*dt;
        nextP[7][21] = P[7][21] + P[4][21]*dt;
        nextP[8][21] = P[8][21] + P[5][21]*dt;
        nextP[9][21] = P[9][21];
        nextP[10][21] = P[10][21];
        nextP[11][21] = P[11][21];
        nextP[12][21] = P[12][21];
        nextP[13][21] = P[13][21];
        nextP[14][21] = P[14][21];
        nextP[15][21] = P[15][21];
        nextP[16][21] = P[16][21];
        nextP[17][21] = P[17][21];
        nextP[18][21] = P[18][21];
        nextP[19][21] = P[19][21];
        nextP[20][21] = P[20][21];
        nextP[21][21] = P[21][21];

        if (stateIndexLim > 21) {
            nextP[0][22] = P[0][22]*SPP[5] - P[1][22]*SPP[4] + P[2][22]*SPP[8] + P[9][22]*SPP[22] + P[12][22]*SPP[18];
            nextP[1][22] = P[1][22]*SPP[6] - P[0][22]*SPP[2] - P[2][22]*SPP[9] + P[10][22]*SPP[22] + P[13][22]*SPP[17];
            nextP[2][22] = P[0][22]*SPP[14] - P[1][22]*SPP[3] + P[2][22]*SPP[13] + P[11][22]*SPP[22] + P[14][22]*SPP[16];
            nextP[3][22] = P[3][22] + P[0][22]*SPP[1] + P[1][22]*SPP[19] + P[2][22]*SPP[15] - P[15][22]*SPP[21];
            nextP[4][22] = P[4][22] + P[15][22]*SF[22] + P[0][22]*SPP[20] + P[1][22]*SPP[12] + P[2][22]*SPP[11];
            nextP[5][22] = P[5][22] + P[15][22]*SF[20] - P[0][22]*SPP[7] + P[1][22]*SPP[10] + P[2][22]*SPP[0];
            nextP[6][22] = P[6][22] + P[3][22]*dt;
            nextP[7][22] = P[7][22] + P[4][22]*dt;
            nextP[8][22] = P[8][22] + P[5][22]*dt;
            nextP[9][22] = P[9][22];
            nextP[10][22] = P[10][22];
            nextP[11][22] = P[11][22];
            nextP[12][22] = P[12][22];
            nextP[13][22] = P[13][22];
            nextP[14][22] = P[14][22];
            nextP[15][22] = P[15][22];
            nextP[16][22] = P[16][22];
            nextP[17][22] = P[17][22];
            nextP[18][22] = P[18][22];
            nextP[19][22] = P[19][22];
            nextP[20][22] = P[20][22];
            nextP[21][22] = P[21][22];
            nextP[22][22] = P[22][22];
            nextP[0][23] = P[0][23]*SPP[5] - P[1][23]*SPP[4] + P[2][23]*SPP[8] + P[9][23]*SPP[22] + P[12][23]*SPP[18];
            nextP[1][23] = P[1][23]*SPP[6] - P[0][23]*SPP[2] - P[2][23]*SPP[9] + P[10][23]*SPP[22] + P[13][23]*SPP[17];
            nextP[2][23] = P[0][23]*SPP[14] - P[1][23]*SPP[3] + P[2][23]*SPP[13] + P[11][23]*SPP[22] + P[14][23]*SPP[16];
            nextP[3][23] = P[3][23] + P[0][23]*SPP[1] + P[1][23]*SPP[19] + P[2][23]*SPP[15] - P[15][23]*SPP[21];
            nextP[4][23] = P[4][23] + P[15][23]*SF[22] + P[0][23]*SPP[20] + P[1][23]*SPP[12] + P[2][23]*SPP[11];
            nextP[5][23] = P[5][23] + P[15][23]*SF[20] - P[0][23]*SPP[7] + P[1][23]*SPP[10] + P[2][23]*SPP[0];
            nextP[6][23] = P[6][23] + P[3][23]*dt;
            nextP[7][23] = P[7][23] + P[4][23]*dt;
            nextP[8][23] = P[8][23] + P[5][23]*dt;
            nextP[9][23] = P[9][23];
            nextP[10][23] = P[10][23];
            nextP[11][23] = P[11][23];
            nextP[12][23] = P[12][23];
            nextP[13][23] = P[13][23];
            nextP[14][23] = P[14][23];
            nextP[15][23] = P[15][23];
            nextP[16][23] = P[16][23];
            nextP[17][23] = P[17][23];
            nextP[18][23] = P[18][23];
            nextP[19][23] = P[19][23];
            nextP[20][23] = P[20][23];
            nextP[21][23] = P[21][23];
            nextP[22][23] = P[22][23];
            nextP[23][23] = P[23][23];
        }
    }

    // Copy upper diagonal to lower diagonal taking advantage of symmetry
    for (uint8_t colIndex=0; colIndex<=stateIndexLim; colIndex++)
    {
        for (uint8_t rowIndex=0; rowIndex<colIndex; rowIndex++)
        {
            nextP[colIndex][rowIndex] = nextP[rowIndex][colIndex];
         }
    }

    // add the general state process noise variances
    for (uint8_t i=0; i<=stateIndexLim; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[6][6] + P[7][7]) > 1e4f)
    {
        for (uint8_t i=6; i<=7; i++)
        {
            for (uint8_t j=0; j<=stateIndexLim; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // copy covariances to output
    CopyCovariances();

    // constrain diagonals to prevent ill-conditioning
    ConstrainVariances();

    // set the flag to indicate that covariance prediction has been performed and reset the increments used by the covariance prediction
    covPredStep = true;
    summedDelAng.zero();
    summedDelVel.zero();
    dt = 0.0f;

    perf_end(_perf_CovariancePrediction);
}

// fuse selected position, velocity and height measurements
void NavEKF::FuseVelPosNED()
{
    // start performance timer
    perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    Vector3f velInnov;
    Vector3f velInnov1;
    Vector3f velInnov2;

    // declare variables used to control access to arrays
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    float posErr;
    Vector6 R_OBS; // Measurement variances used for fusion
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only
    Vector6 observation;
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData) {

        // set the GPS data timeout depending on whether airspeed data is present
        uint32_t gpsRetryTime;
        if (useAirspeed()) gpsRetryTime = gpsRetryTimeUseTAS;
        else gpsRetryTime = gpsRetryTimeNoTAS;

        // form the observation vector and zero velocity and horizontal position observations if in constant position mode
        // If in constant velocity mode, hold the last known horizontal velocity vector
        if (!constPosMode && !constVelMode) {
            observation[0] = gpsDataDelayed.vel.x + gpsVelGlitchOffset.x;
            observation[1] = gpsDataDelayed.vel.y + gpsVelGlitchOffset.y;
            observation[2] = gpsDataDelayed.vel.z;
            observation[3] = gpsDataDelayed.pos.x + gpsPosGlitchOffsetNE.x;
            observation[4] = gpsDataDelayed.pos.y + gpsPosGlitchOffsetNE.y;
        } else if (constPosMode){
            for (uint8_t i=0; i<=4; i++) observation[i] = 0.0f;
        } else if (constVelMode) {
            observation[0] = heldVelNE.x;
            observation[1] = heldVelNE.y;
            for (uint8_t i=2; i<=4; i++) observation[i] = 0.0f;
        }
        observation[5] = -baroDataDelayed.hgt;

        // calculate additional error in GPS position caused by manoeuvring
        posErr = gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // if the GPS is able to report a speed error, we use it to adjust the observation noise for GPS velocity
        // otherwise we scale it using manoeuvre acceleration
        if (gpsSpdAccuracy > 0.0f) {
            // use GPS receivers reported speed accuracy - floor at value set by gps noise parameter
            R_OBS[0] = sq(constrain_float(gpsSpdAccuracy, _gpsHorizVelNoise, 50.0f));
            R_OBS[2] = sq(constrain_float(gpsSpdAccuracy, _gpsVertVelNoise, 50.0f));
        } else {
            // calculate additional error in GPS velocity caused by manoeuvring
            R_OBS[0] = sq(constrain_float(_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(gpsNEVelVarAccScale * accNavMag);
            R_OBS[2] = sq(constrain_float(_gpsVertVelNoise,  0.05f, 5.0f)) + sq(gpsDVelVarAccScale  * accNavMag);
        }
        R_OBS[1] = R_OBS[0];
        R_OBS[3] = sq(constrain_float(_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = sq(constrain_float(_baroAltNoise, 0.1f, 10.0f));

        // reduce weighting (increase observation noise) on baro if we are likely to be in ground effect
        if ((getTakeoffExpected() || getTouchdownExpected()) && filterArmed) {
            R_OBS[5] *= gndEffectBaroScaler;
        }

        // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
        // For horizontal GPs velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPs perfomrance
        // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
        for (uint8_t i=0; i<=1; i++) R_OBS_DATA_CHECKS[i] = sq(constrain_float(_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(gpsNEVelVarAccScale * accNavMag);
        for (uint8_t i=2; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];


        // if vertical GPS velocity data is being used, check to see if the GPS vertical velocity and barometer
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        if (useGpsVertVel && fuseVelData && (imuSampleTime_ms - lastHgtReceived_ms) <  (2 * msecHgtAvg)) {
            // calculate innovations for height and vertical GPS vel measurements
            float hgtErr  = stateStruct.position.z - observation[5];
            float velDErr = stateStruct.velocity.z - observation[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[8][8] + R_OBS_DATA_CHECKS[5])) && (sq(velDErr) > 9.0f * (P[5][5] + R_OBS_DATA_CHECKS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }

        // calculate innovations and check GPS data validity using an innovation consistency check
        // test position measurements
        if (fusePosData) {
            // test horizontal position measurements
            innovVelPos[3] = stateStruct.position.x - observation[3];
            innovVelPos[4] = stateStruct.position.y - observation[4];
            varInnovVelPos[3] = P[6][6] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[7][7] + R_OBS_DATA_CHECKS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            float maxPosInnov2 = sq(_gpsPosInnovGate)*(varInnovVelPos[3] + varInnovVelPos[4]);
            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // declare a timeout condition if we have been too long without data or not aiding
            posTimeout = (((imuSampleTime_ms - lastPosPassTime) > gpsRetryTime) || PV_AidingMode == AID_NONE);
            // use position data if healthy, timed out, or in constant position mode
            if (posHealth || posTimeout || constPosMode) {
                posHealth = true;
                // only reset the failed time and do glitch timeout checks if we are doing full aiding
                if (PV_AidingMode == AID_ABSOLUTE) {
                    lastPosPassTime = imuSampleTime_ms;
                    // if timed out or outside the specified glitch radius, increment the offset applied to GPS data to compensate for large GPS position jumps
                    if (posTimeout || (maxPosInnov2 > sq(float(_gpsGlitchRadiusMax)))) {
                        gpsPosGlitchOffsetNE.x += innovVelPos[3];
                        gpsPosGlitchOffsetNE.y += innovVelPos[4];
                        // limit the radius of the offset and decay the offset to zero radially
                        decayGpsOffset();
                        // reset the position to the current GPS position which will include the glitch correction offset
                        ResetPosition();
                        // reset the velocity to the GPS velocity
                        ResetVelocity();
                        // don't fuse data on this time step
                        fusePosData = false;
                        // record the fail time
                        lastPosFailTime = imuSampleTime_ms;
                        // Reset the normalised innovation to avoid false failing the bad position fusion test
                        posTestRatio = 0.0f;
                    }
                }
            } else {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            if (_fusionModeGPS == 1 || constVelMode) {
                imax = 1;
            }
            float innovVelSumSq = 0; // sum of squares of velocity innovations
            float varVelSum = 0; // sum of velocity innovation variances
            for (uint8_t i = 0; i<=imax; i++) {
                // velocity states start at index 3
                stateIndex   = i + 3;
                // calculate innovations using blended and single IMU predicted states
                velInnov[i]  = stateStruct.velocity[i] - observation[i]; // blended
                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                // sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov[i]);
                varVelSum += varInnovVelPos[i];
            }
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(_gpsVelInnovGate));
            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);
            // declare a timeout if we have not fused velocity data for too long or not aiding
            velTimeout = (((imuSampleTime_ms - lastVelPassTime) > gpsRetryTime) || PV_AidingMode == AID_NONE);
            // if data is healthy  or in constant velocity mode we fuse it
            if (velHealth || velTimeout || constVelMode) {
                velHealth = true;
                // restart the timeout count
                lastVelPassTime = imuSampleTime_ms;
            } else if (velTimeout && !posHealth && PV_AidingMode == AID_ABSOLUTE) {
                // if data is not healthy and timed out and position is unhealthy and we are using aiding, we reset the velocity, but do not fuse data on this time step
                ResetVelocity();
                fuseVelData =  false;
            } else {
                // if data is unhealthy and position is healthy, we do not fuse it
                velHealth = false;
            }
        }

        // test height measurements
        if (fuseHgtData) {
            // calculate height innovations
            innovVelPos[5] = stateStruct.position.z - observation[5];

            varInnovVelPos[5] = P[8][8] + R_OBS_DATA_CHECKS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(_hgtInnovGate) * varInnovVelPos[5]);
            // fail if the ratio is > 1, but don't fail if bad IMU data
            hgtHealth = ((hgtTestRatio < 1.0f) || badIMUdata);
            hgtTimeout = (imuSampleTime_ms - lastHgtPassTime) > hgtRetryTime;
            // Fuse height data if healthy or timed out or in constant position mode
            if (hgtHealth || hgtTimeout || constPosMode) {
                hgtHealth = true;
                lastHgtPassTime = imuSampleTime_ms;
                // if timed out, reset the height, but do not fuse data on this time step
                if (hgtTimeout) {
                    ResetHeight();
                    fuseHgtData = false;
                }
            }
            else {
                hgtHealth = false;
            }
        }

        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
        if (fuseVelData && velHealth) {
            fuseData[0] = true;
            fuseData[1] = true;
            if (useGpsVertVel) {
                fuseData[2] = true;
            }
            tiltErrVec.zero();
        }
        if (fusePosData && posHealth) {
            fuseData[3] = true;
            fuseData[4] = true;
            tiltErrVec.zero();
        }
        if (fuseHgtData && hgtHealth) {
            fuseData[5] = true;
        }
        if (constVelMode) {
            fuseData[0] = true;
            fuseData[1] = true;
            tiltErrVec.zero();
        }

        // fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 3 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                }
                else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - observation[obsIndex];
                    if (obsIndex == 5) {
                        static const float gndMaxBaroErr = 4.0f;
                        static const float gndBaroInnovFloor = -0.5f;

                        if(getTouchdownExpected()) {
                            // when a touchdown is expected, floor the barometer innovation at gndBaroInnovFloor
                            // constrain the correction between 0 and gndBaroInnovFloor+gndMaxBaroErr
                            // this function looks like this:
                            //         |/
                            //---------|---------
                            //    ____/|
                            //   /     |
                            //  /      |
                            innovVelPos[5] += constrain_float(-innovVelPos[5]+gndBaroInnovFloor, 0.0f, gndBaroInnovFloor+gndMaxBaroErr);
                        }
                    }
                }

                // calculate the Kalman gain and calculate innovation variances
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=15; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                if (!inhibitMagStates) {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = 0.0f;
                    }
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                if (!inhibitWindStates) {
                    Kfusion[22] = P[22][stateIndex]*SK;
                    Kfusion[23] = P[23][stateIndex]*SK;
                } else {
                    Kfusion[22] = 0.0f;
                    Kfusion[23] = 0.0f;
                }

                // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
                stateStruct.angErr.zero();

                // calculate state corrections and re-normalise the quaternions for states predicted using the blended IMU data
                // Don't apply corrections to Z bias state as this has been done already as part of the single IMU calculations
                for (uint8_t i = 0; i<=stateIndexLim; i++) {
                        statesArray[i] = statesArray[i] - Kfusion[i] * innovVelPos[obsIndex];
                }

                // the first 3 states represent the angular misalignment vector. This is
                // is used to correct the estimated quaternion
                correctQuatStates(stateStruct.angErr);

                // sum the attitude error from velocity and position fusion only
                // used as a metric for convergence monitoring
                if (obsIndex != 5) {
                    tiltErrVec += stateStruct.angErr;
                }

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++) {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop performance timer
    perf_end(_perf_FuseVelPosNED);
}

// fuse magnetometer measurements and apply innovation consistency checks
// fuse each axis on consecutive time steps to spread computional load
void NavEKF::FuseMagnetometer()
{
    // declarations
    ftype &q0 = mag_state.q0;
    ftype &q1 = mag_state.q1;
    ftype &q2 = mag_state.q2;
    ftype &q3 = mag_state.q3;
    ftype &magN = mag_state.magN;
    ftype &magE = mag_state.magE;
    ftype &magD = mag_state.magD;
    ftype &magXbias = mag_state.magXbias;
    ftype &magYbias = mag_state.magYbias;
    ftype &magZbias = mag_state.magZbias;
    uint8_t &obsIndex = mag_state.obsIndex;
    Matrix3f &DCM = mag_state.DCM;
    Vector3f &MagPred = mag_state.MagPred;
    ftype &R_MAG = mag_state.R_MAG;
    ftype *SH_MAG = &mag_state.SH_MAG[0];
    Vector24 H_MAG;
    Vector6 SK_MX;
    Vector6 SK_MY;
    Vector6 SK_MZ;

    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    // calculate observation jacobians and Kalman gains
    if (obsIndex == 0)
    {
        // copy required states to local variable names
        q0       = stateStruct.quat[0];
        q1       = stateStruct.quat[1];
        q2       = stateStruct.quat[2];
        q3       = stateStruct.quat[3];
        magN     = stateStruct.earth_magfield[0];
        magE     = stateStruct.earth_magfield[1];
        magD     = stateStruct.earth_magfield[2];
        magXbias = stateStruct.body_magfield[0];
        magYbias = stateStruct.body_magfield[1];
        magZbias = stateStruct.body_magfield[2];

        // rotate predicted earth components into body axes and calculate
        // predicted measurements
        DCM[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
        DCM[0][1] = 2*(q1*q2 + q0*q3);
        DCM[0][2] = 2*(q1*q3-q0*q2);
        DCM[1][0] = 2*(q1*q2 - q0*q3);
        DCM[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        DCM[1][2] = 2*(q2*q3 + q0*q1);
        DCM[2][0] = 2*(q1*q3 + q0*q2);
        DCM[2][1] = 2*(q2*q3 - q0*q1);
        DCM[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        MagPred[0] = DCM[0][0]*magN + DCM[0][1]*magE  + DCM[0][2]*magD + magXbias;
        MagPred[1] = DCM[1][0]*magN + DCM[1][1]*magE  + DCM[1][2]*magD + magYbias;
        MagPred[2] = DCM[2][0]*magN + DCM[2][1]*magE  + DCM[2][2]*magD + magZbias;

        // scale magnetometer observation error with total angular rate
        R_MAG = sq(constrain_float(_magNoise, 0.01f, 0.5f)) + sq(magVarRateScale*imuDataDelayed.delAng.length() / dtIMUavg);

        // calculate observation jacobians
        SH_MAG[0] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
        SH_MAG[1] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_MAG[2] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
        SH_MAG[3] = 2*q0*q1 + 2*q2*q3;
        SH_MAG[4] = 2*q0*q3 + 2*q1*q2;
        SH_MAG[5] = 2*q0*q2 + 2*q1*q3;
        SH_MAG[6] = magE*(2*q0*q1 - 2*q2*q3);
        SH_MAG[7] = 2*q1*q3 - 2*q0*q2;
        SH_MAG[8] = 2*q0*q3;
        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
        H_MAG[1] = SH_MAG[6] - magD*SH_MAG[2] - magN*SH_MAG[5];
        H_MAG[2] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2);
        H_MAG[16] = SH_MAG[1];
        H_MAG[17] = SH_MAG[4];
        H_MAG[18] = SH_MAG[7];
        H_MAG[19] = 1;

        // calculate Kalman gain
        varInnovMag[0] = (P[19][19] + R_MAG - P[1][19]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][19]*SH_MAG[1] + P[17][19]*SH_MAG[4] + P[18][19]*SH_MAG[7] + P[2][19]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2)) - (magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5])*(P[19][1] - P[1][1]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][1]*SH_MAG[1] + P[17][1]*SH_MAG[4] + P[18][1]*SH_MAG[7] + P[2][1]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2))) + SH_MAG[1]*(P[19][16] - P[1][16]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][16]*SH_MAG[1] + P[17][16]*SH_MAG[4] + P[18][16]*SH_MAG[7] + P[2][16]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2))) + SH_MAG[4]*(P[19][17] - P[1][17]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][17]*SH_MAG[1] + P[17][17]*SH_MAG[4] + P[18][17]*SH_MAG[7] + P[2][17]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2))) + SH_MAG[7]*(P[19][18] - P[1][18]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][18]*SH_MAG[1] + P[17][18]*SH_MAG[4] + P[18][18]*SH_MAG[7] + P[2][18]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2))) + (magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2))*(P[19][2] - P[1][2]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][2]*SH_MAG[1] + P[17][2]*SH_MAG[4] + P[18][2]*SH_MAG[7] + P[2][2]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2))));
        if (varInnovMag[0] >= R_MAG) {
            SK_MX[0] = 1.0f / varInnovMag[0];
            faultStatus.bad_xmag = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            obsIndex = 1;
            faultStatus.bad_xmag = true;
            return;
        }
        SK_MX[1] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2);
        SK_MX[2] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
        SK_MX[3] = SH_MAG[7];
        Kfusion[0] = SK_MX[0]*(P[0][19] + P[0][16]*SH_MAG[1] + P[0][17]*SH_MAG[4] - P[0][1]*SK_MX[2] + P[0][2]*SK_MX[1] + P[0][18]*SK_MX[3]);
        Kfusion[1] = SK_MX[0]*(P[1][19] + P[1][16]*SH_MAG[1] + P[1][17]*SH_MAG[4] - P[1][1]*SK_MX[2] + P[1][2]*SK_MX[1] + P[1][18]*SK_MX[3]);
        Kfusion[2] = SK_MX[0]*(P[2][19] + P[2][16]*SH_MAG[1] + P[2][17]*SH_MAG[4] - P[2][1]*SK_MX[2] + P[2][2]*SK_MX[1] + P[2][18]*SK_MX[3]);
        Kfusion[3] = SK_MX[0]*(P[3][19] + P[3][16]*SH_MAG[1] + P[3][17]*SH_MAG[4] - P[3][1]*SK_MX[2] + P[3][2]*SK_MX[1] + P[3][18]*SK_MX[3]);
        Kfusion[4] = SK_MX[0]*(P[4][19] + P[4][16]*SH_MAG[1] + P[4][17]*SH_MAG[4] - P[4][1]*SK_MX[2] + P[4][2]*SK_MX[1] + P[4][18]*SK_MX[3]);
        Kfusion[5] = SK_MX[0]*(P[5][19] + P[5][16]*SH_MAG[1] + P[5][17]*SH_MAG[4] - P[5][1]*SK_MX[2] + P[5][2]*SK_MX[1] + P[5][18]*SK_MX[3]);
        Kfusion[6] = SK_MX[0]*(P[6][19] + P[6][16]*SH_MAG[1] + P[6][17]*SH_MAG[4] - P[6][1]*SK_MX[2] + P[6][2]*SK_MX[1] + P[6][18]*SK_MX[3]);
        Kfusion[7] = SK_MX[0]*(P[7][19] + P[7][16]*SH_MAG[1] + P[7][17]*SH_MAG[4] - P[7][1]*SK_MX[2] + P[7][2]*SK_MX[1] + P[7][18]*SK_MX[3]);
        Kfusion[8] = SK_MX[0]*(P[8][19] + P[8][16]*SH_MAG[1] + P[8][17]*SH_MAG[4] - P[8][1]*SK_MX[2] + P[8][2]*SK_MX[1] + P[8][18]*SK_MX[3]);
        Kfusion[9] = SK_MX[0]*(P[9][19] + P[9][16]*SH_MAG[1] + P[9][17]*SH_MAG[4] - P[9][1]*SK_MX[2] + P[9][2]*SK_MX[1] + P[9][18]*SK_MX[3]);
        Kfusion[10] = SK_MX[0]*(P[10][19] + P[10][16]*SH_MAG[1] + P[10][17]*SH_MAG[4] - P[10][1]*SK_MX[2] + P[10][2]*SK_MX[1] + P[10][18]*SK_MX[3]);
        Kfusion[11] = SK_MX[0]*(P[11][19] + P[11][16]*SH_MAG[1] + P[11][17]*SH_MAG[4] - P[11][1]*SK_MX[2] + P[11][2]*SK_MX[1] + P[11][18]*SK_MX[3]);
        Kfusion[12] = SK_MX[0]*(P[12][19] + P[12][16]*SH_MAG[1] + P[12][17]*SH_MAG[4] - P[12][1]*SK_MX[2] + P[12][2]*SK_MX[1] + P[12][18]*SK_MX[3]);
        Kfusion[13] = SK_MX[0]*(P[13][19] + P[13][16]*SH_MAG[1] + P[13][17]*SH_MAG[4] - P[13][1]*SK_MX[2] + P[13][2]*SK_MX[1] + P[13][18]*SK_MX[3]);
        Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][16]*SH_MAG[1] + P[14][17]*SH_MAG[4] - P[14][1]*SK_MX[2] + P[14][2]*SK_MX[1] + P[14][18]*SK_MX[3]);
        // this term has been zeroed to improve stability of the Z accel bias
        Kfusion[15] = 0.0f;//SK_MX[0]*(P[15][19] + P[15][16]*SH_MAG[1] + P[15][17]*SH_MAG[4] - P[15][1]*SK_MX[2] + P[15][2]*SK_MX[1] + P[15][18]*SK_MX[3]);
        // zero Kalman gains to inhibit wind state estimation
        if (!inhibitWindStates) {
            Kfusion[22] = SK_MX[0]*(P[22][19] + P[22][16]*SH_MAG[1] + P[22][17]*SH_MAG[4] - P[22][1]*SK_MX[2] + P[22][2]*SK_MX[1] + P[22][18]*SK_MX[3]);
            Kfusion[23] = SK_MX[0]*(P[23][19] + P[23][16]*SH_MAG[1] + P[23][17]*SH_MAG[4] - P[23][1]*SK_MX[2] + P[23][2]*SK_MX[1] + P[23][18]*SK_MX[3]);
        } else {
            Kfusion[22] = 0.0f;
            Kfusion[23] = 0.0f;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][16]*SH_MAG[1] + P[16][17]*SH_MAG[4] - P[16][1]*SK_MX[2] + P[16][2]*SK_MX[1] + P[16][18]*SK_MX[3]);
            Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][16]*SH_MAG[1] + P[17][17]*SH_MAG[4] - P[17][1]*SK_MX[2] + P[17][2]*SK_MX[1] + P[17][18]*SK_MX[3]);
            Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][16]*SH_MAG[1] + P[18][17]*SH_MAG[4] - P[18][1]*SK_MX[2] + P[18][2]*SK_MX[1] + P[18][18]*SK_MX[3]);
            Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][16]*SH_MAG[1] + P[19][17]*SH_MAG[4] - P[19][1]*SK_MX[2] + P[19][2]*SK_MX[1] + P[19][18]*SK_MX[3]);
            Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][16]*SH_MAG[1] + P[20][17]*SH_MAG[4] - P[20][1]*SK_MX[2] + P[20][2]*SK_MX[1] + P[20][18]*SK_MX[3]);
            Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][16]*SH_MAG[1] + P[21][17]*SH_MAG[4] - P[21][1]*SK_MX[2] + P[21][2]*SK_MX[1] + P[21][18]*SK_MX[3]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // reset the observation index to 0 (we start by fusing the X measurement)
        obsIndex = 0;

        // set flags to indicate to other processes that fusion has been performed and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = true;
    }
    else if (obsIndex == 1) // we are now fusing the Y measurement
    {
        // calculate observation jacobians
        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
        H_MAG[0] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
        H_MAG[2] = - magE*SH_MAG[4] - magD*SH_MAG[7] - magN*SH_MAG[1];
        H_MAG[16] = 2*q1*q2 - SH_MAG[8];
        H_MAG[17] = SH_MAG[0];
        H_MAG[18] = SH_MAG[3];
        H_MAG[20] = 1;

        // calculate Kalman gain
        varInnovMag[1] = (P[20][20] + R_MAG + P[0][20]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][20]*SH_MAG[0] + P[18][20]*SH_MAG[3] - (SH_MAG[8] - 2*q1*q2)*(P[20][16] + P[0][16]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][16]*SH_MAG[0] + P[18][16]*SH_MAG[3] - P[2][16]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][16]*(SH_MAG[8] - 2*q1*q2)) - P[2][20]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) + (magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5])*(P[20][0] + P[0][0]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][0]*SH_MAG[0] + P[18][0]*SH_MAG[3] - P[2][0]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][0]*(SH_MAG[8] - 2*q1*q2)) + SH_MAG[0]*(P[20][17] + P[0][17]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][17]*SH_MAG[0] + P[18][17]*SH_MAG[3] - P[2][17]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][17]*(SH_MAG[8] - 2*q1*q2)) + SH_MAG[3]*(P[20][18] + P[0][18]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][18]*SH_MAG[0] + P[18][18]*SH_MAG[3] - P[2][18]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][18]*(SH_MAG[8] - 2*q1*q2)) - P[16][20]*(SH_MAG[8] - 2*q1*q2) - (magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1])*(P[20][2] + P[0][2]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][2]*SH_MAG[0] + P[18][2]*SH_MAG[3] - P[2][2]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][2]*(SH_MAG[8] - 2*q1*q2)));
        if (varInnovMag[1] >= R_MAG) {
            SK_MY[0] = 1.0f / varInnovMag[1];
            faultStatus.bad_ymag = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            obsIndex = 2;
            faultStatus.bad_ymag = true;
            return;
        }
        SK_MY[1] = magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1];
        SK_MY[2] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
        SK_MY[3] = SH_MAG[8] - 2*q1*q2;
        Kfusion[0] = SK_MY[0]*(P[0][20] + P[0][17]*SH_MAG[0] + P[0][18]*SH_MAG[3] + P[0][0]*SK_MY[2] - P[0][2]*SK_MY[1] - P[0][16]*SK_MY[3]);
        Kfusion[1] = SK_MY[0]*(P[1][20] + P[1][17]*SH_MAG[0] + P[1][18]*SH_MAG[3] + P[1][0]*SK_MY[2] - P[1][2]*SK_MY[1] - P[1][16]*SK_MY[3]);
        Kfusion[2] = SK_MY[0]*(P[2][20] + P[2][17]*SH_MAG[0] + P[2][18]*SH_MAG[3] + P[2][0]*SK_MY[2] - P[2][2]*SK_MY[1] - P[2][16]*SK_MY[3]);
        Kfusion[3] = SK_MY[0]*(P[3][20] + P[3][17]*SH_MAG[0] + P[3][18]*SH_MAG[3] + P[3][0]*SK_MY[2] - P[3][2]*SK_MY[1] - P[3][16]*SK_MY[3]);
        Kfusion[4] = SK_MY[0]*(P[4][20] + P[4][17]*SH_MAG[0] + P[4][18]*SH_MAG[3] + P[4][0]*SK_MY[2] - P[4][2]*SK_MY[1] - P[4][16]*SK_MY[3]);
        Kfusion[5] = SK_MY[0]*(P[5][20] + P[5][17]*SH_MAG[0] + P[5][18]*SH_MAG[3] + P[5][0]*SK_MY[2] - P[5][2]*SK_MY[1] - P[5][16]*SK_MY[3]);
        Kfusion[6] = SK_MY[0]*(P[6][20] + P[6][17]*SH_MAG[0] + P[6][18]*SH_MAG[3] + P[6][0]*SK_MY[2] - P[6][2]*SK_MY[1] - P[6][16]*SK_MY[3]);
        Kfusion[7] = SK_MY[0]*(P[7][20] + P[7][17]*SH_MAG[0] + P[7][18]*SH_MAG[3] + P[7][0]*SK_MY[2] - P[7][2]*SK_MY[1] - P[7][16]*SK_MY[3]);
        Kfusion[8] = SK_MY[0]*(P[8][20] + P[8][17]*SH_MAG[0] + P[8][18]*SH_MAG[3] + P[8][0]*SK_MY[2] - P[8][2]*SK_MY[1] - P[8][16]*SK_MY[3]);
        Kfusion[9] = SK_MY[0]*(P[9][20] + P[9][17]*SH_MAG[0] + P[9][18]*SH_MAG[3] + P[9][0]*SK_MY[2] - P[9][2]*SK_MY[1] - P[9][16]*SK_MY[3]);
        Kfusion[10] = SK_MY[0]*(P[10][20] + P[10][17]*SH_MAG[0] + P[10][18]*SH_MAG[3] + P[10][0]*SK_MY[2] - P[10][2]*SK_MY[1] - P[10][16]*SK_MY[3]);
        Kfusion[11] = SK_MY[0]*(P[11][20] + P[11][17]*SH_MAG[0] + P[11][18]*SH_MAG[3] + P[11][0]*SK_MY[2] - P[11][2]*SK_MY[1] - P[11][16]*SK_MY[3]);
        Kfusion[12] = SK_MY[0]*(P[12][20] + P[12][17]*SH_MAG[0] + P[12][18]*SH_MAG[3] + P[12][0]*SK_MY[2] - P[12][2]*SK_MY[1] - P[12][16]*SK_MY[3]);
        Kfusion[13] = SK_MY[0]*(P[13][20] + P[13][17]*SH_MAG[0] + P[13][18]*SH_MAG[3] + P[13][0]*SK_MY[2] - P[13][2]*SK_MY[1] - P[13][16]*SK_MY[3]);
        Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][17]*SH_MAG[0] + P[14][18]*SH_MAG[3] + P[14][0]*SK_MY[2] - P[14][2]*SK_MY[1] - P[14][16]*SK_MY[3]);
        // this term has been zeroed to improve stability of the Z accel bias
        Kfusion[15] = 0.0f;//SK_MY[0]*(P[15][20] + P[15][17]*SH_MAG[0] + P[15][18]*SH_MAG[3] + P[15][0]*SK_MY[2] - P[15][2]*SK_MY[1] - P[15][16]*SK_MY[3]);
        // zero Kalman gains to inhibit wind state estimation
        if (!inhibitWindStates) {
            Kfusion[22] = SK_MY[0]*(P[22][20] + P[22][17]*SH_MAG[0] + P[22][18]*SH_MAG[3] + P[22][0]*SK_MY[2] - P[22][2]*SK_MY[1] - P[22][16]*SK_MY[3]);
            Kfusion[23] = SK_MY[0]*(P[23][20] + P[23][17]*SH_MAG[0] + P[23][18]*SH_MAG[3] + P[23][0]*SK_MY[2] - P[23][2]*SK_MY[1] - P[23][16]*SK_MY[3]);
        } else {
            Kfusion[22] = 0.0f;
            Kfusion[23] = 0.0f;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][17]*SH_MAG[0] + P[16][18]*SH_MAG[3] + P[16][0]*SK_MY[2] - P[16][2]*SK_MY[1] - P[16][16]*SK_MY[3]);
            Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][17]*SH_MAG[0] + P[17][18]*SH_MAG[3] + P[17][0]*SK_MY[2] - P[17][2]*SK_MY[1] - P[17][16]*SK_MY[3]);
            Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][17]*SH_MAG[0] + P[18][18]*SH_MAG[3] + P[18][0]*SK_MY[2] - P[18][2]*SK_MY[1] - P[18][16]*SK_MY[3]);
            Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][17]*SH_MAG[0] + P[19][18]*SH_MAG[3] + P[19][0]*SK_MY[2] - P[19][2]*SK_MY[1] - P[19][16]*SK_MY[3]);
            Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][17]*SH_MAG[0] + P[20][18]*SH_MAG[3] + P[20][0]*SK_MY[2] - P[20][2]*SK_MY[1] - P[20][16]*SK_MY[3]);
            Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][17]*SH_MAG[0] + P[21][18]*SH_MAG[3] + P[21][0]*SK_MY[2] - P[21][2]*SK_MY[1] - P[21][16]*SK_MY[3]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // set flags to indicate to other processes that fusion has been performede and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = true;
    }
    else if (obsIndex == 2) // we are now fusing the Z measurement
    {
        // calculate observation jacobians
        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
        H_MAG[0] = magN*(SH_MAG[8] - 2*q1*q2) - magD*SH_MAG[3] - magE*SH_MAG[0];
        H_MAG[1] = magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3);
        H_MAG[16] = SH_MAG[5];
        H_MAG[17] = 2*q2*q3 - 2*q0*q1;
        H_MAG[18] = SH_MAG[2];
        H_MAG[21] = 1;

        // calculate Kalman gain
        varInnovMag[2] = (P[21][21] + R_MAG + P[16][21]*SH_MAG[5] + P[18][21]*SH_MAG[2] + SH_MAG[5]*(P[21][16] + P[16][16]*SH_MAG[5] + P[18][16]*SH_MAG[2] - P[0][16]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2)) - P[17][16]*(2*q0*q1 - 2*q2*q3) + P[1][16]*(magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3))) + SH_MAG[2]*(P[21][18] + P[16][18]*SH_MAG[5] + P[18][18]*SH_MAG[2] - P[0][18]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2)) - P[17][18]*(2*q0*q1 - 2*q2*q3) + P[1][18]*(magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3))) - P[0][21]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2)) - (magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2))*(P[21][0] + P[16][0]*SH_MAG[5] + P[18][0]*SH_MAG[2] - P[0][0]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2)) - P[17][0]*(2*q0*q1 - 2*q2*q3) + P[1][0]*(magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3))) - P[17][21]*(2*q0*q1 - 2*q2*q3) - (2*q0*q1 - 2*q2*q3)*(P[21][17] + P[16][17]*SH_MAG[5] + P[18][17]*SH_MAG[2] - P[0][17]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2)) - P[17][17]*(2*q0*q1 - 2*q2*q3) + P[1][17]*(magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3))) + P[1][21]*(magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3)) + (magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3))*(P[21][1] + P[16][1]*SH_MAG[5] + P[18][1]*SH_MAG[2] - P[0][1]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2)) - P[17][1]*(2*q0*q1 - 2*q2*q3) + P[1][1]*(magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3))));
        if (varInnovMag[2] >= R_MAG) {
            SK_MZ[0] = 1.0f / varInnovMag[2];
            faultStatus.bad_zmag = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            obsIndex = 3;
            faultStatus.bad_zmag = true;
            return;
        }
        SK_MZ[1] = magE*SH_MAG[4] + magN*SH_MAG[1] - magD*(2*q0*q2 - 2*q1*q3);
        SK_MZ[2] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2*q1*q2);
        SK_MZ[3] = 2*q0*q1 - 2*q2*q3;
        Kfusion[0] = SK_MZ[0]*(P[0][21] + P[0][18]*SH_MAG[2] + P[0][16]*SH_MAG[5] - P[0][0]*SK_MZ[2] + P[0][1]*SK_MZ[1] - P[0][17]*SK_MZ[3]);
        Kfusion[1] = SK_MZ[0]*(P[1][21] + P[1][18]*SH_MAG[2] + P[1][16]*SH_MAG[5] - P[1][0]*SK_MZ[2] + P[1][1]*SK_MZ[1] - P[1][17]*SK_MZ[3]);
        Kfusion[2] = SK_MZ[0]*(P[2][21] + P[2][18]*SH_MAG[2] + P[2][16]*SH_MAG[5] - P[2][0]*SK_MZ[2] + P[2][1]*SK_MZ[1] - P[2][17]*SK_MZ[3]);
        Kfusion[3] = SK_MZ[0]*(P[3][21] + P[3][18]*SH_MAG[2] + P[3][16]*SH_MAG[5] - P[3][0]*SK_MZ[2] + P[3][1]*SK_MZ[1] - P[3][17]*SK_MZ[3]);
        Kfusion[4] = SK_MZ[0]*(P[4][21] + P[4][18]*SH_MAG[2] + P[4][16]*SH_MAG[5] - P[4][0]*SK_MZ[2] + P[4][1]*SK_MZ[1] - P[4][17]*SK_MZ[3]);
        Kfusion[5] = SK_MZ[0]*(P[5][21] + P[5][18]*SH_MAG[2] + P[5][16]*SH_MAG[5] - P[5][0]*SK_MZ[2] + P[5][1]*SK_MZ[1] - P[5][17]*SK_MZ[3]);
        Kfusion[6] = SK_MZ[0]*(P[6][21] + P[6][18]*SH_MAG[2] + P[6][16]*SH_MAG[5] - P[6][0]*SK_MZ[2] + P[6][1]*SK_MZ[1] - P[6][17]*SK_MZ[3]);
        Kfusion[7] = SK_MZ[0]*(P[7][21] + P[7][18]*SH_MAG[2] + P[7][16]*SH_MAG[5] - P[7][0]*SK_MZ[2] + P[7][1]*SK_MZ[1] - P[7][17]*SK_MZ[3]);
        Kfusion[8] = SK_MZ[0]*(P[8][21] + P[8][18]*SH_MAG[2] + P[8][16]*SH_MAG[5] - P[8][0]*SK_MZ[2] + P[8][1]*SK_MZ[1] - P[8][17]*SK_MZ[3]);
        Kfusion[9] = SK_MZ[0]*(P[9][21] + P[9][18]*SH_MAG[2] + P[9][16]*SH_MAG[5] - P[9][0]*SK_MZ[2] + P[9][1]*SK_MZ[1] - P[9][17]*SK_MZ[3]);
        Kfusion[10] = SK_MZ[0]*(P[10][21] + P[10][18]*SH_MAG[2] + P[10][16]*SH_MAG[5] - P[10][0]*SK_MZ[2] + P[10][1]*SK_MZ[1] - P[10][17]*SK_MZ[3]);
        Kfusion[11] = SK_MZ[0]*(P[11][21] + P[11][18]*SH_MAG[2] + P[11][16]*SH_MAG[5] - P[11][0]*SK_MZ[2] + P[11][1]*SK_MZ[1] - P[11][17]*SK_MZ[3]);
        Kfusion[12] = SK_MZ[0]*(P[12][21] + P[12][18]*SH_MAG[2] + P[12][16]*SH_MAG[5] - P[12][0]*SK_MZ[2] + P[12][1]*SK_MZ[1] - P[12][17]*SK_MZ[3]);
        Kfusion[13] = SK_MZ[0]*(P[13][21] + P[13][18]*SH_MAG[2] + P[13][16]*SH_MAG[5] - P[13][0]*SK_MZ[2] + P[13][1]*SK_MZ[1] - P[13][17]*SK_MZ[3]);
        Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][18]*SH_MAG[2] + P[14][16]*SH_MAG[5] - P[14][0]*SK_MZ[2] + P[14][1]*SK_MZ[1] - P[14][17]*SK_MZ[3]);
        // this term has been zeroed to improve stability of the Z accel bias
        Kfusion[15] = 0.0f;//SK_MZ[0]*(P[15][21] + P[15][18]*SH_MAG[2] + P[15][16]*SH_MAG[5] - P[15][0]*SK_MZ[2] + P[15][1]*SK_MZ[1] - P[15][17]*SK_MZ[3]);
        // zero Kalman gains to inhibit wind state estimation
        if (!inhibitWindStates) {
            Kfusion[22] = SK_MZ[0]*(P[22][21] + P[22][18]*SH_MAG[2] + P[22][16]*SH_MAG[5] - P[22][0]*SK_MZ[2] + P[22][1]*SK_MZ[1] - P[22][17]*SK_MZ[3]);
            Kfusion[23] = SK_MZ[0]*(P[23][21] + P[23][18]*SH_MAG[2] + P[23][16]*SH_MAG[5] - P[23][0]*SK_MZ[2] + P[23][1]*SK_MZ[1] - P[23][17]*SK_MZ[3]);
        } else {
            Kfusion[22] = 0.0f;
            Kfusion[23] = 0.0f;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][18]*SH_MAG[2] + P[16][16]*SH_MAG[5] - P[16][0]*SK_MZ[2] + P[16][1]*SK_MZ[1] - P[16][17]*SK_MZ[3]);
            Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][18]*SH_MAG[2] + P[17][16]*SH_MAG[5] - P[17][0]*SK_MZ[2] + P[17][1]*SK_MZ[1] - P[17][17]*SK_MZ[3]);
            Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][18]*SH_MAG[2] + P[18][16]*SH_MAG[5] - P[18][0]*SK_MZ[2] + P[18][1]*SK_MZ[1] - P[18][17]*SK_MZ[3]);
            Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][18]*SH_MAG[2] + P[19][16]*SH_MAG[5] - P[19][0]*SK_MZ[2] + P[19][1]*SK_MZ[1] - P[19][17]*SK_MZ[3]);
            Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][18]*SH_MAG[2] + P[20][16]*SH_MAG[5] - P[20][0]*SK_MZ[2] + P[20][1]*SK_MZ[1] - P[20][17]*SK_MZ[3]);
            Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][18]*SH_MAG[2] + P[21][16]*SH_MAG[5] - P[21][0]*SK_MZ[2] + P[21][1]*SK_MZ[1] - P[21][17]*SK_MZ[3]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // set flags to indicate to other processes that fusion has been performede and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = false;
    }
    // calculate the measurement innovation
    innovMag[obsIndex] = MagPred[obsIndex] - magDataDelayed.mag[obsIndex];
    // calculate the innovation test ratio
    magTestRatio[obsIndex] = sq(innovMag[obsIndex]) / (sq(_magInnovGate) * varInnovMag[obsIndex]);
    // check the last values from all components and set magnetometer health accordingly
    magHealth = (magTestRatio[0] < 1.0f && magTestRatio[1] < 1.0f && magTestRatio[2] < 1.0f);
    // Don't fuse unless all componenets pass. The exception is if the bad health has timed out and we are not a fly forward vehicle
    // In this case we might as well try using the magnetometer, but with a reduced weighting
    if (magHealth || ((magTestRatio[obsIndex] < 1.0f) && !assume_zero_sideslip() && magTimeout)) {

        // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
        stateStruct.angErr.zero();

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            // If we are forced to use a bad compass in flight, we reduce the weighting by a factor of 4
            if (!magHealth && !constPosMode) {
                Kfusion[j] *= 0.25f;
            }
            // If in the air and there is no other form of heading reference or we are yawing rapidly which creates larger inertial yaw errors,
            // we strengthen the magnetometer attitude correction
            if (filterArmed && (constPosMode || highYawRate) && j <= 3) {
                Kfusion[j] *= 4.0f;
            }
            statesArray[j] = statesArray[j] - Kfusion[j] * innovMag[obsIndex];
        }

        // the first 3 states represent the angular misalignment vector. This is
        // is used to correct the estimated quaternion on the current time step
        correctQuatStates(stateStruct.angErr);

        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        for (uint8_t i = 0; i<=stateIndexLim; i++) {
            for (uint8_t j = 0; j<=2; j++) {
                KH[i][j] = Kfusion[i] * H_MAG[j];
            }
            for (uint8_t j = 3; j<=15; j++) {
                KH[i][j] = 0.0f;
            }
            for (uint8_t j = 16; j<=21; j++) {
                if (!inhibitMagStates) {
                    KH[i][j] = Kfusion[i] * H_MAG[j];
                } else {
                    KH[i][j] = 0.0f;
                }
            }
            for (uint8_t j = 22; j<=23; j++) {
                KH[i][j] = 0.0f;
            }
        }
        for (uint8_t i = 0; i<=stateIndexLim; i++) {
            for (uint8_t j = 0; j<=stateIndexLim; j++) {
                KHP[i][j] = 0;
                for (uint8_t k = 0; k<=2; k++) {
                    KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                }
                if (!inhibitMagStates) {
                    for (uint8_t k = 16; k<=21; k++) {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                }
            }
        }
        for (uint8_t i = 0; i<=stateIndexLim; i++) {
            for (uint8_t j = 0; j<=stateIndexLim; j++) {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optiocal flow rates and range finder measurements
*/
void NavEKF::EstimateTerrainOffset()
{
    // start performance timer
    perf_begin(_perf_OpticalFlowEKF);

    // constrain height above ground to be above range measured on ground
    float heightAboveGndEst = max((terrainState - stateStruct.position.z), rngOnGnd);

    // calculate a predicted LOS rate squared
    float velHorizSq = sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y);
    float losRateSq = velHorizSq / sq(heightAboveGndEst);

    // don't update terrain offset state if there is no range finder and not generating enough LOS rate, or without GPS, as it is poorly observable
    if (!fuseRngData && (gpsNotAvailable || PV_AidingMode == AID_RELATIVE || velHorizSq < 25.0f || losRateSq < 0.01f || onGround)) {
        inhibitGndState = true;
    } else {
        inhibitGndState = false;
        // record the time we last updated the terrain offset state
        gndHgtValidTime_ms = imuSampleTime_ms;

        // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
        // limit distance to prevent intialisation afer bad gps causing bad numerical conditioning
        float distanceTravelledSq = sq(stateStruct.position[0] - prevPosN) + sq(stateStruct.position[1] - prevPosE);
        distanceTravelledSq = min(distanceTravelledSq, 100.0f);
        prevPosN = stateStruct.position[0];
        prevPosE = stateStruct.position[1];

        // in addition to a terrain gradient error model, we also have a time based error growth that is scaled using the gradient parameter
        float timeLapsed = min(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f);
        float Pincrement = (distanceTravelledSq * sq(0.01f*float(_gndGradientSigma))) + sq(float(_gndGradientSigma) * timeLapsed);
        Popt += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (fuseRngData) {
            // predict range
            float predRngMeas = max((terrainState - stateStruct.position[2]),rngOnGnd) / Tnb_flow.c.z;

            // Copy required states to local variable names
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            float R_RNG = 0.5f;

            // calculate Kalman gain
            float SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            float K_RNG = Popt/(SK_RNG*(R_RNG + Popt/sq(SK_RNG)));

            // Calculate the innovation variance for data logging
            varInnovRng = (R_RNG + Popt/sq(SK_RNG));

            // constrain terrain height to be below the vehicle
            terrainState = max(terrainState, stateStruct.position[2] + rngOnGnd);

            // Calculate the measurement innovation
            innovRng = predRngMeas - rngMea;

            // calculate the innovation consistency test ratio
            auxRngTestRatio = sq(innovRng) / (sq(_rngInnovGate) * varInnovRng);

            // Check the innovation for consistency and don't fuse if > 5Sigma
            if ((sq(innovRng)*SK_RNG) < 25.0f)
            {
                // correct the state
                terrainState -= K_RNG * innovRng;

                // constrain the state
                terrainState = max(terrainState, stateStruct.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - sq(Popt)/(SK_RNG*(R_RNG + Popt/sq(SK_RNG))*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));

                // prevent the state variance from becoming negative
                Popt = max(Popt,0.0f);

            }
        }

        if (fuseOptFlowData) {

            Vector3f vel; // velocity of sensor relative to ground in NED axes
            Vector3f relVelSensor; // velocity of sensor relative to ground in sensor axes
            float losPred; // predicted optical flow angular rate measurement
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time
            float K_OPT;
            float H_OPT;

            // Correct velocities for GPS glitch recovery offset
            vel.x          = stateStruct.velocity[0] - gpsVelGlitchOffset.x;
            vel.y          = stateStruct.velocity[1] - gpsVelGlitchOffset.y;
            vel.z          = stateStruct.velocity[2];

            // predict range to centre of image
            float flowRngPred = max((terrainState - stateStruct.position[2]),rngOnGnd) / Tnb_flow.c.z;

            // constrain terrain height to be below the vehicle
            terrainState = max(terrainState, stateStruct.position[2] + rngOnGnd);

            // calculate relative velocity in sensor frame
            relVelSensor = Tnb_flow*vel;

            // divide velocity by range, subtract body rates and apply scale factor to
            // get predicted sensed angular optical rates relative to X and Y sensor axes
            losPred =   relVelSensor.length()/flowRngPred;

            // calculate innovations
            auxFlowObsInnov = losPred - sqrtf(sq(flowRadXYcomp[0]) + sq(flowRadXYcomp[1]));

            // calculate observation jacobian
            float t3 = sq(q0);
            float t4 = sq(q1);
            float t5 = sq(q2);
            float t6 = sq(q3);
            float t10 = q0*q3*2.0f;
            float t11 = q1*q2*2.0f;
            float t14 = t3+t4-t5-t6;
            float t15 = t14*vel.x;
            float t16 = t10+t11;
            float t17 = t16*vel.y;
            float t18 = q0*q2*2.0f;
            float t19 = q1*q3*2.0f;
            float t20 = t18-t19;
            float t21 = t20*vel.z;
            float t2 = t15+t17-t21;
            float t7 = t3-t4-t5+t6;
            float t8 = stateStruct.position[2]-terrainState;
            float t9 = 1.0f/sq(t8);
            float t24 = t3-t4+t5-t6;
            float t25 = t24*vel.y;
            float t26 = t10-t11;
            float t27 = t26*vel.x;
            float t28 = q0*q1*2.0f;
            float t29 = q2*q3*2.0f;
            float t30 = t28+t29;
            float t31 = t30*vel.z;
            float t12 = t25-t27+t31;
            float t13 = sq(t7);
            float t22 = sq(t2);
            float t23 = 1.0f/(t8*t8*t8);
            float t32 = sq(t12);
            H_OPT = 0.5f*(t13*t22*t23*2.0f+t13*t23*t32*2.0f)/sqrtf(t9*t13*t22+t9*t13*t32);

            // calculate innovation variances
            auxFlowObsInnovVar = H_OPT*Popt*H_OPT + R_LOS;

            // calculate Kalman gain
            K_OPT = Popt*H_OPT/auxFlowObsInnovVar;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio = sq(auxFlowObsInnov) / (sq(_flowInnovGate) * auxFlowObsInnovVar);

            // don't fuse if optical flow data is outside valid range
            if (max(flowRadXY[0],flowRadXY[1]) < _maxFlowRate) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov;

                // constrain the state
                terrainState = max(terrainState, stateStruct.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming negative
                Popt = max(Popt,0.0f);
            }
        }
    }

    // stop the performance timer
    perf_end(_perf_OpticalFlowEKF);
}

// zero specified range of rows in the state covariance matrix
void NavEKF::zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0])*24);
    }
}

// zero specified range of columns in the state covariance matrix
void NavEKF::zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=23; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));
    }
}

// store output data in the FIFO
void NavEKF::StoreOutput()
{
    storedOutput[fifoIndexNow] = outputDataNew;
}

// reset the output data to the current EKF state
void NavEKF::StoreOutputReset()
{
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;
    // write current measurement to entire table
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i] = outputDataNew;
    }
    outputDataDelayed = outputDataNew;
}

// Reset the stored output quaternion history to current EKF state
void NavEKF::StoreQuatReset()
{
    outputDataNew.quat = stateStruct.quat;
    // write current measurement to entire table
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i].quat = outputDataNew.quat;
    }
    outputDataDelayed.quat = outputDataNew.quat;
}

// recall output data from the FIFO
void NavEKF::RecallOutput()
{
    outputDataDelayed = storedOutput[fifoIndexDelayed];
}

// store imu in the FIFO
void NavEKF::StoreIMU()
{
    fifoIndexDelayed = fifoIndexNow;
    fifoIndexNow = fifoIndexNow + 1;
    if (fifoIndexNow >= IMU_BUFFER_LENGTH) {
        fifoIndexNow = 0;
    }
    storedIMU[fifoIndexNow] = imuDataNew;
}

// reset the stored imu history and store the current value
void NavEKF::StoreIMU_reset()
{
    // write current measurement to entire table
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedIMU[i] = imuDataNew;
    }
    imuDataDelayed = imuDataNew;
    fifoIndexDelayed = fifoIndexNow+1;
    if (fifoIndexDelayed >= IMU_BUFFER_LENGTH) {
        fifoIndexDelayed = 0;
    }
}

// recall IMU data from the FIFO
void NavEKF::RecallIMU()
{
    imuDataDelayed = storedIMU[fifoIndexDelayed];
}

// store baro in a history array
void NavEKF::StoreBaro()
{
        if (baroStoreIndex >= OBS_BUFFER_LENGTH) {
            baroStoreIndex = 0;
        }
        storedBaro[baroStoreIndex] = baroDataNew;
        baroStoreIndex += 1;
}

// return newest un-used baro data that has fallen behind the fusion time horizon
// if no un-used data is available behind the fusion horizon, return false
bool NavEKF::RecallBaro()
{
    baro_elements dataTemp;
    baro_elements dataTempZero;
    dataTempZero.time_ms = 0;
    uint32_t temp_ms = 0;
    for (uint8_t i=0; i<OBS_BUFFER_LENGTH; i++) {
        dataTemp = storedBaro[i];
        // find a measurement older than the fusion time horizon that we haven't checked before
        if (dataTemp.time_ms != 0 && dataTemp.time_ms <= imuDataDelayed.time_ms) {
            // zero the time stamp so we won't use it again
            storedBaro[i]=dataTempZero;
            // Find the most recent non-stale measurement that meets the time horizon criteria
            if (((imuDataDelayed.time_ms - dataTemp.time_ms) < 500) && dataTemp.time_ms > temp_ms) {
                baroDataDelayed = dataTemp;
                temp_ms = dataTemp.time_ms;
            }
        }
    }
    if (temp_ms != 0) {
        return true;
    } else {
        return false;
    }
}

// store magnetometer data in a history array
void NavEKF::StoreMag()
{
        if (magStoreIndex >= OBS_BUFFER_LENGTH) {
            magStoreIndex = 0;
        }
        storedMag[magStoreIndex] = magDataNew;
        magStoreIndex += 1;
}

// return newest un-used magnetometer data that has fallen behind the fusion time horizon
// if no un-used data is available behind the fusion horizon, return false
bool NavEKF::RecallMag()
{
    mag_elements dataTemp;
    mag_elements dataTempZero;
    dataTempZero.time_ms = 0;
    uint32_t temp_ms = 0;
    for (uint8_t i=0; i<OBS_BUFFER_LENGTH; i++) {
        dataTemp = storedMag[i];
        // find a measurement older than the fusion time horizon that we haven't checked before
        if (dataTemp.time_ms != 0 && dataTemp.time_ms <= imuDataDelayed.time_ms) {
            // zero the time stamp so we won't use it again
            storedMag[i]=dataTempZero;
            // Find the most recent non-stale measurement that meets the time horizon criteria
            if (((imuDataDelayed.time_ms - dataTemp.time_ms) < 500) && dataTemp.time_ms > temp_ms) {
                magDataDelayed = dataTemp;
                temp_ms = dataTemp.time_ms;
            }
        }
    }
    if (temp_ms != 0) {
        return true;
    } else {
        return false;
    }
}

// store GPS data in a history array
void NavEKF::StoreGPS()
{
        if (gpsStoreIndex >= OBS_BUFFER_LENGTH) {
            gpsStoreIndex = 0;
        }
        storedGPS[gpsStoreIndex] = gpsDataNew;
        gpsStoreIndex += 1;
}

// return newest un-used GPS data that has fallen behind the fusion time horizon
// if no un-used data is available behind the fusion horizon, return false
bool NavEKF::RecallGPS()
{
    gps_elements dataTemp;
    gps_elements dataTempZero;
    dataTempZero.time_ms = 0;
    uint32_t temp_ms = 0;
    for (uint8_t i=0; i<OBS_BUFFER_LENGTH; i++) {
        dataTemp = storedGPS[i];
        // find a measurement older than the fusion time horizon that we haven't checked before
        if (dataTemp.time_ms != 0 && dataTemp.time_ms <= imuDataDelayed.time_ms) {
            // zero the time stamp so we won't use it again
            storedGPS[i]=dataTempZero;
            // Find the most recent non-stale measurement that meets the time horizon criteria
            if (((imuDataDelayed.time_ms - dataTemp.time_ms) < 500) && dataTemp.time_ms > temp_ms) {
                gpsDataDelayed = dataTemp;
                temp_ms = dataTemp.time_ms;
            }
        }
    }
    if (temp_ms != 0) {
        return true;
    } else {
        return false;
    }
}

// calculate nav to body quaternions from body to nav rotation matrix
void NavEKF::quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const
{
    // Calculate the body to nav cosine matrix
    quat.rotation_matrix(Tbn);
}

// return the Euler roll, pitch and yaw angle in radians
void NavEKF::getEulerAngles(Vector3f &euler) const
{
    outputDataNew.quat.to_euler(euler.x, euler.y, euler.z);
    euler = euler - _ahrs->get_trim();
}

// This returns the specific forces in the NED frame
void NavEKF::getAccelNED(Vector3f &accelNED) const {
    accelNED = velDotNED;
    accelNED.z -= GRAVITY_MSS;
}

// return NED velocity in m/s
//
void NavEKF::getVelNED(Vector3f &vel) const
{
    vel = outputDataNew.velocity;
}

// Return the last calculated NED position relative to the reference point (m).
// if a calculated solution is not available, use the best available data and return false
bool NavEKF::getPosNED(Vector3f &pos) const
{
    // The EKF always has a height estimate regardless of mode of operation
    pos.z = outputDataNew.position.z;
    // There are three modes of operation, absolute position (GPS fusion), relative position (optical flow fusion) and constant position (no position estimate available)
    nav_filter_status status;
    getFilterStatus(status);
    if (status.flags.horiz_pos_abs || status.flags.horiz_pos_rel) {
        // This is the normal mode of operation where we can use the EKF position states
        pos.x = outputDataNew.position.x;
        pos.y = outputDataNew.position.y;
        return true;
    } else {
        // In constant position mode the EKF position states are at the origin, so we cannot use them as a position estimate
        if(validOrigin) {
            if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
                // If the origin has been set and we have GPS, then return the GPS position relative to the origin
                const struct Location &gpsloc = _ahrs->get_gps().location();
                Vector2f tempPosNE = location_diff(EKF_origin, gpsloc);
                pos.x = tempPosNE.x;
                pos.y = tempPosNE.y;
                return false;
            } else {
                // If no GPS fix is available, all we can do is provide the last known position
                pos.x = outputDataNew.position.x + lastKnownPositionNE.x;
                pos.y = outputDataNew.position.y + lastKnownPositionNE.y;
                return false;
            }
        } else {
            // If the origin has not been set, then we have no means of providing a relative position
            pos.x = 0.0f;
            pos.y = 0.0f;
            return false;
        }
    }
    return false;
}

// return body axis gyro bias estimates in rad/sec
void NavEKF::getGyroBias(Vector3f &gyroBias) const
{
    if (dtIMUavg < 1e-6f) {
        gyroBias.zero();
        return;
    }
    gyroBias = stateStruct.gyro_bias / dtIMUavg;
}

// return body axis gyro scale factor estimates
void NavEKF::getGyroScale(Vector3f &gyroScale) const
{
    gyroScale.x = 1.0f/stateStruct.gyro_scale.x - 1.0f;
    gyroScale.y = 1.0f/stateStruct.gyro_scale.y - 1.0f;
    gyroScale.z = 1.0f/stateStruct.gyro_scale.z - 1.0f;
}

// return tilt error convergence metric
void NavEKF::getTiltError(float &ang) const
{
    ang = tiltErrFilt;
}

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
void NavEKF::resetGyroBias(void)
{
    stateStruct.gyro_bias.zero();
    zeroRows(P,9,11);
    zeroCols(P,9,11);
    P[9][9] = sq(radians(InitialGyroBiasUncertainty() * dtIMUavg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
}

// Reset the baro so that it reads zero at the current height
// Reset the EKF height to zero
// Adjust the EKf origin height so that the EKF height + origin height is the same as before
// Return true if the height datum reset has been performed
// If using a range finder for height do not reset and return false
bool NavEKF::resetHeightDatum(void)
{
    // if we are using a range finder for height, return false
    if (_altSource == 1) {
        return false;
    }
    // record the old height estimate
    float oldHgt = -stateStruct.position.z;
    // reset the barometer so that it reads zero at the current height
    _baro.update_calibration();
    // reset the height state
    stateStruct.position.z = 0.0f;
    // adjust the height of the EKF origin so that the origin plus baro height before and afer the reset is the same
    if (validOrigin) {
        EKF_origin.alt += oldHgt*100;
    }
    return true;
}

// Commands the EKF to not use GPS.
// This command must be sent prior to arming
// This command is forgotten by the EKF each time the vehicle disarms
// Returns 0 if command rejected
// Returns 1 if attitude, vertical velocity and vertical position will be provided
// Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
uint8_t NavEKF::setInhibitGPS(void)
{
    if(!filterArmed) {
        return 0;
    }
    if (optFlowDataPresent()) {
        _fusionModeGPS = 3;
        return 2;
    } else {
        return 1;
    }
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (PV_AidingMode == AID_RELATIVE) {
        // allow 1.0 rad/sec margin for angular motion
        ekfGndSpdLimit = max((_maxFlowRate - 1.0f), 0.0f) * max((terrainState - stateStruct.position[2]), rngOnGnd);
        // use standard gains up to 5.0 metres height and reduce above that
        ekfNavVelGainScaler = 4.0f / max((terrainState - stateStruct.position[2]),4.0f);
    } else {
        ekfGndSpdLimit = 400.0f; //return 80% of max filter speed
        ekfNavVelGainScaler = 1.0f;
    }
}

// return weighting of first IMU in blending function
void NavEKF::getIMU1Weighting(float &ret) const
{
    ret = IMU1_weighting;
}

// return the individual Z-accel bias estimates in m/s^2
void NavEKF::getAccelZBias(float &zbias1, float &zbias2) const {
    if (dtIMUavg > 0) {
        zbias1 = stateStruct.accel_zbias / dtIMUavg;
        zbias2 = 0;
    } else {
        zbias1 = 0;
        zbias2 = 0;
    }
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF::getWind(Vector3f &wind) const
{
    wind.x = stateStruct.wind_vel.x;
    wind.y = stateStruct.wind_vel.y;
    wind.z = 0.0f; // currently don't estimate this
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF::getMagNED(Vector3f &magNED) const
{
    magNED = stateStruct.earth_magfield * 1000.0f;
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF::getMagXYZ(Vector3f &magXYZ) const
{
    magXYZ = stateStruct.body_magfield*1000.0f;
}

// return magnetometer offsets
// return true if offsets are valid
bool NavEKF::getMagOffsets(Vector3f &magOffsets) const
{
    // compass offsets are valid if we have finalised magnetic field initialisation and magnetic field learning is not prohibited and primary compass is valid
    if (secondMagYawInit && (_magCal != 2) && _ahrs->get_compass()->healthy(0)) {
        magOffsets = _ahrs->get_compass()->get_offsets(0) - stateStruct.body_magfield*1000.0f;
        return true;
    } else {
        magOffsets = _ahrs->get_compass()->get_offsets(0);
        return false;
    }
}

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool NavEKF::getLLH(struct Location &loc) const
{
    if(validOrigin) {
        // Altitude returned is an absolute altitude relative to the WGS-84 spherioid
        loc.alt = EKF_origin.alt - outputDataNew.position.z*100;
        loc.flags.relative_alt = 0;
        loc.flags.terrain_alt = 0;

        // there are three modes of operation, absolute position (GPS fusion), relative position (optical flow fusion) and constant position (no aiding)
        nav_filter_status status;
        getFilterStatus(status);
        if (status.flags.horiz_pos_abs || status.flags.horiz_pos_rel) {
            loc.lat = EKF_origin.lat;
            loc.lng = EKF_origin.lng;
            location_offset(loc, outputDataNew.position.x, outputDataNew.position.y);
            return true;
        } else {
            // we could be in constant position mode  becasue the vehicle has taken off without GPS, or has lost GPS
            // in this mode we cannot use the EKF states to estimate position so will return the best available data
            if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
                // we have a GPS position fix to return
                const struct Location &gpsloc = _ahrs->get_gps().location();
                loc.lat = gpsloc.lat;
                loc.lng = gpsloc.lng;
                return true;
            } else {
                // if no GPS fix, provide last known position before entering the mode
                location_offset(loc, lastKnownPositionNE.x, lastKnownPositionNE.y);
                return false;
            }
        }
    } else {
        // If no origin has been defined for the EKF, then we cannot use its position states so return a raw
        // GPS reading if available and return false
        if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D)) {
            const struct Location &gpsloc = _ahrs->get_gps().location();
            loc = gpsloc;
            loc.flags.relative_alt = 0;
            loc.flags.terrain_alt = 0;
        }
        return false;
    }
}

// return the estimated height above ground level
bool NavEKF::getHAGL(float &HAGL) const
{
    HAGL = terrainState - outputDataNew.position.z;
    // If we know the terrain offset and altitude, then we have a valid height above ground estimate
    return !hgtTimeout && gndOffsetValid && healthy();
}

// return data for debugging optical flow fusion
void NavEKF::getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov, float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    varFlow = max(flowTestRatio[0],flowTestRatio[1]);
    gndOffset = terrainState;
    flowInnovX = innovOptFlow[0];
    flowInnovY = innovOptFlow[1];
    auxInnov = auxFlowObsInnov;
    HAGL = terrainState - stateStruct.position.z;
    rngInnov = innovRng;
    range = rngMea;
    gndOffsetErr = sqrtf(Popt); // note Popt is constrained to be non-negative in EstimateTerrainOffset()
}

// calculate whether the flight vehicle is on the ground or flying from height, airspeed and GPS speed
void NavEKF::SetFlightAndFusionModes()
{
    // determine if the vehicle is manoevring
    if (accNavMagHoriz > 0.5f){
        manoeuvring = true;
    } else {
        manoeuvring = false;
    }
    // if we are a fly forward type vehicle, then in-air mode can be determined through a combination of speed and height criteria
    if (assume_zero_sideslip()) {
        // Evaluate a numerical score that defines the likelihood we are in the air
        float gndSpdSq = sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y);
        bool highGndSpd = false;
        bool highAirSpd = false;
        bool largeHgtChange = false;

        // trigger at 8 m/s airspeed
        if (_ahrs->airspeed_sensor_enabled()) {
            const AP_Airspeed *airspeed = _ahrs->get_airspeed();
            if (airspeed->get_airspeed() * airspeed->get_EAS2TAS() > 10.0f) {
                highAirSpd = true;
            }
        }

        // trigger at 10 m/s GPS velocity, but not if GPS is reporting bad velocity errors
        if (gndSpdSq > 100.0f && gpsSpdAccuracy < 1.0f) {
            highGndSpd = true;
        }

        // trigger if more than 10m away from initial height
        if (fabsf(baroDataDelayed.hgt) > 10.0f) {
            largeHgtChange = true;
        }

        // to go to in-air mode we also need enough GPS velocity to be able to calculate a reliable ground track heading and either a lerge height or airspeed change
        if (onGround && highGndSpd && (highAirSpd || largeHgtChange)) {
            onGround = false;
        }
        // if is possible we are in flight, set the time this condition was last detected
        if (highGndSpd || highAirSpd || largeHgtChange) {
            airborneDetectTime_ms = imuSampleTime_ms;
        }
        // after 5 seconds of not detecting a possible flight condition, we transition to on-ground mode
        if(!onGround && ((imuSampleTime_ms - airborneDetectTime_ms) > 5000)) {
            onGround = true;
        }
        // perform a yaw alignment check against GPS if exiting on-ground mode, bu tonly if we have enough ground speed
        // this is done to protect against unrecoverable heading alignment errors due to compass faults
        if (!onGround && prevOnGround) {
            alignYawGPS();
        }
    }
    // store current on-ground status for next time
    prevOnGround = onGround;
    // Wind state estimation disabled in this prototype untl we get the airspeed and sythetic sideslip fusion integrated
    inhibitWindStates = true;
    // request mag calibration for both in-air and manoeuvre threshold options
    bool magCalRequested = ((_magCal == 0) && !onGround) || ((_magCal == 1) && manoeuvring)  || (_magCal == 3);
    // deny mag calibration request if we aren't using the compass, are in the pre-arm constant position mode or it has been inhibited by the user
    bool magCalDenied = !use_compass() || constPosMode || (_magCal == 2);
    // inhibit the magnetic field calibration if not requested or denied
    inhibitMagStates = (!magCalRequested || magCalDenied);

    if (inhibitMagStates) {
        stateIndexLim = 15;
    } else if (inhibitWindStates) {
        stateIndexLim = 21;
    } else {
        stateIndexLim = 23;
    }
}

// initialise the covariance matrix
void NavEKF::CovarianceInit()
{
    // zero the matrix
    for (uint8_t i=1; i<=stateIndexLim; i++)
    {
        for (uint8_t j=0; j<=stateIndexLim; j++)
        {
            P[i][j] = 0.0f;
        }
    }
    // attitude error
    P[0][0]   = 0.1f;
    P[1][1]   = 0.1f;
    P[2][2]   = 0.1f;
    // velocities
    P[3][3]   = sq(0.7f);
    P[4][4]   = P[3][3];
    P[5][5]   = sq(0.7f);
    // positions
    P[6][6]   = sq(15.0f);
    P[7][7]   = P[6][6];
    P[8][8]   = sq(_baroAltNoise);
    // gyro delta angle biases
    P[9][9] = sq(radians(InitialGyroBiasUncertainty() * dtIMUavg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
    // gyro scale factor biases
    P[12][12] = sq(1e-3);
    P[13][13] = P[12][12];
    P[14][14] = P[12][12];
    // Z delta velocity bias
    P[15][15] = sq(INIT_ACCEL_BIAS_UNCERTAINTY * dtIMUavg);
    // earth magnetic field
    P[16][16] = 0.0f;
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field
    P[19][19] = 0.0f;
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
    // wind velocities
    P[22][22] = 0.0f;
    P[23][23]  = P[22][22];


    // optical flow ground height covariance
    Popt = 0.25f;

}

// force symmetry on the covariance matrix to prevent ill-conditioning
void NavEKF::ForceSymmetry()
{
    for (uint8_t i=1; i<=stateIndexLim; i++)
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            float temp = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// copy covariances across from covariance prediction calculation
void NavEKF::CopyCovariances()
{
    // copy predicted covariances
    for (uint8_t i=0; i<=stateIndexLim; i++) {
        for (uint8_t j=0; j<=stateIndexLim; j++)
        {
            P[i][j] = nextP[i][j];
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
void NavEKF::ConstrainVariances()
{
    for (uint8_t i=0; i<=2; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f); // attitude error
    for (uint8_t i=3; i<=5; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // velocities
    for (uint8_t i=6; i<=8; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e6f); // positions
    for (uint8_t i=9; i<=11; i++) P[i][i] = constrain_float(P[i][i],0.0f,sq(0.175f * dtIMUavg)); // delta angle biases
    for (uint8_t i=12; i<=14; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // delta angle scale factors
    P[15][15] = constrain_float(P[15][15],0.0f,sq(10.0f * dtIMUavg)); // delta velocity bias
    for (uint8_t i=16; i<=18; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // earth magnetic field
    for (uint8_t i=19; i<=21; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // body magnetic field
    for (uint8_t i=22; i<=23; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // wind velocity
}

// constrain states to prevent ill-conditioning
void NavEKF::ConstrainStates()
{
    // attitude errors are limited between +-1
    for (uint8_t i=0; i<=2; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i=3; i<=5; i++) statesArray[i] = constrain_float(statesArray[i],-5.0e2f,5.0e2f);
    // position limit 1000 km - TODO apply circular limit
    for (uint8_t i=6; i<=7; i++) statesArray[i] = constrain_float(statesArray[i],-1.0e6f,1.0e6f);
    // height limit covers home alt on everest through to home alt at SL and ballon drop
    stateStruct.position.z = constrain_float(stateStruct.position.z,-4.0e4f,1.0e4f);
    // gyro bias limit ~6 deg/sec (this needs to be set based on manufacturers specs)
    for (uint8_t i=9; i<=11; i++) statesArray[i] = constrain_float(statesArray[i],-0.1f*dtIMUavg,0.1f*dtIMUavg);
    // gyro scale factor limit of +-5% (this needs to be set based on manufacturers specs)
    for (uint8_t i=12; i<=14; i++) statesArray[i] = constrain_float(statesArray[i],0.95f,1.05f);
    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    stateStruct.accel_zbias = constrain_float(stateStruct.accel_zbias,-1.0f*dtIMUavg,1.0f*dtIMUavg);
    // earth magnetic field limit
    for (uint8_t i=16; i<=18; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // body magnetic field limit
    for (uint8_t i=19; i<=21; i++) statesArray[i] = constrain_float(statesArray[i],-0.5f,0.5f);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=22; i<=23; i++) statesArray[i] = constrain_float(statesArray[i],-100.0f,100.0f);
    // constrain the terrain offset state
    terrainState = max(terrainState, stateStruct.position.z + rngOnGnd);
}

bool NavEKF::readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt) {
    const AP_InertialSensor &ins = _ahrs->get_ins();

    if (ins_index < ins.get_accel_count()) {
        ins.get_delta_velocity(ins_index,dVel);
        dVel_dt = max(ins.get_delta_velocity_dt(ins_index),1.0e-4f);
        return true;
    }
    return false;
}

bool NavEKF::readDeltaAngle(uint8_t ins_index, Vector3f &dAng) {
    const AP_InertialSensor &ins = _ahrs->get_ins();

    if (ins_index < ins.get_gyro_count()) {
        ins.get_delta_angle(ins_index,dAng);
        return true;
    }
    return false;
}

// update IMU delta angle and delta velocity measurements
void NavEKF::readIMUData()
{
    const AP_InertialSensor &ins = _ahrs->get_ins();

    // average IMU sampling rate
    dtIMUavg = 1.0f/ins.get_sample_rate();

    // the imu sample time is used as a common time reference throughout the filter
    imuSampleTime_ms = hal.scheduler->millis();

    // Get delta velocity data
    readDeltaVelocity(ins.get_primary_accel(), imuDataNew.delVel, imuDataNew.delVelDT);

    // Get delta angle data
    readDeltaAngle(ins.get_primary_gyro(), imuDataNew.delAng);
    imuDataNew.delAngDT = max(ins.get_delta_time(),1.0e-4f);

    // get current time stamp
    imuDataNew.time_ms = imuSampleTime_ms;

    // save data in the FIFO buffer
    StoreIMU();

    // extract the oldest available data from the FIFO buffer
    imuDataDelayed = storedIMU[fifoIndexDelayed];

}

// check for new valid GPS data and update stored measurement if available
void NavEKF::readGpsData()
{
    bool goodToAlign = false;
    // check for new GPS data
    if ((_ahrs->get_gps().last_message_time_ms() != lastTimeGpsReceived_ms) &&
            (_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D))
    {
        // store fix time from previous read
        secondLastGpsTime_ms = lastTimeGpsReceived_ms;

        // get current fix time
        lastTimeGpsReceived_ms = _ahrs->get_gps().last_message_time_ms();

        // estimate when the GPS fix was valid, allowing for GPS processing and other delays
        // ideally we should be using a timing signal from the GPS receiver to set this time
        gpsFixTime_ms = lastTimeGpsReceived_ms - _msecGpsDelay;

        // read the NED velocity from the GPS
        gpsDataNew.vel = _ahrs->get_gps().velocity();

        // Use the speed accuracy from the GPS if available, otherwise set it to zero.
        // Apply a decaying envelope filter with a 5 second time constant to the raw speed accuracy data
        float alpha = constrain_float(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms),0.0f,1.0f);
        gpsSpdAccuracy *= (1.0f - alpha);
        float gpsSpdAccRaw;
        if (!_ahrs->get_gps().speed_accuracy(gpsSpdAccRaw)) {
            gpsSpdAccuracy = 0.0f;
        } else {
            gpsSpdAccuracy = max(gpsSpdAccuracy,gpsSpdAccRaw);
        }

        // check if we have enough GPS satellites and increase the gps noise scaler if we don't
        if (_ahrs->get_gps().num_sats() >= 6 && !constPosMode) {
            gpsNoiseScaler = 1.0f;
        } else if (_ahrs->get_gps().num_sats() == 5 && !constPosMode) {
            gpsNoiseScaler = 1.4f;
        } else { // <= 4 satellites or in constant position mode
            gpsNoiseScaler = 2.0f;
        }

        // Check if GPS can output vertical velocity and set GPS fusion mode accordingly
        if (_ahrs->get_gps().have_vertical_velocity() && _fusionModeGPS == 0) {
            useGpsVertVel = true;
        } else {
            useGpsVertVel = false;
        }

        // Monitor quality of the GPS velocity data for alignment
        goodToAlign = calcGpsGoodToAlign();

        // read latitutde and longitude from GPS and convert to local NE position relative to the stored origin
        // If we don't have an origin, then set it to the current GPS coordinates
        const struct Location &gpsloc = _ahrs->get_gps().location();
        if (validOrigin) {
            gpsDataNew.pos = location_diff(EKF_origin, gpsloc);
        } else if (goodToAlign){
            // Set the NE origin to the current GPS position
            setOrigin();
            // Now we know the location we have an estimate for the magnetic field declination and adjust the earth field accordingly
            alignMagStateDeclination();
            // Set the height of the NED origin to ‘height of baro height datum relative to GPS height datum'
            EKF_origin.alt = gpsloc.alt - baroDataNew.hgt;
            // We are by definition at the origin at the instant of alignment so set NE position to zero
            gpsDataNew.pos.zero();
            // If the vehicle is in flight (use arm status to determine) and GPS useage isn't explicitly prohibited, we switch to absolute position mode
            if (filterArmed && _fusionModeGPS != 3) {
                constPosMode = false;
                PV_AidingMode = AID_ABSOLUTE;
                gpsNotAvailable = false;
                // Initialise EKF position and velocity states
                ResetPosition();
                ResetVelocity();
            }
        }

        // calculate a position offset which is applied to NE position and velocity wherever it is used throughout code to allow GPS position jumps to be accommodated gradually
        decayGpsOffset();

        // save baro measurement to buffer to be fused later
        gpsDataNew.time_ms = gpsFixTime_ms;
        StoreGPS();

    }

    // If no previous GPS lock or told not to use it, or EKF origin not set, we declare the  GPS unavailable for use
    if ((_ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) || _fusionModeGPS == 3 || !validOrigin) {
        gpsNotAvailable = true;
    } else {
        gpsNotAvailable = false;
    }
}

// check for new altitude measurement data and update stored measurement if available
void NavEKF::readHgtData()
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    if (_baro.get_last_update() != lastHgtReceived_ms) {
        // Don't use Baro height if operating in optical flow mode as we use range finder instead
        if (_fusionModeGPS == 3 && _altSource == 1) {
            if ((imuSampleTime_ms - rngValidMeaTime_ms) < 2000) {
                // adjust range finder measurement to allow for effect of vehicle tilt and height of sensor
                baroDataNew.hgt = max(rngMea * Tnb_flow.c.z, rngOnGnd);
                // calculate offset to baro data that enables baro to be used as a backup
                // filter offset to reduce effect of baro noise and other transient errors on estimate
                baroHgtOffset = 0.1f * (_baro.get_altitude() + stateStruct.position.z) + 0.9f * baroHgtOffset;
            } else if (filterArmed && takeOffDetected) {
                // use baro measurement and correct for baro offset - failsafe use only as baro will drift
                baroDataNew.hgt = max(_baro.get_altitude() - baroHgtOffset, rngOnGnd);
            } else {
                // If we are on ground and have no range finder reading, assume the nominal on-ground height
                baroDataNew.hgt = rngOnGnd;
                // calculate offset to baro data that enables baro to be used as a backup
                // filter offset to reduce effect of baro noise and other transient errors on estimate
                baroHgtOffset = 0.1f * (_baro.get_altitude() + stateStruct.position.z) + 0.9f * baroHgtOffset;
            }
        } else {
            // use baro measurement and correct for baro offset
            baroDataNew.hgt = _baro.get_altitude();
        }

        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        if (!getTakeoffExpected()) {
            static const float gndHgtFiltTC = 0.5f;
            static const float dtBaro = msecHgtAvg*1.0e-3f;
            float alpha = constrain_float(dtBaro / (dtBaro+gndHgtFiltTC),0.0f,1.0f);
            meaHgtAtTakeOff += (baroDataDelayed.hgt-meaHgtAtTakeOff)*alpha;
        } else if (filterArmed && getTakeoffExpected()) {
            // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
            // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
            baroDataNew.hgt = max(baroDataNew.hgt, meaHgtAtTakeOff);
        }

        // time stamp used to check for new measurement
        lastHgtReceived_ms = _baro.get_last_update();

        // estimate of time height measurement was taken, allowing for delays
        hgtMeasTime_ms = lastHgtReceived_ms - msecHgtDelay;

        // save baro measurement to buffer to be fused later
        baroDataNew.time_ms = hgtMeasTime_ms;
        StoreBaro();
    }
}

// check for new magnetometer data and update store measurements if available
void NavEKF::readMagData()
{
    if (use_compass() && _ahrs->get_compass()->last_update_usec() != lastMagUpdate) {
        // store time of last measurement update
        lastMagUpdate = _ahrs->get_compass()->last_update_usec();

        // estimate of time magnetometer measurement was taken, allowing for delays
        magMeasTime_ms = imuSampleTime_ms - msecMagDelay;

        // read compass data and scale to improve numerical conditioning
        magDataNew.mag = _ahrs->get_compass()->get_field() * 0.001f;

        // check if compass offsets have been changed and adjust EKF bias states to maintain consistent innovations
        if (_ahrs->get_compass()->healthy(0)) {
            Vector3f nowMagOffsets = _ahrs->get_compass()->get_offsets(0);
            bool changeDetected = (!is_equal(nowMagOffsets.x,lastMagOffsets.x) || !is_equal(nowMagOffsets.y,lastMagOffsets.y) || !is_equal(nowMagOffsets.z,lastMagOffsets.z));
            // Ignore bias changes before final mag field and yaw initialisation, as there may have been a compass calibration
            if (changeDetected && secondMagYawInit) {
                stateStruct.body_magfield.x += (nowMagOffsets.x - lastMagOffsets.x) * 0.001f;
                stateStruct.body_magfield.y += (nowMagOffsets.y - lastMagOffsets.y) * 0.001f;
                stateStruct.body_magfield.z += (nowMagOffsets.z - lastMagOffsets.z) * 0.001f;
            }
        lastMagOffsets = nowMagOffsets;
        }

        // save magnetometer measurement to buffer to be fused later
        magDataNew.time_ms = magMeasTime_ms;
        StoreMag();
    }
}

// write the raw optical flow measurements
// this needs to be called externally.
void NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
{
    // The raw measurements need to be optical flow rates in radians/second averaged across the time since the last update
    // The PX4Flow sensor outputs flow rates with the following axis and sign conventions:
    // A positive X rate is produced by a positive sensor rotation about the X axis
    // A positive Y rate is produced by a positive sensor rotation about the Y axis
    // This filter uses a different definition of optical flow rates to the sensor with a positive optical flow rate produced by a
    // negative rotation about that axis. For example a positive rotation of the flight vehicle about its X (roll) axis would produce a negative X flow rate
    flowMeaTime_ms = imuSampleTime_ms;
    flowQuality = rawFlowQuality;
    // calculate bias errors on flow sensor gyro rates, but protect against spikes in data
    flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrain_float((rawGyroRates.x - stateStruct.omega.x),-0.1f,0.1f);
    flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrain_float((rawGyroRates.y - stateStruct.omega.y),-0.1f,0.1f);
    // check for takeoff if relying on optical flow and zero measurements until takeoff detected
    // if we haven't taken off - constrain position and velocity states
    if (_fusionModeGPS == 3) {
        detectOptFlowTakeoff();
    }
    // calculate rotation matrices at mid sample time for flow observations
    stateStruct.quat.rotation_matrix(Tbn_flow);
    Tnb_flow = Tbn_flow.transposed();
    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && rawFlowRates.length() < 4.2f && rawGyroRates.length() < 4.2f) {
        // correct flow sensor rates for bias
        omegaAcrossFlowTime.x = rawGyroRates.x - flowGyroBias.x;
        omegaAcrossFlowTime.y = rawGyroRates.y - flowGyroBias.y;
        // write uncorrected flow rate measurements that will be used by the focal length scale factor estimator
        // note correction for different axis and sign conventions used by the px4flow sensor
        flowRadXY[0] = - rawFlowRates.x; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        flowRadXY[1] = - rawFlowRates.y; // raw (non motion compensated) optical flow angular rate about the Y axis (rad/sec)
        // write flow rate measurements corrected for body rates
        flowRadXYcomp[0] = flowRadXY[0] + omegaAcrossFlowTime.x;
        flowRadXYcomp[1] = flowRadXY[1] + omegaAcrossFlowTime.y;
        // set flag that will trigger observations
        newDataFlow = true;
        flowValidMeaTime_ms = imuSampleTime_ms;
    } else {
        newDataFlow = false;
    }
}

// calculate the NED earth spin vector in rad/sec
void NavEKF::calcEarthRateNED(Vector3f &omega, int32_t latitude) const
{
    float lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosf(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinf(lat_rad);
}

// initialise the earth magnetic field states using declination, suppled roll/pitch
// and magnetometer measurements and return initial attitude quaternion
// if no magnetometer data, do not update magnetic field states and assume zero yaw angle
Quaternion NavEKF::calcQuatAndFieldStates(float roll, float pitch)
{
    // declare local variables required to calculate initial orientation and magnetic field
    float yaw;
    Matrix3f Tbn;
    Vector3f initMagNED;
    Quaternion initQuat;

    if (use_compass()) {
        // calculate rotation matrix from body to NED frame
        Tbn.from_euler(roll, pitch, 0.0f);

        // read the magnetometer data
        readMagData();

        // rotate the magnetic field into NED axes
        initMagNED = Tbn * magDataDelayed.mag;

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;
        yawAligned = true;
        // calculate initial filter quaternion states using yaw from magnetometer if mag heading healthy
        // otherwise use existing heading
        if (!badMag) {
            // store the yaw change so that it can be retrieved externally for use by the control loops to prevent yaw disturbances following a reset
            Vector3f tempEuler;
            stateStruct.quat.to_euler(tempEuler.x, tempEuler.y, tempEuler.z);
            yawResetAngle = wrap_PI(yaw - tempEuler.z);
            // set the flag to indicate that an in-flight yaw reset has been performed
            // this will be cleared when the reset value is retrieved
            yawResetAngleWaiting = true;
            // calculate an initial quaternion using the new yaw value
            initQuat.from_euler(roll, pitch, yaw);
        } else {
            initQuat = stateStruct.quat;
        }

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        initQuat.rotation_matrix(Tbn);
        stateStruct.earth_magfield = Tbn * magDataDelayed.mag;

        // align the NE earth magnetic field states with the published declination
        alignMagStateDeclination();

        // clear bad magnetometer status
        badMag = false;
    } else {
        initQuat.from_euler(roll, pitch, 0.0f);
        yawAligned = false;
    }

    // return attitude quaternion
    return initQuat;
}

// this function is used to do a forced alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF::alignYawGPS()
{
    if ((sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y)) > 25.0f) {
        float roll;
        float pitch;
        float oldYaw;
        float newYaw;
        float yawErr;
        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        stateStruct.quat.to_euler(roll, pitch, oldYaw);
        // calculate course yaw angle
        oldYaw = atan2f(stateStruct.velocity.y,stateStruct.velocity.x);
        // calculate yaw angle from GPS velocity
        newYaw = atan2f(gpsDataNew.vel.y,gpsDataNew.vel.x);
        // estimate the yaw error
        yawErr = wrap_PI(newYaw - oldYaw);
        // If the inertial course angle disagrees with the GPS by more than 45 degrees, we declare the compass as bad
        badMag = (fabsf(yawErr) > 0.7854f);
        // correct yaw angle using GPS ground course compass failed or if not previously aligned
        if (badMag || !yawAligned) {
            // correct the yaw angle
            newYaw = oldYaw + yawErr;
            // calculate new filter quaternion states from Euler angles
            stateStruct.quat.from_euler(roll, pitch, newYaw);
            // the yaw angle is now aligned so update its status
            yawAligned =  true;
            // reset the position and velocity states
            ResetPosition();
            ResetVelocity();
            // reset the covariance for the quaternion, velocity and position states
            // zero the matrix entries
            zeroRows(P,0,9);
            zeroCols(P,0,9);
            // velocities - we could have a big error coming out of constant position mode due to GPS lag
            P[3][3]   = 400.0f;
            P[4][4]   = P[3][3];
            P[5][5]   = sq(0.7f);
            // positions - we could have a big error coming out of constant position mode due to GPS lag
            P[6][6]   = 400.0f;
            P[7][7]   = P[6][6];
            P[8][8]   = sq(5.0f);
        }
        // Update magnetic field states if the magnetometer is bad
        if (badMag) {
            Vector3f eulerAngles;
            getEulerAngles(eulerAngles);
            calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        }
    }
}

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF::getRotationBodyToNED(Matrix3f &mat) const
{
    Vector3f trim = _ahrs->get_trim();
    outputDataNew.quat.rotation_matrix(mat);
    mat.rotateXYinv(trim);
}

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
void  NavEKF::getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    velInnov.x = innovVelPos[0];
    velInnov.y = innovVelPos[1];
    velInnov.z = innovVelPos[2];
    posInnov.x = innovVelPos[3];
    posInnov.y = innovVelPos[4];
    posInnov.z = innovVelPos[5];
    magInnov.x = 1e3f*innovMag[0]; // Convert back to sensor units
    magInnov.y = 1e3f*innovMag[1]; // Convert back to sensor units
    magInnov.z = 1e3f*innovMag[2]; // Convert back to sensor units
    tasInnov   = innovVtas;
    yawInnov   = innovYaw;
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
// this indicates the amount of margin available when tuning the various error traps
// also return the current offsets applied to the GPS position measurements
void  NavEKF::getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    velVar   = sqrtf(velTestRatio);
    posVar   = sqrtf(posTestRatio);
    hgtVar   = sqrtf(hgtTestRatio);
    magVar.x = sqrtf(magTestRatio.x);
    magVar.y = sqrtf(magTestRatio.y);
    magVar.z = sqrtf(magTestRatio.z);
    tasVar   = sqrtf(tasTestRatio);
    offset   = gpsPosGlitchOffsetNE;
}

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF::InitialiseVariables()
{
    // initialise time stamps
    imuSampleTime_ms = hal.scheduler->millis();
    lastHealthyMagTime_ms = imuSampleTime_ms;
    TASmsecPrev = imuSampleTime_ms;
    BETAmsecPrev = imuSampleTime_ms;
    lastMagUpdate = 0;
    lastHgtReceived_ms = imuSampleTime_ms;
    lastAirspeedUpdate = 0;
    lastVelPassTime = imuSampleTime_ms;
    lastPosPassTime = imuSampleTime_ms;
    lastPosFailTime = 0;
    lastHgtPassTime = imuSampleTime_ms;
    lastTasPassTime = imuSampleTime_ms;
    lastStateStoreTime_ms = imuSampleTime_ms;
    lastTimeGpsReceived_ms = 0;
    secondLastGpsTime_ms = 0;
    lastDecayTime_ms = imuSampleTime_ms;
    timeAtLastAuxEKF_ms = imuSampleTime_ms;
    flowValidMeaTime_ms = imuSampleTime_ms;
    rngValidMeaTime_ms = imuSampleTime_ms;
    flowMeaTime_ms = 0;
    prevFlowFuseTime_ms = imuSampleTime_ms;
    gndHgtValidTime_ms = 0;
    ekfStartTime_ms = imuSampleTime_ms;
    lastGpsVelFail_ms = 0;
    lastGpsAidBadTime_ms = 0;
    gpsFixTime_ms = imuSampleTime_ms;
    hgtMeasTime_ms = imuSampleTime_ms;
    magMeasTime_ms = imuSampleTime_ms;

    // initialise other variables
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    magTimeout = true;
    tasTimeout = true;
    badMag = false;
    badIMUdata = false;
    firstArmComplete = false;
    firstMagYawInit = false;
    secondMagYawInit = false;
    dtIMUavg = 0.0025f;
    dt = 0;
    lastGyroBias.zero();
    lastAngRate.zero();
    lastAccel1.zero();
    lastAccel2.zero();
    velDotNEDfilt.zero();
    summedDelAng.zero();
    summedDelVel.zero();
    gpsPosGlitchOffsetNE.zero();
    lastKnownPositionNE.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&nextP[0][0], 0, sizeof(nextP));
    memset(&processNoise[0], 0, sizeof(processNoise));
    newDataFlow = false;
    flowDataValid = false;
    newDataRng  = false;
    flowFusePerformed = false;
    fuseOptFlowData = false;
    Popt = 0.0f;
    terrainState = 0.0f;
    prevPosN = stateStruct.position.x;
    prevPosE = stateStruct.position.y;
    fuseRngData = false;
    inhibitGndState = true;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    constVelMode = false;
    lastConstVelMode = false;
    heldVelNE.zero();
    PV_AidingMode = AID_NONE;
    posTimeout = true;
    velTimeout = true;
    gpsVelGlitchOffset.zero();
    filterArmed = false;
    prevFilterArmed = false;
    constPosMode = true;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
    IMU1_weighting = 0.5f;
    onGround = true;
    manoeuvring = false;
    yawAligned = false;
    inhibitWindStates = true;
    inhibitMagStates = true;
    gndOffsetValid =  false;
    flowXfailed = false;
    validOrigin = false;
    takeoffExpectedSet_ms = 0;
    expectGndEffectTakeoff = false;
    touchdownExpectedSet_ms = 0;
    expectGndEffectTouchdown = false;
    gpsSpdAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    gpsAidingBad = false;
    highYawRate = false;
    yawRateFilt = 0.0f;
    yawResetAngle = 0.0f;
    yawResetAngleWaiting = false;
    tiltErrFilt = 1.0f;
    tiltAlignComplete = false;
    yawAlignComplete = false;
    stateIndexLim = 23;
    imuDataNew.frame = 0;
    baroStoreIndex = 0;
    magStoreIndex = 0;
    gpsStoreIndex = 0;
    delAngCorrection.zero();
    delVelCorrection.zero();
    velCorrection.zero();

}

// return true if we should use the airspeed sensor
bool NavEKF::useAirspeed(void) const
{
    return false;
}

// return true if we should use the range finder sensor
bool NavEKF::useRngFinder(void) const
{
    // TO-DO add code to set this based in setting of optical flow use parameter and presence of sensor
    return true;
}

// return true if optical flow data is available
bool NavEKF::optFlowDataPresent(void) const
{
    return false;
}

// return true if the filter to be ready to use gps
bool NavEKF::readyToUseGPS(void) const
{
    return validOrigin && tiltAlignComplete && yawAlignComplete;
}

// return true if we should use the compass
bool NavEKF::use_compass(void) const
{
    return _ahrs->get_compass() && _ahrs->get_compass()->use_for_yaw();
}

// decay GPS horizontal position offset to close to zero at a rate of 1 m/s for copters and 5 m/s for planes
// limit radius to a maximum of 50m
void NavEKF::decayGpsOffset()
{
    float offsetDecaySpd;
    if (assume_zero_sideslip()) {
        offsetDecaySpd = 5.0f;
    } else {
        offsetDecaySpd = 1.0f;
    }
    float lapsedTime = 0.001f*float(imuSampleTime_ms - lastDecayTime_ms);
    lastDecayTime_ms = imuSampleTime_ms;
    float offsetRadius = pythagorous2(gpsPosGlitchOffsetNE.x, gpsPosGlitchOffsetNE.y);
    // decay radius if larger than offset decay speed multiplied by lapsed time (plus a margin to prevent divide by zero)
    if (offsetRadius > (offsetDecaySpd * lapsedTime + 0.1f)) {
        // Calculate the GPS velocity offset required. This is necessary to prevent the position measurement being rejected for inconsistency when the radius is being pulled back in.
        gpsVelGlitchOffset = -gpsPosGlitchOffsetNE*offsetDecaySpd/offsetRadius;
        // calculate scale factor to be applied to both offset components
        float scaleFactor = constrain_float((offsetRadius - offsetDecaySpd * lapsedTime), 0.0f, 50.0f) / offsetRadius;
        gpsPosGlitchOffsetNE.x *= scaleFactor;
        gpsPosGlitchOffsetNE.y *= scaleFactor;
    } else {
        gpsVelGlitchOffset.zero();
        gpsPosGlitchOffsetNE.zero();
    }
}

/*
  should we assume zero sideslip?
 */
bool NavEKF::assume_zero_sideslip(void) const
{
    // we don't assume zero sideslip for ground vehicles as EKF could
    // be quite sensitive to a rapid spin of the ground vehicle if
    // traction is lost
    return _ahrs->get_fly_forward() && _ahrs->get_vehicle_class() != AHRS_VEHICLE_GROUND;
}


/* 
   vehicle specific initial gyro bias uncertainty in deg/sec
 */
float NavEKF::InitialGyroBiasUncertainty(void) const
{
    return 5.0f;
}

/*
return the filter fault status as a bitmasked integer
 0 = quaternions are NaN
 1 = velocities are NaN
 2 = badly conditioned X magnetometer fusion
 3 = badly conditioned Y magnetometer fusion
 5 = badly conditioned Z magnetometer fusion
 6 = badly conditioned airspeed fusion
 7 = badly conditioned synthetic sideslip fusion
 7 = filter is not initialised
*/
void  NavEKF::getFilterFaults(uint8_t &faults) const
{
    faults = (stateStruct.quat.is_nan()<<0 |
              stateStruct.velocity.is_nan()<<1 |
              faultStatus.bad_xmag<<2 |
              faultStatus.bad_ymag<<3 |
              faultStatus.bad_zmag<<4 |
              faultStatus.bad_airspeed<<5 |
              faultStatus.bad_sideslip<<6 |
              !statesInitialised<<7);
}

/*
return filter timeout status as a bitmasked integer
 0 = position measurement timeout
 1 = velocity measurement timeout
 2 = height measurement timeout
 3 = magnetometer measurement timeout
 4 = true airspeed measurement timeout
 5 = unassigned
 6 = unassigned
 7 = unassigned
*/
void  NavEKF::getFilterTimeouts(uint8_t &timeouts) const
{
    timeouts = (posTimeout<<0 |
                velTimeout<<1 |
                hgtTimeout<<2 |
                magTimeout<<3 |
                tasTimeout<<4);
}

/*
return filter function status as a bitmasked integer
 0 = attitude estimate valid
 1 = horizontal velocity estimate valid
 2 = vertical velocity estimate valid
 3 = relative horizontal position estimate valid
 4 = absolute horizontal position estimate valid
 5 = vertical position estimate valid
 6 = terrain height estimate valid
 7 = constant position mode
*/
void  NavEKF::getFilterStatus(nav_filter_status &status) const
{
    // init return value
    status.value = 0;

    bool doingFlowNav = (PV_AidingMode == AID_RELATIVE) && flowDataValid;
    bool doingWindRelNav = !tasTimeout && assume_zero_sideslip();
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool notDeadReckoning = !constVelMode && !constPosMode;
    bool someVertRefData = (!velTimeout && useGpsVertVel) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout) || doingFlowNav;
    bool optFlowNavPossible = flowDataValid && (_fusionModeGPS == 3);
    bool gpsNavPossible = !gpsNotAvailable && (_fusionModeGPS <= 2);
    bool filterHealthy = healthy();

    // set individual flags
    status.flags.attitude = !stateStruct.quat.is_nan() && filterHealthy;   // attitude valid (we need a better check)
    status.flags.horiz_vel = someHorizRefData && notDeadReckoning && filterHealthy;      // horizontal velocity estimate valid
    status.flags.vert_vel = someVertRefData && filterHealthy;        // vertical velocity estimate valid
    status.flags.horiz_pos_rel = ((doingFlowNav && gndOffsetValid) || doingWindRelNav || doingNormalGpsNav) && notDeadReckoning && filterHealthy;   // relative horizontal position estimate valid
    status.flags.horiz_pos_abs = !gpsAidingBad && doingNormalGpsNav && notDeadReckoning && filterHealthy; // absolute horizontal position estimate valid
    status.flags.vert_pos = !hgtTimeout && filterHealthy;            // vertical position estimate valid
    status.flags.terrain_alt = gndOffsetValid && filterHealthy;		// terrain height estimate valid
    status.flags.const_pos_mode = constPosMode && filterHealthy;     // constant position mode
    status.flags.pred_horiz_pos_rel = (optFlowNavPossible || gpsNavPossible) && filterHealthy; // we should be able to estimate a relative position when we enter flight mode
    status.flags.pred_horiz_pos_abs = gpsNavPossible && filterHealthy; // we should be able to estimate an absolute position when we enter flight mode
    status.flags.takeoff_detected = takeOffDetected; // takeoff for optical flow navigation has been detected
    status.flags.takeoff = expectGndEffectTakeoff; // The EKF has been told to expect takeoff and is in a ground effect mitigation mode
    status.flags.touchdown = expectGndEffectTouchdown; // The EKF has been told to detect touchdown and is in a ground effect mitigation mode
    status.flags.using_gps = (imuSampleTime_ms - lastPosPassTime) < 4000;
}

// send an EKF_STATUS message to GCS
void NavEKF::send_status_report(mavlink_channel_t chan)
{
    // get filter status
    nav_filter_status filt_state;
    getFilterStatus(filt_state);

    // prepare flags
    uint16_t flags = 0;
    if (filt_state.flags.attitude) { flags |= EKF_ATTITUDE; }
    if (filt_state.flags.horiz_vel) { flags |= EKF_VELOCITY_HORIZ; }
    if (filt_state.flags.vert_vel) { flags |= EKF_VELOCITY_VERT; }
    if (filt_state.flags.horiz_pos_rel) { flags |= EKF_POS_HORIZ_REL; }
    if (filt_state.flags.horiz_pos_abs) { flags |= EKF_POS_HORIZ_ABS; }
    if (filt_state.flags.vert_pos) { flags |= EKF_POS_VERT_ABS; }
    if (filt_state.flags.terrain_alt) { flags |= EKF_POS_VERT_AGL; }
    if (filt_state.flags.const_pos_mode) { flags |= EKF_CONST_POS_MODE; }
    if (filt_state.flags.pred_horiz_pos_rel) { flags |= EKF_PRED_POS_HORIZ_REL; }
    if (filt_state.flags.pred_horiz_pos_abs) { flags |= EKF_PRED_POS_HORIZ_ABS; }

    // get variances
    float velVar, posVar, hgtVar, tasVar;
    Vector3f magVar;
    Vector2f offset;
    getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);

    // send message
    mavlink_msg_ekf_status_report_send(chan, flags, velVar, posVar, hgtVar, magVar.length(), tasVar);

}

// Check arm status and perform required checks and mode changes
void NavEKF::performArmingChecks()
{
    // don't allow filter to arm until it has been running for long enough to stabilise
    prevFilterArmed = filterArmed;
    filterArmed = ((readyToUseGPS() || _fusionModeGPS == 3) && (imuSampleTime_ms - ekfStartTime_ms) > 1000);

    // check to see if arm status has changed and reset states if it has
    if (filterArmed != prevFilterArmed) {
        // only reset the magnetic field and heading on the first arm. This prevents in-flight learning being forgotten for vehicles that do multiple short flights and disarm in-between.
        if (filterArmed && !firstArmComplete) {
            firstArmComplete = true;
            Vector3f eulerAngles;
            getEulerAngles(eulerAngles);
            stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
            StoreQuatReset();
        }
        // store vertical position at arming to use as a reference for ground relative cehcks
        if (filterArmed) {
            posDownAtArming = stateStruct.position.z;
        }
        // zero stored velocities used to do dead-reckoning
        heldVelNE.zero();
        // reset the flag that indicates takeoff for use by optical flow navigation
        takeOffDetected = false;
        // set various  useage modes based on the condition at arming. These are then held until the vehicle is disarmed.
        if (!filterArmed) {
            PV_AidingMode = AID_NONE; // When dis-armed, we only estimate orientation & height using the constant position mode
            posTimeout = true;
            velTimeout = true;
            constPosMode = true;
            constVelMode = false; // always clear constant velocity mode if constant position mode is active
            lastConstVelMode = false;
            // store the current position to be used to keep reporting the last known position when disarmed
            lastKnownPositionNE.x = stateStruct.position.x;
            lastKnownPositionNE.y = stateStruct.position.y;
            // initialise filtered altitude used to provide a takeoff reference to current baro on disarm
            // this reduces the time required for the filter to settle before the estimate can be used
            meaHgtAtTakeOff = baroDataDelayed.hgt;
            // reset the vertical position state to faster recover from baro errors experienced during touchdown
            stateStruct.position.z = -meaHgtAtTakeOff;
        } else if (_fusionModeGPS == 3) { // arming when GPS useage has been prohibited
            if (optFlowDataPresent()) {
                hal.console->printf("EKF is using optical flow\n");
                PV_AidingMode = AID_RELATIVE; // we have optical flow data and can estimate all vehicle states
                posTimeout = true;
                velTimeout = true;
                constPosMode = false;
                constVelMode = false;
            } else {
                hal.console->printf("EKF cannot use aiding\n");
                PV_AidingMode = AID_NONE; // we don't have optical flow data and will only be able to estimate orientation and height
                posTimeout = true;
                velTimeout = true;
                constPosMode = true;
                constVelMode = false; // always clear constant velocity mode if constant position mode is active
            }
            // Reset the last valid flow measurement time
            flowValidMeaTime_ms = imuSampleTime_ms;
            // Reset the last valid flow fusion time
            prevFlowFuseTime_ms = imuSampleTime_ms;
            // this avoids issues casued by the time delay associated with arming that can trigger short timeouts
            rngValidMeaTime_ms = imuSampleTime_ms;
            // store the range finder measurement which will be used as a reference to detect when we have taken off
            rangeAtArming = rngMea;
            // set the time at which we arm to assist with takeoff detection
            timeAtArming_ms =  imuSampleTime_ms;
        } else { // arming when GPS useage is allowed
            if (gpsNotAvailable) {
                hal.console->printf("EKF cannot use aiding\n");
                PV_AidingMode = AID_NONE; // we don't have have GPS data and will only be able to estimate orientation and height
                posTimeout = true;
                velTimeout = true;
                constPosMode = true;
                constVelMode = false; // always clear constant velocity mode if constant position mode is active
            } else {
                hal.console->printf("EKF is using GPS\n");
                PV_AidingMode = AID_ABSOLUTE; // we have GPS data and can estimate all vehicle states
                posTimeout = false;
                velTimeout = false;
                constPosMode = false;
                constVelMode = false;
                // we need to reset the GPS timers to prevent GPS timeout logic being invoked on entry into GPS aiding
                // this is becasue the EKF can be interrupted for an arbitrary amount of time during vehicle arming checks
                lastTimeGpsReceived_ms = imuSampleTime_ms;
                secondLastGpsTime_ms = imuSampleTime_ms;
                // reset the last valid position fix time to prevent unwanted activation of GPS glitch logic
                lastPosPassTime = imuSampleTime_ms;
                // reset the fail time to prevent premature reporting of loss of position accruacy
                lastPosFailTime = 0;
            }
        }
        if (filterArmed) {
            // Reset filter position to GPS when transitioning into flight mode
            // We need to do this becasue the vehicle may have moved since the EKF origin was set
            ResetPosition();
        } else {
            // Reset all position and velocity states when transitioning out of flight mode
            // We need to do this becasue we are going into a mode that assumes zero position and velocity
            ResetVelocity();
            ResetPosition();
        }

    } else if (filterArmed && !firstMagYawInit && (stateStruct.position.z  - posDownAtArming) < -1.5f && !assume_zero_sideslip()) {
        // Do the first in-air yaw and earth mag field initialisation when the vehicle has gained 1.5m of altitude after arming if it is a non-fly forward vehicle (vertical takeoff)
        // This is done to prevent magnetic field distoration from steel roofs and adjacent structures causing bad earth field and initial yaw values
        Vector3f eulerAngles;
        getEulerAngles(eulerAngles);
        stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        StoreQuatReset();
        firstMagYawInit = true;
    } else if (filterArmed && !secondMagYawInit && (stateStruct.position.z - posDownAtArming) < -5.0f && !assume_zero_sideslip()) {
        // Do the second and final yaw and earth mag field initialisation when the vehicle has gained 5.0m of altitude after arming if it is a non-fly forward vehicle (vertical takeoff)
        // This second and final correction is needed for flight from large metal structures where the magnetic field distortion can extend up to 5m
        Vector3f eulerAngles;
        getEulerAngles(eulerAngles);
        stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        StoreQuatReset();
        secondMagYawInit = true;
    }

    // Always turn aiding off when the vehicle is disarmed
    if (!filterArmed) {
        PV_AidingMode = AID_NONE;
        posTimeout = true;
        velTimeout = true;
        // set constant position mode if aiding is inhibited
        constPosMode = true;
        constVelMode = false; // always clear constant velocity mode if constant position mode is active
        lastConstVelMode = false;
    }

}

// Set the NED origin to be used until the next filter reset
void NavEKF::setOrigin()
{
    // assume origin at current GPS location (no averaging)
    EKF_origin = _ahrs->get_gps().location();
    // define Earth rotation vector in the NED navigation frame at the origin
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);
    validOrigin = true;
    hal.console->printf("EKF Origin Set\n");
}

// return the LLH location of the filters NED origin
bool NavEKF::getOriginLLH(struct Location &loc) const
{
    if (validOrigin) {
        loc = EKF_origin;
    }
    return validOrigin;
}

// set the LLH location of the filters NED origin
bool NavEKF::setOriginLLH(struct Location &loc)
{
    if (filterArmed) {
        return false;
    }
    EKF_origin = loc;
    validOrigin = true;
    return true;
}

// determine if a takeoff is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF::getTakeoffExpected()
{
    if (expectGndEffectTakeoff && imuSampleTime_ms - takeoffExpectedSet_ms > gndEffectTimeout_ms) {
        expectGndEffectTakeoff = false;
    }

    return expectGndEffectTakeoff;
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF::setTakeoffExpected(bool val)
{
    takeoffExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTakeoff = val;
}


// determine if a touchdown is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF::getTouchdownExpected()
{
    if (expectGndEffectTouchdown && imuSampleTime_ms - touchdownExpectedSet_ms > gndEffectTimeout_ms) {
        expectGndEffectTouchdown = false;
    }

    return expectGndEffectTouchdown;
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF::setTouchdownExpected(bool val)
{
    touchdownExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTouchdown = val;
}

// Monitor GPS data to see if quality is good enough to initialise the EKF
// Monitor magnetometer innovations to to see if the heading is good enough to use GPS
// Return true if all criteria pass for 10 seconds
bool NavEKF::calcGpsGoodToAlign(void)
{
    // calculate absolute difference between GPS vert vel and inertial vert vel
    float velDiffAbs;
    if (_ahrs->get_gps().have_vertical_velocity()) {
        velDiffAbs = fabsf(gpsDataDelayed.vel.z - stateStruct.velocity.z);
    } else {
        velDiffAbs = 0.0f;
    }
    // fail if velocity difference or reported speed accuracy greater than threshold
    bool gpsVelFail = (velDiffAbs > 1.0f) || (gpsSpdAccuracy > 1.0f);
    // fail if not enough sats
    bool numSatsFail = _ahrs->get_gps().num_sats() < 6;
    // fail if horiziontal position accuracy not sufficient
    float hAcc = 0.0f;
    bool hAccFail;
    if (_ahrs->get_gps().horizontal_accuracy(hAcc)) {
        hAccFail = hAcc > 5.0f;
    } else {
        hAccFail =  false;
    }
    // fail if magnetometer innovations are outside limits indicating bad yaw
    // with bad yaw we are unable to use GPS
    bool yawFail;
    if (magTestRatio.x > 1.0f || magTestRatio.y > 1.0f) {
        yawFail = true;
    } else {
        yawFail = false;
    }
    // record time of fail
    // assume  fail first time called
    if (gpsVelFail || numSatsFail || hAccFail || yawFail || lastGpsVelFail_ms == 0) {
        lastGpsVelFail_ms = imuSampleTime_ms;
    }
    // continuous period without fail required to return healthy
    if (imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
        return true;
    } else {
        return false;
    }
}

// Read the range finder and take new measurements if available
// Read at 20Hz and apply a median filter
void NavEKF::readRangeFinder(void)
{
    static float storedRngMeas[3];
    static uint32_t storedRngMeasTime_ms[3];
    static uint32_t lastRngMeasTime_ms = 0;
    static uint8_t rngMeasIndex = 0;
    uint8_t midIndex;
    uint8_t maxIndex;
    uint8_t minIndex;
    // get theoretical correct range when the vehicle is on the ground
    rngOnGnd = _rng.ground_clearance_cm() * 0.01f;
    if (_rng.status() == RangeFinder::RangeFinder_Good && (imuSampleTime_ms - lastRngMeasTime_ms) > 50) {
        // store samples and sample time into a ring buffer
        rngMeasIndex ++;
        if (rngMeasIndex > 2) {
            rngMeasIndex = 0;
        }
        storedRngMeasTime_ms[rngMeasIndex] = imuSampleTime_ms;
        storedRngMeas[rngMeasIndex] = _rng.distance_cm() * 0.01f;
        // check for three fresh samples and take median
        bool sampleFresh[3];
        for (uint8_t index = 0; index <= 2; index++) {
            sampleFresh[index] = (imuSampleTime_ms - storedRngMeasTime_ms[index]) < 500;
        }
        if (sampleFresh[0] && sampleFresh[1] && sampleFresh[2]) {
            if (storedRngMeas[0] > storedRngMeas[1]) {
                minIndex = 1;
                maxIndex = 0;
            } else {
                maxIndex = 0;
                minIndex = 1;
            }
            if (storedRngMeas[2] > storedRngMeas[maxIndex]) {
                midIndex = maxIndex;
            } else if (storedRngMeas[2] < storedRngMeas[minIndex]) {
                midIndex = minIndex;
            } else {
                midIndex = 2;
            }
            rngMea = max(storedRngMeas[midIndex],rngOnGnd);
            newDataRng = true;
            rngValidMeaTime_ms = imuSampleTime_ms;
        } else if (!filterArmed) {
            // if not armed and no return, we assume on ground range
            rngMea = rngOnGnd;
            newDataRng = true;
            rngValidMeaTime_ms = imuSampleTime_ms;
        } else {
            newDataRng = false;
        }
        lastRngMeasTime_ms =  imuSampleTime_ms;
    }
}

// Detect takeoff for optical flow navigation
void NavEKF::detectOptFlowTakeoff(void)
{
    if (filterArmed && !takeOffDetected && (imuSampleTime_ms - timeAtArming_ms) > 1000) {
        const AP_InertialSensor &ins = _ahrs->get_ins();
        Vector3f angRateVec;
        Vector3f gyroBias;
        getGyroBias(gyroBias);
        bool dual_ins = ins.get_gyro_health(0) && ins.get_gyro_health(1);
        if (dual_ins) {
                angRateVec = (ins.get_gyro(0) + ins.get_gyro(1)) * 0.5f - gyroBias;
        } else {
                angRateVec = ins.get_gyro() - gyroBias;
        }

        takeOffDetected = (takeOffDetected || (angRateVec.length() > 0.1f) || (rngMea > (rangeAtArming + 0.1f)));
    }
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool NavEKF::getHeightControlLimit(float &height) const
{
    // only ask for limiting if we are doing optical flow navigation
    if (_fusionModeGPS == 3) {
        // If are doing optical flow nav, ensure the height above ground is within range finder limits after accounting for vehicle tilt and control errors
        height = max(float(_rng.max_distance_cm()) * 0.007f - 1.0f, 1.0f);
        return true;
    } else {
        return false;
    }
}

// provides the delta quaternion that was used by the INS calculation to rotate from the previous orientation to the orientation at the current time
// the delta quaternion returned will be a zero rotation if the INS calculation was not performed on that time step
Quaternion NavEKF::getDeltaQuaternion(void) const
{
    // Note: correctedDelAngQuat is reset to a zero rotation at the start of every update cycle in UpdateFilter()
    return correctedDelAngQuat;
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
void NavEKF::getQuaternion(Quaternion& ret) const
{
    ret = outputDataNew.quat;
}

// align the NE earth magnetic field states with the published declination
void NavEKF::alignMagStateDeclination()
{
    // get the magnetic declination
    float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // rotate the NE values so that the declination matches the published value
    Vector3f initMagNED = stateStruct.earth_magfield;
    float magLengthNE = pythagorous2(initMagNED.x,initMagNED.y);
    stateStruct.earth_magfield.x = magLengthNE * cosf(magDecAng);
    stateStruct.earth_magfield.y = magLengthNE * sinf(magDecAng);
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns true if a reset yaw angle has been updated and not queried
// this function should not have more than one client
bool NavEKF::getLastYawResetAngle(float &yawAng)
{
    if (yawResetAngleWaiting) {
        yawAng = yawResetAngle;
        yawResetAngleWaiting = false;
        return true;
    } else {
        yawAng = yawResetAngle;
        return false;
    }
}

// correct the quaternion using an attitude error vector
void NavEKF::correctQuatStates(Vector3f &errVec)
{
    // Convert the error rotation vector to its equivalent quaternion where
    // truth = estimate + error
    float rotationMag = errVec.length();
    if (rotationMag > 1e-6f) {
        Quaternion deltaQuat;
        float temp = sinf(0.5f*rotationMag) / rotationMag;
        deltaQuat[0] = cosf(0.5f*rotationMag);
        deltaQuat[1] = errVec.x*temp;
        deltaQuat[2] = errVec.y*temp;
        deltaQuat[3] = errVec.z*temp;

        // Update the quaternion states by rotating from the previous attitude through the error quaternion
        stateStruct.quat *= deltaQuat;

        // re-normalise the quaternion
        stateStruct.quat.normalize();
    }
}

// Fuse compass measurements usinga simple declination observation model that doesn't use magnetic field states
void NavEKF::fuseCompass()
{
    float q0 = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];

    float magX = magDataDelayed.mag.x;
    float magY = magDataDelayed.mag.y;
    float magZ = magDataDelayed.mag.z;

    // compass measurement error variance (rad^2)
    const float R_MAG = 3e-2f;

    // Calculate observation Jacobian
    float t2 = q0*q0;
    float t3 = q1*q1;
    float t4 = q2*q2;
    float t5 = q3*q3;
    float t6 = q0*q2*2.0f;
    float t7 = q1*q3*2.0f;
    float t8 = t6+t7;
    float t9 = q0*q3*2.0f;
    float t13 = q1*q2*2.0f;
    float t10 = t9-t13;
    float t11 = t2+t3-t4-t5;
    float t12 = magX*t11;
    float t14 = magZ*t8;
    float t19 = magY*t10;
    float t15 = t12+t14-t19;
    float t16 = t2-t3+t4-t5;
    float t17 = q0*q1*2.0f;
    float t24 = q2*q3*2.0f;
    float t18 = t17-t24;
    float t20 = 1.0f/t15;
    float t21 = magY*t16;
    float t22 = t9+t13;
    float t23 = magX*t22;
    float t28 = magZ*t18;
    float t25 = t21+t23-t28;
    float t29 = t20*t25;
    float t26 = tan(t29);
    float t27 = 1.0f/(t15*t15);
    float t30 = t26*t26;
    float t31 = t30+1.0f;
    float H_MAG[3];
    H_MAG[0] = -t31*(t20*(magZ*t16+magY*t18)+t25*t27*(magY*t8+magZ*t10));
    H_MAG[1] = t31*(t20*(magX*t18+magZ*t22)+t25*t27*(magX*t8-magZ*t11));
    H_MAG[2] = t31*(t20*(magX*t16-magY*t22)+t25*t27*(magX*t10+magY*t11));

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
    float PH[3];
    float varInnov = R_MAG;
    for (uint8_t rowIndex=0;rowIndex<=2;rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0;colIndex<=2;colIndex++) {
            PH[rowIndex] += P[rowIndex][colIndex]*H_MAG[colIndex];
        }
        varInnov += H_MAG[rowIndex]*PH[rowIndex];
    }
    float varInnovInv = 1.0f / varInnov;
    for (uint8_t rowIndex=0;rowIndex<=stateIndexLim;rowIndex++) {
        Kfusion[rowIndex] = 0.0f;
        for (uint8_t colIndex=0;colIndex<=2;colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_MAG[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }

    // Calculate the innovation
    float innovation = calcMagHeadingInnov();

    // Copy raw value to output variable used for data logging
    innovYaw = innovation;

    // limit the innovation so that initial corrections are not too large
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the state vector
    stateStruct.angErr.zero();
    for (uint8_t i=0;i<=stateIndexLim;i++) {
        statesArray[i] -= Kfusion[i] * innovation;
    }

    // the first 3 states represent the angular misalignment vector. This is
    // is used to correct the estimated quaternion on the current time step
    correctQuatStates(stateStruct.angErr);

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    float HP[24];
    for (uint8_t colIndex=0;colIndex<=stateIndexLim;colIndex++) {
        HP[colIndex] = 0.0f;
        for (uint8_t rowIndex=0;rowIndex<=2;rowIndex++) {
            HP[colIndex] += H_MAG[rowIndex]*P[rowIndex][colIndex];
        }
    }
    for (uint8_t rowIndex=0;rowIndex<=stateIndexLim;rowIndex++) {
        for (uint8_t colIndex=0;colIndex<=stateIndexLim;colIndex++) {
            P[rowIndex][colIndex] -= Kfusion[rowIndex] * HP[colIndex];
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();
}

// Calculate magnetic heading innovation
float NavEKF::calcMagHeadingInnov()
{
    // rotate predicted earth components into body axes and calculate
    // predicted measurements
    Matrix3f Tbn_temp;
    stateStruct.quat.rotation_matrix(Tbn_temp);
    Vector3f magMeasNED = Tbn_temp*magDataDelayed.mag;

    // calculate the innovation where the predicted measurement is the angle wrt magnetic north of the horizontal component of the measured field
    float innovation = atan2f(magMeasNED.y,magMeasNED.x) - _ahrs->get_compass()->get_declination();

    // wrap the innovation so it sits on the range from +-pi
    if (innovation > 3.1415927f) {
        innovation = innovation - 6.2831853f;
    } else if (innovation < -3.1415927f) {
        innovation = innovation + 6.2831853f;
    }

    return innovation;
}

#endif // HAL_CPU_CLASS
