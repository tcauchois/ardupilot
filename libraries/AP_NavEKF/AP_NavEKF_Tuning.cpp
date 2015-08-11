/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF_Tuning.h"
#include <AP_Vehicle.h>

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
#define GYRO_PNOISE_DEFAULT     0.01f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    1E-05f
#define ABIAS_PNOISE_DEFAULT    0.00005f
#define MAG_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        5
#define POS_GATE_DEFAULT        5
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         3
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
#define GYRO_PNOISE_DEFAULT     0.01f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    8E-06f
#define ABIAS_PNOISE_DEFAULT    0.00005f
#define MAG_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        5
#define POS_GATE_DEFAULT        5
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         2
#define GLITCH_RADIUS_DEFAULT   25
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
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         25
#define FLOW_NOISE_DEFAULT      0.3f
#define FLOW_GATE_DEFAULT       3

#endif // APM_BUILD_DIRECTORY

// Define tuning parameters
const AP_Param::GroupInfo NavEKF_Tuning::var_info[] PROGMEM = {

    // @Param: VELNE_NOISE
    // @DisplayName: GPS horizontal velocity measurement noise scaler
    // @Description: This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("VELNE_NOISE",    0, NavEKF_Tuning, _gpsHorizVelNoise, VELNE_NOISE_DEFAULT),

    // @Param: VELD_NOISE
    // @DisplayName: GPS vertical velocity measurement noise scaler
    // @Description: This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the weighting on this measurement.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("VELD_NOISE",    1, NavEKF_Tuning, _gpsVertVelNoise, VELD_NOISE_DEFAULT),

    // @Param: POSNE_NOISE
    // @DisplayName: GPS horizontal position measurement noise (m)
    // @Description: This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("POSNE_NOISE",    2, NavEKF_Tuning, _gpsHorizPosNoise, POSNE_NOISE_DEFAULT),

    // @Param: ALT_NOISE
    // @DisplayName: Altitude measurement noise (m)
    // @Description: This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("ALT_NOISE",    3, NavEKF_Tuning, _baroAltNoise, ALT_NOISE_DEFAULT),

    // @Param: MAG_NOISE
    // @DisplayName: Magnetometer measurement noise (Gauss)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MAG_NOISE",    4, NavEKF_Tuning, _magNoise, MAG_NOISE_DEFAULT),

    // @Param: EAS_NOISE
    // @DisplayName: Equivalent airspeed measurement noise (m/s)
    // @Description: This is the RMS value of noise in equivalent airspeed measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("EAS_NOISE",    5, NavEKF_Tuning, _easNoise, 1.4f),

    // @Param: WIND_PNOISE
    // @DisplayName: Wind velocity process noise (m/s^2)
    // @Description: This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
    // @Range: 0.01 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PNOISE",    6, NavEKF_Tuning, _windVelProcessNoise, 0.1f),

    // @Param: WIND_PSCALE
    // @DisplayName: Height rate to wind procss noise scaler
    // @Description: Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
    // @Range: 0.0 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PSCALE",    7, NavEKF_Tuning, _wndVarHgtRateScale, 0.5f),

    // @Param: GYRO_PNOISE
    // @DisplayName: Rate gyro noise (rad/s)
    // @Description: This noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
    // @Range: 0.001 0.05
    // @Increment: 0.001
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("GYRO_PNOISE",    8, NavEKF_Tuning, _gyrNoise, GYRO_PNOISE_DEFAULT),

    // @Param: ACC_PNOISE
    // @DisplayName: Accelerometer noise (m/s^2)
    // @Description: This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
    // @Range: 0.05 1.0
    // @Increment: 0.01
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ACC_PNOISE",    9, NavEKF_Tuning, _accNoise, ACC_PNOISE_DEFAULT),

    // @Param: GBIAS_PNOISE
    // @DisplayName: Rate gyro bias process noise (rad/s)
    // @Description: This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier.
    // @Range: 0.0000001 0.00001
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("GBIAS_PNOISE",    10, NavEKF_Tuning, _gyroBiasProcessNoise, GBIAS_PNOISE_DEFAULT),

    // @Param: ABIAS_PNOISE
    // @DisplayName: Accelerometer bias process noise (m/s^2)
    // @Description: This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
    // @Range: 0.00001 0.001
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ABIAS_PNOISE",    11, NavEKF_Tuning, _accelBiasProcessNoise, ABIAS_PNOISE_DEFAULT),

    // @Param: MAG_PNOISE
    // @DisplayName: Magnetic field process noise (gauss/s)
    // @Description: This noise controls the growth of magnetic field state error estimates. Increasing it makes magnetic field bias estimation faster and noisier.
    // @Range: 0.0001 0.01
    // @User: Advanced
    // @Units: gauss/s
    AP_GROUPINFO("MAG_PNOISE",    12, NavEKF_Tuning, _magProcessNoise, MAG_PNOISE_DEFAULT),

    // @Param: GSCL_PNOISE
    // @DisplayName: Rate gyro scale factor process noise (1/s)
    // @Description: This noise controls the rate of gyro scale factor learning. Increasing it makes rate gyro scale factor estimation faster and noisier.
    // @Range: 0.0000001 0.00001
    // @User: Advanced
    // @Units: 1/s
   AP_GROUPINFO("GSCL_PNOISE",    13, NavEKF_Tuning, _gyroScaleProcessNoise, 1e-6f),

    // @Param: GPS_DELAY
    // @DisplayName: GPS measurement delay (msec)
    // @Description: This is the number of msec that the GPS measurements lag behind the inertial measurements.
    // @Range: 0 500
    // @Increment: 10
    // @User: Advanced
    // @Units: milliseconds
    AP_GROUPINFO("VEL_DELAY",    14, NavEKF_Tuning, _msecGpsDelay, 220),

    // this slot has been deprecated and reserved for later use

    // @Param: GPS_TYPE
    // @DisplayName: GPS mode control
    // @Description: This parameter controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
    // @Values: 0:GPS 3D Vel and 2D Pos, 1:GPS 2D vel and 2D pos, 2:GPS 2D pos, 3:No GPS use optical flow
    // @User: Advanced
    AP_GROUPINFO("GPS_TYPE",    16, NavEKF_Tuning, _fusionModeGPS, 0),

    // @Param: VEL_GATE
    // @DisplayName: GPS velocity measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VEL_GATE",    17, NavEKF_Tuning, _gpsVelInnovGate, VEL_GATE_DEFAULT),

    // @Param: POS_GATE
    // @DisplayName: GPS position measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POS_GATE",    18, NavEKF_Tuning, _gpsPosInnovGate, POS_GATE_DEFAULT),

    // @Param: HGT_GATE
    // @DisplayName: Height measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("HGT_GATE",    19, NavEKF_Tuning, _hgtInnovGate, HGT_GATE_DEFAULT),

    // @Param: MAG_GATE
    // @DisplayName: Magnetometer measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAG_GATE",    20, NavEKF_Tuning, _magInnovGate, MAG_GATE_DEFAULT),

    // @Param: EAS_GATE
    // @DisplayName: Airspeed measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EAS_GATE",    21, NavEKF_Tuning, _tasInnovGate, 10),

    // @Param: MAG_CAL
    // @DisplayName: Magnetometer calibration mode
    // @Description: EKF_MAG_CAL = 0 enables calibration based on flying speed and altitude and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition and is recommended if in-flight magnetometer calibration is unreliable.
    // @Values: 0:Speed and Height,1:Acceleration,2:Never,3:Always
    // @User: Advanced
    AP_GROUPINFO("MAG_CAL",    22, NavEKF_Tuning, _magCal, MAG_CAL_DEFAULT),

    // this slot has been deprecated and reserved for later use

    // @Param: GLITCH_RAD
    // @DisplayName: GPS glitch radius gate size (m)
    // @Description: This parameter controls the maximum amount of difference in horizontal position (in m) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and an offset is applied to the GPS measurement to compensate. Position steps smaller than this value will be temporarily ignored, but will then be accepted and the filter will move to the new position. Position steps larger than this value will be ignored initially, but the filter will then apply an offset to the GPS position measurement.
    // @Range: 10 50
    // @Increment: 5
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("GLITCH_RAD",    24, NavEKF_Tuning, _gpsGlitchRadiusMax, GLITCH_RADIUS_DEFAULT),

    // @Param: GND_GRADIENT
    // @DisplayName: Terrain Gradient % RMS
    // @Description: This parameter sets the RMS terrain gradient percentage assumed by the terrain height estimation. Terrain height can be estimated using optical flow and/or range finder sensor data if fitted. Smaller values cause the terrain height estimate to be slower to respond to changes in measurement. Larger values casue the terrain height estimate to be faster to respond, but also more noisy. Generally this value can be reduced if operating over very flat terrain and increased if operating over uneven terrain.
    // @Range: 1 - 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GND_GRADIENT",    25, NavEKF_Tuning, _gndGradientSigma, 2),

    // @Param: FLOW_NOISE
    // @DisplayName: Optical flow measurement noise (rad/s)
    // @Description: This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 - 1.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("FLOW_NOISE",    26, NavEKF_Tuning, _flowNoise, FLOW_NOISE_DEFAULT),

    // @Param: FLOW_GATE
    // @DisplayName: Optical Flow measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLOW_GATE",    27, NavEKF_Tuning, _flowInnovGate, FLOW_GATE_DEFAULT),

    // @Param: FLOW_DELAY
    // @DisplayName: Optical Flow measurement delay (msec)
    // @Description: This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
    // @Range: 0 - 500
    // @Increment: 10
    // @User: Advanced
    // @Units: milliseconds
    AP_GROUPINFO("FLOW_DELAY",    28, NavEKF_Tuning, _msecFlowDelay, FLOW_MEAS_DELAY),

    // @Param: RNG_GATE
    // @DisplayName: Range finder measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RNG_GATE",    29, NavEKF_Tuning, _rngInnovGate, 5),

    // @Param: MAX_FLOW
    // @DisplayName: Maximum valid optical flow rate
    // @Description: This parameter sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
    // @Range: 1.0 - 4.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("MAX_FLOW",    30, NavEKF_Tuning, _maxFlowRate, 2.5f),

    // @Param: FALLBACK
    // @DisplayName: Fallback strictness
    // @Description: This parameter controls the conditions necessary to trigger a fallback to DCM and INAV. A value of 1 will cause fallbacks to occur on loss of GPS and other conditions. A value of 0 will trust the EKF more.
    // @Values: 0:Trust EKF more, 1:Trust DCM more
    // @User: Advanced
    AP_GROUPINFO("FALLBACK",    31, NavEKF_Tuning, _fallback, 1),

    // @Param: ALT_SOURCE
    // @DisplayName: Primary height source
    // @Description: This parameter controls which height sensor is used by the EKF during optical flow navigation (when EKF_GPS_TYPE = 3). A value of will 0 cause it to always use baro altitude. A value of 1 will casue it to use range finder if available.
    // @Values: 0:Use Baro, 1:Use Range Finder
    // @User: Advanced
    AP_GROUPINFO("ALT_SOURCE",    32, NavEKF_Tuning, _altSource, 1),


    AP_GROUPEND
};

NavEKF_Tuning::NavEKF_Tuning() :
    gpsNEVelVarAccScale(0.05f),     // Scale factor applied to horizontal velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsDVelVarAccScale(0.07f),      // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsPosVarAccScale(0.05f),       // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    msecHgtDelay(60),               // Height measurement delay (msec)
    msecMagDelay(60),               // Magnetometer measurement delay (msec)
    msecTasDelay(240),              // Airspeed measurement delay (msec)
    gpsRetryTimeUseTAS(10000),      // GPS retry time with airspeed measurements (msec)
    gpsRetryTimeNoTAS(7000),        // GPS retry time without airspeed measurements (msec)
    gpsFailTimeWithFlow(1000),      // If we have no GPS for longer than this and we have optical flow, then we will switch across to using optical flow (msec)
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
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif //HAL_CPU_CLASS
