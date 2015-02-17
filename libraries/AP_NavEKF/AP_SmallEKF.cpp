/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

// uncomment this to force the optimisation of this code, note that
// this makes debugging harder
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#pragma GCC optimize("O0")
#else
#pragma GCC optimize("O3")
#endif

#include "AP_SmallEKF.h"
#include <AP_AHRS.h>
#include <AP_Param.h>
#include <AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


// Define tuning parameters
const AP_Param::GroupInfo SmallEKF::var_info[] PROGMEM = {
    AP_GROUPEND
};

// constructor
SmallEKF::SmallEKF(const AP_AHRS_NavEKF &ahrs) :
    _ahrs(ahrs),
    _main_ekf(ahrs.get_NavEKF_const()),
    state(*reinterpret_cast<struct state_elements *>(&states)),
    FiltInit(false),
    lastMagUpdate(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// run a 9-state EKF used to calculate orientation
void SmallEKF::RunEKF(float delta_time, const Vector3f &delta_angles, const Vector3f &delta_velocity, const Vector3f &joint_angles)
{
    imuSampleTime_ms = hal.scheduler->millis();
    dtIMU = delta_time;

    // initialise variables and constants
    if (!FiltInit) {
        StartTime_ms =  imuSampleTime_ms;
        newDataMag = false;
        YawAligned = false;
        state.quat[0] = 1.0f;
        const float Sigma_velNED = 0.5f; // 1 sigma uncertainty in horizontal velocity components
        const float Sigma_dAngBias  = 0.01745f*dtIMU; // 1 Sigma uncertainty in delta angle bias
        const float Sigma_dVelBias  = 0.1f*dtIMU; // 1 Sigma uncertainty in delta velocity bias
        const float Sigma_angErr = 1.0f; // 1 Sigma uncertainty in angular misalignment (rad)
        for (uint8_t i=0; i <= 2; i++) Cov[i][i] = sq(Sigma_angErr);
        for (uint8_t i=3; i <= 5; i++) Cov[i][i] = sq(Sigma_velNED);
        for (uint8_t i=6; i <= 8; i++) Cov[i][i] = sq(Sigma_dAngBias);
        for (uint8_t i=9; i <= 11; i++) Cov[i][i] = sq(Sigma_dVelBias);
        FiltInit = true;
        hal.console->printf("SmallEKF Alignment Started\n");
    }

    // We are using the IMU data from the flight vehicle and setting joint angles to zero for the time being
    gSense.gPsi = joint_angles.z; // yaw
    gSense.gPhi = joint_angles.x; // roll
    gSense.gTheta = joint_angles.y; // pitch
    cosPhi = cosf(gSense.gPhi);
    cosTheta = cosf(gSense.gTheta);
    sinPhi = sinf(gSense.gPhi);
    sinTheta = sinf(gSense.gTheta);
    sinPsi = sinf(gSense.gPsi);
    cosPsi = cosf(gSense.gPsi);
    gSense.delAng = delta_angles;
    gSense.delVel = delta_velocity;

    // predict states
    predictStates();

    // predict the covariance
    predictCovariance();

    // fuse SmallEKF velocity data
    fuseVelocity(YawAligned);

    // Align the heading once there has been enough time for the filter to settle and the tilt corrections have dropped below a threshold
    if ((((imuSampleTime_ms - StartTime_ms) > 5000 && TiltCorrection < 1e-4f) || ((imuSampleTime_ms - StartTime_ms) > 30000)) && !YawAligned) {
        //calculate the initial heading using magnetometer, estimated tilt and declination
        alignHeading();
        YawAligned = true;
        hal.console->printf("SmallEKF Alignment Completed\n");
    }

    // Fuse magnetometer data if  we have new measurements and an aligned heading
    readMagData();
    if (newDataMag && YawAligned) {
        fuseCompass();
        newDataMag = false;
    }

}

// state prediction
void SmallEKF::predictStates()
{
    static Vector3f gimDelAngCorrected;
    static Vector3f gimDelAngPrev;

    // NED gravity vector m/s^2
    const Vector3f gravityNED(0, 0, GRAVITY_MSS);

    // apply corrections for bias and coning errors
    // % * - and + operators have been overloaded
    gimDelAngCorrected   = gSense.delAng - state.delAngBias - (gimDelAngPrev % gimDelAngCorrected) * 8.333333e-2f;
    gimDelAngPrev        = gSense.delAng - state.delAngBias;

    // convert the rotation vector to its equivalent quaternion
    float rotationMag = gimDelAngCorrected.length();
    Quaternion deltaQuat;
    if (rotationMag < 1e-6f)
    {
        deltaQuat[0] = 1.0f;
        deltaQuat[1] = 0.0f;
        deltaQuat[2] = 0.0f;
        deltaQuat[3] = 0.0f;
    }
    else
    {
        deltaQuat[0] = cosf(0.5f * rotationMag);
        float rotScaler = (sinf(0.5f * rotationMag)) / rotationMag;
        deltaQuat[1] = gimDelAngCorrected.x * rotScaler;
        deltaQuat[2] = gimDelAngCorrected.y * rotScaler;
        deltaQuat[3] = gimDelAngCorrected.z * rotScaler;
    }

    // update the quaternions by rotating from the previous attitude through
    // the delta angle rotation quaternion
    state.quat *= deltaQuat;

    // normalise the quaternions and update the quaternion states
    state.quat.normalize();

    // calculate the sensor to NED cosine matrix
    state.quat.rotation_matrix(Tsn);

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    Vector3f delVelNav  = Tsn*(gSense.delVel - state.delVelBias) + gravityNED*dtIMU;

    // sum delta velocities to get velocity
    state.velocity += delVelNav;
}

// covariance prediction using optimised algebraic toolbox expressions
// equivalent to P = F*P*transpose(P) + G*imu_errors*transpose(G) +
// gyro_bias_state_noise
void SmallEKF::predictCovariance()
{
    float delAngBiasVariance = sq(dtIMU*dtIMU*5E-4f);
    float delVelBiasVariance = sq(dtIMU*dtIMU*1E-3f);

    float daxNoise = sq(dtIMU*0.0087f);
    float dayNoise = sq(dtIMU*0.0087f);
    float dazNoise = sq(dtIMU*0.0087f);

    float dvxNoise = sq(dtIMU*0.5f);
    float dvyNoise = sq(dtIMU*0.5f);
    float dvzNoise = sq(dtIMU*0.5f);
    float dvx = gSense.delVel.x;
    float dvy = gSense.delVel.y;
    float dvz = gSense.delVel.z;
    float dax = gSense.delAng.x;
    float day = gSense.delAng.y;
    float daz = gSense.delAng.z;
    float q0 = state.quat[0];
    float q1 = state.quat[1];
    float q2 = state.quat[2];
    float q3 = state.quat[3];
    float dax_b = state.delAngBias.x;
    float day_b = state.delAngBias.y;
    float daz_b = state.delAngBias.z;
    float dvx_b = state.delVelBias.x;
    float dvy_b = state.delVelBias.y;
    float dvz_b = state.delVelBias.z;

    float t2 = dax*0.5f;
    float t3 = dax_b*0.5f;
    float t4 = t2-t3;
    float t5 = day*0.5f;
    float t6 = day_b*0.5f;
    float t7 = t5-t6;
    float t8 = daz*0.5f;
    float t9 = daz_b*0.5f;
    float t10 = t8-t9;
    float t11 = q2*t4*0.5f;
    float t12 = q1*t7*0.5f;
    float t13 = q0*t10*0.5f;
    float t14 = q2*0.5f;
    float t15 = q3*t4*0.5f;
    float t16 = q1*t10*0.5f;
    float t17 = q1*0.5f;
    float t18 = q0*t4*0.5f;
    float t19 = q3*t7*0.5f;
    float t20 = q0*0.5f;
    float t21 = q2*t7*0.5f;
    float t22 = q3*t10*0.5f;
    float t23 = q0*t7*0.5f;
    float t24 = q3*0.5f;
    float t25 = q1*t4*0.5f;
    float t26 = q2*t10*0.5f;
    float t27 = t11+t12+t13-t24;
    float t28 = t14+t15+t16-t23;
    float t29 = q2*t28*2.0f;
    float t30 = t17+t18+t19-t26;
    float t31 = q1*t30*2.0f;
    float t32 = t20+t21+t22-t25;
    float t33 = q0*t32*2.0f;
    float t40 = q3*t27*2.0f;
    float t34 = t29+t31+t33-t40;
    float t35 = q0*q0;
    float t36 = q1*q1;
    float t37 = q2*q2;
    float t38 = q3*q3;
    float t39 = t35+t36+t37+t38;
    float t41 = t11+t12-t13+t24;
    float t42 = t14-t15+t16+t23;
    float t43 = q1*t42*2.0f;
    float t44 = -t17+t18+t19+t26;
    float t45 = q2*t44*2.0f;
    float t46 = t20-t21+t22+t25;
    float t47 = q3*t46*2.0f;
    float t57 = q0*t41*2.0f;
    float t48 = t43+t45+t47-t57;
    float t49 = -t14+t15+t16+t23;
    float t50 = q0*t49*2.0f;
    float t51 = t11-t12+t13+t24;
    float t52 = t20+t21-t22+t25;
    float t53 = q2*t52*2.0f;
    float t54 = t17-t18+t19+t26;
    float t55 = q3*t54*2.0f;
    float t58 = q1*t51*2.0f;
    float t56 = t50+t53+t55-t58;
    float t59 = Cov[0][0]*t34;
    float t60 = Cov[1][0]*t48;
    float t66 = Cov[6][0]*t39;
    float t67 = Cov[2][0]*t56;
    float t61 = t59+t60-t66-t67;
    float t62 = Cov[0][1]*t34;
    float t63 = Cov[1][1]*t48;
    float t64 = Cov[0][2]*t34;
    float t65 = Cov[1][2]*t48;
    float t72 = Cov[6][1]*t39;
    float t73 = Cov[2][1]*t56;
    float t68 = t62+t63-t72-t73;
    float t69 = t35+t36-t37-t38;
    float t86 = Cov[6][2]*t39;
    float t87 = Cov[2][2]*t56;
    float t70 = t64+t65-t86-t87;
    float t71 = dvx-dvx_b;
    float t74 = q0*q3*2.0f;
    float t81 = q1*q2*2.0f;
    float t75 = t74-t81;
    float t76 = q0*q2*2.0f;
    float t77 = q1*q3*2.0f;
    float t78 = t76+t77;
    float t79 = dvy-dvy_b;
    float t80 = dvz-dvz_b;
    float t82 = Cov[0][10]*t34;
    float t83 = Cov[1][10]*t48;
    float t103 = Cov[6][10]*t39;
    float t104 = Cov[2][10]*t56;
    float t84 = t82+t83-t103-t104;
    float t85 = t35-t36+t37-t38;
    float t88 = t74+t81;
    float t89 = Cov[0][9]*t34;
    float t90 = Cov[1][9]*t48;
    float t100 = Cov[6][9]*t39;
    float t101 = Cov[2][9]*t56;
    float t91 = t89+t90-t100-t101;
    float t92 = q0*q1*2.0f;
    float t96 = q2*q3*2.0f;
    float t93 = t92-t96;
    float t94 = Cov[0][11]*t34;
    float t95 = Cov[1][11]*t48;
    float t114 = Cov[6][11]*t39;
    float t115 = Cov[2][11]*t56;
    float t97 = t94+t95-t114-t115;
    float t98 = t35-t36-t37+t38;
    float t99 = t76-t77;
    float t102 = t92+t96;
    float t105 = Cov[0][6]*t34;
    float t106 = Cov[1][6]*t48;
    float t411 = Cov[6][6]*t39;
    float t107 = t105+t106-t411-Cov[2][6]*t56;
    float t108 = Cov[0][7]*t34;
    float t109 = Cov[1][7]*t48;
    float t412 = Cov[6][7]*t39;
    float t110 = t108+t109-t412-Cov[2][7]*t56;
    float t111 = Cov[0][8]*t34;
    float t112 = Cov[1][8]*t48;
    float t413 = Cov[6][8]*t39;
    float t113 = t111+t112-t413-Cov[2][8]*t56;
    float t116 = q0*t27*2.0f;
    float t117 = q1*t28*2.0f;
    float t118 = q3*t32*2.0f;
    float t128 = q2*t30*2.0f;
    float t119 = t116+t117+t118-t128;
    float t120 = q0*t46*2.0f;
    float t121 = q2*t42*2.0f;
    float t122 = q3*t41*2.0f;
    float t129 = q1*t44*2.0f;
    float t123 = t120+t121+t122-t129;
    float t124 = q1*t52*2.0f;
    float t125 = q2*t51*2.0f;
    float t126 = q3*t49*2.0f;
    float t130 = q0*t54*2.0f;
    float t127 = t124+t125+t126-t130;
    float t131 = Cov[7][0]*t39;
    float t132 = Cov[0][0]*t119;
    float t141 = Cov[1][0]*t123;
    float t142 = Cov[2][0]*t127;
    float t133 = t131+t132-t141-t142;
    float t134 = Cov[7][1]*t39;
    float t135 = Cov[0][1]*t119;
    float t147 = Cov[1][1]*t123;
    float t148 = Cov[2][1]*t127;
    float t136 = t134+t135-t147-t148;
    float t137 = Cov[7][2]*t39;
    float t138 = Cov[0][2]*t119;
    float t153 = Cov[1][2]*t123;
    float t154 = Cov[2][2]*t127;
    float t139 = t137+t138-t153-t154;
    float t140 = t39*t39;
    float t143 = q1*t27*2.0f;
    float t144 = q2*t32*2.0f;
    float t145 = q3*t30*2.0f;
    float t204 = q0*t28*2.0f;
    float t146 = t143+t144+t145-t204;
    float t149 = q0*t44*2.0f;
    float t150 = q1*t46*2.0f;
    float t151 = q2*t41*2.0f;
    float t205 = q3*t42*2.0f;
    float t152 = t149+t150+t151-t205;
    float t155 = q0*t52*2.0f;
    float t156 = q1*t54*2.0f;
    float t157 = q3*t51*2.0f;
    float t206 = q2*t49*2.0f;
    float t158 = t155+t156+t157-t206;
    float t159 = t69*t79;
    float t160 = t71*t75;
    float t161 = t159+t160;
    float t162 = t69*t80;
    float t222 = t71*t78;
    float t163 = t162-t222;
    float t164 = t78*t79;
    float t165 = t75*t80;
    float t166 = t164+t165;
    float t167 = Cov[7][10]*t39;
    float t168 = Cov[0][10]*t119;
    float t193 = Cov[1][10]*t123;
    float t194 = Cov[2][10]*t127;
    float t169 = t167+t168-t193-t194;
    float t170 = t71*t85;
    float t226 = t79*t88;
    float t171 = t170-t226;
    float t172 = t80*t85;
    float t173 = t79*t93;
    float t174 = t172+t173;
    float t175 = Cov[7][9]*t39;
    float t176 = Cov[0][9]*t119;
    float t191 = Cov[1][9]*t123;
    float t192 = Cov[2][9]*t127;
    float t177 = t175+t176-t191-t192;
    float t178 = Cov[7][11]*t39;
    float t179 = Cov[0][11]*t119;
    float t184 = Cov[1][11]*t123;
    float t185 = Cov[2][11]*t127;
    float t180 = t178+t179-t184-t185;
    float t181 = t71*t93;
    float t182 = t80*t88;
    float t183 = t181+t182;
    float t186 = t71*t98;
    float t187 = t80*t99;
    float t188 = t186+t187;
    float t189 = t79*t98;
    float t233 = t80*t102;
    float t190 = t189-t233;
    float t195 = t71*t102;
    float t196 = t79*t99;
    float t197 = t195+t196;
    float t198 = Cov[7][6]*t39;
    float t199 = Cov[0][6]*t119;
    float t200 = Cov[7][7]*t39;
    float t201 = Cov[0][7]*t119;
    float t202 = Cov[7][8]*t39;
    float t203 = Cov[0][8]*t119;
    float t207 = Cov[8][0]*t39;
    float t208 = Cov[1][0]*t152;
    float t216 = Cov[0][0]*t146;
    float t217 = Cov[2][0]*t158;
    float t209 = t207+t208-t216-t217;
    float t210 = Cov[8][1]*t39;
    float t211 = Cov[1][1]*t152;
    float t218 = Cov[0][1]*t146;
    float t219 = Cov[2][1]*t158;
    float t212 = t210+t211-t218-t219;
    float t213 = Cov[8][2]*t39;
    float t214 = Cov[1][2]*t152;
    float t220 = Cov[0][2]*t146;
    float t221 = Cov[2][2]*t158;
    float t215 = t213+t214-t220-t221;
    float t223 = Cov[0][10]*t146;
    float t224 = Cov[2][10]*t158;
    float t236 = Cov[8][10]*t39;
    float t237 = Cov[1][10]*t152;
    float t225 = t223+t224-t236-t237;
    float t227 = Cov[0][9]*t146;
    float t228 = Cov[2][9]*t158;
    float t234 = Cov[8][9]*t39;
    float t235 = Cov[1][9]*t152;
    float t229 = t227+t228-t234-t235;
    float t230 = Cov[0][11]*t146;
    float t231 = Cov[2][11]*t158;
    float t244 = Cov[8][11]*t39;
    float t245 = Cov[1][11]*t152;
    float t232 = t230+t231-t244-t245;
    float t238 = Cov[8][6]*t39;
    float t239 = Cov[1][6]*t152;
    float t240 = Cov[8][7]*t39;
    float t241 = Cov[1][7]*t152;
    float t242 = Cov[8][8]*t39;
    float t243 = Cov[1][8]*t152;
    float t246 = Cov[1][0]*t163;
    float t247 = Cov[10][0]*t75;
    float t248 = Cov[0][0]*t166;
    float t258 = Cov[9][0]*t69;
    float t259 = Cov[2][0]*t161;
    float t260 = Cov[11][0]*t78;
    float t249 = Cov[3][0]+t246+t247+t248-t258-t259-t260;
    float t250 = Cov[1][1]*t163;
    float t251 = Cov[10][1]*t75;
    float t252 = Cov[0][1]*t166;
    float t261 = Cov[9][1]*t69;
    float t262 = Cov[2][1]*t161;
    float t263 = Cov[11][1]*t78;
    float t253 = Cov[3][1]+t250+t251+t252-t261-t262-t263;
    float t254 = Cov[1][2]*t163;
    float t255 = Cov[10][2]*t75;
    float t256 = Cov[0][2]*t166;
    float t264 = Cov[9][2]*t69;
    float t265 = Cov[2][2]*t161;
    float t266 = Cov[11][2]*t78;
    float t257 = Cov[3][2]+t254+t255+t256-t264-t265-t266;
    float t267 = Cov[1][9]*t163;
    float t268 = Cov[10][9]*t75;
    float t269 = Cov[0][9]*t166;
    float t279 = Cov[9][9]*t69;
    float t280 = Cov[2][9]*t161;
    float t281 = Cov[11][9]*t78;
    float t270 = Cov[3][9]+t267+t268+t269-t279-t280-t281;
    float t271 = Cov[1][11]*t163;
    float t272 = Cov[10][11]*t75;
    float t273 = Cov[0][11]*t166;
    float t285 = Cov[9][11]*t69;
    float t286 = Cov[2][11]*t161;
    float t287 = Cov[11][11]*t78;
    float t274 = Cov[3][11]+t271+t272+t273-t285-t286-t287;
    float t275 = Cov[1][10]*t163;
    float t276 = Cov[10][10]*t75;
    float t277 = Cov[0][10]*t166;
    float t282 = Cov[9][10]*t69;
    float t283 = Cov[2][10]*t161;
    float t284 = Cov[11][10]*t78;
    float t278 = Cov[3][10]+t275+t276+t277-t282-t283-t284;
    float t288 = Cov[1][6]*t163;
    float t289 = Cov[10][6]*t75;
    float t290 = Cov[0][6]*t166;
    float t291 = Cov[3][6]+t288+t289+t290-Cov[9][6]*t69-Cov[2][6]*t161-Cov[11][6]*t78;
    float t292 = Cov[1][7]*t163;
    float t293 = Cov[10][7]*t75;
    float t294 = Cov[0][7]*t166;
    float t295 = Cov[3][7]+t292+t293+t294-Cov[9][7]*t69-Cov[2][7]*t161-Cov[11][7]*t78;
    float t296 = Cov[1][8]*t163;
    float t297 = Cov[10][8]*t75;
    float t298 = Cov[0][8]*t166;
    float t299 = Cov[3][8]+t296+t297+t298-Cov[9][8]*t69-Cov[2][8]*t161-Cov[11][8]*t78;
    float t300 = Cov[2][0]*t171;
    float t301 = Cov[11][0]*t93;
    float t302 = Cov[1][0]*t183;
    float t312 = Cov[10][0]*t85;
    float t313 = Cov[0][0]*t174;
    float t314 = Cov[9][0]*t88;
    float t303 = Cov[4][0]+t300+t301+t302-t312-t313-t314;
    float t304 = Cov[2][1]*t171;
    float t305 = Cov[11][1]*t93;
    float t306 = Cov[1][1]*t183;
    float t315 = Cov[10][1]*t85;
    float t316 = Cov[0][1]*t174;
    float t317 = Cov[9][1]*t88;
    float t307 = Cov[4][1]+t304+t305+t306-t315-t316-t317;
    float t308 = Cov[2][2]*t171;
    float t309 = Cov[11][2]*t93;
    float t310 = Cov[1][2]*t183;
    float t318 = Cov[10][2]*t85;
    float t319 = Cov[0][2]*t174;
    float t320 = Cov[9][2]*t88;
    float t311 = Cov[4][2]+t308+t309+t310-t318-t319-t320;
    float t321 = dvxNoise*t69*t88;
    float t322 = Cov[2][9]*t171;
    float t323 = Cov[11][9]*t93;
    float t324 = Cov[1][9]*t183;
    float t334 = Cov[10][9]*t85;
    float t335 = Cov[0][9]*t174;
    float t336 = Cov[9][9]*t88;
    float t325 = Cov[4][9]+t322+t323+t324-t334-t335-t336;
    float t326 = Cov[2][11]*t171;
    float t327 = Cov[11][11]*t93;
    float t328 = Cov[1][11]*t183;
    float t340 = Cov[10][11]*t85;
    float t341 = Cov[0][11]*t174;
    float t342 = Cov[9][11]*t88;
    float t329 = Cov[4][11]+t326+t327+t328-t340-t341-t342;
    float t330 = Cov[2][10]*t171;
    float t331 = Cov[11][10]*t93;
    float t332 = Cov[1][10]*t183;
    float t337 = Cov[10][10]*t85;
    float t338 = Cov[0][10]*t174;
    float t339 = Cov[9][10]*t88;
    float t333 = Cov[4][10]+t330+t331+t332-t337-t338-t339;
    float t343 = Cov[2][6]*t171;
    float t344 = Cov[11][6]*t93;
    float t345 = Cov[1][6]*t183;
    float t346 = Cov[4][6]+t343+t344+t345-Cov[0][6]*t174-Cov[9][6]*t88-Cov[10][6]*t85;
    float t347 = Cov[2][7]*t171;
    float t348 = Cov[11][7]*t93;
    float t349 = Cov[1][7]*t183;
    float t350 = Cov[4][7]+t347+t348+t349-Cov[0][7]*t174-Cov[9][7]*t88-Cov[10][7]*t85;
    float t351 = Cov[2][8]*t171;
    float t352 = Cov[11][8]*t93;
    float t353 = Cov[1][8]*t183;
    float t354 = Cov[4][8]+t351+t352+t353-Cov[0][8]*t174-Cov[9][8]*t88-Cov[10][8]*t85;
    float t355 = Cov[0][0]*t190;
    float t356 = Cov[9][0]*t99;
    float t357 = Cov[2][0]*t197;
    float t367 = Cov[11][0]*t98;
    float t368 = Cov[1][0]*t188;
    float t369 = Cov[10][0]*t102;
    float t358 = Cov[5][0]+t355+t356+t357-t367-t368-t369;
    float t359 = Cov[0][1]*t190;
    float t360 = Cov[9][1]*t99;
    float t361 = Cov[2][1]*t197;
    float t370 = Cov[11][1]*t98;
    float t371 = Cov[1][1]*t188;
    float t372 = Cov[10][1]*t102;
    float t362 = Cov[5][1]+t359+t360+t361-t370-t371-t372;
    float t363 = Cov[0][2]*t190;
    float t364 = Cov[9][2]*t99;
    float t365 = Cov[2][2]*t197;
    float t373 = Cov[11][2]*t98;
    float t374 = Cov[1][2]*t188;
    float t375 = Cov[10][2]*t102;
    float t366 = Cov[5][2]+t363+t364+t365-t373-t374-t375;
    float t376 = dvzNoise*t78*t98;
    float t377 = Cov[0][9]*t190;
    float t378 = Cov[9][9]*t99;
    float t379 = Cov[2][9]*t197;
    float t390 = Cov[11][9]*t98;
    float t391 = Cov[1][9]*t188;
    float t392 = Cov[10][9]*t102;
    float t380 = Cov[5][9]+t377+t378+t379-t390-t391-t392;
    float t381 = Cov[0][11]*t190;
    float t382 = Cov[9][11]*t99;
    float t383 = Cov[2][11]*t197;
    float t396 = Cov[11][11]*t98;
    float t397 = Cov[1][11]*t188;
    float t398 = Cov[10][11]*t102;
    float t384 = Cov[5][11]+t381+t382+t383-t396-t397-t398;
    float t385 = Cov[0][10]*t190;
    float t386 = Cov[9][10]*t99;
    float t387 = Cov[2][10]*t197;
    float t393 = Cov[11][10]*t98;
    float t394 = Cov[1][10]*t188;
    float t395 = Cov[10][10]*t102;
    float t388 = Cov[5][10]+t385+t386+t387-t393-t394-t395;
    float t389 = dvyNoise*t85*t102;
    float t399 = Cov[0][6]*t190;
    float t400 = Cov[9][6]*t99;
    float t401 = Cov[2][6]*t197;
    float t402 = Cov[5][6]+t399+t400+t401-Cov[1][6]*t188-Cov[10][6]*t102-Cov[11][6]*t98;
    float t403 = Cov[0][7]*t190;
    float t404 = Cov[9][7]*t99;
    float t405 = Cov[2][7]*t197;
    float t406 = Cov[5][7]+t403+t404+t405-Cov[1][7]*t188-Cov[10][7]*t102-Cov[11][7]*t98;
    float t407 = Cov[0][8]*t190;
    float t408 = Cov[9][8]*t99;
    float t409 = Cov[2][8]*t197;
    float t410 = Cov[5][8]+t407+t408+t409-Cov[1][8]*t188-Cov[10][8]*t102-Cov[11][8]*t98;

    nextCov[0][0] = daxNoise*t140+t34*t61+t48*t68-t56*t70-t39*t107;
    nextCov[0][1] = -t39*t110-t61*t119+t68*t123+t70*t127;
    nextCov[0][2] = -t39*t113-t68*t152+t70*t158+t146*(t59+t60-t66-t67);
    nextCov[0][3] = Cov[0][3]*t34+Cov[1][3]*t48-Cov[2][3]*t56-Cov[6][3]*t39+t75*t84-t69*t91-t78*t97-t70*t161+t166*(t59+t60-t66-t67)+t163*(t62+t63-t72-t73);
    nextCov[0][4] = Cov[0][4]*t34+Cov[1][4]*t48-Cov[2][4]*t56-Cov[6][4]*t39-t84*t85-t88*t91+t93*t97-t61*t174+t70*t171+t183*(t62+t63-t72-t73);
    nextCov[0][5] = Cov[0][5]*t34+Cov[1][5]*t48-Cov[2][5]*t56-Cov[6][5]*t39-t84*t102-t97*t98+t61*t190-t68*t188+t99*(t89+t90-t100-t101)+t197*(t64+t65-t86-t87);
    nextCov[0][6] = t107;
    nextCov[0][7] = t110;
    nextCov[0][8] = t113;
    nextCov[0][9] = t91;
    nextCov[0][10] = t84;
    nextCov[0][11] = t97;
    nextCov[1][0] = -t34*t133-t48*t136+t56*t139+t39*(t198+t199-Cov[1][6]*t123-Cov[2][6]*t127);
    nextCov[1][1] = dayNoise*t140+t119*t133-t123*t136-t127*t139+t39*(t200+t201-Cov[1][7]*t123-Cov[2][7]*t127);
    nextCov[1][2] = -t133*t146+t136*t152-t139*t158+t39*(t202+t203-Cov[1][8]*t123-Cov[2][8]*t127);
    nextCov[1][3] = -Cov[7][3]*t39-Cov[0][3]*t119+Cov[1][3]*t123+Cov[2][3]*t127-t75*t169+t69*t177+t78*t180-t133*t166-t136*t163+t139*t161;
    nextCov[1][4] = -Cov[7][4]*t39-Cov[0][4]*t119+Cov[1][4]*t123+Cov[2][4]*t127+t85*t169+t88*t177-t93*t180+t133*t174-t139*t171-t136*t183;
    nextCov[1][5] = -Cov[7][5]*t39-Cov[0][5]*t119+Cov[1][5]*t123+Cov[2][5]*t127+t102*t169-t99*t177+t98*t180-t133*t190+t136*t188-t139*t197;
    nextCov[1][6] = -t198-t199+Cov[1][6]*t123+Cov[2][6]*t127;
    nextCov[1][7] = -t200-t201+Cov[1][7]*t123+Cov[2][7]*t127;
    nextCov[1][8] = -t202-t203+Cov[1][8]*t123+Cov[2][8]*t127;
    nextCov[1][9] = -t175-t176+t191+t192;
    nextCov[1][10] = -t167-t168+t193+t194;
    nextCov[1][11] = -t178-t179+t184+t185;
    nextCov[2][0] = -t34*t209-t48*t212+t56*t215+t39*(t238+t239-Cov[0][6]*t146-Cov[2][6]*t158);
    nextCov[2][1] = t119*t209-t123*t212-t127*t215+t39*(t240+t241-Cov[0][7]*t146-Cov[2][7]*t158);
    nextCov[2][2] = dazNoise*t140-t146*t209+t152*t212-t158*t215+t39*(t242+t243-Cov[0][8]*t146-Cov[2][8]*t158);
    nextCov[2][3] = -Cov[8][3]*t39+Cov[0][3]*t146-Cov[1][3]*t152+Cov[2][3]*t158-t69*t229+t75*t225-t78*t232-t163*t212-t166*t209+t161*t215;
    nextCov[2][4] = -Cov[8][4]*t39+Cov[0][4]*t146-Cov[1][4]*t152+Cov[2][4]*t158-t85*t225-t88*t229+t93*t232+t174*t209-t171*t215-t183*t212;
    nextCov[2][5] = -Cov[8][5]*t39+Cov[0][5]*t146-Cov[1][5]*t152+Cov[2][5]*t158-t102*t225-t98*t232-t190*t209+t188*t212-t197*t215+t99*(t227+t228-t234-t235);
    nextCov[2][6] = -t238-t239+Cov[0][6]*t146+Cov[2][6]*t158;
    nextCov[2][7] = -t240-t241+Cov[0][7]*t146+Cov[2][7]*t158;
    nextCov[2][8] = -t242-t243+Cov[0][8]*t146+Cov[2][8]*t158;
    nextCov[2][9] = t229;
    nextCov[2][10] = t225;
    nextCov[2][11] = t232;
    nextCov[3][0] = t34*t249+t48*t253-t56*t257-t39*t291;
    nextCov[3][1] = -t39*t295-t119*t249+t123*t253+t127*t257;
    nextCov[3][2] = -t39*t299+t146*t249-t152*t253+t158*t257;
    nextCov[3][3] = Cov[3][3]-Cov[9][3]*t69+Cov[0][3]*t166+Cov[1][3]*t163+Cov[10][3]*t75-Cov[2][3]*t161-Cov[11][3]*t78-t69*t270-t78*t274+t75*t278+t166*t249+t163*t253-t161*t257+dvxNoise*(t69*t69)+dvyNoise*(t75*t75)+dvzNoise*(t78*t78);
    nextCov[3][4] = Cov[3][4]+t321-Cov[9][4]*t69+Cov[0][4]*t166+Cov[1][4]*t163+Cov[10][4]*t75-Cov[2][4]*t161-Cov[11][4]*t78-t88*t270-t85*t278+t93*t274-t174*t249+t171*t257+t183*t253-dvyNoise*t75*t85-dvzNoise*t78*t93;
    nextCov[3][5] = Cov[3][5]+t376-Cov[9][5]*t69+Cov[0][5]*t166+Cov[1][5]*t163+Cov[10][5]*t75-Cov[2][5]*t161-Cov[11][5]*t78+t99*t270-t98*t274-t102*t278+t190*t249-t188*t253+t197*t257-dvxNoise*t69*t99-dvyNoise*t75*t102;
    nextCov[3][6] = t291;
    nextCov[3][7] = t295;
    nextCov[3][8] = t299;
    nextCov[3][9] = t270;
    nextCov[3][10] = t278;
    nextCov[3][11] = t274;
    nextCov[4][0] = t34*t303+t48*t307-t56*t311-t39*t346;
    nextCov[4][1] = -t39*t350-t119*t303+t123*t307+t127*t311;
    nextCov[4][2] = -t39*t354+t146*t303-t152*t307+t158*t311;
    nextCov[4][3] = Cov[4][3]+t321-Cov[0][3]*t174-Cov[9][3]*t88-Cov[10][3]*t85+Cov[2][3]*t171+Cov[1][3]*t183+Cov[11][3]*t93-t69*t325-t78*t329+t75*t333+t166*t303+t163*t307-t161*t311-dvyNoise*t75*t85-dvzNoise*t78*t93;
    nextCov[4][4] = Cov[4][4]-Cov[0][4]*t174-Cov[9][4]*t88-Cov[10][4]*t85+Cov[2][4]*t171+Cov[1][4]*t183+Cov[11][4]*t93-t88*t325-t85*t333+t93*t329-t174*t303+t171*t311+t183*t307+dvxNoise*(t88*t88)+dvyNoise*(t85*t85)+dvzNoise*(t93*t93);
    nextCov[4][5] = Cov[4][5]+t389-Cov[0][5]*t174-Cov[9][5]*t88-Cov[10][5]*t85+Cov[2][5]*t171+Cov[1][5]*t183+Cov[11][5]*t93+t99*t325-t98*t329-t102*t333+t190*t303-t188*t307+t197*t311-dvxNoise*t88*t99-dvzNoise*t93*t98;
    nextCov[4][6] = t346;
    nextCov[4][7] = t350;
    nextCov[4][8] = t354;
    nextCov[4][9] = t325;
    nextCov[4][10] = t333;
    nextCov[4][11] = t329;
    nextCov[5][0] = t34*t358+t48*t362-t56*t366-t39*t402;
    nextCov[5][1] = -t39*t406-t119*t358+t123*t362+t127*t366;
    nextCov[5][2] = -t39*t410+t146*t358-t152*t362+t158*t366;
    nextCov[5][3] = Cov[5][3]+t376+Cov[9][3]*t99+Cov[0][3]*t190-Cov[1][3]*t188-Cov[10][3]*t102-Cov[11][3]*t98+Cov[2][3]*t197-t69*t380-t78*t384+t75*t388+t166*t358+t163*t362-t161*t366-dvxNoise*t69*t99-dvyNoise*t75*t102;
    nextCov[5][4] = Cov[5][4]+t389+Cov[9][4]*t99+Cov[0][4]*t190-Cov[1][4]*t188-Cov[10][4]*t102-Cov[11][4]*t98+Cov[2][4]*t197-t88*t380-t85*t388+t93*t384-t174*t358+t171*t366+t183*t362-dvxNoise*t88*t99-dvzNoise*t93*t98;
    nextCov[5][5] = Cov[5][5]+Cov[9][5]*t99+Cov[0][5]*t190-Cov[1][5]*t188-Cov[10][5]*t102-Cov[11][5]*t98+Cov[2][5]*t197+t99*t380-t98*t384-t102*t388+t190*t358-t188*t362+t197*t366+dvxNoise*(t99*t99)+dvyNoise*(t102*t102)+dvzNoise*(t98*t98);
    nextCov[5][6] = t402;
    nextCov[5][7] = t406;
    nextCov[5][8] = t410;
    nextCov[5][9] = t380;
    nextCov[5][10] = t388;
    nextCov[5][11] = t384;
    nextCov[6][0] = -t411+Cov[6][0]*t34+Cov[6][1]*t48-Cov[6][2]*t56;
    nextCov[6][1] = -t412-Cov[6][0]*t119+Cov[6][1]*t123+Cov[6][2]*t127;
    nextCov[6][2] = -t413+Cov[6][0]*t146-Cov[6][1]*t152+Cov[6][2]*t158;
    nextCov[6][3] = Cov[6][3]-Cov[6][2]*t161+Cov[6][1]*t163+Cov[6][0]*t166-Cov[6][9]*t69+Cov[6][10]*t75-Cov[6][11]*t78;
    nextCov[6][4] = Cov[6][4]+Cov[6][2]*t171-Cov[6][0]*t174+Cov[6][1]*t183-Cov[6][10]*t85-Cov[6][9]*t88+Cov[6][11]*t93;
    nextCov[6][5] = Cov[6][5]-Cov[6][1]*t188+Cov[6][0]*t190+Cov[6][2]*t197+Cov[6][9]*t99-Cov[6][11]*t98-Cov[6][10]*t102;
    nextCov[6][6] = Cov[6][6];
    nextCov[6][7] = Cov[6][7];
    nextCov[6][8] = Cov[6][8];
    nextCov[6][9] = Cov[6][9];
    nextCov[6][10] = Cov[6][10];
    nextCov[6][11] = Cov[6][11];
    nextCov[7][0] = -t198+Cov[7][0]*t34+Cov[7][1]*t48-Cov[7][2]*t56;
    nextCov[7][1] = -t200-Cov[7][0]*t119+Cov[7][1]*t123+Cov[7][2]*t127;
    nextCov[7][2] = -t202+Cov[7][0]*t146-Cov[7][1]*t152+Cov[7][2]*t158;
    nextCov[7][3] = Cov[7][3]-Cov[7][2]*t161+Cov[7][1]*t163+Cov[7][0]*t166-Cov[7][9]*t69+Cov[7][10]*t75-Cov[7][11]*t78;
    nextCov[7][4] = Cov[7][4]+Cov[7][2]*t171-Cov[7][0]*t174+Cov[7][1]*t183-Cov[7][10]*t85-Cov[7][9]*t88+Cov[7][11]*t93;
    nextCov[7][5] = Cov[7][5]-Cov[7][1]*t188+Cov[7][0]*t190+Cov[7][2]*t197+Cov[7][9]*t99-Cov[7][11]*t98-Cov[7][10]*t102;
    nextCov[7][6] = Cov[7][6];
    nextCov[7][7] = Cov[7][7];
    nextCov[7][8] = Cov[7][8];
    nextCov[7][9] = Cov[7][9];
    nextCov[7][10] = Cov[7][10];
    nextCov[7][11] = Cov[7][11];
    nextCov[8][0] = -t238+Cov[8][0]*t34+Cov[8][1]*t48-Cov[8][2]*t56;
    nextCov[8][1] = -t240-Cov[8][0]*t119+Cov[8][1]*t123+Cov[8][2]*t127;
    nextCov[8][2] = -t242+Cov[8][0]*t146-Cov[8][1]*t152+Cov[8][2]*t158;
    nextCov[8][3] = Cov[8][3]-Cov[8][2]*t161+Cov[8][1]*t163+Cov[8][0]*t166-Cov[8][9]*t69+Cov[8][10]*t75-Cov[8][11]*t78;
    nextCov[8][4] = Cov[8][4]+Cov[8][2]*t171-Cov[8][0]*t174+Cov[8][1]*t183-Cov[8][10]*t85-Cov[8][9]*t88+Cov[8][11]*t93;
    nextCov[8][5] = Cov[8][5]-Cov[8][1]*t188+Cov[8][0]*t190+Cov[8][2]*t197+Cov[8][9]*t99-Cov[8][11]*t98-Cov[8][10]*t102;
    nextCov[8][6] = Cov[8][6];
    nextCov[8][7] = Cov[8][7];
    nextCov[8][8] = Cov[8][8];
    nextCov[8][9] = Cov[8][9];
    nextCov[8][10] = Cov[8][10];
    nextCov[8][11] = Cov[8][11];
    nextCov[9][0] = Cov[9][0]*t34-Cov[9][6]*t39+Cov[9][1]*t48-Cov[9][2]*t56;
    nextCov[9][1] = -Cov[9][7]*t39-Cov[9][0]*t119+Cov[9][1]*t123+Cov[9][2]*t127;
    nextCov[9][2] = -Cov[9][8]*t39+Cov[9][0]*t146-Cov[9][1]*t152+Cov[9][2]*t158;
    nextCov[9][3] = Cov[9][3]-t279-Cov[9][2]*t161+Cov[9][1]*t163+Cov[9][0]*t166+Cov[9][10]*t75-Cov[9][11]*t78;
    nextCov[9][4] = Cov[9][4]-t336+Cov[9][2]*t171-Cov[9][0]*t174+Cov[9][1]*t183-Cov[9][10]*t85+Cov[9][11]*t93;
    nextCov[9][5] = Cov[9][5]+t378-Cov[9][1]*t188+Cov[9][0]*t190+Cov[9][2]*t197-Cov[9][11]*t98-Cov[9][10]*t102;
    nextCov[9][6] = Cov[9][6];
    nextCov[9][7] = Cov[9][7];
    nextCov[9][8] = Cov[9][8];
    nextCov[9][9] = Cov[9][9];
    nextCov[9][10] = Cov[9][10];
    nextCov[9][11] = Cov[9][11];
    nextCov[10][0] = Cov[10][0]*t34-Cov[10][6]*t39+Cov[10][1]*t48-Cov[10][2]*t56;
    nextCov[10][1] = -Cov[10][7]*t39-Cov[10][0]*t119+Cov[10][1]*t123+Cov[10][2]*t127;
    nextCov[10][2] = -Cov[10][8]*t39+Cov[10][0]*t146-Cov[10][1]*t152+Cov[10][2]*t158;
    nextCov[10][3] = Cov[10][3]+t276-Cov[10][2]*t161+Cov[10][1]*t163+Cov[10][0]*t166-Cov[10][9]*t69-Cov[10][11]*t78;
    nextCov[10][4] = Cov[10][4]-t337+Cov[10][2]*t171-Cov[10][0]*t174+Cov[10][1]*t183-Cov[10][9]*t88+Cov[10][11]*t93;
    nextCov[10][5] = Cov[10][5]-t395-Cov[10][1]*t188+Cov[10][0]*t190+Cov[10][2]*t197+Cov[10][9]*t99-Cov[10][11]*t98;
    nextCov[10][6] = Cov[10][6];
    nextCov[10][7] = Cov[10][7];
    nextCov[10][8] = Cov[10][8];
    nextCov[10][9] = Cov[10][9];
    nextCov[10][10] = Cov[10][10];
    nextCov[10][11] = Cov[10][11];
    nextCov[11][0] = Cov[11][0]*t34-Cov[11][6]*t39+Cov[11][1]*t48-Cov[11][2]*t56;
    nextCov[11][1] = -Cov[11][7]*t39-Cov[11][0]*t119+Cov[11][1]*t123+Cov[11][2]*t127;
    nextCov[11][2] = -Cov[11][8]*t39+Cov[11][0]*t146-Cov[11][1]*t152+Cov[11][2]*t158;
    nextCov[11][3] = Cov[11][3]-t287-Cov[11][2]*t161+Cov[11][1]*t163+Cov[11][0]*t166-Cov[11][9]*t69+Cov[11][10]*t75;
    nextCov[11][4] = Cov[11][4]+t327+Cov[11][2]*t171-Cov[11][0]*t174+Cov[11][1]*t183-Cov[11][10]*t85-Cov[11][9]*t88;
    nextCov[11][5] = Cov[11][5]-t396-Cov[11][1]*t188+Cov[11][0]*t190+Cov[11][2]*t197+Cov[11][9]*t99-Cov[11][10]*t102;
    nextCov[11][6] = Cov[11][6];
    nextCov[11][7] = Cov[11][7];
    nextCov[11][8] = Cov[11][8];
    nextCov[11][9] = Cov[11][9];
    nextCov[11][10] = Cov[11][10];
    nextCov[11][11] = Cov[11][11];

    // Add the delta angle bias state noise
    for (uint8_t i=6;i<=8;i++) {
        nextCov[i][i] = nextCov[i][i] + delAngBiasVariance;
    }

    // Add the delta velocity bias state noise
    for (uint8_t i=9;i<=11;i++) {
        nextCov[i][i] = nextCov[i][i] + delVelBiasVariance;
    }

    // copy predicted variances whilst constraining to be non-negative
    for (uint8_t index=0; index<=11; index++) {
        if (nextCov[index][index] < 0.0f) {
            Cov[index][index] = 0.0f;
        } else {
            Cov[index][index] = nextCov[index][index];
        }
    }

    // copy elements to covariance matrix whilst enforcing symmetry
    for (uint8_t rowIndex=1; rowIndex<=11; rowIndex++) {
        for (uint8_t colIndex=0; colIndex<=rowIndex-1; colIndex++) {
            Cov[rowIndex][colIndex] = 0.5f*(nextCov[rowIndex][colIndex] + nextCov[colIndex][rowIndex]);
            Cov[colIndex][rowIndex] = Cov[rowIndex][colIndex];
        }
    }

}

// Fuse the SmallEKF velocity estimates - this enables alevel reference to be maintained during constant turns
void SmallEKF::fuseVelocity(bool yawInit)
{
    float R_OBS = 0.25f;
    float innovation[3];
    float varInnov[3];
    Vector3f angErrVec;
    uint8_t stateIndex;
    float K[12];
    // Fuse measurements sequentially
    for (uint8_t obsIndex=0;obsIndex<=2;obsIndex++) {
        stateIndex = 3 + obsIndex;

        // Calculate the velocity measurement innovation using the SmallEKF estimate as the observation
        // if heading isn't aligned, use zero velocity (static assumption)
        if (yawInit) {
            Vector3f measVelNED;
            _main_ekf.getVelNED(measVelNED);
            innovation[obsIndex] = state.velocity[obsIndex] - measVelNED[obsIndex];
        } else {
            innovation[obsIndex] = state.velocity[obsIndex];
        }

        // Zero the attitude error states - they represent the incremental error so must be zero before corrections are applied
        state.angErr.zero();
        // Calculate the innovation variance
        varInnov[obsIndex] = Cov[stateIndex][stateIndex] + R_OBS;
        // Calculate the Kalman gain and correct states, taking advantage of direct state observation
        for (uint8_t rowIndex=0;rowIndex<=8;rowIndex++) {
            K[rowIndex] = Cov[rowIndex][stateIndex]/varInnov[obsIndex];
            states[rowIndex] -= K[rowIndex] * innovation[obsIndex];
        }

        // Store tilt error estimate for external monitoring
        angErrVec = angErrVec + state.angErr;

        // the first 3 states represent the angular misalignment vector. This is
        // is used to correct the estimated quaternion
        // Convert the error rotation vector to its equivalent quaternion
        // truth = estimate + error
        float rotationMag = state.angErr.length();
        if (rotationMag > 1e-6f) {
            Quaternion deltaQuat;
            float temp = sinf(0.5f*rotationMag) / rotationMag;
            deltaQuat[0] = cosf(0.5f*rotationMag);
            deltaQuat[1] = state.angErr.x*temp;
            deltaQuat[2] = state.angErr.y*temp;
            deltaQuat[3] = state.angErr.z*temp;

            // Update the quaternion states by rotating from the previous attitude through the error quaternion
            state.quat *= deltaQuat;

            // re-normalise the quaternion
            state.quat.normalize();
        }

        // Update the covariance
        for (uint8_t rowIndex=0;rowIndex<=11;rowIndex++) {
            for (uint8_t colIndex=0;colIndex<=11;colIndex++) {
                Cov[rowIndex][colIndex] = Cov[rowIndex][colIndex] - K[rowIndex]*Cov[stateIndex][colIndex];
            }
        }

        // force symmetry and constrain diagonals to be non-negative
        fixCovariance();
    }

    // calculate tilt component of angle correction
    TiltCorrection = sqrtf(sq(angErrVec.x) + sq(angErrVec.y));
}

// check for new magnetometer data and update store measurements if available
void SmallEKF::readMagData()
{
    if (_ahrs.get_compass() && 
        _ahrs.get_compass()->use_for_yaw() && 
        _ahrs.get_compass()->last_update != lastMagUpdate) {
        // store time of last measurement update
        lastMagUpdate = _ahrs.get_compass()->last_update;

        // read compass data and scale to improve numerical conditioning
        magData = _ahrs.get_compass()->get_field() * 0.001f;

        // let other processes know that new compass data has arrived
        newDataMag = true;
    } else {
        newDataMag = false;
    }
}

// Fuse compass measurements from autopilot
void SmallEKF::fuseCompass()
{
    float q0 = state.quat[0];
    float q1 = state.quat[1];
    float q2 = state.quat[2];
    float q3 = state.quat[3];

    float magX = magData.x;
    float magY = magData.y;
    float magZ = magData.z;

    const float R_MAG = 3e-2f;

    // Calculate observation Jacobian
    float t6 = q0*q0;
    float t7 = q1*q1;
    float t8 = q2*q2;
    float t9 = q3*q3;
    float t10 = t6+t7-t8-t9;
    float t13 = q0*q2*2.0f;
    float t14 = q1*q3*2.0f;
    float t15 = t13+t14;
    float t16 = q0*q3*2.0f;
    float t18 = q1*q2*2.0f;
    float t17 = t16-t18;
    float t19 = cosTheta*sinPsi;
    float t20 = sinPhi*sinTheta*cosPsi;
    float t21 = t19+t20;
    float t22 = t16+t18;
    float t23 = sinTheta*sinPsi;
    float t41 = cosTheta*sinPhi*cosPsi;
    float t24 = t23-t41;
    float t25 = q0*q1*2.0f;
    float t31 = q2*q3*2.0f;
    float t26 = t25-t31;
    float t27 = t6-t7+t8-t9;
    float t28 = sinTheta*cosPsi;
    float t29 = cosTheta*sinPhi*sinPsi;
    float t30 = t28+t29;
    float t32 = cosTheta*cosPsi;
    float t46 = sinPhi*sinTheta*sinPsi;
    float t33 = t32-t46;
    float t35 = sinPhi*t17;
    float t36 = cosPhi*sinTheta*t10;
    float t37 = cosPhi*cosTheta*t15;
    float t38 = t35+t36-t37;
    float t39 = magZ*t38;
    float t40 = t10*t21;
    float t42 = t15*t24;
    float t43 = cosPhi*cosPsi*t17;
    float t44 = t40+t42-t43;
    float t45 = magY*t44;
    float t47 = t10*t33;
    float t48 = t15*t30;
    float t49 = cosPhi*sinPsi*t17;
    float t50 = t47+t48+t49;
    float t51 = magX*t50;
    float t52 = -t39+t45+t51;
    float t53 = 1.0f/t52;
    float t54 = sinPhi*t27;
    float t55 = cosPhi*cosTheta*t26;
    float t56 = cosPhi*sinTheta*t22;
    float t57 = -t54+t55+t56;
    float t58 = magZ*t57;
    float t59 = t21*t22;
    float t60 = t24*t26;
    float t61 = cosPhi*cosPsi*t27;
    float t62 = t59-t60+t61;
    float t63 = magY*t62;
    float t64 = t26*t30;
    float t65 = t22*t33;
    float t66 = cosPhi*sinPsi*t27;
    float t67 = t64-t65+t66;
    float t68 = magX*t67;
    float t69 = t58-t63+t68;
    float t70 = t53*t69;
    float t34 = tan(t70);
    float t71 = t34*t34;
    float t72 = t71+1.0f;
    float t73 = 1.0f/(t52*t52);
    float H_MAG[3];
    H_MAG[0] = -t72*(t53*(magZ*(sinPhi*t26+cosPhi*cosTheta*t27)+magY*(t24*t27+cosPhi*cosPsi*t26)+magX*(t27*t30-cosPhi*sinPsi*t26))-t69*t73*(magZ*(sinPhi*t15+cosPhi*cosTheta*t17)+magY*(t17*t24+cosPhi*cosPsi*t15)+magX*(t17*t30-cosPhi*sinPsi*t15)));
    H_MAG[1] = t72*(t53*(magZ*(cosPhi*cosTheta*t22-cosPhi*sinTheta*t26)+magY*(t22*t24+t21*t26)+magX*(t22*t30+t26*t33))+t69*t73*(magZ*(cosPhi*cosTheta*t10+cosPhi*sinTheta*t15)+magY*(t10*t24-t15*t21)+magX*(t10*t30-t15*t33)));
    H_MAG[2] = t72*(t53*(-magZ*(sinPhi*t22+cosPhi*sinTheta*t27)+magY*(t21*t27-cosPhi*cosPsi*t22)+magX*(t27*t33+cosPhi*sinPsi*t22))-t69*t73*(magZ*(sinPhi*t10-cosPhi*sinTheta*t17)+magY*(t17*t21+cosPhi*t10*cosPsi)+magX*(t17*t33-cosPhi*t10*sinPsi)));

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
    float PH[3];
    float varInnov = R_MAG;
    for (uint8_t rowIndex=0;rowIndex<=2;rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0;colIndex<=2;colIndex++) {
            PH[rowIndex] += Cov[rowIndex][colIndex]*H_MAG[colIndex];
        }
        varInnov += H_MAG[rowIndex]*PH[rowIndex];
    }
    float K_MAG[12];
    float varInnovInv = 1.0f / varInnov;
    for (uint8_t rowIndex=0;rowIndex<=11;rowIndex++) {
        K_MAG[rowIndex] = 0.0f;
        for (uint8_t colIndex=0;colIndex<=2;colIndex++) {
            K_MAG[rowIndex] += Cov[rowIndex][colIndex]*H_MAG[colIndex];
        }
        K_MAG[rowIndex] *= varInnovInv;
    }

    // Calculate the innovation
    float innovation = calcMagHeadingInnov();

    // limit the innovation so that initial corrections are not too large
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the state vector
    state.angErr.zero();
    for (uint8_t i=0;i<=11;i++) {
        states[i] -= K_MAG[i] * innovation;
    }

    // the first 3 states represent the angular error vector where truth = estimate + error. This is is used to correct the estimated quaternion
    float rotationMag = state.angErr.length();
    if (rotationMag > 1e-6f) {
        // Convert the error rotation vector to its equivalent quaternion
        Quaternion deltaQuat;
        float temp = sinf(0.5f*rotationMag) / rotationMag;
        deltaQuat[0] = cosf(0.5f*rotationMag);
        deltaQuat[1] = state.angErr.x*temp;
        deltaQuat[2] = state.angErr.y*temp;
        deltaQuat[3] = state.angErr.z*temp;

        // Bring the quaternion state estimate back to 'truth' by adding the error
        state.quat *= deltaQuat;

        // re-normalise the quaternion
        state.quat.normalize();
    }

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    float HP[12];
    for (uint8_t colIndex=0;colIndex<=11;colIndex++) {
        HP[colIndex] = 0.0f;
        for (uint8_t rowIndex=0;rowIndex<=2;rowIndex++) {
            HP[colIndex] += H_MAG[rowIndex]*Cov[rowIndex][colIndex];
        }
    }
    for (uint8_t rowIndex=0;rowIndex<=11;rowIndex++) {
        for (uint8_t colIndex=0;colIndex<=11;colIndex++) {
            Cov[rowIndex][colIndex] -= K_MAG[rowIndex] * HP[colIndex];
        }
    }

    // force symmetry and constrain diagonals to be non-negative
    fixCovariance();
}

// Perform an initial heading alignment using the magnetic field and assumed declination
void SmallEKF::alignHeading()
{
    // calculate the correction rotation vector in NED frame
    Vector3f deltaRotNED;
    deltaRotNED.z = -calcMagHeadingInnov();

    // rotate into sensor frame
    Vector3f angleCorrection = Tsn.transposed()*deltaRotNED;

    // apply the correction to the quaternion state
    float rotationMag = deltaRotNED.length();
    if (rotationMag > 1e-6f) {
        // Convert the error rotation vector to its equivalent quaternion
        Quaternion deltaQuat;
        float temp = sinf(0.5f*rotationMag) / rotationMag;
        deltaQuat[0] = cosf(0.5f*rotationMag);
        deltaQuat[1] = angleCorrection.x*temp;
        deltaQuat[2] = angleCorrection.y*temp;
        deltaQuat[3] = angleCorrection.z*temp;

        // Bring the quaternion state estimate back to 'truth' by adding the error
        state.quat *= deltaQuat;

        // re-normalise the quaternion
        state.quat.normalize();
    }
}


// Calculate magnetic heading innovation
float SmallEKF::calcMagHeadingInnov()
{
    // Define rotation from magnetometer to sensor using a 312 rotation sequence
    Matrix3f Tms;
    Tms[0][0] = cosTheta*cosPsi-sinPsi*sinPhi*sinTheta;
    Tms[1][0] = -sinPsi*cosPhi;
    Tms[2][0] = cosPsi*sinTheta+cosTheta*sinPsi*sinPhi;
    Tms[0][1] = cosTheta*sinPsi+cosPsi*sinPhi*sinTheta;
    Tms[1][1] = cosPsi*cosPhi;
    Tms[2][1] = sinPsi*sinTheta-cosTheta*cosPsi*sinPhi;
    Tms[0][2] = -sinTheta*cosPhi;
    Tms[1][2] = sinPhi;
    Tms[2][2] = cosTheta*cosPhi;

    // get earth and body magnetic fields
    Vector3f earth_magfield, body_magfield;
    _main_ekf.getMagNED(earth_magfield);
    _main_ekf.getMagXYZ(body_magfield);

    earth_magfield *= 0.001f;
    body_magfield *= 0.001f;

    // Define rotation from magnetometer to NED axes
    Matrix3f Tmn = Tsn*Tms;
    // rotate magentic field measured at top plate into NED axes afer applying bias values learnt by SmallEKF
    Vector3f magMeasNED = Tmn*(magData - body_magfield);
    // the predicted measurement is the angle wrt magnetic north of the horizontal component of the measured field
    float innovation = atan2(magMeasNED.y,magMeasNED.x) - atan2(earth_magfield.y,earth_magfield.x);

    // Unwrap the innovation so it sits on the range from +-pi
    if (innovation > 3.1415927f) {
        innovation = innovation - 6.2831853f;
    } else if (innovation < -3.1415927f) {
        innovation = innovation + 6.2831853f;
    }

    return innovation;
}

// Force symmmetry and non-negative diagonals on state covarinace matrix
void SmallEKF::fixCovariance()
{
    // force symmetry
    for (uint8_t rowIndex=1; rowIndex<=11; rowIndex++) {
        for (uint8_t colIndex=0; colIndex<=rowIndex-1; colIndex++) {
            Cov[rowIndex][colIndex] = 0.5f*(Cov[rowIndex][colIndex] + Cov[colIndex][rowIndex]);
            Cov[colIndex][rowIndex] = Cov[rowIndex][colIndex];
        }
    }

    // constrain diagonals to be non-negative
    for (uint8_t index=1; index<=11; index++) {
        if (Cov[index][index] < 0.0f) {
            Cov[index][index] = 0.0f;
        }
    }
}

// return data for debugging EKF
void SmallEKF::getDebug(float &tilt, Vector3f &velocity, Vector3f &euler, Vector3f &gyroBias, Vector3f &accelBias) const
{
    tilt = TiltCorrection;
    velocity = state.velocity;
    state.quat.to_euler(euler.x, euler.y, euler.z);
    if (dtIMU < 1.0e-6f) {
        gyroBias.zero();
        accelBias.zero();
    } else {
        gyroBias = state.delAngBias / dtIMU;
        accelBias = state.delVelBias / dtIMU;
    }
}

// get gyro bias data
void SmallEKF::getGyroBias(Vector3f &gyroBias) const
{
    if (dtIMU < 1.0e-6f) {
        gyroBias.zero();
    } else {
        gyroBias = state.delAngBias / dtIMU;
    }
}

// get quaternion data
void SmallEKF::getQuat(Quaternion &quat) const
{
    quat = state.quat;
}

#endif // HAL_CPU_CLASS
