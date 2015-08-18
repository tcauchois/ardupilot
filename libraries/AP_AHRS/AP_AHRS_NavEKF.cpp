/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_AHRS.h"
#include <AP_Vehicle/AP_Vehicle.h>

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1 || sel == AHRS_SELECTED_EKF2)
        return _gyro_estimate;
    else //if(sel == AHRS_SELECTED_DCM)
        return AP_AHRS_DCM::get_gyro();
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1 || sel == AHRS_SELECTED_EKF2)
        return _dcm_matrix;
    else //if(sel == AHRS_SELECTED_DCM)
        return AP_AHRS_DCM::get_dcm_matrix();
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1 || sel == AHRS_SELECTED_EKF2)
        return _gyro_bias;
    else //if(sel == AHRS_SELECTED_DCM)
        return AP_AHRS_DCM::get_gyro_drift();
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
    // update DCM
    AP_AHRS_DCM::reset_gyro_drift();

    // reset the EKF gyro bias states
    EKF.resetGyroBias();
    if(EKF2 != NULL)
        EKF2->resetGyroBias();
}

void AP_AHRS_NavEKF::update(void)
{
    // we need to restore the old DCM attitude values as these are
    // used internally in DCM to calculate error values for gyro drift
    // correction
    roll = _dcm_attitude.x;
    pitch = _dcm_attitude.y;
    yaw = _dcm_attitude.z;
    update_cd_values();

    AP_AHRS_DCM::update();

    // keep DCM attitude available for get_secondary_attitude()
    _dcm_attitude(roll, pitch, yaw);

    if (!ekf1_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = hal.scheduler->millis();
        }
        if (hal.scheduler->millis() - start_time_ms > startup_delay_ms) {
            ekf1_started = EKF.InitialiseFilterDynamic();
        }
    }
    if (!ekf2_started && EKF2 != NULL) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (hal.scheduler->millis() - start_time_ms > startup_delay_ms) {
            ekf2_started = EKF2->InitialiseFilterBootstrap();
        }
    }
    if (ekf1_started) {
        EKF.UpdateFilter();
    }
    if (ekf2_started && EKF2 != NULL) {
        EKF2->UpdateFilter();
    }

    Vector3f eulers;
    float abias1, abias2;
    float IMU1_weighting;

    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1) {
        EKF.getRotationBodyToNED(_dcm_matrix);
        EKF.getEulerAngles(eulers);
        EKF.getGyroBias(_gyro_bias);
        EKF.getAccelZBias(abias1, abias2);
        EKF.getIMU1Weighting(IMU1_weighting);
    } else if(sel == AHRS_SELECTED_EKF2 && EKF2 != NULL) {
        EKF2->getRotationBodyToNED(_dcm_matrix);
        EKF2->getEulerAngles(eulers);
        EKF2->getGyroBias(_gyro_bias);
        EKF2->getAccelZBias(abias1, abias2);
        EKF2->getIMU1Weighting(IMU1_weighting);
    } else {
        return;
    }

    roll  = eulers.x;
    pitch = eulers.y;
    yaw   = eulers.z;
    update_cd_values();
    update_trig();

    // keep _gyro_bias for get_gyro_drift()
    _gyro_bias = -_gyro_bias;

    // calculate corrected gryo estimate for get_gyro()
    _gyro_estimate.zero();
    uint8_t healthy_count = 0;
    for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
        if (_ins.get_gyro_health(i) && healthy_count < 2) {
            _gyro_estimate += _ins.get_gyro(i);
            healthy_count++;
        }
    }
    if (healthy_count > 1) {
        _gyro_estimate /= healthy_count;
    }
    _gyro_estimate += _gyro_bias;

    // update _accel_ef_ekf
    for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
        Vector3f accel = _ins.get_accel(i);
        if (i==0) {
            accel.z -= abias1;
        } else if (i==1) {
            accel.z -= abias2;
        }
        if (_ins.get_accel_health(i)) {
            _accel_ef_ekf[i] = _dcm_matrix * accel;
        }
    }

    if(_ins.get_accel_health(0) && _ins.get_accel_health(1)) {
        _accel_ef_ekf_blended = _accel_ef_ekf[0] * IMU1_weighting + _accel_ef_ekf[1] * (1.0f-IMU1_weighting);
    } else {
        _accel_ef_ekf_blended = _accel_ef_ekf[_ins.get_primary_accel()];
    }
}

// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef(uint8_t i) const
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1 || sel == AHRS_SELECTED_EKF2)
        return _accel_ef_ekf[i];
    else //if(sel == AHRS_SELECTED_DCM)
        return AP_AHRS_DCM::get_accel_ef(i);
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended(void) const
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1 || sel == AHRS_SELECTED_EKF2)
        return _accel_ef_ekf_blended;
    else //if(sel == AHRS_SELECTED_DCM)
        return AP_AHRS_DCM::get_accel_ef_blended();
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
    if (ekf1_started) {
        ekf1_started = EKF.InitialiseFilterBootstrap();
    }
    if (ekf2_started && EKF2 != NULL) {
        ekf2_started = EKF2->InitialiseFilterBootstrap();
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    if (ekf1_started) {
        ekf1_started = EKF.InitialiseFilterBootstrap();
    }
    if (ekf2_started && EKF2 != NULL) {
        ekf2_started = EKF2->InitialiseFilterBootstrap();
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1) {
        Vector3f ned_pos;
        if(EKF.getLLH(loc) && EKF.getPosNED(ned_pos)) {
            // fixup altitude using relative position from AHRS home, not
            // EKF origin
            loc.alt = get_home().alt - ned_pos.z*100;
            return true;
        }
    } else if(sel == AHRS_SELECTED_EKF2 && EKF2 != NULL) {
        Vector3f ned_pos;
        if(EKF2->getLLH(loc) && EKF2->getPosNED(ned_pos)) {
            loc.alt = get_home().alt - ned_pos.z*100;
            return true;
        }
    }
    //else if(sel == AHRS_SELECTED_DCM) {}
    return AP_AHRS_DCM::get_position(loc);
}

// status reporting of estimated errors
float AP_AHRS_NavEKF::get_error_rp(void) const
{
    return AP_AHRS_DCM::get_error_rp();
}

float AP_AHRS_NavEKF::get_error_yaw(void) const
{
    return AP_AHRS_DCM::get_error_yaw();
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS_NavEKF::wind_estimate(void)
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1) {
        Vector3f wind;
        EKF.getWind(wind);
        return wind;
    } else if(sel == AHRS_SELECTED_EKF2 && EKF2 != NULL) {
        Vector3f wind;
        EKF2->getWind(wind);
        return wind;
    } else {
        // EKF does not estimate wind speed when there is no airspeed
        // sensor active
        return AP_AHRS_DCM::wind_estimate();
    }
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float *airspeed_ret) const
{
    return AP_AHRS_DCM::airspeed_estimate(airspeed_ret);
}

// true if compass is being used
bool AP_AHRS_NavEKF::use_compass(void)
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1)
        return EKF.use_compass();
    else if(sel == AHRS_SELECTED_EKF2 && EKF2 != NULL)
        return EKF2->use_compass();
    else
        return AP_AHRS_DCM::use_compass();
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers)
{
    if (using_EKF() != AHRS_SELECTED_DCM) {
        // return DCM attitude
        eulers = _dcm_attitude;
        return true;
    }
    if (ekf1_started) {
        // EKF is secondary
        EKF.getEulerAngles(eulers);
        return true;
    }
    // no secondary available
    return false;
}

// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc)
{
    if (using_EKF() != AHRS_SELECTED_DCM) {
        // return DCM position
        AP_AHRS_DCM::get_position(loc);
        return true;
    }    
    if (ekf1_started) {
        // EKF is secondary
        EKF.getLLH(loc);
        return true;
    }
    // no secondary available
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1) {
        Vector3f vec;
        EKF.getVelNED(vec);
        return Vector2f(vec.x, vec.y);
    } else if(sel == AHRS_SELECTED_EKF2 && EKF2 != NULL) {
        Vector3f vec;
        EKF2->getVelNED(vec);
        return Vector2f(vec.x, vec.y);
    } else {
        return AP_AHRS_DCM::groundspeed_vector();
    }
}

void AP_AHRS_NavEKF::set_home(const Location &loc)
{
    AP_AHRS_DCM::set_home(loc);
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const 
{
    AHRS_selected sel = using_EKF();
    return (sel == AHRS_SELECTED_EKF1 || sel == AHRS_SELECTED_EKF2);
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1) {
        EKF.getVelNED(vec);
        return true;
    } else if(sel == AHRS_SELECTED_EKF2 && EKF2 != NULL) {
        EKF2->getVelNED(vec);
        return true;
    } else
        return false;
}

// return a relative ground position in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_relative_position_NED(Vector3f &vec) const
{
    AHRS_selected sel = using_EKF();
    if(sel == AHRS_SELECTED_EKF1) {
        return EKF.getPosNED(vec);
    } else if(sel == AHRS_SELECTED_EKF2 && EKF2 != NULL) {
        return EKF2->getPosNED(vec);
    } else
        return false;
}

bool AP_AHRS_NavEKF::vehicle_ekf_checks(nav_filter_status filt_state) const
{
    if (_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
        _vehicle_class == AHRS_VEHICLE_GROUND) {
        if (hal.util->get_soft_armed() && !filt_state.flags.using_gps && _gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            // if the EKF is not fusing GPS and we have a 3D lock, then
            // plane and rover would prefer to use the GPS position from
            // DCM. This is a safety net while some issues with the EKF
            // get sorted out
            return false;
        }
        if (hal.util->get_soft_armed() && filt_state.flags.const_pos_mode) {
            return false;
        }
        if (!filt_state.flags.attitude ||
            !filt_state.flags.horiz_vel ||
            !filt_state.flags.vert_vel ||
            !filt_state.flags.horiz_pos_abs ||
            !filt_state.flags.vert_pos) {
            return false;
        }
    }
    return true;
}

AP_AHRS_NavEKF::AHRS_selected AP_AHRS_NavEKF::using_EKF(void) const
{
    //Try to use EKF2
    if(_ekf_use == EKF_USE_PROTOTYPE_WITHOUT_FALLBACK && EKF2 != NULL) {
        uint8_t ekf2_faults;
        EKF2->getFilterFaults(ekf2_faults);
        nav_filter_status filt_state;
        EKF2->getFilterStatus(filt_state);

        //For EKF2, we switch away if the EKF encounters an internal processing error.
        if(ekf2_started && vehicle_ekf_checks(filt_state) && (ekf2_faults == 0))
            return AHRS_SELECTED_EKF2;
    }

    //Try to use EKF1
    if(_ekf_use == EKF_USE_PROTOTYPE_WITHOUT_FALLBACK || _ekf_use == EKF_USE_WITHOUT_FALLBACK ||
            _ekf_use == EKF_USE_WITH_FALLBACK) {
        uint8_t ekf_faults;
        EKF.getFilterFaults(ekf_faults);
        nav_filter_status filt_state;
        EKF.getFilterStatus(filt_state);

        // If EKF is started we switch away if it reports unhealthy. This could be due to bad
        // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
        // an internal processing error, but not for bad sensor data.
        if(ekf1_started && vehicle_ekf_checks(filt_state) &&
                ((_ekf_use == EKF_USE_WITH_FALLBACK && EKF.healthy()) ||
                 (_ekf_use != EKF_USE_WITH_FALLBACK && (ekf_faults == 0))))
            return AHRS_SELECTED_EKF1;
    }

    return AHRS_SELECTED_DCM;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_NavEKF::healthy(void) const
{
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    if (_ekf_use == EKF_USE_PROTOTYPE_WITHOUT_FALLBACK) {
        if(EKF2 == NULL || !ekf2_started || !EKF2->healthy())
            return false;
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
             _vehicle_class == AHRS_VEHICLE_GROUND) &&
            using_EKF() != AHRS_SELECTED_EKF2) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return true;
    } else if(_ekf_use == EKF_USE_WITH_FALLBACK || _ekf_use == EKF_USE_WITHOUT_FALLBACK) {
        if(!ekf1_started || !EKF.healthy())
            return false;
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
             _vehicle_class == AHRS_VEHICLE_GROUND) &&
            using_EKF() != AHRS_SELECTED_EKF1) {
            return false;
        }
        return true;
    } else /* if(_ekf_use == EKF_DO_NOT_USE) */
        return AP_AHRS_DCM::healthy();
}

void AP_AHRS_NavEKF::set_ekf_use(int8_t setting)
{
    _ekf_use.set(setting);
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) const
{
    // initialisation complete 10sec after ekf has started
    return (ekf1_started && (ekf2_started || EKF2 == NULL) &&
            (hal.scheduler->millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
};

// write optical flow data to EKF
void  AP_AHRS_NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
{
    EKF.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
    if(EKF2 != NULL)
        EKF2->writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
}

// inhibit GPS useage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    uint8_t st1 = EKF.setInhibitGPS(), st2 = 0;
    if(EKF2 != NULL)
        st2 = EKF2->setInhibitGPS();
    if(using_EKF() == AHRS_SELECTED_EKF2)
        return st2;
    else
        return st1;
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler)
{
    if(using_EKF() == AHRS_SELECTED_EKF2 && EKF2 != NULL)
        EKF2->getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
    else
        EKF.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS_NavEKF::getMagOffsets(Vector3f &magOffsets)
{
    if(using_EKF() == AHRS_SELECTED_EKF2 && EKF2 != NULL)
        return EKF2->getMagOffsets(magOffsets);
    else
        return EKF.getMagOffsets(magOffsets);
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

