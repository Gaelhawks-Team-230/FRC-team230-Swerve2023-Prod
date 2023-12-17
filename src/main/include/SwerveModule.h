#pragma once

#include <frc/DigitalInput.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/CANCoder.h"
#include <cmath>
#include <frc/DutyCycle.h>
#include "Common.h"

// #define STEER_kP (9.0)
// #define STEER_kV (12.0)
// #define STEER_kR (3000.0)
// #define STEER_TAU (0.03)

#define STEER_kP (8.0)
#define STEER_kV (12.0)
#define STEER_kR (1800.0)
#define STEER_TAU (0.04)

#define DRIVE_kV (10.0)
#define DRIVE_kR (200.0)
#define DRIVE_TAU (0.05)
#define kAV (0.0185)


#define COUNTS_100MS (1.0/100.0)
#define MILLISECONDS_SEC (100.0 / 0.1)
#define REV_COUNTS (1.0 / 2048.0)
#define RAD_REV (2.0 * M_PI)
#define WHEEL_CIRCUMFERENCE (2.00)
#define GEAR_RATIO (1.0 / 6.75)


#define DRIVE_VELOCITY_MEAS_WINDOW (32.0)
#define STEER_VELOCITY_MEAS_WINDOW (16.0)

#define DRIVE_DEADBAND (0.001)
#define STEER_DEADBAND (0.001)

#define DRIVE_STATOR_CURRENT_LIMIT (10.0)

class TalonXXVI;

class SwerveModule
{

public:
    SwerveModule(TalonXXVI *pRobot, unsigned int steerFalconID, unsigned int driveFalconID, unsigned int absoluteEncoderID, unsigned int swerveID, double steerOffset);
    void LocalReset();
    void UpdateDash();
    void StopAll();
    void StartingConfig();

    void SetDriveCmd(double cmd);
    void SetWheelCmd(double cmd);
    void Analyze();
    void DriveControl(double vel, double angle);

    double GetDriveVel() { return m_driveVel; };
    double GetSteerEncoderVel() { return m_steerEncoderVel; };
    double GetSteerEncoderAbsolutePosition() { return m_steerEncoderPosition; };
    double GetVelocityScaleFactor() { return COUNTS_100MS * MILLISECONDS_SEC * REV_COUNTS * RAD_REV * WHEEL_CIRCUMFERENCE * GEAR_RATIO; };
    double GetPositionVelocityScaleFactor() { return (1.0 / LOOPTIME) * REV_COUNTS * RAD_REV * WHEEL_CIRCUMFERENCE * GEAR_RATIO; };

private:
    TalonXXVI *mainRobot;
    unsigned int m_moduleID;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_steer;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *m_drive;
    ctre::phoenix::sensors::CANCoder *m_steerEncoder;

    double m_driveLastPosition;
    double m_driveCurrentPosition;
    double m_driveRaw;
    double m_driveOffset;
    double m_drivePosition;
    double m_driveVel;
    double m_driveMotorVel;
    double m_drivePosVel;

    double m_steerMotorPosition;
    double m_steerEncoderRawPosition;
    double m_steerMotorVel;
    double m_steerEncoderPosition;
    // double m_steerEncoderAbsolutePosition;
    double m_steerEncoderVel;
    double m_steerOffset;


    double m_steer_pErr;
    double m_steer_vCmd;
    double m_steer_vErr;
    double m_steer_vErrIntegrator;
    double m_steer_cmd;

    bool m_rev;

    double m_drive_pErr;
    double m_drive_vCmd;
    double m_drive_vErr;
    double m_drive_vErrIntegrator;
    double m_drive_cmd;

    double drivecntrl_vel;
    double drivecmtrl_angle;
};