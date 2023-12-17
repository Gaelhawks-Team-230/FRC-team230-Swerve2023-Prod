#include "TalonXXVI.h"
#include "frc/RobotController.h"
#include <fmt/core.h>
#include "Common.h"
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

TalonXXVI::TalonXXVI() : TimedRobot(units::second_t LOOPTIME)
{
  m_drivetrain = new Drivetrain(this);
  m_userInput = new Joystick(this);
  m_pdp = new frc::PowerDistribution(PDP_MODULE, frc::PowerDistribution::ModuleType::kCTRE);
  m_vision = new Vision(this);
  m_logger = new CLoggers("log");

  LocalReset();
}

void TalonXXVI::RobotInit()
{
  LocalReset();
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_limelight_chooser.SetDefaultOption(kPipelineDefault, kPipelineDefault);
  m_limelight_chooser.AddOption(kPipelineAprilTag, kPipelineAprilTag);
  frc::SmartDashboard::PutData("Limelight Pipeline", &m_limelight_chooser);

  m_logger->Init();
}
void TalonXXVI::LocalReset()
{
  // * Global loopcount
  m_count = 0;
  m_alliance = frc::DriverStation::GetAlliance();
  m_drivetrain->LocalReset();
  m_userInput->LocalReset();
  m_vision->LocalReset();
}
/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void TalonXXVI::RobotPeriodic()
{}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void TalonXXVI::AutonomousInit()
{}

void TalonXXVI::AutonomousPeriodic()
{}

void TalonXXVI::TeleopInit()
{
  LocalReset();
  // m_vision->SetTargetFilter(TargetFilter::BEST_TARGET);
  // m_vision->SetFiducialTargetLock(1);
  m_pipelineSelected = m_limelight_chooser.GetSelected();
  // if (m_pipelineSelected == kPipelineAprilTag)
  // {
  //   m_vision->SetCameraMode(APRIL_TAG_CAM);
  // }
  // else
  // {
  //   m_vision->SetCameraMode(DRIVER_CAM);
  // }
  // m_vision->SetVisionFrame(VisionFrame::ROBOT_FRAME);
}

void TalonXXVI::TeleopPeriodic()
{
  double xdot, ydot, psidot;
  double vxBeep, vyBeep;
  m_count++;

  // Get sensor data
  m_drivetrain->Analyze();
  m_userInput->Analyze();
  m_vision->Analyze();

  // * Drive train control
  xdot = m_userInput->GetFlightCtrl_CMD_X();
  ydot = m_userInput->GetFlightCtrl_CMD_Y();
  psidot = m_userInput->GetFlightCtrl_CMD_R();

  // check Beep right button on flight ctrt
  if (m_userInput->GetFlightCtrlButton(BEEP_FOREWARD_BUTTON) == kPressing)
  {
    vxBeep = 2.0 / LOOPTIME;
  }
  else if (m_userInput->GetFlightCtrlButton(BEEP_BACKWARD_BUTTON) == kPressing)
  {
    vxBeep = -2.0 / LOOPTIME;
  }
  else
  {
    vxBeep = 0.0;
  }
  xdot += vxBeep;
  if (m_userInput->GetFlightCtrlButton(BEEP_RIGHT_BUTTON) == kPressing)
  {
    vyBeep = 2.0 / LOOPTIME;
  }
  else if (m_userInput->GetFlightCtrlButton(BEEP_LEFT_BUTTON) == kPressing)
  {
    vyBeep = -2.0 / LOOPTIME;
  }
  else
  {
    vyBeep = 0.0;
  }
  ydot += vyBeep;

  // * Drive control convert to field right off the bat
  if (m_userInput->GetFlightCtrlButton(FIELD_ROBOT_SWITCH))
  {
    m_drivetrain->SetZeroVelDebugModeOff();
  }
  else
  {
    m_drivetrain->ToFieldCoordinates(&xdot, &ydot);
    m_drivetrain->SetZeroVelDebugModeOff();
  }

  // * Vision Stuff
  // if (m_userInput->GetFlightCtrlButton(VISION_CENTER_BUTTON))
  if (m_userInput->GetFlightCtrlButton(VISION_CENTER_BUTTON)) // center
  {
    m_vision->SetMiddleGoalPos();
    m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  }
  else if (m_userInput->GetFlightCtrlButton(VISION_DEBUG_BUTTON))
  {
    m_vision->SetCenterGoalPos();
    m_vision->SetVisionFrame(FIELD_CENTER);
    m_vision->DriveTargetting(&xdot, &ydot, &psidot);

  }

if (m_userInput->GetFlightCtrlButton(VISION_PIPELINE_UP) == kPressing)
{
  m_vision->IncreaseCameraPipeline();
}

  // else if (m_userInput->GetFlightCtrlButton(VISION_LEFT_BUTTON)) // left target in field frame
  // {
  //   m_vision->SetRightGoalPos();
  //   m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  // }
  // else if (m_userInput->GetFlightCtrlButton(VISION_RIGHT_BUTTON))
  // {
  //   // m_vision->SetRightGoalPos();
  //   autoScoreStage = 1;
  // }
  // else if (m_userInput->GetFlightCtrlButton(VISION_LEFT_BUTTON))
  // {
  //   // m_vision->SetLeftGoalPos();
  //   autoScoreStage = 1;
  // }
  // else if (m_userInput->GetFlightCtrlButton(DATA_DEBUG_BUTTON))
  // {
  //   m_vision->SetDataGoalPos();
  //   m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  //   xdot = 0.0;
  //   ydot = 0.0;
  //   psidot = 0.0;
  // }

  // goal_pos.Set(0.0, 0.0, 180.0);
  // m_vision->SetGoalPos(goal_pos);
  // m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  // double elaspedTime;
  // switch (autoScoreStage)
  // {
  // case 0:
  //   //  DO NOTHING
  //   break;
  // case 1:
  //   autoScoreCountOffset = m_count;
  //   autoScoreStage++;
  //   break;
  // case 2:
  //   if (m_vision->TargetFound() < 0.5)
  //   {
  //     break;
  //   }
  //   m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  //   elaspedTime = (m_count - autoScoreCountOffset) * LOOPTIME;
  //   if (elaspedTime >= 2.0)
  //   {
  //     autoScoreStage = 0;
  //   }
  //   break;
  // }

  // * Begin Drive control
  if (m_userInput->GetFlightCtrlButton(RESET_GYRO_BUTTON))
  {
    m_drivetrain->GyroReset();
  }

  if (m_userInput->GetFlightCtrlButton(GYRO_BUTTON_SWITCH))
  {
    m_drivetrain->EnableGyro();
  }
  else
  {
    m_drivetrain->DisableGyro();
  }
  if (m_userInput->GetFlightCtrlButton(GRID_ALIGN_BUTTON_1) || m_userInput->GetFlightCtrlButton(GRID_ALIGN_BUTTON_2))
  {
    double pErr = GRID_ALIGN_ANGLE - Wrap(m_drivetrain->GetGyroReading());
    double vCmd = pErr * GYRO_ANGLE_KP;
    psidot = Limit(-120.0, 120.0, vCmd);
  }

  m_drivetrain->DriveControl(xdot, ydot, psidot);
  ServiceDash();
}
void TalonXXVI::DisabledInit() {}

void TalonXXVI::DisabledPeriodic() {}

void TalonXXVI::TestInit() {}

void TalonXXVI::TestPeriodic() {}

void TalonXXVI::SimulationInit() {}

void TalonXXVI::SimulationPeriodic() {}

void TalonXXVI::ServiceDash()
{
  m_drivetrain->UpdateDash();
  m_userInput->UpdateDash();
  m_vision->UpdateDash();

  frc::SmartDashboard::PutNumber("Loop count", m_count);
  frc::SmartDashboard::PutNumber("CAN pct utilization", (frc::RobotController::GetCANStatus().percentBusUtilization * 100));
  frc::SmartDashboard::PutBoolean("Alliance", m_alliance);
}
/**
 * @brief Wrap value in degrees
 *
 * @param val value in degrees
 * @return double wrapped value in degrees
 */
double TalonXXVI::Wrap(double val)
{
  double rVal = val * (M_PI / 180);
  double rWrap = atan2(sin(rVal), cos(rVal));
  return rWrap * (180 / M_PI);
}
/**
 * @brief Limit the value between a range
 *
 * @param min minimum value
 * @param max maximum value
 * @param val value to compare
 * @return double
 */
double TalonXXVI::Limit(double min, double max, double val)
{
  if (val > max)
  {
    return max;
  }
  if (val < min)
  {
    return min;
  }
  return val;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<TalonXXVI>();
}
#endif
