#pragma once


#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/DriverStation.h"
#include "frc/PowerDistribution.h"

#include "Drivetrain.h"
#include "Joystick.h"
#include <Loggers.h>
#include "Vision.h"

class TalonXXVI : public frc::TimedRobot
{
public:
  TalonXXVI();

  frc::PowerDistribution *m_pdp;

  Joystick *m_userInput;
  Drivetrain *m_drivetrain;
  Vision *m_vision;

  CLoggers *m_logger;

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void ServiceDash();
  void LocalReset();

  unsigned int GetLoopCount() { return m_count; };
  frc::DriverStation::Alliance GetAlliance() { return m_alliance; };
  static double Wrap(double val);
  static double Limit(double min, double max, double val);

private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Do Nothing";
  
  std::string m_autoSelected;

  frc::SendableChooser<std::string> m_limelight_chooser;
  const std::string kPipelineDefault = "Driver Cam";
  const std::string kPipelineAprilTag = "April Tag Tracking";

  std::string m_pipelineSelected;

  unsigned int m_count;
  // kBlue or kRed
  frc::DriverStation::Alliance m_alliance;

};