package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerve.DriveSwerveCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @SuppressWarnings("unused")
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Intake.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SwerveDrivetrainSubsystem.getInstance().resetEncoders();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SwerveDrivetrainSubsystem.getInstance().fixOffsetAuto();

    CommandScheduler.getInstance().setDefaultCommand(
      SwerveDrivetrainSubsystem.getInstance(), 
      new DriveSwerveCommand(
        RobotContainer.driverController::getLeftX,
        RobotContainer.driverController::getLeftY,
        RobotContainer.driverController::getRightX));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
