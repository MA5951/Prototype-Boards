package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.proto.Proto;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    Proto.getInstance();
    Proto.getInstance().activateMotors();
    Proto.getInstance().shuffleBoardOutputs();
  }

  @Override
  public void robotPeriodic() {
    Proto.getInstance().shuffleBoardOutputs();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Proto.getInstance().activateMotors();
  }

  @Override
  public void teleopPeriodic() {
    Proto.getInstance().shuffleBoardOutputs();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
