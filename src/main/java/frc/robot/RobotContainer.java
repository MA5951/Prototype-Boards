// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.commands.setIntake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final CommandJoystick leftJoystick = new CommandJoystick(0);
  public static final CommandJoystick rightJoystick = new CommandJoystick(1);
  public static final CommandPS4Controller joystick = new CommandPS4Controller(2);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // leftJoystick.button(1).whileTrue(new setIntake());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}