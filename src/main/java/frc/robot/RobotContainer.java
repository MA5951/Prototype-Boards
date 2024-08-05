package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.proto.Proto;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class RobotContainer {
  public static final CommandPS5Controller
    driverController = new CommandPS5Controller(PortMap.Controllers.driveID);

  private void registerCommands() {
  }

  public RobotContainer() {
    registerCommands();

    SwerveDrivetrainSubsystem.getInstance();

    configureBindings();
  }

  private void configureBindings() {
    driverController.R2().whileTrue(new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(
        0.4)
    )).whileFalse(
      new InstantCommand(
      () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(
        1)
    ));

    driverController.triangle().whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::updateOffset)
    );


    driverController.square().whileTrue(
      new InstantCommand(
        () -> Proto.getInstance().activateMotors()
      )
    ).whileFalse(
      new InstantCommand(
        () -> Proto.getInstance().stopMotors()
      )
    );
  }
}
