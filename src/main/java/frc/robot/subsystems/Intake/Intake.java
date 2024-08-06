// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class Intake extends SubsystemBase {
  private static Intake intake;



  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;


  public Intake() {
    upperMotor = new CANSparkMax(12, MotorType.kBrushless);
    lowerMotor = new CANSparkMax(14, MotorType.kBrushless);


  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  public void turnOn() {
    upperMotor.set(-0.7);
    lowerMotor.set(0.7);
  }

  public void turnOFF() {
    upperMotor.set(0);
    lowerMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
