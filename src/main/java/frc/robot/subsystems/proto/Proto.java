// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.proto;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.MotorSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class Proto extends SubsystemBase implements MotorSubsystem{

  private static Proto instance;
  private CANSparkMax MotorOne;
  private CANSparkMax MotorTwo;
  private CANSparkMax MotorThree;
  private CANSparkMax MotorFour;

  private double numForShuffleBoard;

  private MAShuffleboard board;
  private MAShuffleboard settingBoard;

  private double time;
  private double prevTime;
  private double[] prevVelocity;

  private boolean settingsInitialized = false;

  private Proto() {
    time = Timer.getFPGATimestamp();
    prevTime = time;
    prevVelocity = new double[] {0, 0, 0, 0};

    settingBoard = new MAShuffleboard("Settings");

    numForShuffleBoard = 0;

    settingBoard.addNum("Motor 1 Precent Speed", numForShuffleBoard);
    settingBoard.addNum("Motor 2 Precent Speed", numForShuffleBoard);
    settingBoard.addNum("Motor 3 Precent Speed", numForShuffleBoard);
    settingBoard.addNum("Motor 4 Precent Speed", numForShuffleBoard);

    settingBoard.addNum("Motor 1 Gear Ratio", numForShuffleBoard);
    settingBoard.addNum("Motor 2 Gear Ratio", numForShuffleBoard);
    settingBoard.addNum("Motor 3 Gear Ratio", numForShuffleBoard);
    settingBoard.addNum("Motor 4 Gear Ratio", numForShuffleBoard);

    settingBoard.addBrushModeChooser("Motor 1 Type");
    settingBoard.addBrushModeChooser("Motor 2 Type");
    settingBoard.addBrushModeChooser("Motor 3 Type");
    settingBoard.addBrushModeChooser("Motor 4 Type");

    settingBoard.addNeutralModeChooser("Motor 1 Neutral Mode");
    settingBoard.addNeutralModeChooser("Motor 2 Neutral Mode");
    settingBoard.addNeutralModeChooser("Motor 3 Neutral Mode");
    settingBoard.addNeutralModeChooser("Motor 4 Neutral Mode");

    settingBoard.addBooleanChooser("Motor 1 Enabled");
    settingBoard.addBooleanChooser("Motor 2 Enabled");
    settingBoard.addBooleanChooser("Motor 3 Enabled");
    settingBoard.addBooleanChooser("Motor 4 Enabled");

    board = new MAShuffleboard("Prototype");
  }

  public void initializeBySettings() {
    try {
      MotorOne = new CANSparkMax(1, settingBoard.getBrushMode("Motor 1 Type"));
      MotorTwo = new CANSparkMax(2, settingBoard.getBrushMode("Motor 2 Type"));
      MotorThree = new CANSparkMax(3, settingBoard.getBrushMode("Motor 3 Type"));
      MotorFour = new CANSparkMax(4, settingBoard.getBrushMode("Motor 4 Type"));
    } catch (Exception e) {
      System.err.println("dident initialize motors beacuse of error: " + e);
      System.err.println("Beacuse this isent the first init");
    }

    MotorOne.setIdleMode(settingBoard.getNeutralMode("Motor 1 Neutral Mode"));
    MotorTwo.setIdleMode(settingBoard.getNeutralMode("Motor 2 Neutral Mode"));
    MotorThree.setIdleMode(settingBoard.getNeutralMode("Motor 3 Neutral Mode"));
    MotorFour.setIdleMode(settingBoard.getNeutralMode("Motor 4 Neutral Mode"));

    settingsInitialized = true;
  }

  public void stop () {
    MotorOne.set(0);
    MotorTwo.set(0);
    MotorThree.set(0);
    MotorFour.set(0);
  }

  public static Proto getInstance() {
    if (instance == null) {
      instance = new Proto();
    }
    return instance;
  }

  @Override
  public void periodic() {
    if (settingsInitialized) {
      time = Timer.getFPGATimestamp();

      if (settingBoard.getBrushMode("Motor 1 Type") !=  MotorType.kBrushed) {
        board.addNum("motor 1 velocity", MotorOne.getEncoder().getVelocity() * settingBoard.getNum("Motor 1 Gear Ratio"));
        board.addNum("motor 1 distance", MotorOne.getEncoder().getPosition() * settingBoard.getNum("Motor 1 Gear Ratio"));
        board.addNum("motor 1 acceleration", (MotorOne.getEncoder().getVelocity() - prevVelocity[0]) * settingBoard.getNum("Motor 1 Gear Ratio") / (time - prevTime));
        prevVelocity[0] = MotorOne.getEncoder().getVelocity();
      } else {
        board.addNum("motor 1 velocity", 666666);
        board.addNum("motor 1 distance", 666666);
        board.addNum("motor 1 acceleration", 666666);
      }
      if (settingBoard.getBrushMode("Motor 2 Type") !=  MotorType.kBrushed) {
        board.addNum("motor 2 velocity", MotorTwo.getEncoder().getVelocity() * settingBoard.getNum("Motor 2 Gear Ratio"));
        board.addNum("motor 2 distance", MotorTwo.getEncoder().getPosition() * settingBoard.getNum("Motor 2 Gear Ratio"));
        board.addNum("motor 2 acceleration", (MotorTwo.getEncoder().getVelocity() - prevVelocity[1]) * settingBoard.getNum("Motor 2 Gear Ratio") / (time - prevTime));
        prevVelocity[1] = MotorTwo.getEncoder().getVelocity();
      } else {
        board.addNum("motor 2 velocity", 666666);
        board.addNum("motor 2 distance", 666666);
        board.addNum("motor 2 acceleration", 666666);
      }
      if (settingBoard.getBrushMode("Motor 3 Type") !=  MotorType.kBrushed) {
        board.addNum("motor 3 velocity", MotorThree.getEncoder().getVelocity() * settingBoard.getNum("Motor 3 Gear Ratio"));
        board.addNum("motor 3 distance", MotorThree.getEncoder().getPosition() * settingBoard.getNum("Motor 3 Gear Ratio"));
        board.addNum("motor 3 acceleration", (MotorThree.getEncoder().getVelocity() - prevVelocity[2]) * settingBoard.getNum("Motor 3 Gear Ratio") / (time - prevTime));
        prevVelocity[2] = MotorThree.getEncoder().getVelocity();
      } else {
        board.addNum("motor 3 velocity", 666666);
        board.addNum("motor 3 distance", 666666);
        board.addNum("motor 3 acceleration", 666666);
      }
      if (settingBoard.getBrushMode("Motor 4 Type") !=  MotorType.kBrushed) {
        board.addNum("motor 4 velocity", MotorFour.getEncoder().getVelocity() * settingBoard.getNum("Motor 4 Gear Ratio"));
        board.addNum("motor 4 distance", MotorFour.getEncoder().getPosition() * settingBoard.getNum("Motor 4 Gear Ratio"));
        board.addNum("motor 4 acceleration", (MotorFour.getEncoder().getVelocity() - prevVelocity[3]) * settingBoard.getNum("Motor 4 Gear Ratio") / (time - prevTime));
        prevVelocity[3] = MotorFour.getEncoder().getVelocity();
      } else {
        board.addNum("motor 4 velocity", 666666);
        board.addNum("motor 4 distance", 666666);
        board.addNum("motor 4 acceleration", 666666);
      }

      prevTime = time;
    
      setVoltage(6);
    }
  }

  @Override
  public boolean canMove() {
    return true;
  }

  @Override
  public void setVoltage(double voltage) {
    if (settingBoard.getBooleanChooser("Motor 1 Enabled")) {
      MotorOne.setVoltage(settingBoard.getNum("Motor 1 Precent Speed") * 12);
    } else {
      MotorOne.set(0);
    }
    if (settingBoard.getBooleanChooser("Motor 2 Enabled")) {
      MotorTwo.setVoltage(settingBoard.getNum("Motor 2 Precent Speed") * 12);
    } else {
      MotorTwo.set(0);
    }
    if (settingBoard.getBooleanChooser("Motor 3 Enabled")) {
      MotorThree.setVoltage(settingBoard.getNum("Motor 3 Precent Speed") * 12);
    } else {
      MotorThree.set(0);
    }
    if (settingBoard.getBooleanChooser("Motor 4 Enabled")) {
      MotorFour.setVoltage(settingBoard.getNum("Motor 4 Precent Speed") * 12);
    } else {
      MotorFour.set(0);
    }
  }
}
