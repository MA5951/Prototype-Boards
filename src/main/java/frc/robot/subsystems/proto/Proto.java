// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.proto;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class Proto extends SubsystemBase{

  private static Proto instance;
  private CANSparkMax MotorOne;//A
  private CANSparkMax MotorTwo;//B
  private CANSparkMax MotorThree;//C
  private CANSparkMax MotorFour;//D
  private CANSparkMax[] motorsArry;
  private String[] motorsNameArry = new String[] {"A" ,"B" , "C" , "D"};


  private MAShuffleboard board;

  private double time;
  private double prevTime ;
  private double[] prevVelocity = new double[] {0, 0, 0, 0};

  private boolean settingsInitialized = false;

  private Proto() {
    time = Timer.getFPGATimestamp();
    prevTime = time;
    board = new MAShuffleboard("PrototypeBoard");
    

    for (int i = 0; i < 4; i++) {
      board.addNum(motorsNameArry[i]  + "--Speed", 0);
      board.addNum(motorsNameArry[i] +" Ratio", 1);
      board.addBrushModeChooser(motorsNameArry[i] +" Type");
      board.addNeutralModeChooser(motorsNameArry[i] +" Neutral Mode");
      board.addBooleanChooser(motorsNameArry[i] +" Enabled");
      board.addNum(motorsNameArry[i] + " Velocity", 0);
      board.addNum(motorsNameArry[i] + " Distance", 0);
      board.addNum(motorsNameArry[i] + " Acceleration", 0);
      System.out.println(motorsNameArry[i] + " Motor Init");
    }

  }

  public void initializeBySettings() {
    try {
      MotorOne = new CANSparkMax(1, board.getBrushMode("A Type"));
      MotorTwo = new CANSparkMax(2, board.getBrushMode("B Type"));
      MotorThree = new CANSparkMax(3, board.getBrushMode("C Type"));
      MotorFour = new CANSparkMax(4, board.getBrushMode("D Type"));
      motorsArry = new CANSparkMax[] {MotorOne , MotorTwo , MotorThree , MotorFour};
    } catch (Exception e) {
      System.err.println("dident initialize motors beacuse of error: " + e);
      System.err.println("Beacuse this isent the first init");
    }

    for (int i = 0; i < 4; i++) {
      motorsArry[i].setIdleMode(board.getNeutralMode(motorsNameArry[i] +" Neutral Mode"));
    }

    settingsInitialized = true;
  }

  public void stop () {
    for (int i = 0; i < 4; i++) {
      motorsArry[i].set(0);
    }
  }

  public void setVoltage() {
    for (int i = 0; i < 4; i++) {
      if (board.getBooleanChooser(motorsNameArry[i] +" Enabled")) {
        motorsArry[i].setVoltage(board.getNum(motorsNameArry[i]  + "--Speed") * 12);
      } else {
        motorsArry[i].setVoltage(0);
      }
    }
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
      // for (int i = 0; i < 4; i++) {
      //   if (board.getBrushMode(motorsNameArry[i] +" Type") ==  MotorType.kBrushless) {
      //     board.addNum(motorsNameArry[i] + " Velocity", motorsArry[i].getEncoder().getVelocity() * board.getNum(motorsNameArry[i] +" Ratio"));
      //     board.addNum(motorsNameArry[i] + " Distance", motorsArry[i].getEncoder().getPosition() * board.getNum(motorsNameArry[i] +" Ratio"));
      //     board.addNum(motorsNameArry[i] + " Acceleration", (motorsArry[i].getEncoder().getVelocity() - prevVelocity[i]) * board.getNum(motorsNameArry[i] +" Ratio") / (time - prevTime));
      //     prevVelocity[i] = motorsArry[i].getEncoder().getVelocity();
      //   } else {
      //     board.addNum("motor 1 velocity", 666666);
      //     board.addNum("motor 1 distance", 666666);
      //     board.addNum("motor 1 acceleration", 666666);
      //   }
      // }

      prevTime = time;
  
    }
  }
}
