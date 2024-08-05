package frc.robot.subsystems.proto;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;

public class Proto {
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

    private Proto() {
        MotorOne = new CANSparkMax(4, MotorType.kBrushless);
        MotorTwo = new CANSparkMax(30, MotorType.kBrushless);
        MotorThree = new CANSparkMax(14, MotorType.kBrushless);
        MotorFour = new CANSparkMax(3, MotorType.kBrushless);

        time = Timer.getFPGATimestamp();
        prevTime = time;
        prevVelocity = new double[] {0, 0, 0, 0};

        settingBoard = new MAShuffleboard("Settings");

        numForShuffleBoard = 1;

        settingBoard.addNum("Motor 1 Precent Speed", numForShuffleBoard);
        settingBoard.addNum("Motor 2 Precent Speed", numForShuffleBoard);
        settingBoard.addNum("Motor 3 Precent Speed", numForShuffleBoard);
        settingBoard.addNum("Motor 4 Precent Speed", numForShuffleBoard);

        settingBoard.addNum("Motor 1 Gear Ratio", numForShuffleBoard);
        settingBoard.addNum("Motor 2 Gear Ratio", numForShuffleBoard);
        settingBoard.addNum("Motor 3 Gear Ratio", numForShuffleBoard);
        settingBoard.addNum("Motor 4 Gear Ratio", numForShuffleBoard);

        settingBoard.addNeutralModeChooser("Motor 1 Neutral Mode");
        settingBoard.addNeutralModeChooser("Motor 2 Neutral Mode");
        settingBoard.addNeutralModeChooser("Motor 3 Neutral Mode");
        settingBoard.addNeutralModeChooser("Motor 4 Neutral Mode");

        settingBoard.addBooleanChooser("Motor 1 Enabled");
        settingBoard.addBooleanChooser("Motor 2 Enabled");
        settingBoard.addBooleanChooser("Motor 3 Enabled");
        settingBoard.addBooleanChooser("Motor 4 Enabled");

        board = new MAShuffleboard("Prototype");

        board.addNum("motor 1 velocity", 0);
        board.addNum("motor 1 distance", 0);
        board.addNum("motor 1 acceleration", 0);

        board.addNum("motor 2 velocity", 0);
        board.addNum("motor 2 distance", 0);
        board.addNum("motor 2 acceleration", 0);

        board.addNum("motor 3 velocity", 0);
        board.addNum("motor 3 distance", 0);
        board.addNum("motor 3 acceleration", 0);

        board.addNum("motor 4 velocity", 0);
        board.addNum("motor 4 distance", 0);
        board.addNum("motor 4 acceleration", 0);
    }

    public void activateMotors() {
        MotorOne.setIdleMode(settingBoard.getNeutralMode("Motor 1 Neutral Mode"));
        MotorTwo.setIdleMode(settingBoard.getNeutralMode("Motor 2 Neutral Mode"));
        MotorThree.setIdleMode(settingBoard.getNeutralMode("Motor 3 Neutral Mode"));
        MotorFour.setIdleMode(settingBoard.getNeutralMode("Motor 4 Neutral Mode"));
        
        if (settingBoard.getBooleanChooser("Motor 1 Enabled")) {
        MotorOne.setVoltage(settingBoard.getNum("Motor 1 Precent Speed") * 12);
        } else {
        MotorOne.setVoltage(0);
        }
        if (settingBoard.getBooleanChooser("Motor 2 Enabled")) {
        MotorTwo.setVoltage(settingBoard.getNum("Motor 2 Precent Speed") * 12);
        } else {
        MotorTwo.setVoltage(0);
        }
        if (settingBoard.getBooleanChooser("Motor 3 Enabled")) {
        MotorThree.setVoltage(settingBoard.getNum("Motor 3 Precent Speed") * 12);
        } else {
        MotorThree.setVoltage(0);
        }
        if (settingBoard.getBooleanChooser("Motor 4 Enabled")) {
        MotorFour.setVoltage(settingBoard.getNum("Motor 4 Precent Speed") * 12);
        } else {
        MotorFour.setVoltage(0);
        }

        time = Timer.getFPGATimestamp();
        board.addNum("motor 1 velocity", MotorOne.getEncoder().getVelocity() * settingBoard.getNum("Motor 1 Gear Ratio"));
        board.addNum("motor 1 distance", MotorOne.getEncoder().getPosition() * settingBoard.getNum("Motor 1 Gear Ratio"));
        board.addNum("motor 1 acceleration", (MotorOne.getEncoder().getVelocity() - prevVelocity[0]) * settingBoard.getNum("Motor 1 Gear Ratio") / (time - prevTime));
        prevVelocity[0] = MotorOne.getEncoder().getVelocity();

        board.addNum("motor 2 velocity", MotorTwo.getEncoder().getVelocity() * settingBoard.getNum("Motor 2 Gear Ratio"));
        board.addNum("motor 2 distance", MotorTwo.getEncoder().getPosition() * settingBoard.getNum("Motor 2 Gear Ratio"));
        board.addNum("motor 2 acceleration", (MotorTwo.getEncoder().getVelocity() - prevVelocity[1]) * settingBoard.getNum("Motor 2 Gear Ratio") / (time - prevTime));
        prevVelocity[1] = MotorTwo.getEncoder().getVelocity();
        
        board.addNum("motor 3 velocity", MotorThree.getEncoder().getVelocity() * settingBoard.getNum("Motor 3 Gear Ratio"));
        board.addNum("motor 3 distance", MotorThree.getEncoder().getPosition() * settingBoard.getNum("Motor 3 Gear Ratio"));
        board.addNum("motor 3 acceleration", (MotorThree.getEncoder().getVelocity() - prevVelocity[2]) * settingBoard.getNum("Motor 3 Gear Ratio") / (time - prevTime));
        prevVelocity[2] = MotorThree.getEncoder().getVelocity();
        
        board.addNum("motor 4 velocity", MotorFour.getEncoder().getVelocity() * settingBoard.getNum("Motor 4 Gear Ratio"));
        board.addNum("motor 4 distance", MotorFour.getEncoder().getPosition() * settingBoard.getNum("Motor 4 Gear Ratio"));
        board.addNum("motor 4 acceleration", (MotorFour.getEncoder().getVelocity() - prevVelocity[3]) * settingBoard.getNum("Motor 4 Gear Ratio") / (time - prevTime));
        prevVelocity[3] = MotorFour.getEncoder().getVelocity();

        prevTime = time;
    }

    public void stopMotors() {
        MotorOne.setVoltage(0);
        MotorTwo.setVoltage(0);
        MotorThree.setVoltage(0);
        MotorFour.setVoltage(0);
    }
    
    public static Proto getInstance() {
        if (instance == null) {
            instance = new Proto();
        }
        return instance;
    }
}
