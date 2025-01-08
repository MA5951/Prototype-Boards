package frc.robot.subsystems.proto;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Proto {
    private static Proto instance;

    private SparkMax MotorOne;
    private SparkMax MotorTwo;

    private DigitalInput sensor1;
    private DigitalInput sensor2;
    private DigitalInput sensor3;
    private DigitalInput sensor4;



    private double numForShuffleBoard;

    private MAShuffleboard board;
    private MAShuffleboard settingBoard;

    private double time;
    private double prevTime;
    private double[] prevVelocity;

    private Proto() {
        MotorOne = new SparkMax(5, MotorType.kBrushless);
        MotorTwo = new SparkMax(6, MotorType.kBrushless);

        sensor1 = new DigitalInput(1);
        sensor2 = new DigitalInput(2);
        sensor3 = new DigitalInput(3);
        sensor3 = new DigitalInput(4);



        time = Timer.getFPGATimestamp();
        prevTime = time;
        prevVelocity = new double[] {0, 0}; 

        settingBoard = new MAShuffleboard("Settings");

        numForShuffleBoard = 1;

        settingBoard.addNum("Motor 1 Precent Speed", numForShuffleBoard);
        settingBoard.addNum("Motor 2 Precent Speed", numForShuffleBoard);

        settingBoard.addNum("Motor 1 Gear Ratio", numForShuffleBoard);
        settingBoard.addNum("Motor 2 Gear Ratio", numForShuffleBoard);


        settingBoard.addNeutralModeChooser("Motor 1 Neutral Mode");
        settingBoard.addNeutralModeChooser("Motor 2 Neutral Mode");


        settingBoard.addBooleanChooser("Motor 1 Enabled");
        settingBoard.addBooleanChooser("Motor 2 Enabled");


        board = new MAShuffleboard("Prototype");

        board.addNum("motor 1 velocity", 0);
        board.addNum("motor 1 distance", 0);
        board.addNum("motor 1 acceleration", 0);

        board.addNum("motor 2 velocity", 0);
        board.addNum("motor 2 distance", 0);
        board.addNum("motor 2 acceleration", 0);


    }

    private void configureMotor(SparkMax motor, IdleMode idleMode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode); 
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void activateMotors() {
        configureMotor(MotorOne, settingBoard.getNeutralMode("Motor 1 Neutral Mode"));
        configureMotor(MotorTwo, settingBoard.getNeutralMode("Motor 2 Neutral Mode"));

        
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
    }

    public boolean getSensore1() {
        return sensor1.get();
    }

    public boolean getSensore2() {
        return sensor2.get();
    }

    public boolean getSensore3() {
        return sensor3.get();
    }

    public boolean getSensore4() {
        return sensor4.get();
    }

    public void shuffleBoardOutputs() {
        time = Timer.getFPGATimestamp();
        board.addNum("motor 1 velocity", MotorOne.getEncoder().getVelocity() *  settingBoard.getNum("Motor 1 Gear Ratio"));
        board.addNum("motor 1 distance", MotorOne.getEncoder().getPosition() * settingBoard.getNum("Motor 1 Gear Ratio"));
        board.addNum("motor 1 acceleration", (MotorOne.getEncoder().getVelocity() - prevVelocity[0]) * settingBoard.getNum("Motor 1 Gear Ratio") / (time - prevTime));
        prevVelocity[0] = MotorOne.getEncoder().getVelocity();

        board.addNum("motor 2 velocity", MotorTwo.getEncoder().getVelocity() * settingBoard.getNum("Motor 2 Gear Ratio"));
        board.addNum("motor 2 distance", MotorTwo.getEncoder().getPosition() * settingBoard.getNum("Motor 2 Gear Ratio"));
        board.addNum("motor 2 acceleration", (MotorTwo.getEncoder().getVelocity() - prevVelocity[1]) * settingBoard.getNum("Motor 2 Gear Ratio") / (time - prevTime));
        prevVelocity[1] = MotorTwo.getEncoder().getVelocity();
        

        board.addBoolean("sensor 1", getSensore1());
        board.addBoolean("sensor 2", getSensore2());
        board.addBoolean("sensor 3", getSensore3());
        board.addBoolean("sensor 4", getSensore4());

        
        prevTime = time;
    }

    public void stopMotors() {
        MotorOne.setVoltage(0);
        MotorTwo.setVoltage(0);

    }
    
    public static Proto getInstance() {
        if (instance == null) {
            instance = new Proto();
        }
        return instance;
    }
}
