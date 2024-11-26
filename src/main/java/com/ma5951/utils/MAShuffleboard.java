package com.ma5951.utils;

import java.util.HashMap;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MAShuffleboard {
    private ShuffleboardTab board;
    private HashMap<String, GenericEntry> values;
    private SendableChooser<Command> commandChooser;
    private HashMap<String, SendableChooser<IdleMode>> neutralModeChoosers;
    private HashMap<String, SendableChooser<MotorType>> brushModeChoosers;
    private HashMap<String, SendableChooser<Boolean>> booleanChoosers;

    public MAShuffleboard(String tab) {
        board = Shuffleboard.getTab(tab);
        values = new HashMap<>();
        commandChooser = new SendableChooser<>();
        neutralModeChoosers = new HashMap<>();
        brushModeChoosers = new HashMap<>(); 
        booleanChoosers = new HashMap<>(); 
    }

    public void addNum(String title, double num) {
        if (!values.containsKey(title)) {
            values.put(title, board.add(title, num).getEntry());
        } else {
            values.get(title).setDouble(num);
        }
    }

    public void addString(String title, String str) {
        if(!values.containsKey(title)) {
            values.put(title, board.add(title, str).getEntry());
        } else {
            values.get(title).setString(str);
        }
    }

    public void addBoolean(String title, boolean bol) {
        if (!values.containsKey(title)) {
            values.put(title, board.add(title, bol).getEntry());
        } else {
            values.get(title).setBoolean(bol);
        }
    }

    public double getNum(String title) {
        if (values.containsKey(title)) {
            return values.get(title).getDouble(0);
        }
        System.err.println("none existing title: " + title);
        return 0;
    }

    public String getString(String title) {
        if (values.containsKey(title)) {
            return values.get(title).getString("null");
        }
        System.err.println("none existing title: " + title);
        return "null";
    }

    public Boolean getBoolean(String title) {
        if (values.containsKey(title)) {
            return values.get(title).getBoolean(false);
        }
        System.err.println("none existing title: " + title);
        return false;
    }

    public void addNeutralModeChooser(String title) {
        SendableChooser<IdleMode> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Brake", IdleMode.kBrake);
        chooser.addOption("Coast", IdleMode.kCoast);
        neutralModeChoosers.put(title, chooser);
        board.add(title, chooser);
    }
    
    public IdleMode getNeutralMode(String title) {
        if (neutralModeChoosers.containsKey(title)) {
            return neutralModeChoosers.get(title).getSelected();
        }
        System.err.println("none existing title: " + title);
        return IdleMode.kBrake;
    }

    public void addBrushModeChooser(String title) {
        SendableChooser<MotorType> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Brushed", MotorType.kBrushed);
        chooser.addOption("Brushless", MotorType.kBrushless);
        brushModeChoosers.put(title, chooser);
        board.add(title, chooser);
    }

    public MotorType getBrushMode(String title) {
        if (brushModeChoosers.containsKey(title)) {
            return brushModeChoosers.get(title).getSelected();
        }
        System.err.println("none existing title: " + title);
        return MotorType.kBrushed;
    }

    public void addBooleanChooser(String title) {
        SendableChooser<Boolean> chooser = new SendableChooser<>();
        chooser.setDefaultOption("True", true);
        chooser.addOption("False", false);
        booleanChoosers.put(title, chooser);
        board.add(title, chooser);
    }

    public Boolean getBooleanChooser(String title) {
        if (booleanChoosers.containsKey(title)) {
            return booleanChoosers.get(title).getSelected();
        }
        System.err.println("none existing title: " + title);
        return false;
    }

    public void createButton(String title, Command command) {
        SmartDashboard.putData(title, command);
    }

    public void initSendableChooser(String title) {
        board.add(title, commandChooser);
    }

    public void addOptionToChooser(String optionName, Command command) {
        commandChooser.addOption(optionName, command);
    }

    public void addDefaultOptionToChooser(String optionName, Command command) {
        commandChooser.setDefaultOption(optionName, command);
    }

    public Command getSelectedCommand() {
        return commandChooser.getSelected();
    }
}
