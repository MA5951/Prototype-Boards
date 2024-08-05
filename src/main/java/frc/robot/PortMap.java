package frc.robot;

public class PortMap {

  public static class CanBus {
    public static final String CANivoreBus = "Swerve";
    public static final String RioBus = "rio";
  }

  public static class Controllers {
    public static final int driveID = 0;
  }

  public static class Swerve {
    public static final int leftFrontAbsoluteEncoder = 22;
    public static final int leftFrontDriveID = 8; //8
    public static final int leftFrontTurningID = 5; //5

    public static final int leftBackAbsoluteEncoder = 21;
    public static final int leftBackDriveID = 4;
    public static final int leftBackTurningID = 9;

    public static final int rightFrontAbsoluteEncoder = 23;
    public static final int rightFrontDriveID = 7;
    public static final int rightFrontTurningID = 6;

    public static final int rightBackAbsoluteEncoder = 24;
    public static final int rightBackDriveID = 2;
    public static final int rightBackTurningID = 3;

    // public static final int Pigeon2ID = 12;
    // public static final int leftFrontAbsoluteEncoder = 30;
    // public static final int leftFrontDriveID = 31;
    // public static final int leftFrontTurningID = 32;

    // public static final int leftBackAbsoluteEncoder = 33;
    // public static final int leftBackDriveID = 34;
    // public static final int leftBackTurningID = 35;

    // public static final int rightFrontAbsoluteEncoder = 36;
    // public static final int rightFrontDriveID = 37;
    // public static final int rightFrontTurningID = 38;

    // public static final int rightBackAbsoluteEncoder = 39;
    // public static final int rightBackDriveID = 40;
    // public static final int rightBackTurningID = 41;

    public static final int Pigeon2ID = 12;
  }
}