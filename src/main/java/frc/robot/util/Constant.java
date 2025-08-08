package frc.robot.util;

public class Constant {

  public static final class elevatorConstants {
    public static int masterID = 21;
    public static int followerID = 23;

    public static double FEED = -.3;
    // public static double L1Level = 0.0; // Placeholder replace with real value from robot
    public static double L2Level = -1; // Placeholder replace with real value from robot
    public static double L1AlgeaLevel = -2; // Placeholder replace with real value from robot
    public static double L3Level = -3; // Placeholder replace with real value from robot
    public static double L2AlgeaLevel = -4; // possibly the real value
    public static double L4Level = -5; // Placeholder replace with real value from robot

    public static final double kMaxSpeedMetersPerSecond = 4.6;

    public static final double kDirectionSlewRate = 1.2; // radians per second
  }

  public static final class ArmConstants {
    public static int masterID = 22;

    public static double FEED = 2;
    public static double L1Level = 1.4; // Placeholder replace with real value from robot
    public static double L2Level = 1; // Placeholder replace with real value from robot
    public static double L1AlgeaLevel = 1; // Placeholder replace with real value from robot
    public static double L3Level = .4; // Placeholder replace with real value from robot
    public static double L2AlgeaLevel = .4; // possibly the real value
    public static double L4Level = .3; // Placeholder replace with real value from robot

    public static final double kMaxSpeedMetersPerSecond = 4.6;

    public static final double kDirectionSlewRate = 1.2; // radians per second
  }
}
