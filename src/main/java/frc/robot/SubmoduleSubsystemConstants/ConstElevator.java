package frc.robot.SubmoduleSubsystemConstants;

public class ConstElevator {
    public static final int kMotorPortElevator1 = 14; //left
    public static final int kMotorPortElevator2 = 4; //right

    //Does the motor need to be inverted?
    public static final boolean kElevator1Inverted = false;
    public static final boolean kElevator2Inverted = true;

    //Velocity Conversion Factor
    public static final double velFactorElevator1 = 1.0;
    public static final double velFactorElevator2 = 1.0;

    // PID Controller Constants
  public static final double kP = 6e-4;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kIz = 0;
  public static final double kFF = 1.0 / 5700.0;
  public static final double kMaxOut = 1;
  public static final double kMinOut = -1;
  public static final double maxRPM = 5700;
}
