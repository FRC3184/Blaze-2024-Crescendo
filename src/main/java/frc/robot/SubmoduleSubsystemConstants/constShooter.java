package frc.robot.SubmoduleSubsystemConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class ConstShooter {

  // Constants for the shooter
  public static final int kMotorPortWheel1 = 10;
  public static final int kMotorPortWheel2 = 11;

  // Does the motor need to be inverted?
  public static final boolean invertedWheel1 = true;
  public static final boolean invertedWheel2 = true;

  // Velocity Conversion Factor
  public static final double velFactorWheel1 = 1.0;
  public static final double velFactorWheel2 = 1.0;

  // PID Controller Constants
  public static final double kP = 6e-5;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kIz = 0;
  public static final double kFF = 1.0 / 5700.0;
  public static final double kMaxOut = 1;
  public static final double kMinOut = -1;
  public static final double maxRPM = 5700;
}