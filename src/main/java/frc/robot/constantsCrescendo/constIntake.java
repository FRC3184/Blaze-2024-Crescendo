package frc.robot.constantsCrescendo;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class constIntake {

    // Constants for the shooter
    public static final int kIntakePort1 = 19;
    public static final int kIntakePort2 = 20;

    // Does the motor need to be inverted? 
    public static final boolean kIntakeInverted1 = false;
    public static final boolean kIntakeInverted2 = false;

    // Velocity Conversion Factor
    public static final double kIntakeVelFactor1 = 1.0;
    public static final double kIntakeVelFactor2 = 1.0;

    //
    public static final double kMaxPower = 1.0;
    public static final double kMinPower = -1.0;
}