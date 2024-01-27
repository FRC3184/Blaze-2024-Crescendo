package frc.robot.constants;

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
public final class constShooter {

    // Constants for the shooter
    public static final class ShooterConstants {
        public static final int kMotorPortWheel1 = 11;
        public static final int kMotorPortWheel2 = 10;

        // Does the motor need to be inverted? 
        public static final boolean invertedWheel1 = true;
        public static final boolean invertedWheel2 = true;

        // Velocity Conversion Factor
        public static final double velFactorWheel1 = 1.0;
        public static final double velFactorWheel2 = 1.0;
    }

}