package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RobotMap {
    public static class Color{
        public static edu.wpi.first.wpilibj.DriverStation.Alliance AllianceColor = edu.wpi.first.wpilibj.DriverStation.Alliance.Red;  
    }

    public class constsJoysticks {

        public static final int kDriverControllerPort = 0;
        public static final int kGunnerControllerPort = 1;
        public static final int kButtonControllerPort = 2;
        public static final double kDriveDeadband = 0.05;
    
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
      } 

      public static final class LEDConstants {
        public static final int LeftLedPort = 9;
        public static final int RightLedPort = 8;
        public static final int LEDLen = 150; // length of current full LED strip
      }

      public final class constShooter {

        // Constants for the shooter
        public static final int kMotorPortWheel1 = 5;
        public static final int kMotorPortWheel2 = 6;
    
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
        public static final double kFF = 1.0/5700.0;
        public static final double kMaxOut = 1;
        public static final double kMinOut = -1;
        public static final double maxRPM = 5700;
    }

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

    public class constAuto {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}

public class constMode {
    public static final boolean intakeDebug = false;
    public static final boolean wristDebug = false;
    public static final boolean extensionDebug = false;
    public static final boolean shoulderDebug = false;
    public static final boolean drivetrainDebug = false;
    public static final boolean limelightDebug = true;
    public static final boolean armDebug = false;
}


}
