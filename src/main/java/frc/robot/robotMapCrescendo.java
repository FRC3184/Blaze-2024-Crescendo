package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The SwerveMap class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class robotMapCrescendo {

  // sets the robot mode, and debug outputs
    public static final class ModeConstants {
      public static final boolean intakeDebug = false;
      public static final boolean wristDebug = false;
      public static final boolean extensionDebug = false;
      public static final boolean shoulderDebug = false;
      public static final boolean drivetrainDebug = false;
      public static final boolean limelightDebug = true;
      public static final boolean flipperDebug = false;
  } 

    public static class Color{
      public static edu.wpi.first.wpilibj.DriverStation.Alliance AllianceColor = edu.wpi.first.wpilibj.DriverStation.Alliance.Red;  
  }
    public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxDefaultMPS = 2.5;
    public static final double kMaxSlowMPS = 1;
    public static final double kMaxGrannyMPS = 0.4;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    //Origional Amounts
//Direction 1.2
//Magnitude 1.8
//Rotation 2.0
    public static final double kDirectionSlewRate = 2.4; // radians per second
    public static final double kMagnitudeSlewRate = 7.2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 12.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    // TODO: Rough tuning complete, needs more refined tuning
    public static final double kFrontLeftChassisAngularOffset = -(Math.PI / 2.0);
    public static final double kFrontRightChassisAngularOffset = (0);
    public static final double kBackLeftChassisAngularOffset = (Math.PI);
    public static final double kBackRightChassisAngularOffset = (Math.PI / 2.0);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = true;

    public static boolean kFieldCentric = true;

    public static boolean kRightStickNormalMode = true;
    
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kGunnerControllerPort = 1;
    public static final int kButtonControllerPort = 2;
    public static final double kDriveDeadband = 0.05;
  }


  public static final class AutoConstants {
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

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }  

  public static final class LEDConstants {
    public static final int LeftLedPort = 9;
    public static final int RightLedPort = 8;
    public static final int LEDLen = 150; // length of current full LED strip
  }

  public static final class ArmConstants {
    // length of arm components
    // constants are in centimeters
    public static final double kGroundToShoulder = 0; 
    public static final double kShoulderToWrist = 0;
    public static final double kWristToConeEnd = 0;
    public static final double kWristToCubeEnd = 0;

    // Flipper constants
    public static final int kFlipperMotorCanId = 14;
    public static final double kFlipperMaxSpeed = 0.025;

    // wrist constants
    public static final int kWristCanId = 12;
    public static final int kWristEncToDeg = 0;
    public static final double kWristMaxPower = 0.35;
    public static final double kWristOutDegLmt = 275;
    public static final double kWristInDegLmt = 45;
    public static double kWristOutAbsLmt = 0.900;
    public static double kWristInAbsLmt = 0.318;
    public static final double kWristAbsEncTol = 0.01;
    public static final double kWristOffsetDeg = (90 - 10.60) + 180;

    public static final double kWristKp = 5;
    public static final double kWristKi = 0;
    public static final double kWristKd = 1;
    public static final double kWristKiz = 0;
    public static final double kWristKff = 0;
    public static final double kWristTolerance = 0.5;

    public static final double kWristManualIncrement = 0.002;

    // extension constants
    public static final int kExtensionCanId = 11;
    public static final int kExtensionEncToCm = 0;
    public static final int kExtensionEncRetracted = 0;
    public static int kExtensionEncExtended = -66;
    public static final double kExtendLimitInches = 23.25;
    public static final double kExtendLimitCm = 59.5;
    public static final double kExtendMaxPower = 1;
    public static final int kRetractedLimitPort = 0;

    public static final double kExtensionKp = 0.4;
    public static final double kExtensionKi = 0;
    public static final double kExtensionKd = 0;
    public static final double kExtensionKiz = 0;
    public static final double kExtensionKff = 0;
    public static final double kExtensionTolerance = 0.5;

    public static final double kExtensionManualIncrement = 0.3;

    // Shoulder constants
    public static final int kShoulderLeftCanId = 9;
    public static final boolean kShoulderLefInverted = true;
    public static final int kShoulderRightCanId = 10;
    public static final int kShoulderEncToDeg = 0;
    public static final double kShoulderMaxPower = 0.35;
    public static final double kShoulderUpAbsLmt = 0.128;
    public static final double kShoulderDownAbsLmt = 0.485;
    public static final double kShoulderUpDegLmt = 202.2;
    public static final double kShoulderDownDegLmt = 330;
    public static final double kShoulderAbsEncTol = 0.01;
    public static final double kShoulderOffsetDeg = (90 - 54.41) + 180;

    public static final double kShoulderKp = 7;
    public static final double kShoulderKi = 0;
    public static final double kShoulderKd = 0.1;
    public static final double kShoulderKiz = 0;
    public static final double kShoulderKff = 0;
    public static final double kShoulderTolerance = 0.5;

    public static final boolean kShoulderPidWrap = true;
    public static final double kShoulderPidMax = 0.999;
    public static final double kShoulderPidMin = 0;

    public static final double kShoulderManualIncrement = 0.001;

  }

  public static final class armPosConstants{
    // Stowed
    public static final double stowShoulder = 0;
    public static final double stowWrist = 0;
    // Floor Intake / Low Score 
    public static final double floorShoulder = 0.48;
    public static final double floorWrist = 0.548; //0.532
    public static final double floorExtension = 0;
    // Floor intake for 3 piece auto
    public static final double extendFloorShoulder = 0.48;
    public static final double extendFloorWrist = 0.58;
    public static final double extendFloorExtension = 0;
    // Prep arm for extention in auto
    public static final double extendFloorShoulderPrep = 0.374;
    public static final double extendFloorWristPrep = 0.642;
    public static final double extendFloorExtensionPrep = 0;
    //Mid Shoot Arm Position
    public static final double MidshootArmShoulder = .13;
    public static final double MidshootArmWrist = .385; //.367
    public static final double MidshootArmExtension = 0;
    // Low Shoot Arm Position
    public static final double LowshootArmShoulder = .13;
    public static final double LowshootArmWrist = .318;
    public static final double LowshootArmExtension = 0;
    // New Floor Intake
    public static final double floor2Shoulder = 0.41;
    public static final double floor2Wrist = 0.652;
    public static final double floor2Extension = 0;
    //Balance arm position
    public static final double BalanceShoulder = 0.48;
    public static final double BalanceWrist = 0.35;
    public static final double BalanceExtension = 0;
    // Shelf Intake
    public static final double shelfShoulder = 0.13;
    public static final double shelfWrist = 0.9;
    public static final double shelfExtension = 0;
    // Mid Score
    public static final double midShoulder = 0.223;
    public static final double midWrist = 0.842;//.826
    public static final double midExtension = -12.3;
    // High Score
    public static final double highShoulder = 0.224;
    public static final double highWrist = 0.802;//.786
    public static final double highExtension = -65.9;
  }  

  public static final class IntakeConstants {
    public static final int kIntakeCanId = 13;
    public static final double kIntakeMaxPower = 0.3;
    public static final double kIntakeShootPower = 0.6;
  }

  public static enum GridPosition {
    ONE(0.670),
    TWO(1.072),
    THREE(1.674),
    FOUR(2.276),
    FIVE(2.748),
    SIX(3.348),
    SEVEN(3.9),
    EIGHT(4.424),
    NINE(5);

    public final double offset;

    private GridPosition(double offset) {
      this.offset = offset;
    }
  }

  public static final class TrajectoryValues{
    public static double goalX;
    public static double goalY = 0;
  }

  public static final class AprilTags{
    public static final Map<Integer, Pose3d> aprilTags =
      Map.of(
          1,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          2,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          3,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI)),
          4,
          new Pose3d(
              Units.inchesToMeters(636.96),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0.0, 0.0, Math.PI)),
          5,
          new Pose3d(
              Units.inchesToMeters(14.25),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d()),
          6,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d()),
          7,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d()),
          8,
          new Pose3d(
              Units.inchesToMeters(40.45), //1.027
              Units.inchesToMeters(42.19), //1.072
              Units.inchesToMeters(18.22), //0.463
              new Rotation3d())
        );
  }

  public static final class DebugConstants {
    public static final boolean driveDebug = true;
    public static final boolean armDebug = false;
  }
}