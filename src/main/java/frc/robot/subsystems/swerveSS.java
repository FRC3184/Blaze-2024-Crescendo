package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight;
import java.time.Instant;
import java.util.HashMap;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.commands.VisionCommands.SwerveTrajectoryCommand;
import frc.robot.maps.SwerveMap.AutoConstants;
import frc.robot.maps.SwerveMap.DebugConstants;
import frc.robot.maps.SwerveMap.DriveConstants;
import frc.robot.maps.SwerveMap.ModeConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveSSSwerve extends SubsystemBase {
  public static SwerveControllerCommand YEs;
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // private final ADIS16470_IMU m_navxGyro = new ADIS16470_IMU();
  private final AHRS m_navxGyro = new AHRS();
  private final Limelight m_limelight = new Limelight();


  private int speedMode = 1;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private SwerveControllerCommand cmd;
  private boolean trajectoryRunning = false;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
      
    
  /** Creates a new DriveSubsystem. */
  public DriveSSSwerve() {
    dashboardDebug();
  }

  double currHeading;

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    currHeading = getHeading();
    absGyro360();
    absGyro180();

    if (DebugConstants.driveDebug) {
      dashboardDebug();
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getTagPose() {
    return new Pose2d(m_limelight.botposeblue[0],m_limelight.botposeblue[1],Rotation2d.fromDegrees(m_limelight.botposeblue[5]));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
     double xSpeedDelivered = xSpeedCommanded * (getSpeed());
     double ySpeedDelivered = ySpeedCommanded * (getSpeed());
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, (getSpeed()));

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  private double getSpeed(){
    switch(speedMode){
      case 0:
        return DriveConstants.kMaxSlowMPS;
      case 2:
        return DriveConstants.kMaxSpeedMetersPerSecond;
      case 3:
        return DriveConstants.kMaxGrannyMPS;
      default:
        return DriveConstants.kMaxDefaultMPS;
    }
  }
  public double getPitch(){
    return m_navxGyro.getPitch();
  }

  public double getX(){
    return m_limelight.getX();
  }
  public double getY(){
    return m_limelight.getY();
  }

  public double getS(){
    return m_limelight.getS();
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setStraight() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  // public boolean isStraight() {
  //   return (m_frontLeft.getState() == )
  // }

  /**
   * Alternative heading calculations that are not continuous
   */
  private double heading360 = 0;
  private double heading180 = 0;
  
  private double absGyro360() {
    heading360 = (currHeading >= 0 ?  currHeading % 360 : 360 + (currHeading % 360));
    return heading360;
  }

  private double absGyro180() {
    heading180 = (heading360 <= 180 ? heading360 % 180 : -(180 - (heading360 % 180)));
    return heading180;
  }

  /**
   * Rotate Robot to a specified angle
   */
  private double tolerance = 2.0;
  private double rotSpeed = 0;
  private double distToTarget = 0;

  public void rotateToAngle(double target, double xSpeed, double ySpeed) {
    rotSpeed = (target - heading180) / 180;

    if (Math.abs(rotSpeed) < (tolerance / 360.0)) {
      rotSpeed = 0;
    }
    
    drive(xSpeed, ySpeed, rotSpeed, true, true);
  }

  public void rotateToAngle2(double target, double xSpeed, double ySpeed) {
    distToTarget = ((target - heading360) + 360.0) % 360.0;
    if (distToTarget < 180){
      rotSpeed = distToTarget / 180.0;
    } else {
      rotSpeed = -((360.0 - distToTarget) / 180.0);
    }

    if (Math.abs(rotSpeed) < (tolerance / 360.0)) {
      rotSpeed = 0;
    }
    
    drive(xSpeed, ySpeed, rotSpeed, true, true);
  }
  

  public void limelightAlignX() {
    // double aligned = 0;
    // double speedMax = 0.65; 
    // double xMax = -30;
    // if (getX()<aligned - 3) {
    //     drive(0, (getX() / xMax) * speedMax, 0, false, false);
    // } else if(getX()>aligned + 3) {
    //     drive(0, (getX() / xMax) * speedMax, 0, false, false);
    // } else {
    //     //drive(0, 0, 0, false, false);
    // }
    double x = m_limelight.botposeblue[0];
    double y = m_limelight.botposeblue[1];
    double yaw = m_limelight.botposeblue[5];
    
    Pose2d Here = new Pose2d(x,y,Rotation2d.fromDegrees(yaw));
    Pose2d There = new Pose2d(1.8,1.1,Rotation2d.fromDegrees(180));
    //Pose2d HereNorThere = Here
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    var Trajectory = TrajectoryGenerator.generateTrajectory(Here, List.of(new Translation2d(There.getX(), There.getY())), There, config);
     YEs = new SwerveControllerCommand(Trajectory, this::getTagPose,DriveConstants.kDriveKinematics,new PIDController(8, 0, 0), new PIDController(0, 8, 0), new ProfiledPIDController(0,0,0,AutoConstants.kThetaControllerConstraints),this::setModuleStates,this);

  }
public void limelightAlignS() {
  double aligned = 0;
  double speedMax = 0.65; 
  double SkewMax = 0;
  if (getS()<aligned - 3) {
      drive(0, 0, 0, false, false);
  } else if(getX()>aligned + 3) {
      drive(0, 0, 0.1, false, false);
  } else {
      //drive(0, 0, 0, false, false);
  }
}

  public void autoBalance() {
    double balanced = 1.5;
    double delta = Math.abs(balanced-getPitch());
    if (getPitch()<balanced - 12){
        drive(-.162/DriveConstants.kMaxSlowMPS, 0, 0, false, false);
    }
    else if(getPitch()>balanced + 12){
        drive(.162/DriveConstants.kMaxSlowMPS, 0, 0, false, false);
    }
    // else if((getPitch() > balanced - 1 &&  getPitch() < balanced - 2)){
    //   drive(-DriveConstants.kMaxSlowMPS*(0.3), 0, 0, false, false);
    // }
    // else if((getPitch() > balanced + 1 &&  getPitch() < balanced + 2)) {
    //   drive(DriveConstants.kMaxSlowMPS*(0.3), 0, 0, false, false);
    // }
    else{
        drive(0, 0, 0, true, false);
        setX();
    }
  }

  public void driveToBalance() {
    double balanced = 1.5;
    if (getPitch() < -12.9){ //-12.9
      drive(-DriveConstants.kMaxSlowMPS/5.0, 0, 0, false, true);
    } else if(getPitch() > 15.4){ //15.4
      drive(DriveConstants.kMaxSlowMPS/5.0, 0, 0, false, true);
    } else {
      drive(0, 0, 0, false, true);
    }
  }

  public void DriveToCharge() {
    if (getPitch()>-13){
      drive(-DriveConstants.kMaxSlowMPS/2.0, 0, 0, false, false);
    }
    else {
      drive(0, 0, 0, true, false);
    }
  }
public Command followpath(PathPlannerTrajectory traj, Boolean isFirstPath) {
      return new SequentialCommandGroup(
              new InstantCommand(() -> {
                if(isFirstPath){
                  this.resetOdometry(traj.getInitialHolonomicPose());
                }
    
              }),
      new PPSwerveControllerCommand(
           traj, 
           this::getPose, // Pose supplier
           DriveConstants.kDriveKinematics, // SwerveDriveKinematics
           new PIDController(8, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(8, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(6, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           this::setModuleStates, // Module states consumer
           false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           this// Requires this drive subsystem
       ));
}

public void goToPoint(Trajectory traj){
  PIDController xController = new PIDController(8, 0, 0);
  PIDController yController = new PIDController(8, 0, 0);
  ProfiledPIDController thetaController = new ProfiledPIDController(8, 0, 0, AutoConstants.kThetaControllerConstraints);

  cmd = new SwerveControllerCommand(
    traj, 
    this::getTagPose,
    DriveConstants.kDriveKinematics,
    xController, 
    yController,
    thetaController,
    this::setModuleStates,
    this
  );
  cmd.schedule();
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, (getSpeed()));
  m_frontLeft.setDesiredState(desiredStates[0]);
  m_frontRight.setDesiredState(desiredStates[1]);
  m_rearLeft.setDesiredState(desiredStates[2]);
  m_rearRight.setDesiredState(desiredStates[3]);
}


  public void setFieldcentric(boolean fieldcentric)
  {
    DriveConstants.kFieldCentric = fieldcentric;
  }

  public void setRightStickNormalMode(boolean rightstickmode)
  {
    DriveConstants.kRightStickNormalMode = rightstickmode;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  
  public SwerveModuleState[]  getModuleStates() {
    return new SwerveModuleState[]{m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()};
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_navxGyro.reset();
    m_navxGyro.setAngleAdjustment(0);
  }

  // rotates heading by 180 degrees 
  public void flipHeading() {
    m_navxGyro.setAngleAdjustment(180);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navxGyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public void setSpeedMode(Integer Mode){
    speedMode = Mode;
  }

  public boolean isTrajectoryRunning() {
    return trajectoryRunning;
  }

  public Command getTrajectoryCommand() {
    return cmd == null ? new WaitCommand(0.01) : cmd;
  }

  public void dashboardDebug(){
    if (ModeConstants.drivetrainDebug) {
      SmartDashboard.putNumber("Speed Mode", speedMode);
      SmartDashboard.putNumber("Heading", getHeading());
      SmartDashboard.putNumber("Heading360", heading360);
      SmartDashboard.putNumber("Rotate Speed", rotSpeed);
      SmartDashboard.putNumber("Pitch", getPitch());
      SmartDashboard.putBoolean("FieldCentric", DriveConstants.kFieldCentric);

      SmartDashboard.putNumber("X Coordinate", getPose().getX());
      SmartDashboard.putNumber("Y Coordinate", getPose().getY());

      trajectoryRunning = cmd != null ? cmd.isScheduled() : false;

      SmartDashboard.putBoolean("is scheduled", trajectoryRunning);

      SmartDashboard.putNumber("front left rotation", getModuleStates()[0].angle.getDegrees());
      SmartDashboard.putNumber("front right rotation", getModuleStates()[1].angle.getDegrees());
      SmartDashboard.putNumber("rear left rotation", getModuleStates()[2].angle.getDegrees());
      SmartDashboard.putNumber("rear right rotation", getModuleStates()[3].angle.getDegrees());
    }
  }

  public void dashboardComp() {
    SmartDashboard.putNumber("Speed Mode", speedMode);

  }
}