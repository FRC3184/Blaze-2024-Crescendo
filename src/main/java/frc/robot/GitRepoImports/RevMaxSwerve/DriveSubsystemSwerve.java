package frc.robot.GitRepoImports.RevMaxSwerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.GitRepoImports.RevMaxSwerve.constantsMaxSwerve.DriveConstants;
import frc.robot.RobotMap.ModeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveSubsystemSwerve extends SubsystemBase {
  public static SwerveControllerCommand YEs;
  // Create MAXSwerveModules
  // IMPORTANT - Redefine each module based on which motor controller you are using
  private final MAXSwerveModuleWithSparkMax m_frontLeft = new MAXSwerveModuleWithSparkMax(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModuleWithSparkMax m_frontRight = new MAXSwerveModuleWithSparkMax(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModuleWithSparkMax m_rearLeft = new MAXSwerveModuleWithSparkMax(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModuleWithSparkMax m_rearRight = new MAXSwerveModuleWithSparkMax(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // private final ADIS16470_IMU m_navxGyro = new ADIS16470_IMU();
  private final AHRS m_navxGyro = new AHRS();

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
  public DriveSubsystemSwerve() {
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

    if (ModeConstants.drivetrainDebug) {
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
      double angleDif = swerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = swerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = swerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = swerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
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

