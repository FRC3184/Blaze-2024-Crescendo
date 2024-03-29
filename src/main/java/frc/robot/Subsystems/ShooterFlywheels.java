package frc.robot.Subsystems;

import edu.wpi.first.math.Pair;
import frc.Mechanisms.configurationTypes.TwoMotorSSwithSparkMax;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;

public class ShooterFlywheels extends TwoMotorSSwithSparkMax {

  public ShooterFlywheels() {
    super(ConstShooter.kMotorPortWheel1, ConstShooter.kMotorPortWheel2, 
          ConstShooter.invertedWheel1, ConstShooter.invertedWheel2, 
          ConstShooter.velFactorWheel1, ConstShooter.velFactorWheel2, "Flywheels");
  }

  // all this should prob be moved somewhere else at some point,
  // i just put it here because it needed to be written

  // this math is based on this wikipedia article
  // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)

  // some of these constants need values

  // mult this by rpm to get expected speed for note when shot
  // in m/s
  // motor-flywheel gear ratio, flywheel diameter in inches, conversion to meters, pi, factor to adjust if needed   
  public static final double VELOCITY_RPM_FACTOR = (42/24) * 4 / 39.37009424 * Math.PI * 1;
  // standard gravity
  public static final double GRAVITY = 9.80665;

  // min and max angle that the shooter can reach in RADIANS
  public static final double MAX_ANGLE = 0;
  public static final double MIN_ANGLE = 0;
  
  // NOTE: these functions expect meters as the units for distance and height

  // calculates angle that allows lowest launch velocity
  // WARNING: the returned angle is in RADIANS!!!
  public Pair<Double, Double> calculateAngleVelocity(double distance, double height) {
    double speed = Math.sqrt(GRAVITY * (height + Math.sqrt(height*height + distance*distance))) / VELOCITY_RPM_FACTOR;
    double theta = (Math.PI / 2) - (((Math.PI / 2) - Math.atan(height/distance))/2);

    return new Pair<Double, Double>(theta, speed);
  }

  // calculates angle given flywheel motor rpm
  // WARNING: the returned angle is in RADIANS!!!
  public double calculateAngle(double distance, double height, double rpm) {
    double noteSpeed = rpm * VELOCITY_RPM_FACTOR;
    double sqrtPart = Math.sqrt(Math.pow(noteSpeed, 4) - GRAVITY * (GRAVITY * distance * distance + 2 * height * noteSpeed * noteSpeed));
    
    double theta1 = Math.atan((noteSpeed * noteSpeed + sqrtPart) / (GRAVITY * distance));
    double theta2 = Math.atan((noteSpeed * noteSpeed - sqrtPart) / (GRAVITY * distance));

    return Math.min(theta1, theta2);
  }
}