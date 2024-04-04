package frc.robot.Sensors.BackLimelight;

import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.Mechanisms.sensorTypes.Limelight;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;

public class ShooterLimelight extends Limelight {
    NetworkTable LimelightTable;
    public double angle;
    public static double[] targetpose_robotspace;
    protected ShuffleboardTab sbTab;
    private GenericEntry shooterAngle;

    private static double offsetX = ((2 + 1/2 + 18.11) * 5.5400554295247);
    private static double offsetY = (8 + 1/4) * 5.5400554295247;
    private static double offsetZ = 0;
    
    public ShooterLimelight () {
        super("limelight-shooter", "Shooter Limelight");
    }

    @Override
    public void periodic() {
        updateLimelight();

        Optional<Double> a = calculateAngle(Math.hypot(targetpose_robotspace[0] - offsetX, targetpose_robotspace[2] - offsetZ), (80.515 * 5.5400554295247) - offsetY, ConstShooter.defVelocity);
        angle = Math.toDegrees(a.orElse(1000000000.0));

        dashboardUpdate();
    }

    @Override
    protected void updateLimelight() {
        // calls OG updateLimelight function so we don't have to keep it up-to-date with Mechanisms
        super.updateLimelight();

        targetpose_robotspace = LimelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    @Override
    protected void dashboardInit() {
        // same as above
        super.dashboardInit();

        shooterAngle = sbTab.add("Target Shooter Angle", angle).getEntry();
    }

    @Override
    protected void dashboardUpdate() { 
        // same as above
        super.dashboardUpdate();

        shooterAngle.setDouble(angle);
    }

    // this math is based on this wikipedia article
    // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)

    // standard gravity
    private static final double GRAVITY = 9.80665;

    // NOTE: these functions expect meters as the units for distance and height

    // calculates angle that allows lowest launch velocity
    // WARNING: the returned angle is in RADIANS!!!
    public Pair<Double, Double> calculateAngleVelocity(double distance, double height) {
        double speed = Math.sqrt(GRAVITY * (height + Math.sqrt(height*height + distance*distance))) / ConstShooter.VelocityRpmFactor;
        double theta = (Math.PI / 2) - (((Math.PI / 2) - Math.atan(height/distance))/2);

        return new Pair<Double, Double>(theta, speed);
    }

    // calculates angle given flywheel motor rpm
    // WARNING: the returned angle is in RADIANS!!!
    public Optional<Double> calculateAngle(double distance, double height, double rpm) {
        double noteSpeed = rpm * ConstShooter.VelocityRpmFactor;
        try {
            double sqrtPart = Math.sqrt(Math.pow(noteSpeed, 4) - GRAVITY * (GRAVITY * distance * distance + 2 * height * noteSpeed * noteSpeed));

            double theta1 = Math.atan((noteSpeed * noteSpeed + sqrtPart) / (GRAVITY * distance));
            double theta2 = Math.atan((noteSpeed * noteSpeed - sqrtPart) / (GRAVITY * distance));

            return Optional.of(Math.min(theta1, theta2));
        }
        catch (ArithmeticException e) {
            return Optional.empty();
        }
    }

    // key: distance, value: pair of angle & rpm
    private static final TreeMap<Double, Pair<Double, Double>> shooter_map = new TreeMap<Double, Pair<Double, Double>>();
    
    static {
        // fill table with distance -> pair of angle & rpm
        shooter_map.put(0.0, new Pair<Double, Double>(0.0, 0.0));
        shooter_map.put(1.0, new Pair<Double, Double>(1.0, 1.0));
    };
}
