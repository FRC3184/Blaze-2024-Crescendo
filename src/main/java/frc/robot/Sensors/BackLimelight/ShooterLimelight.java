package frc.robot.Sensors.BackLimelight;

import frc.Mechanisms.sensorTypes.Limelight;

public class ShooterLimelight extends Limelight {

    public ShooterLimelight () {
        super("limelight-shooter", "Shooter Limelight");
    }

    @Override
    public void periodic() {
        updateLimelight();
        dashboardDebug();
    }

    private void updateLimelight() {
        tid = shooterLimeTable.getEntry("tid");
        tx = shooterLimeTable.getEntry("tx");
        ty = shooterLimeTable.getEntry("ty");
        ta = shooterLimeTable.getEntry("ta");
        ts = shooterLimeTable.getEntry("ts");
        botposeblue = shooterLimeTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        targetpose_robotspace = shooterLimeTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        

        id_num = tid.getDouble(0.0);
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        skew = ts.getDouble(0.0);
    }
    
    public double getShooterX(){
        updateLimelight();
        return x;
    }

    public double getShooterY(){
        updateLimelight();
        return y;
    }

    public double getShooterS(){
        updateLimelight();
        return skew;
    }

    private void dashboardDebug() { 
        updateLimelight();

        // SmartDashboard.putNumberArray("botPoseBlue", botposeblue);
        // SmartDashboard.putNumber("x", botposeblue[0]);
        // SmartDashboard.putNumber("y", botposeblue[1]);
        // SmartDashboard.putNumber("z", botposeblue[2]);
        // SmartDashboard.putNumber("roll", botposeblue[3]);
        // SmartDashboard.putNumber("pitch", botposeblue[4]);
        // SmartDashboard.putNumber("yaw", botposeblue[5]);

        SmartDashboard.putNumber("ShooterLL X", x);
        SmartDashboard.putNumber("ShooterLL Y", y);
        SmartDashboard.putNumber("ShooterLL Area", area);
        SmartDashboard.putNumber("ShooterLL Skew", skew);

        Optional<Double> angle = calculateAngle(Math.hypot(targetpose_robotspace[0], targetpose_robotspace[2]), -targetpose_robotspace[1], 4000.0);
        SmartDashboard.putNumber("angle or whatever", Math.toDegrees(angle.orElse(10000000.0)));
    }

    public static final class alignmentConstants{
        public static final double Aligned = 0.0;

    }

    // all this should prob be moved somewhere else at some point,
    // i just put it here because it needed to be written

    // this math is based on this wikipedia article
    // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)

    // some of these constants need values

    // mult this by rpm to get expected speed for note when shot
    // in m/s
    // motor-flywheel gear ratio, flywheel diameter in inches, conversion to meters, pi, factor to adjust if needed   
    private static final double VELOCITY_RPM_FACTOR = (42/24) * (4 / 39.37009424) * Math.PI * 1 / 60;
    // standard gravity
    private static final double GRAVITY = 9.80665;

    // min and max angle that the shooter can reach in RADIANS
    private static final double MAX_ANGLE = 0;
    private static final double MIN_ANGLE = 0;
    
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
    public Optional<Double> calculateAngle(double distance, double height, double rpm) {
        double noteSpeed = rpm * VELOCITY_RPM_FACTOR;
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
