package frc.robot.Sensors.BackLimelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterLimelight extends SubsystemBase {
    NetworkTable shooterLimeTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    double x, y, area, skew, id_num;
    NetworkTableEntry tx, ty, ta, ts, tid;
    public static double[] botposeblue;
    public ShooterLimelight () {
        updateLimelight();
        dashboardDebug();
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
    }

    public static final class alignmentConstants{
        public static final double Aligned = 0.0;

    }
}
