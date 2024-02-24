package frc.robot.Sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
    NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");
    double x, y, area, skew, id_num;
    NetworkTableEntry tx, ty, ta, ts, tid;
    public static double[] botposeblue;
    public Limelight () {
        updateLimelight();
        dashboardDebug();
    }

    @Override
    public void periodic() {
        updateLimelight();
        dashboardDebug();
    }

    private void updateLimelight() {
        tid = limeTable.getEntry("tid");
        tx = limeTable.getEntry("tx");
        ty = limeTable.getEntry("ty");
        ta = limeTable.getEntry("ta");
        ts = limeTable.getEntry("ts");
        botposeblue = limeTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        

        id_num = tid.getDouble(0.0);
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        skew = ts.getDouble(0.0);
    }
    
    public double getX(){
        updateLimelight();
        return x;
    }

    public double getY(){
        updateLimelight();
        return y;
    }

    public double getS(){
        updateLimelight();
        return skew;
    }

    private void dashboardDebug() {  
        SmartDashboard.putNumberArray("botPoseBlue", botposeblue);
        SmartDashboard.putNumber("x", botposeblue[0]);
        SmartDashboard.putNumber("y", botposeblue[1]);
        SmartDashboard.putNumber("z", botposeblue[2]);
        SmartDashboard.putNumber("roll", botposeblue[3]);
        SmartDashboard.putNumber("pitch", botposeblue[4]);
        SmartDashboard.putNumber("yaw", botposeblue[5]);

        SmartDashboard.putNumber("GETY", getY());
        SmartDashboard.putNumber("GETX", getX());
    }

    public static final class alignmentConstants{
        public static final double Aligned = 0.0;

    }
}
