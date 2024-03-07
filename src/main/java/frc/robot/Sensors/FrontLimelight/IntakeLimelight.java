package frc.robot.Sensors.FrontLimelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeLimelight extends SubsystemBase {
    NetworkTable intakeLimeTable = NetworkTableInstance.getDefault().getTable("limelight-intake");
    double x, y, area, skew, id_num;
    NetworkTableEntry tx, ty, ta, ts, tid;
    public static double[] botposeblue;
    public IntakeLimelight () {
        updateLimelight();
        dashboardDebug();
    }

    @Override
    public void periodic() {
        updateLimelight();
        dashboardDebug();
    }

    private void updateLimelight() {
        tid = intakeLimeTable.getEntry("tid");
        tx = intakeLimeTable.getEntry("tx");
        ty = intakeLimeTable.getEntry("ty");
        ta = intakeLimeTable.getEntry("ta");
        ts = intakeLimeTable.getEntry("ts");
        botposeblue = intakeLimeTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        

        id_num = tid.getDouble(0.0);
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        skew = ts.getDouble(0.0);
    }
    
    public double getIntakeX(){
        updateLimelight();
        return x;
    }

    public double getIntakeY(){
        updateLimelight();
        return y;
    }

    public double getIntakeS(){
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

        SmartDashboard.putNumber("IntakeLL X", x);
        SmartDashboard.putNumber("IntakeLL Y", y);
        SmartDashboard.putNumber("IntakeLL Area", area);
        SmartDashboard.putNumber("IntakeLL Skew", skew);
    }

    public static final class alignmentConstants{
        public static final double Aligned = 0.0;

    }
}
