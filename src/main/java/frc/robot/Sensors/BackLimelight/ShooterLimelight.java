package frc.robot.Sensors.BackLimelight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.Mechanisms.sensorTypes.Limelight;

public class ShooterLimelight extends Limelight {

    public ShooterLimelight () {
        super("limelight-shooter", "Shooter Limelight");
        if(DriverStation.getAlliance().get() == Alliance.Red){
            setPriorityID(4);
        }
        else if(DriverStation.getAlliance().get() == Alliance.Blue){
            setPriorityID(7);
        }
    }

}
