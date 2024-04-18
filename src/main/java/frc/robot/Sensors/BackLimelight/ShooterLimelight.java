package frc.robot.Sensors.BackLimelight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.Mechanisms.sensorTypes.Limelight;

public class ShooterLimelight extends Limelight {

    public ShooterLimelight () {
        super("limelight-shooter", "Shooter Limelight");

        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if(currentAlliance == Alliance.Red) {
            setPriorityID(4);
        }
        else {
            setPriorityID(7);
        }
    }

}
