package frc.robot.Commands;

import frc.Mechanisms.flywheel.backward;
import frc.robot.Subsystems.ShooterFlywheels;

public class shooterFlywheelBackward extends backward {

    public shooterFlywheelBackward(ShooterFlywheels subsystem){
        super(subsystem);
    }

    public void initialize() {}
}
