package frc.robot.Commands;

import frc.Mechanisms.rollers.off;
import frc.robot.Subsystems.CarriageBelt;

public class carriageBeltOff extends off {

    public carriageBeltOff(CarriageBelt subsystem){
        super(subsystem);
        addRequirements(subsystem); 
    }

    public void initialize() {}
}
