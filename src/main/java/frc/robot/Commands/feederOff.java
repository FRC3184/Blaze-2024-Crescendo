package frc.robot.Commands;

import frc.Mechanisms.rollers.off;
import frc.robot.Subsystems.Feeder;

public class feederOff extends off {

    public feederOff(Feeder subsystem){
        super(subsystem);

        addRequirements(subsystem); 
    }

    public void initialize() {}
}
