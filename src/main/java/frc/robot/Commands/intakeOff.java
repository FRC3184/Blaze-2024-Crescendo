package frc.robot.Commands;

import frc.Mechanisms.rollers.off;
import frc.robot.Subsystems.Intake;

public class intakeOff extends off {

    public intakeOff(Intake subsystem){
        super(subsystem);

        addRequirements(subsystem); 
    }

    public void initialize() {}
}
