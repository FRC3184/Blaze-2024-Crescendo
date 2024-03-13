package frc.robot.Commands;

import frc.Mechanisms.rollers.backward;
import frc.robot.Subsystems.Intake;

public class intakeBackward extends backward {

    public intakeBackward(Intake subsystem){
        super(subsystem);
    }

    public void initialize() {}
}
