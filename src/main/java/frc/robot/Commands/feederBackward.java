package frc.robot.Commands;

import frc.Mechanisms.rollers.backward;
import frc.robot.Subsystems.Feeder;

public class feederBackward extends backward {

    public feederBackward(Feeder subsystem){
        super(subsystem);
    }

    public void initialize() {}
}
