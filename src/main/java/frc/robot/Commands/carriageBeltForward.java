package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstCarriage;
import frc.robot.SubmoduleSubsystemConstants.ConstFeeder;
import frc.robot.Subsystems.CarriageBelt;

public class carriageBeltForward extends Command {
    private final CarriageBelt carriageBelt;

    public carriageBeltForward(CarriageBelt subsystem){
        carriageBelt = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        carriageBelt.setVelocity(ConstCarriage.defaultVelocity);
    }

    @Override
    public void execute() {
        carriageBelt.runVel();
    }

    @Override
    public void end(boolean interrupted) {
        carriageBelt.stop();
    }
}
