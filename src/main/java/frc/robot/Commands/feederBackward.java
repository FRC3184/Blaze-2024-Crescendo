package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstFeeder;
import frc.robot.Subsystems.Feeder;

public class feederBackward extends Command {
    private final Feeder feeder;

    public feederBackward(Feeder subsystem){
        feeder = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        feeder.setSpeed(-0.5);
    }

    @Override
    public void execute() {
        feeder.runSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
