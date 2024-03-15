package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstIntake;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;

public class climbDown extends Command {
    private final Climber climber;

    public climbDown(Climber subsystem){
        climber = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        climber.setSpeed(-0.5);
    }

    @Override
    public void execute() {
        climber.run();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
