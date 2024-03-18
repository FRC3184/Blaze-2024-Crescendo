package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstIntake;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;

public class elevatorDown extends Command {
    private final Elevator Elevator;

    public elevatorDown(Elevator subsystem){
        Elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        Elevator.setSpeed(-0.5);
    }

    @Override
    public void execute() {
        Elevator.run();
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.stop();
    }
}
