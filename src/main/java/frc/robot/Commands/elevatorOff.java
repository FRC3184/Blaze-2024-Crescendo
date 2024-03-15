package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class elevatorOff extends Command {
    private final Elevator elevator;

    public elevatorOff(Elevator subsystem){
        elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        elevator.setSpeed(0.5);
    }

    @Override
    public void execute() {
        elevator.run();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
