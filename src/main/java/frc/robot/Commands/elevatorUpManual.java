package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class elevatorUpManual extends Command {
    private final Elevator elevator;

    public elevatorUpManual(Elevator subsystem){
        elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        elevator.setSpeed(0.25);
    }

    @Override
    public void execute() {
        elevator.setSpeed(0.25);
        elevator.run();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}