package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class elevatorUpPID extends Command {
    private final Elevator elevator;

    public elevatorUpPID(Elevator subsystem){
        elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        // elevator.setSpeed(0.25);
    }

    @Override
    public void execute() {
        elevator.seekPosition(0);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}