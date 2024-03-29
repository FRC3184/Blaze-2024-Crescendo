package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class elevatorDownPID extends Command {
    private final Elevator Elevator;

    public elevatorDownPID(Elevator subsystem){
        Elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        // Elevator.setSpeed(-0.25);
    }

    @Override
    public void execute() {
        Elevator.seekPosition(0);
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.stop();
    }
}
