package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstElevator;
import frc.robot.Subsystems.Elevator;

public class elevatorUpManual extends Command {
    private final Elevator elevator;

    public elevatorUpManual(Elevator subsystem){
        elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        elevator.setSpeed(ConstElevator.elevatorSpeed);
    }

    @Override
    public void execute() {
        elevator.setSpeed(ConstElevator.elevatorSpeed);
        elevator.run();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}