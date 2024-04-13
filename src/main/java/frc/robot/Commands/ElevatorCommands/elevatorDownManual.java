package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstElevator;
import frc.robot.Subsystems.Elevator;

public class elevatorDownManual extends Command {
    private final Elevator elevator;

    public elevatorDownManual(Elevator subsystem){
        elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        
        if(elevator.getPosition()<ConstElevator.elevatorSlowZoneDown && elevator.getPosition()>ConstElevator.loweredPos){
            elevator.setSpeed(-ConstElevator.elevatorFinishSpeed);
            elevator.run();
        } else if(elevator.getPosition()>ConstElevator.loweredPos){
            elevator.setSpeed(-ConstElevator.elevatorFallSpeed);
            elevator.run();
        } else {
            elevator.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}