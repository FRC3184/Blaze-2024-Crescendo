package frc.robot.Commands.ElevatorCommands;

import frc.robot.SubmoduleSubsystemConstants.ConstElevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class elevatorUpManual extends Command {
    private final Elevator elevator;

    public elevatorUpManual(Elevator subsystem){
        elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        if(elevator.getPosition()<ConstElevator.raisedPos){
            elevator.setSpeed(ConstElevator.elevatorRiseSpeed);
        elevator.run();
        // }
        // else if(elevator.getPosition()>ConstElevator.raisedPos-20 && elevator.getPosition()<ConstElevator.raisedPos){
        //     elevator.setSpeed(ConstElevator.elevatorFinishSpeed);
        // elevator.run();
        } else {
            elevator.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}