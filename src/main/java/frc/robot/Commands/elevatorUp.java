package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;

public class elevatorUp extends Command {
    private final Elevator elevator;

    public elevatorUp(Elevator subsystem){
        elevator = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        elevator.setSpeed(0);
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