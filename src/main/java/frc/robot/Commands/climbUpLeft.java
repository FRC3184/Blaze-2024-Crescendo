package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class climbUpLeft extends Command {
    private final Climber climber;

    public climbUpLeft(Climber subsystem){
        climber = subsystem;
        // addRequirements(subsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        climber.runMotor2Seperate(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor2();
    }
}
