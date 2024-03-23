package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class climbDownRight extends Command {
    private final Climber climber;

    public climbDownRight(Climber subsystem){
        climber = subsystem;
        // addRequirements(subsystem); 
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        climber.runMotor1Seperate(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor1();
    }
}
