package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstClimber;
import frc.robot.Subsystems.Climber;

public class climbUpRight extends Command {
    private final Climber climber;

    public climbUpRight(Climber subsystem){
        climber = subsystem;
        // addRequirements(subsystem); 
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        climber.runMotor1Seperate(ConstClimber.climbPower);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor1();
    }
}
