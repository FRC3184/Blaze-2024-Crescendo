package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class climbUpMotor1 extends Command {
    private final Climber climber;

    public climbUpMotor1(Climber subsystem){
        climber = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        climber.setSpeed(0.5);
    }

    @Override
    public void execute() {
        climber.runMotor1();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
