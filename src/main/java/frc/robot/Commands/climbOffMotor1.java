package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;

public class climbOffMotor1 extends Command {
    private final Climber intake;

    public climbOffMotor1(Climber subsystem){
        intake = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        intake.setSpeed(0);
    }

    @Override
    public void execute() {
        intake.runMotor1();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}