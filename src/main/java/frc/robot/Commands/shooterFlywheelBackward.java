package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.ShooterFlywheels;

public class shooterFlywheelBackward extends Command {
    private final ShooterFlywheels shooterFlywheels;

    public shooterFlywheelBackward(ShooterFlywheels subsystem){
        shooterFlywheels = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        shooterFlywheels.setTargetVelocity(-ConstShooter.defVelocity);
    }

    @Override
    public void execute() {
        shooterFlywheels.runVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        shooterFlywheels.stop();
    }
}