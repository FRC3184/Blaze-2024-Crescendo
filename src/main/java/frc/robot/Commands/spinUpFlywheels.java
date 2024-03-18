package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstFeeder;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;

public class spinUpFlywheels extends Command {

    private final ShooterFlywheels flywheels; 

    public spinUpFlywheels(ShooterFlywheels flywheel){
        this.flywheels = flywheel;
    }

    public void initialize() {
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
    }

    @Override
    public void execute() {
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        flywheels.runVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
    }
}
