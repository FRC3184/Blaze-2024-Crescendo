package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstCarriage;
import frc.robot.Subsystems.ShooterPitch;

public class shooterPitchUp extends Command {
    private final ShooterPitch shooterPitch;

    public shooterPitchUp(ShooterPitch subsystem){
        shooterPitch = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        shooterPitch.setSpeed(0.1);
    }

    @Override
    public void execute() {
        shooterPitch.run();
    }

    @Override
    public void end(boolean interrupted) {
        shooterPitch.stop();
    }
}
