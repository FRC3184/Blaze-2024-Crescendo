package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class shooterPitchUp extends Command {
    private final ShooterPitch shooterPitch;

    public shooterPitchUp(ShooterPitch subsystem){
        shooterPitch = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        shooterPitch.setSpeed(ConstShooter.pivotSpeed);
    }

    @Override
    public void execute() {
        if (shooterPitch.getAbsAngle()>0.2 && shooterPitch.getAbsAngle()>0){
            shooterPitch.run();
        }
        else{
            shooterPitch.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterPitch.stop();
    }
}
