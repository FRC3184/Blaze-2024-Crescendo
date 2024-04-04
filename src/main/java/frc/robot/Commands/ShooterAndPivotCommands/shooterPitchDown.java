package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterPitch;

public class shooterPitchDown extends Command {
    private final ShooterPitch shooterPitch;

    public shooterPitchDown(ShooterPitch subsystem){
        shooterPitch = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        shooterPitch.setSpeed(-0.1);
    }

    @Override
    public void execute() {
        if (shooterPitch.getAbsAngle()<0.567 && shooterPitch.getAbsAngle()>0){
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
