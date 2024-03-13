package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;

public class shoot extends Command {

    private final ShooterFlywheels flywheels; 
    private final ShooterPitch pitch;
    private final Feeder feeder;

    public shoot(ShooterFlywheels flywheel, ShooterPitch pitch, Feeder feeder){
        this.flywheels = flywheel;
        this.pitch = pitch;
        this.feeder = feeder;
    }

    public void initialize() {

    }

    @Override
    public void execute() {
        flywheels.setVelocity(ConstShooter.defVelocity);
        // check if flywheel is up to speed and pitch is set
        if(true){
            // run feeder
        }

    }

}
