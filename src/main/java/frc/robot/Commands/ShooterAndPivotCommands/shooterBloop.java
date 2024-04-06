package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
// import frc.robot.Subsystems.ShooterPitch;

public class shooterBloop extends Command {

    private final ShooterFlywheels flywheels; 
    // private final ShooterPitch pitch;
    private final Feeder feeder;

    public shooterBloop(ShooterFlywheels flywheel, /*ShooterPitch pitch,*/ Feeder feeder){
        this.flywheels = flywheel;
        // this.pitch = pitch;
        this.feeder = feeder;
    }

    public void initialize() {
        flywheels.setTargetVelocity(1000);
        feeder.setSpeed(-0.5);
    }

    @Override
    public void execute() {
        flywheels.setTargetVelocity(1000);
        flywheels.runVelocity();
        if(flywheels.atSpeed()){
            feeder.setSpeed(-0.5);
            feeder.runSpeed();
        } else {
            feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        feeder.stop();
    }
}
