package frc.robot.Commands.ShooterAndPivotCommands.MiamiValleyPivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterPitch;

public class Pitch40 extends Command {
    
    ShooterPitch pitch;


    public Pitch40(ShooterPitch shooterPitch){
        pitch = shooterPitch;

        addRequirements(pitch);
    }

    public void initialize(){

    }

    public void execute(){
        pitch.seekPosition(0.41);
    }

    public void end(boolean interrupted){
        pitch.stop();
    }
}