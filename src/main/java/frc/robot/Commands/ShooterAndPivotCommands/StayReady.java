package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;

public class StayReady extends Command {
    
    public StayReady(){

    }

    public void initialize(){
        ConstShooter.stayReady = true;
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        ConstShooter.stayReady = false;
    }
}
