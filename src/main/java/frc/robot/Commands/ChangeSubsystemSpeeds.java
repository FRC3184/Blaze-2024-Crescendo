package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstElevator;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.SpeedSS;
import frc.robot.SubmoduleSubsystemConstants.ConstClimber;
// import frc.robot.SubmoduleSubsystemConstants.ConstPivot;

public class ChangeSubsystemSpeeds extends Command {

    XboxController subsystemController = new XboxController(2);

    SpeedSS speedChanger;
    
    public ChangeSubsystemSpeeds(SpeedSS speed){
        speedChanger = speed;
        addRequirements(speedChanger);
    }

public void initialize() {

    }

    @Override
    public void execute() {
        //Change Elevator Speed (Y & A)
        if(subsystemController.getYButtonPressed()){
        ConstElevator.elevatorSpeed += 0.05;
        }
        else if (subsystemController.getAButtonPressed()){
            ConstElevator.elevatorSpeed -= 0.05;
        }

        //Change Climber Speed (X & B)
        if(subsystemController.getXButtonPressed()){
            ConstClimber.climbPower+=0.1;
        }
        else if(subsystemController.getBButtonPressed()){
            ConstClimber.climbPower-=0.1;
        }

        //Change Pivot Speed (POV Up & Down)
        if(subsystemController.getPOV()==0){
            ConstShooter.pivotMaxSpeed+=0.05;
        }
        else if (subsystemController.getPOV()==180){
            ConstShooter.pivotMaxSpeed-=0.05;
        }

        //Change Shooter Speed (POV Left & Right)
        if(subsystemController.getPOV()==270){
            ConstShooter.defVelocity+=250;
        }
        else if (subsystemController.getPOV()==90){
            ConstShooter.defVelocity-=250;
        }

        //Keep Speeds between 0 and 1 / 0 & 5750
        if(ConstElevator.elevatorSpeed>1){
            ConstElevator.elevatorSpeed = 1;
        }
        else if(ConstElevator.elevatorSpeed<0){
        ConstElevator.elevatorSpeed = 0;
        }

        if(ConstClimber.climbPower>1){
            ConstClimber.climbPower = 1;
        }
        else if (ConstClimber.climbPower<0){
            ConstClimber.climbPower = 0;
        }

        if(ConstShooter.pivotMaxSpeed>1){
            ConstShooter.pivotMaxSpeed = 1;
        }
        else if(ConstShooter.pivotMaxSpeed<0){
            ConstShooter.pivotMaxSpeed = 0;
        }

        if(ConstShooter.defVelocity>5750){
            ConstShooter.defVelocity = 5750;
        }
        else if(ConstShooter.defVelocity<0){
            ConstShooter.defVelocity = 0;
        }
    }

}
