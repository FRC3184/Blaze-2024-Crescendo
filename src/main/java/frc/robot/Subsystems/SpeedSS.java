package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubmoduleSubsystemConstants.ConstClimber;
import frc.robot.SubmoduleSubsystemConstants.ConstElevator;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;

public class SpeedSS extends SubsystemBase {
    
    public SpeedSS(){

    }

    public void initialize(){

    }

    @Override
    public void periodic() {
        smartDashboardOut();
    }

    @Override
    public void simulationPeriodic() {

    }

    void smartDashboardOut(){
        SmartDashboard.putNumber("Elevator Speed", ConstElevator.elevatorSpeed);
        SmartDashboard.putNumber("Climber Speed", ConstClimber.climbPower);
        SmartDashboard.putNumber("Pivot Speed", ConstShooter.pivotMaxSpeed);
        SmartDashboard.putNumber("Shooter Speed", ConstShooter.defVelocity);
    }
}
