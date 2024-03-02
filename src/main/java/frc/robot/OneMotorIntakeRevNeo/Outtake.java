package frc.robot.OneMotorIntakeRevNeo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Outtake extends Command {
    private final IntakeSS m_intake;

    private double IntakeRPM = 0.0;
    private int IntakeSpeed = 0;

    public Outtake(IntakeSS subsystem){
        m_intake = subsystem;

        addRequirements(subsystem); 
    }

    public void initialize() {}



    @Override
    public void execute() {
        IntakeRPM = m_intake.getIntakeSpeed();

        SmartDashboard.putNumber("Intake RPM Setpoint", IntakeSpeed);
        SmartDashboard.putNumber("Intake RPM", IntakeRPM);
        
        IntakeSpeed = 500;

        m_intake.setIntakeSpeed(-IntakeSpeed);
        m_intake.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
