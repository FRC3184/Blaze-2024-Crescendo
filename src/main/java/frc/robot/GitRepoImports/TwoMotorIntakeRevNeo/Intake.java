package frc.robot.GitRepoImports.TwoMotorIntakeRevNeo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Command {
    private final IntakeSubsystem m_intake;    

    public Intake(IntakeSubsystem subsystem){
        m_intake = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {}



    @Override
    public void execute() {
        m_intake.run(0.2, 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}