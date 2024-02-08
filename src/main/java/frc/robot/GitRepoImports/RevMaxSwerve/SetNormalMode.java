package frc.robot.GitRepoImports.RevMaxSwerve;

import edu.wpi.first.wpilibj2.command.Command;

public class SetNormalMode extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystemSwerve m_robotDrive;

    public SetNormalMode(DriveSubsystemSwerve subsystem){
        m_robotDrive = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_robotDrive.setSpeedMode(1);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
