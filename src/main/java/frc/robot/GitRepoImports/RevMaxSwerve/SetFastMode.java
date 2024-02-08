package frc.robot.GitRepoImports.RevMaxSwerve;

import edu.wpi.first.wpilibj2.command.Command;

public class SetFastMode extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private DriveSubsystemSwerve m_robotDrive;

    public SetFastMode(DriveSubsystemSwerve subsystem){
        m_robotDrive = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        m_robotDrive.setSpeedMode(2);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
