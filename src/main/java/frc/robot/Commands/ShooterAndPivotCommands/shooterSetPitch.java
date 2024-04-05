package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class shooterSetPitch extends Command {
    private final ShooterPitch shooterPitch;
    private final double target_pitch;

    public shooterSetPitch(ShooterPitch subsystem, double pitch) {
        shooterPitch = subsystem;
        target_pitch = MathUtil.clamp(pitch, ConstShooter.upperLimit, ConstShooter.lowerLimit);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterPitch.seekPosition(target_pitch);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPitch.stop();
    }
}
