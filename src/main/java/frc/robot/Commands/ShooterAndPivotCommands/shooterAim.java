package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class shooterAim extends Command {
    private final ShooterPitch shooterPitch;
    private final ShooterLimelight shooterLimelight;

    private double target_pitch;

    public shooterAim(ShooterPitch pitchSubsystem, ShooterLimelight limelightSubsystem) {
        shooterPitch = pitchSubsystem;
        shooterLimelight = limelightSubsystem;
        target_pitch = 0;
        addRequirements(pitchSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // convert to revolutions
        Pair<Double, Double> r = shooterPitch.inputAngleFromRockerAngle(shooterLimelight.angle / (2 * Math.PI));

        // subsystem doesn't clamp value, so we have to clamp it ourselves to not wreck the bot.
        double revolutions1 = MathUtil.clamp(r.getFirst(), ConstShooter.upperLimit, ConstShooter.lowerLimit);
        target_pitch = revolutions1;

        shooterPitch.seekPosition(target_pitch);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPitch.stop();
    }
}
