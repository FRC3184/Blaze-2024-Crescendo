package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class shooterAimSpeaker extends Command {
    private final ShooterPitch shooterPitch;
    private final ShooterLimelight shooterLimelight;

    private double target_pitch;

    public shooterAimSpeaker(ShooterPitch pitchSubsystem, ShooterLimelight limelightSubsystem) {
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
        // convert to input angle
        Pair<Double, Double> r = shooterPitch.inputAngleFromRockerAngle(Math.PI - shooterLimelight.angle);

        // subsystem doesn't clamp value, so we have to clamp it ourselves to not wreck the bot.
        // also convert to revolutions
        double revolutions1 = MathUtil.clamp(r.getFirst(), ConstShooter.upperLimit, ConstShooter.lowerLimit) / (2 * Math.PI);
        target_pitch = revolutions1;

        shooterPitch.seekPosition(target_pitch);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPitch.stop();
    }
}
