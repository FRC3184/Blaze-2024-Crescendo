package frc.robot.GitRepoImports.RevMaxSwerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GitRepoImports.RevMaxSwerve.DriveSubsystemSwerve;
import com.kauailabs.navx.frc.AHRS;


public class FaceForward extends Command {
    private final AHRS m_navxGyro = new AHRS();


    
    private final DriveSubsystemSwerve m_robotDrive;

    private double xPower;
    private double yPower;
    private double thetaPower;

    // private PIDController xController;
    // private PIDController yController;
    private ProfiledPIDController thetaController;

    private boolean done = false;

    // private Pose2d target;
    private Transform2d offset;

    public FaceForward(DriveSubsystemSwerve subsystem) {
        m_robotDrive = subsystem;
        addRequirements(subsystem);
        
        // xController = new PIDController(1, 0, 0);
        // yController = new PIDController(1, 0, 0);
        thetaController = new ProfiledPIDController(0.6, 0, 0, new Constraints(8, 8));

        // target = new Pose2d(1.85, gridPos.offset, Rotation2d.fromRadians(0));

        offset = new Transform2d(
            new Translation2d(),
            new Rotation2d(Math.toRadians(180))
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // xController.setSetpoint(target.getX());
        // yController.setSetpoint(target.getY());
        thetaController.setGoal(m_navxGyro.getAngle());
    }

    @Override
    public void execute() {

        // Pose2d tag = new Pose2d(m_robotDrive.getTagPose().getTranslation(), m_robotDrive.getTagPose().getRotation());
        // tag = tag.transformBy(offset);

        // xPower = -xController.calculate(tag.getX())/2;
        // yPower = -yController.calculate(tag.getY())/2;
        thetaPower = thetaController.calculate(m_navxGyro.getAngle());

        // if (!(target.getRotation().getRadians() - Math.toRadians(5) > tag.getRotation().getRadians() &&
        //     target.getRotation().getRadians() + Math.toRadians(5) < tag.getRotation().getRadians())) {

            
        if (m_navxGyro.getAngle() !=0) {
            m_robotDrive.setRotation(thetaPower);
            m_robotDrive.drive(yPower, xPower, m_robotDrive.getRotation() , false, false);
        }

    }
  
    @Override
    public boolean isFinished() {
       return (xPower < 0.0025 && yPower < 0.0025 && thetaPower < 0.0025);
    }
}