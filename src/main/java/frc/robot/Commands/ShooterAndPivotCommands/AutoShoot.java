package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Mechanisms.sensorTypes.Limelight;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class AutoShoot extends Command {
    
    ShooterPitch pitch;
    ShooterLimelight shooterLL;
    ShooterFlywheels flywheels;
    Feeder feeder;
    CarriageBelt carriage;
    double pitchError;
    double thetaError;

    private final DriveSubsystemSwerve robotDrive;
    int driverControllerPort = ConstJoysticks.kDriverControllerPort;
    private final XboxController driverController = new XboxController(driverControllerPort);
    private double thetaPower;
    private PIDController thetaController;
    private double target = Limelight.alignmentConstants.Aligned;
    private double pitchTolerance = 0.02;
    private double thetaTolerance = 5;


    public AutoShoot(ShooterPitch shooterPitch, ShooterFlywheels shooterFlywheels, Feeder feederSS, CarriageBelt belt, ShooterLimelight LLShooter, DriveSubsystemSwerve driveSS){
        pitch = shooterPitch;
        shooterLL = LLShooter;
        robotDrive = driveSS;
        flywheels = shooterFlywheels;
        feeder = feederSS;
        carriage = belt;

        thetaController = new PIDController(0.25, CardinalConstants.CardinalI, CardinalConstants.CardinalD);

        addRequirements(pitch, flywheels, feeder, carriage, shooterLL, robotDrive);
    }

    public void initialize(){
        thetaController.setSetpoint(target);
    }

    public void execute(){
        pitchError = shooterLL.getPredictedPivot()-pitch.getAbsPosition();
        thetaError = target-shooterLL.getCamXInches();
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        flywheels.runVelocity();

        thetaPower = thetaController.calculate(shooterLL.getCamXMeters());

        if(shooterLL.getV()==1){
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), 
          ConstJoysticks.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
          thetaPower, true, true);
          pitch.seekPosition(shooterLL.getPredictedPivot());
          } else {
            robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), ConstMaxSwerveDrive.DriveConstants.kFieldCentric, true);
            pitch.seekPosition(ConstShooter.upperLimit);
          }
    

        if(Math.abs(pitchError)<pitchTolerance && Math.abs(thetaError)<thetaTolerance && flywheels.atSpeed()){
            feeder.setSpeed(-0.5);
            feeder.runSpeed();
            carriage.setSpeed(0.5);
            carriage.runSpeed();
        } else {
            feeder.stop();
            carriage.stop();
        }
    }

    public void end(boolean interrupted){
        pitch.stop();
        flywheels.stop();
        feeder.stop();
        carriage.stop();
    }
}