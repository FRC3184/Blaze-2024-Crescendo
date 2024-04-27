package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Mechanisms.sensorTypes.Limelight;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class SmartScore extends Command {
    
    ShooterPitch pitch;
    ShooterLimelight shooterLL;
    ShooterFlywheels flywheels;
    Feeder feeder;
    CarriageBelt carriage;
    Elevator elevator;
    double pitchError;
    double thetaError;

    private final DriveSubsystemSwerve robotDrive;
    int driverControllerPort = ConstJoysticks.kDriverControllerPort;
    private final XboxController driverController = new XboxController(driverControllerPort);
    private double thetaPower;
    private PIDController thetaController;
    private double target = Limelight.alignmentConstants.Aligned;
    private double pitchTolerance = 0.02;
    private double thetaTolerance = 7.5;

    private final Timer feederTimer = new Timer();


    public SmartScore(ShooterPitch shooterPitch, ShooterFlywheels shooterFlywheels, Feeder feederSS, CarriageBelt belt, ShooterLimelight LLShooter, DriveSubsystemSwerve driveSS, Elevator elevator){
        pitch = shooterPitch;
        shooterLL = LLShooter;
        robotDrive = driveSS;
        flywheels = shooterFlywheels;
        feeder = feederSS;
        carriage = belt;
        this.elevator = elevator;

        thetaController = new PIDController(0.25, CardinalConstants.CardinalI, CardinalConstants.CardinalD);

        addRequirements(pitch, flywheels, feeder, carriage, shooterLL, robotDrive);
    }

    public void initialize(){
        thetaController.setSetpoint(target);
        feederTimer.reset();
        ConstShooter.NoteInRobot = false;
    }

    public void execute(){
        if(elevator.getPosition()<30){
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
          feederTimer.start();
          } else {
            feederTimer.stop();
            feederTimer.reset();
            robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), ConstMaxSwerveDrive.DriveConstants.kFieldCentric, true);
            pitch.seekPosition(ConstShooter.upperLimit);
          }
    

        if(shooterLL.getV() == 1 && Math.abs(pitchError)<pitchTolerance && Math.abs(thetaError)<thetaTolerance && flywheels.atSpeed()){
            feeder.setSpeed(-0.5);
            feeder.runSpeed();
            carriage.setSpeed(0.5);
            carriage.runSpeed();
        } else {
            feeder.stop();
            carriage.stop();
        }

        if(feederTimer.get()>0){
            feeder.setSpeed(-0.5);
            feeder.runSpeed();
            carriage.setSpeed(0.5);
            carriage.runSpeed();
        }
    }
    else{
        carriage.setSpeed(-0.5);
        carriage.runSpeed();
    }
    }

    public void end(boolean interrupted){
        pitch.stop();
        flywheels.stop();
        feeder.stop();
        carriage.stop();
    }
}