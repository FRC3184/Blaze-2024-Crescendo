package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class SmartScore extends Command {
    
    ShooterPitch pitch;
    ShooterLimelight shooterLL;
    ShooterFlywheels flywheels;
    Feeder feeder;
    CarriageBelt carriage;
    Intake intake;
    Elevator elevator;
    double pitchError;
    double thetaError;

    private final DriveSubsystemSwerve robotDrive;
    int driverControllerPort = ConstJoysticks.kDriverControllerPort;
    private final XboxController driverController = new XboxController(driverControllerPort);
    private double thetaPower;
    private PIDController thetaController;
    private double target = -0.15;

    private double pitchTolerance = 0.02;
    private double thetaTolerance = 0.2;

    private boolean pitchInRange;
    private boolean thetaInRange;

    private final Timer feederOverrideTimer = new Timer();

    public SmartScore(ShooterPitch shooterPitch, ShooterFlywheels shooterFlywheels, Feeder feederSS, CarriageBelt belt, Intake intakeSS, ShooterLimelight LLShooter, DriveSubsystemSwerve driveSS, Elevator elevator){
        pitch = shooterPitch;
        shooterLL = LLShooter;
        robotDrive = driveSS;
        flywheels = shooterFlywheels;
        feeder = feederSS;
        carriage = belt;
        intake = intakeSS;
        this.elevator = elevator;

        thetaController = new PIDController(0.25, CardinalConstants.CardinalI, CardinalConstants.CardinalD);

        addRequirements(pitch, flywheels, feeder, carriage, intake, shooterLL, robotDrive);
    }

    public void initialize(){
        thetaController.setSetpoint(target);
        ConstShooter.NoteInRobot = false;
        feederOverrideTimer.reset();
    }

    public void execute(){

            pitchInRange = Math.abs(pitchError)<pitchTolerance;
        
            thetaInRange = Math.abs(thetaError)<thetaTolerance;
        

        SmartDashboard.putBoolean("Pitch In Range", pitchInRange);
        SmartDashboard.putBoolean("Theta In Range", thetaInRange);

        SmartDashboard.putNumber("Theta Error", Math.abs(thetaError));
        SmartDashboard.putNumber("Pitch Error", Math.abs(pitchError));


        if(elevator.getPosition()<30){
            pitchError = shooterLL.getPredictedPivot()-pitch.getAbsPosition();
            thetaError = target-shooterLL.getCamXMeters();
            flywheels.setTargetVelocity(ConstShooter.defVelocity);
            flywheels.runVelocity();

            thetaPower = thetaController.calculate(shooterLL.getCamXMeters());

            pitchError = shooterLL.getPredictedPivot()-pitch.getAbsPosition();
            thetaError = target-shooterLL.getCamXMeters();
            flywheels.setTargetVelocity(ConstShooter.defVelocity);
            flywheels.runVelocity();

            thetaPower = thetaController.calculate(shooterLL.getCamXMeters());

            if(shooterLL.getV()==1){
                robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), 
                ConstJoysticks.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
                thetaPower, true, true);
                pitch.seekPosition(shooterLL.getPredictedPivot());
                feederOverrideTimer.start();
            } else {
                feederOverrideTimer.stop();
                feederOverrideTimer.reset();
                robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), ConstMaxSwerveDrive.DriveConstants.kFieldCentric, true);
                pitch.seekPosition(ConstShooter.upperLimit);
            }
        

            if(shooterLL.getV() == 1 && Math.abs(pitchError)<pitchTolerance && Math.abs(thetaError)<thetaTolerance && flywheels.atSpeed()){
                feeder.setSpeed(-0.5);
                feeder.runSpeed();
                carriage.setSpeed(0.5);
                carriage.runSpeed();
                intake.setSpeed(0.5);
                intake.runSpeed();
            } else if (feederOverrideTimer.get() > 2) {
                 feeder.setSpeed(-0.5);
                feeder.runSpeed();
                carriage.setSpeed(0.5);
                carriage.runSpeed();
            } else {
                feeder.stop();
                carriage.stop();
                intake.stop();
            }

            // if(feederOverrideTimer.get()>2){
            //     feeder.setSpeed(-0.5);
            //     feeder.runSpeed();
            //     carriage.setSpeed(0.5);
            //     carriage.runSpeed();
            // }
        } else {
            carriage.setSpeed(-0.5);
            carriage.runSpeed();
        }
    }

    public void end(boolean interrupted){
        pitch.stop();
        flywheels.stop();
        feeder.stop();
        carriage.stop();
        intake.stop();
    }
}