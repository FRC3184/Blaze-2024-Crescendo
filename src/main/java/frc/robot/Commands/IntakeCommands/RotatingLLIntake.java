package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class RotatingLLIntake extends Command {

    String noteLocation = "No Note";
    boolean notePassedIntake = false;

    private final Intake intake;
    private final CarriageBelt carriageBelt;
    private final IntakeODS intakeODS;
    private final CarriageODS carriageODS;
    private final ShooterLimelight shooterLL;
    private final IntakeLimelight intakeLL;

    private final DriveSubsystemSwerve robotDrive;
    int driverControllerPort = ConstJoysticks.kDriverControllerPort;
    private final XboxController driverController = new XboxController(driverControllerPort);
    private double target = 0;
    private PIDController ThetaController;
    private double ThetaPower = 0;

    private final Timer timer = new Timer();

    public RotatingLLIntake(Intake intakeSS, CarriageBelt carriageSS, IntakeODS iODS, CarriageODS cODS, ShooterLimelight LLShooter, IntakeLimelight LLIntake, DriveSubsystemSwerve driveSS){
        intake = intakeSS;
        carriageBelt = carriageSS;
        intakeODS = iODS;
        carriageODS = cODS;
        shooterLL = LLShooter;
        intakeLL = LLIntake;
        robotDrive = driveSS;

        ThetaController = new PIDController(1/75, 0, 0);
        
        addRequirements(robotDrive); //addRequirements(intake, carriageBelt, intakeODS, carriageODS, robotDrive); 

    }

    public void initialize() {
        notePassedIntake = false;
        // intake.setSpeed(0.5);
        // carriageBelt.setSpeed(0.5);
        timer.reset();
        ThetaController.setSetpoint(target);
    }

    @Override
    public void execute() {
        double NoteX = intakeLL.getX();
        ThetaPower = ThetaController.calculate(NoteX);
        // intake.runSpeed();
        // carriageBelt.runSpeed();
        if((intakeODS.getSightStatus()&&carriageODS.getSightStatus())||carriageODS.getSightStatus()){
            carriageODS.setHasNote(true);
            carriageODS.setNoteLocation("Carriage");
            noteLocation = carriageODS.getNoteLocation();
        } else if (intakeODS.getSightStatus()){
            notePassedIntake = true;
            carriageODS.setHasNote(true);
            carriageODS.setNoteLocation("Intake");
            noteLocation = carriageODS.getNoteLocation();
        } else {
            carriageODS.setHasNote(false);
            carriageODS.setNoteLocation("No Note");
            noteLocation = carriageODS.getNoteLocation();
        }

        if (noteLocation.equals("Carriage") && notePassedIntake){
        shooterLL.setLedMode(2);
        intakeLL.setLedMode(2);
        intake.stop();
        carriageBelt.setSpeed(-0.5);
        timer.start();
        }
        else if (noteLocation.equals("Intake")){
        shooterLL.setLedMode(3);
        intakeLL.setLedMode(3);
        }
        else {
        shooterLL.setLedMode(1);
        intakeLL.setLedMode(1);
        intake.runSpeed();
        }

        if(timer.get()>0.25){
        // timer.stop();
        carriageBelt.stop();
        }

        if(timer.get()<0.5){
        shooterLL.setLedMode(2);
        intakeLL.setLedMode(2);
        }
        else{
        shooterLL.setLedMode(1);
        intakeLL.setLedMode(1);
        timer.stop();
        }

        if (intakeLL.getV()==1){
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), ThetaPower, false, true);
        } else {
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), ConstMaxSwerveDrive.DriveConstants.kFieldCentric, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        carriageBelt.stop();
    }
}
