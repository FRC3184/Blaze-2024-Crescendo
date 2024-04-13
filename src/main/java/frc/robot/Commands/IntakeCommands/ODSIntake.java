package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Intake;

public class ODSIntake extends Command {

    String noteLocation = "No Note";

    private final Intake intake;
    private final CarriageBelt carriageBelt;
    private final IntakeODS intakeODS;
    private final CarriageODS carriageODS;
    private final ShooterLimelight shooterLL;
    private final IntakeLimelight intakeLL;

    private final Timer timer = new Timer();

    public ODSIntake(Intake intakeSS, CarriageBelt carriageSS, IntakeODS iODS, CarriageODS cODS, ShooterLimelight LLShooter, IntakeLimelight LLIntake){
        intake = intakeSS;
        carriageBelt = carriageSS;
        intakeODS = iODS;
        carriageODS = cODS;
        shooterLL = LLShooter;
        intakeLL = LLIntake;
        addRequirements(intake, carriageBelt, intakeODS, carriageODS); 
    }

    public void initialize() {
        intake.setSpeed(0.5);
        carriageBelt.setSpeed(0.5);
        timer.reset();
    }

    @Override
    public void execute() {
        intake.runSpeed();
        carriageBelt.runSpeed();
        if((intakeODS.getSightStatus()&&carriageODS.getSightStatus())||carriageODS.getSightStatus()){
            carriageODS.setHasNote(true);
            carriageODS.setNoteLocation("Carriage");
            noteLocation = carriageODS.getNoteLocation();
        } else if (intakeODS.getSightStatus()){
            carriageODS.setHasNote(true);
            carriageODS.setNoteLocation("Intake");
            noteLocation = carriageODS.getNoteLocation();
        } else {
            carriageODS.setHasNote(false);
            carriageODS.setNoteLocation("No Note");
            noteLocation = carriageODS.getNoteLocation();
        }

        if (noteLocation.equals("Carriage")){
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
        carriageBelt.setSpeed(0.5);
        carriageBelt.runSpeed();
        }

        if(timer.get()>0.1){
        timer.stop();
        carriageBelt.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        carriageBelt.stop();
    }
}
