package frc.robot.Commands.SensorAndLEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;

public class locateNoteInRobot extends Command {
    
    String noteLocation = "No Note";

    private final IntakeODS intakeODS;
    private final CarriageODS carriageODS;
    private final ShooterLimelight shooterLL;
    private final IntakeLimelight intakeLL;

    public locateNoteInRobot(IntakeODS intakeSensor, CarriageODS carriageSensor, ShooterLimelight shooterLL, IntakeLimelight intakeLL){
        intakeODS = intakeSensor;
        carriageODS = carriageSensor;
        this.shooterLL = shooterLL;
        this.intakeLL = intakeLL;
        addRequirements(intakeSensor, carriageSensor);
    }

    public void initialize() {
        
    }

    @Override
    public void execute() {
        //Locate the note and update the location and if the robot has a note
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

        // Change the LEDs based on note posession
        if (noteLocation.equals("Carriage")){
        shooterLL.setLedMode(2);
        intakeLL.setLedMode(2);
        }
        else if (noteLocation.equals("Intake")){
        shooterLL.setLedMode(3);
        intakeLL.setLedMode(3);
        }
        else {
        shooterLL.setLedMode(1);
        intakeLL.setLedMode(1);
        }
    }


    @Override
    public void end(boolean interrupted) {

    }
}
