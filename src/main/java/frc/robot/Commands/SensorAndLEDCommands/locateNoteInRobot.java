package frc.robot.Commands.SensorAndLEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;
import frc.robot.SubmoduleSubsystemConstants.ConstLEDs.COLORS;
import frc.robot.Subsystems.LEDs;

public class locateNoteInRobot extends Command {
    
    String noteLocation = "No Note";

    private final IntakeODS intakeODS;
    private final CarriageODS carriageODS;
    private final LEDs leds;

    public locateNoteInRobot(IntakeODS intakeSensor, CarriageODS carriageSensor, LEDs Leds){
        intakeODS = intakeSensor;
        carriageODS = carriageSensor;
        leds = Leds;

        addRequirements(intakeSensor, carriageSensor, Leds);
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
        leds.orangeWave();
        leds.setLeds();
        leds.UpdateLedMode("Orange Wave");
        }
        else if (noteLocation.equals("Intake")){
        leds.setLedBufferByGroup(0, leds.getLedLength(), COLORS.GREEN);  
        leds.setLeds();
        leds.UpdateLedMode("Green");
        }
        else {
        leds.setLedBufferByGroup(0, leds.getLedLength(), COLORS.WHITE);  
        leds.setLeds();
        leds.UpdateLedMode("White");
        }
    }


    @Override
    public void end(boolean interrupted) {

    }
}
