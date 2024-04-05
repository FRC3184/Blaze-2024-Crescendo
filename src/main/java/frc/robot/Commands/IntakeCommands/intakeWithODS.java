package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;
import frc.robot.SubmoduleSubsystemConstants.ConstLEDs.COLORS;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;

public class intakeWithODS extends Command {

    String noteLocation = "No Note";

    private final Intake intake;
    private final CarriageBelt carriageBelt;
    private final IntakeODS intakeODS;
    private final CarriageODS carriageODS;
    private final LEDs leds;

    public intakeWithODS(Intake intakeSS, CarriageBelt carriageSS, IntakeODS iODS, CarriageODS cODS, LEDs ledStrip){
        intake = intakeSS;
        carriageBelt = carriageSS;
        intakeODS = iODS;
        carriageODS = cODS;
        leds = ledStrip;
        addRequirements(intake, carriageBelt, intakeODS, carriageODS, leds); 
    }

    public void initialize() {
        intake.setSpeed(0.5);
        carriageBelt.setSpeed(0.5);
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
        leds.orangeWave();
        leds.setLeds();
        leds.UpdateLedMode("Orange Wave");
        intake.stop();
        carriageBelt.stop();
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
        intake.stop();
        carriageBelt.stop();
    }
}
