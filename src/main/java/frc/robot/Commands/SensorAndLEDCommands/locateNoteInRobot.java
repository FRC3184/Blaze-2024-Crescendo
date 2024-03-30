package frc.robot.Commands.SensorAndLEDCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;
import frc.robot.SubmoduleSubsystemConstants.ConstLEDs.COLORS;
import frc.robot.Subsystems.LEDs;

public class locateNoteInRobot extends Command {
    
    boolean hasNote = false;
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
            hasNote = true;
            noteLocation = "Intake";
        } else if (intakeODS.getSightStatus()){
            hasNote = true;
            noteLocation = "Carriage";
        } else {
            hasNote = false;
            noteLocation = "No Note";
        }

        // Change the LEDs based on note posession
        if (hasNote){
        leds.orangeWave();
        leds.setLeds();
        leds.UpdateLedMode("Orange Wave");
        }
        else {
        leds.setLedBufferByGroup(0, leds.getLedLength(), COLORS.WHITE);  
        leds.setLeds();
        leds.UpdateLedMode("White");
        }
        dashboartOut();
    }

        void dashboartOut(){
            SmartDashboard.putString("noteLocation", noteLocation);
        }

    @Override
    public void end(boolean interrupted) {

    }
}
