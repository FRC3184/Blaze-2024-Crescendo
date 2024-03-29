package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;

public class locateNoteInRobot extends Command {
    
    IntakeODS intakeODS = new IntakeODS();
    CarriageODS carriageODS = new CarriageODS();

    public locateNoteInRobot(IntakeODS intakeSensor, CarriageODS carriageSensor){
        intakeODS = intakeSensor;
        carriageODS = carriageSensor;

        addRequirements(intakeSensor, carriageSensor);
    }

    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {

    }
}
