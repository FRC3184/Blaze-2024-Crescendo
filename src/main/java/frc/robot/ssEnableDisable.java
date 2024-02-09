package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constantsCrescendo.constMode;

public class ssEnableDisable {

    private ShuffleboardTab ssEnableTab;
    private GenericEntry competitionStatus;
    private GenericEntry drivetrainStatus;
    private GenericEntry intakeStatus;
    

    public ssEnableDisable(){
        inCompMode();
        ssEnableTab = Shuffleboard.getTab("Subsystem Status");
        competitionStatus = ssEnableTab.add("Competition Mode", constMode.competitionMode).getEntry();
        drivetrainStatus = ssEnableTab.add("Drivetrain Enabled", constMode.drivetrainRun).getEntry();
        intakeStatus = ssEnableTab.add("Intake Enabled", constMode.intakeRun).getEntry();
    
    }

    public void inCompMode() {
        if (constMode.competitionMode){
            constMode.drivetrainRun = true;
            constMode.intakeRun = true;
        }
    }

    public void updateSSstatus(){
        constMode.competitionMode = competitionStatus.getBoolean(false);
        if (constMode.competitionMode) {
            constMode.drivetrainRun = true;
            constMode.intakeRun = true;
        } else {
            constMode.drivetrainRun = drivetrainStatus.getBoolean(false);
            constMode.intakeRun = intakeStatus.getBoolean(false);
        }

        competitionStatus.setBoolean(constMode.competitionMode);
        intakeStatus.setBoolean(constMode.intakeRun);
        drivetrainStatus.setBoolean(constMode.drivetrainRun);       
    }
    
}
