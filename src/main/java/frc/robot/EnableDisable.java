package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SubmoduleSubsystemConstants.constMode;

/**
 * Class that allows for enabling and disabling system functions through
 * ShuffleBoard.
 */
public class EnableDisable {

  private ShuffleboardTab ssEnableTab;
  private GenericEntry competitionStatus;
  private GenericEntry drivetrainStatus;
  private GenericEntry intakeStatus;

  /**
   * Initialize ShuffleBoard tab and add buttons for each subsystem.
   */
  public EnableDisable() {
    inCompMode();
    ssEnableTab = Shuffleboard.getTab("Subsystem Status");
    competitionStatus = ssEnableTab.add("Competition Mode", constMode.competitionMode).getEntry();
    drivetrainStatus = ssEnableTab.add("Drivetrain Enabled", constMode.drivetrainRun).getEntry();
    intakeStatus = ssEnableTab.add("Intake Enabled", constMode.intakeRun).getEntry();

  }

  /**
   * Check whether the robot is in competition mode.
   */
  public void inCompMode() {
    if (constMode.competitionMode) {
      constMode.drivetrainRun = true;
      constMode.intakeRun = true;
    }
  }

  /**
   * Updates ShuffleBoard.
   */
  public void updateSubsystemStatus() {
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
