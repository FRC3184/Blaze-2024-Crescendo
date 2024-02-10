package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RevMaxSwerve.DriveSubsystemSwerve;

/** An example command that uses an example subsystem. */
public class doNothing extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveSubsystemSwerve m_robotDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSS The subsystem used by this command.
   */
  public doNothing(DriveSubsystemSwerve driveSS) {

    m_robotDrive = driveSS;

    m_robotDrive.drive(0, 0, 0, false, false);
  }
}
