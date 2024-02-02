package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.TwoWheelShooterRevNeo.ShootCommand;
import frc.robot.TwoWheelShooterRevNeo.ShooterSubsystem;
import frc.robot.constantsCrescendo.constsJoysticks;

public class RobotContainer {

  // define subsystems
  // private final DriveSubsystemSwerve m_robotDrive = new DriveSubsystemSwerve();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // Joystick Controller (I/O)
  XboxController m_driverController = new XboxController(constsJoysticks.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(constsJoysticks.kGunnerControllerPort);

  public RobotContainer() {
    configureBindings();

    /*// Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), constsJoysticks.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), constsJoysticks.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), constsJoysticks.kDriveDeadband),
                DriveConstants.kFieldCentric, true),
            m_robotDrive));
            */
    m_shooter.setDefaultCommand(new ShootCommand(m_shooter));

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
