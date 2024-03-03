package frc.robot;

// General Imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import frc.robot.GitRepoImports.RevMaxSwerve.DriveSubsystemSwerve;
import frc.robot.GitRepoImports.RevMaxSwerve.FaceForward;
import frc.robot.GitRepoImports.RevMaxSwerve.SetFastMode;
import frc.robot.GitRepoImports.RevMaxSwerve.SetNormalMode;
import frc.robot.GitRepoImports.RevMaxSwerve.SetSlowMode;
import frc.robot.GitRepoImports.RevMaxSwerve.constantsMaxSwerve.DriveConstants;
import frc.robot.GitRepoImports.TwoMotorIntakeRevNeo.Intake;
import frc.robot.GitRepoImports.TwoMotorIntakeRevNeo.IntakeSubsystem;
import frc.robot.GitRepoImports.TwoWheelShooterRevNeo.ShootCommand;
import frc.robot.GitRepoImports.TwoWheelShooterRevNeo.ShooterSubsystem;
import frc.robot.RobotMap.OIConstants;

public class RobotContainer {

  // define subsystems5
  private final DriveSubsystemSwerve m_robotDrive = new DriveSubsystemSwerve();
  // private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // Joystick Controller (I/O)
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

  public RobotContainer() {
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                DriveConstants.kFieldCentric, true),
            m_robotDrive));
  
    //m_shooter.setDefaultCommand(new ShootCommand(m_shooter));

  }

  private void configureBindings() {
    new JoystickButton(m_gunnerController, Button.kLeftBumper.value).whileTrue(new Intake(m_intake));

    //Set speed modes
    new JoystickButton(m_driverController, Button.kRightBumper.value).onTrue(new SetSlowMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kRightBumper.value).onFalse(new SetNormalMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onTrue(new SetFastMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onFalse(new SetNormalMode(m_robotDrive));

    //Face direction
    new JoystickButton(m_driverController, Button.kY.value).whileTrue(new FaceForward(m_robotDrive));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
