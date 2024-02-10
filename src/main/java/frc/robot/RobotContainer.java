package frc.robot;

// General Imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
// Shooter Imports
// import frc.robot.TwoWheelShooterRevNeo.ShootCommand;
// import frc.robot.TwoWheelShooterRevNeo.ShooterSubsystem;
// Drive Imports
import frc.robot.RevMaxSwerve.DriveSubsystemSwerve;
import frc.robot.RevMaxSwerve.SetFastMode;
import frc.robot.RevMaxSwerve.SetNormalMode;
import frc.robot.RevMaxSwerve.SetSlowMode;
// Intake Imports
import frc.robot.TwoMotorIntakeRevNeo.Intake;
import frc.robot.TwoMotorIntakeRevNeo.IntakeSubsystem;
import frc.robot.constCrescendoBot.constsJoysticks;
import frc.robot.constCrescendoBot.constMaxSwerveDrive.DriveConstants;

public class RobotContainer {

  // define subsystems5
  private final DriveSubsystemSwerve m_robotDrive = new DriveSubsystemSwerve();
  // private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // Joystick Controller (I/O)
  XboxController m_driverController = new XboxController(constsJoysticks.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(constsJoysticks.kGunnerControllerPort);

  public RobotContainer() {
    configureBindings();

    // Configure default commands
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
  
    //m_shooter.setDefaultCommand(new ShootCommand(m_shooter));

  }

  private void configureBindings() {

    //DRIVER CONTROLS
    new JoystickButton(m_driverController, Button.kBack.value).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));
 
    // Set speed modes
    new JoystickButton(m_driverController, Button.kRightBumper.value).onTrue(new SetSlowMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kRightBumper.value).onFalse(new SetNormalMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onTrue(new SetFastMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onFalse(new SetNormalMode(m_robotDrive));

    //GUNNER CONTROLS
    new JoystickButton(m_gunnerController, Button.kLeftBumper.value).whileTrue(new Intake(m_intake));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
