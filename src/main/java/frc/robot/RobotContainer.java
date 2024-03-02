package frc.robot;

// General Imports
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// Drive Imports
import frc.robot.RevMaxSwerve.DriveSubsystemSwerve;
import frc.robot.RevMaxSwerve.SetFastMode;
import frc.robot.RevMaxSwerve.SetNormalMode;
import frc.robot.RevMaxSwerve.SetSlowMode;
// Constants Imports
// import frc.robot.SubmoduleSubsystemConstants.*;
// Shooter Imports
// import frc.robot.TwoWheelShooterRevNeo.*;
// import frc.robot.Sensors.Limelight;
// import frc.robot.SubmoduleSubsystemConstants.constsJoysticks;
import frc.robot.SubmoduleSubsystemConstants.constMaxSwerveDrive.DriveConstants;
// Intake Imports
// import frc.robot.TwoMotorIntakeRevNeo.*;
import frc.robot.SubmoduleSubsystemConstants.constsJoysticks;

/**
 * Contains the robot definition, button bindings for teleop and autonomous configurations.
 */
public class RobotContainer {

  // define subsystems
  private final DriveSubsystemSwerve robotDrive = new DriveSubsystemSwerve();
  // private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // private final IntakeSubsystem m_intake = new IntakeSubsystem();
  // private final Limelight m_limelight = new Limelight();

  // Joystick Controller (I/O)
  XboxController driverController = new XboxController(constsJoysticks.kDriverControllerPort);
  XboxController gunnerController = new XboxController(constsJoysticks.kGunnerControllerPort);

  // Autonomous Chooser
  private ShuffleboardTab sbCompTab = Shuffleboard.getTab("Competition");
  SendableChooser<Command> autoChooserPathPlan = new SendableChooser<>();

  /**
   * Constructor for RobotContainer class.
   */
  public RobotContainer() {
    // TELEOP Setup
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.

    autoChooserPathPlan = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    sbCompTab.add("Choose Path Planner Auto", autoChooserPathPlan);

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
              -MathUtil.applyDeadband(driverController.getLeftY(), constsJoysticks.kDriveDeadband),
              -MathUtil.applyDeadband(driverController.getLeftX(), constsJoysticks.kDriveDeadband),
              -MathUtil.applyDeadband(driverController.getRightX(), constsJoysticks.kDriveDeadband),
              DriveConstants.kFieldCentric, true),
            robotDrive));

    // m_shooter.setDefaultCommand(new ShootCommand(m_shooter));

  }

  private void configureBindings() {

    // DRIVER CONTROLS
    new JoystickButton(driverController, Button.kBack.value)
        .whileTrue(new RunCommand(() -> robotDrive.zeroHeading()));

    new JoystickButton(driverController, Button.kStart.value)
        .whileTrue(new RunCommand(() -> robotDrive.setX(), robotDrive));

    // Set speed modes
    new JoystickButton(driverController, Button.kRightBumper.value)
      .onTrue(new SetSlowMode(robotDrive));
    new JoystickButton(driverController, Button.kRightBumper.value)
      .onFalse(new SetNormalMode(robotDrive));
    new JoystickButton(driverController, Button.kLeftBumper.value)
      .onTrue(new SetFastMode(robotDrive));
    new JoystickButton(driverController, Button.kLeftBumper.value)
      .onFalse(new SetNormalMode(robotDrive));

    // GUNNER CONTROLS
    // new JoystickButton(m_gunnerController,
    // Button.kLeftBumper.value).whileTrue(new Intake(m_intake));

  }

  /**
   * Retrieve the autonomous command that is selected in the Autonomous Chooser.
   *
   * @return Command
   */
  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return autoChooserPathPlan.getSelected();
  }
}
