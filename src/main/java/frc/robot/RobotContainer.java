package frc.robot;

// General Imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
// Autonomous Imports
import frc.robot.Autos.*;
// Constants Imports
import frc.robot.SubmoduleSubsystemConstants.*;
// Shooter Imports
// import frc.robot.TwoWheelShooterRevNeo.*;
// Drive Imports
import frc.robot.RevMaxSwerve.*;
// import frc.robot.SubmoduleSubsystemConstants.constsJoysticks;
import frc.robot.SubmoduleSubsystemConstants.constMaxSwerveDrive.DriveConstants;
// Intake Imports
import frc.robot.TwoMotorIntakeRevNeo.*;

public class RobotContainer {

  private final SendableChooser<Command> autoChooserPathPlan;

  // define subsystems
  private final DriveSubsystemSwerve m_robotDrive = new DriveSubsystemSwerve();
  // private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // Joystick Controller (I/O)
  XboxController m_driverController = new XboxController(constsJoysticks.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(constsJoysticks.kGunnerControllerPort);

  // Autonomous Chooser
  private ShuffleboardTab sbCompTab = Shuffleboard.getTab("Competition");
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  // AUTONOMOUS ROUTINES - FINAL WORKING
  private final Command m_FINALdoNothing = new FINALdoNothing(m_robotDrive);

  // IN PROGRESS AUTOS
  private final Command m_TESTdoNothing = new doNothing(m_robotDrive);


  public RobotContainer() {
    // AUTONOMOUS Setup
    // Competition Ready Autonomous Programs
    m_autoChooser.setDefaultOption("Do Nothing", m_FINALdoNothing);
    // Test Autonomous Programs
    m_autoChooser.addOption("TEST Do Nothing", m_TESTdoNothing);
    // Add data to the dashboard
    sbCompTab.add("Select Autonomous", m_autoChooser);




    // TELEOP Setup
    configureBindings();

     // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooserPathPlan = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooserPathPlan);
  

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
    // return Commands.print("No autonomous command configured");
    return m_autoChooser.getSelected();
  }
}
