package frc.robot;

// General Imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
// Constants Imports
import frc.robot.SubmoduleSubsystemConstants.*;
// Shooter Imports
// Drive Imports
import frc.robot.RevMaxSwerve.*;
import frc.robot.RevMaxSwerve.Commands.FaceForward;
import frc.robot.RevMaxSwerve.Commands.SetFastMode;
import frc.robot.RevMaxSwerve.Commands.SetNormalMode;
import frc.robot.RevMaxSwerve.Commands.SetSlowMode;
import frc.robot.Sensors.Limelight;
// import frc.robot.SubmoduleSubsystemConstants.constsJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.DriveConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.OIConstants;
// Intake Imports
import frc.robot.OneMotorIntakeRevNeo.*;
import frc.robot.TwoWheelShooterRevNeo.ShooterSubsystem;

public class RobotContainer {

  // define subsystems
  private final DriveSubsystemSwerve m_robotDrive = new DriveSubsystemSwerve();
  // private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSS m_intake = new IntakeSS();
  private final Limelight m_limelight = new Limelight();

  
  // Joystick Controller (I/O)
  XboxController m_driverController = new XboxController(ConstJoysticks.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(ConstJoysticks.kGunnerControllerPort);

  // Cardinal Direction functions 
  private Command pointF = Commands.run(() -> m_robotDrive.rotateToAngle2(0,
  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)), m_robotDrive);
  private Command pointL = Commands.run(() -> m_robotDrive.rotateToAngle2(90,
  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)), m_robotDrive);
  private Command pointB = Commands.run(() -> m_robotDrive.rotateToAngle2(180,
  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)), m_robotDrive);
  private Command pointR = Commands.run(() -> m_robotDrive.rotateToAngle2(270,
  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)), m_robotDrive);
  
  private RepeatCommand repeatPointF = new RepeatCommand(pointF);
  private RepeatCommand repeatPointL = new RepeatCommand(pointL);
  private RepeatCommand repeatPointB = new RepeatCommand(pointB);
  private RepeatCommand repeatPointR = new RepeatCommand(pointR);


  // Autonomous Chooser
  private ShuffleboardTab sbCompTab = Shuffleboard.getTab("Competition");
  SendableChooser<Command> autoChooserPathPlan = new SendableChooser<>();

  public RobotContainer() {
    // TELEOP Setup
    configureBindings();

     // Build an auto chooser. This will use Commands.none() as the default option.

    autoChooserPathPlan = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    sbCompTab.add("Choose Path Planner Auto", autoChooserPathPlan);


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), ConstJoysticks.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), ConstJoysticks.kDriveDeadband),
                DriveConstants.kFieldCentric, true),
            m_robotDrive));
  
    //m_shooter.setDefaultCommand(new ShootCommand(m_shooter));
  }

  private void configureBindings() {

    //DRIVER CONTROLS
    new JoystickButton(m_driverController, Button.kBack.value).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));

    new JoystickButton(m_driverController, Button.kStart.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    //Cardinal Directional Buttons
    // rotate robot to face forward
    new JoystickButton(m_driverController, Button.kY.value).whileTrue(repeatPointF);
    // rotate robot to face left
    new JoystickButton(m_driverController, Button.kX.value).whileTrue(repeatPointL);
    // rotate robot to face back
    new JoystickButton(m_driverController, Button.kA.value).whileTrue(repeatPointB);
    // rotate robot to face right
    new JoystickButton(m_driverController, Button.kB.value).whileTrue(repeatPointR);
    // new JoystickButton(m_driverController, Button.kY.value).whileTrue(new FaceForward(m_robotDrive));
    // new JoystickButton(m_driverController, Button.kB.value).whileTrue(new FaceRight(m_robotDrive));
    // new JoystickButton(m_driverController, Button.kX.value).whileTrue(new FaceLeft(m_robotDrive));
    // new JoystickButton(m_driverController, Button.kA.value).whileTrue(new FaceBackwards(m_robotDrive));


 
    // Set speed modes
    new JoystickButton(m_driverController, Button.kRightBumper.value).onTrue(new SetSlowMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kRightBumper.value).onFalse(new SetNormalMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onTrue(new SetFastMode(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onFalse(new SetNormalMode(m_robotDrive));


    //GUNNER CONTROLS
    new JoystickButton(m_gunnerController, Button.kRightBumper.value).whileTrue(new Intake(m_intake));
    new JoystickButton(m_gunnerController, Button.kLeftBumper.value).whileTrue(new Outtake(m_intake));
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return autoChooserPathPlan.getSelected();
  }
}
