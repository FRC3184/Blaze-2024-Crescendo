package frc.robot;

// General Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// Drive Imports
import frc.robot.RevMaxSwerve.DriveSubsystemSwerve;
// Shooter Imports
// Drive Imports
import frc.robot.RevMaxSwerve.Commands.SetFastMode;
import frc.robot.RevMaxSwerve.Commands.SetNormalMode;
import frc.robot.RevMaxSwerve.Commands.SetSlowMode;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.RevMaxSwerve.Commands.AutoIntakeNote;
import frc.robot.RevMaxSwerve.Commands.FaceBackwards;
import frc.robot.RevMaxSwerve.Commands.FaceForward;
import frc.robot.RevMaxSwerve.Commands.FaceRight;
import frc.robot.RevMaxSwerve.Commands.FaceLeft;
// import frc.robot.SubmoduleSubsystemConstants.constsJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.DriveConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.OIConstants;
import frc.robot.TwoWheelShooterRevNeo.Feed;
import frc.robot.TwoWheelShooterRevNeo.FeedAndShoot;
import frc.robot.TwoWheelShooterRevNeo.FeederOff;
import frc.robot.TwoWheelShooterRevNeo.FeederSS;
import frc.robot.TwoWheelShooterRevNeo.Shoot;
import frc.robot.TwoWheelShooterRevNeo.ShooterOff;
import frc.robot.TwoWheelShooterRevNeo.ShooterSubsystem;
// Intake Imports
import frc.robot.OneMotorIntakeRevNeo.*;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstProperties;
/**
 * Contains the robot definition, button bindings for teleop and autonomous configurations.
 */
public class RobotContainer {
  // define subsystems
  private final DriveSubsystemSwerve robotDrive = new DriveSubsystemSwerve();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final FeederSS feeder = new FeederSS();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterLimelight shooterLL = new ShooterLimelight();
  private final IntakeLimelight intakeLL = new IntakeLimelight();
  // Joystick Controller (I/O)
  XboxController driverController = new XboxController(ConstJoysticks.kDriverControllerPort);
  XboxController gunnerController = new XboxController(ConstJoysticks.kGunnerControllerPort);

  // Autonomous Chooser
  private ShuffleboardTab sbCompTab = Shuffleboard.getTab(ConstProperties.CompDashboard.COMPDASHNAME_STRING);
  SendableChooser<Command> autoChooserPathPlan = new SendableChooser<>();

  // Backup Cardinal Directions
  private Command pointF = Commands.run(() -> robotDrive.rotateToAngle(0,
  -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)), robotDrive);
  private Command pointL = Commands.run(() -> robotDrive.rotateToAngle(90,
  -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)), robotDrive);
  private Command pointB = Commands.run(() -> robotDrive.rotateToAngle(180,
  -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)), robotDrive);
  private Command pointR = Commands.run(() -> robotDrive.rotateToAngle(270,
  -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)), robotDrive);
  
  private RepeatCommand repeatPointF = new RepeatCommand(pointF);
  private RepeatCommand repeatPointL = new RepeatCommand(pointL);
  private RepeatCommand repeatPointB = new RepeatCommand(pointB);
  private RepeatCommand repeatPointR = new RepeatCommand(pointR);

  /**
   * Constructor for RobotContainer class.
   */
  public RobotContainer() {
    //Register named autonomous commands
    NamedCommands.registerCommand("Intake", new Intake(intake));
    NamedCommands.registerCommand("IntakeOff", new IntakeOff(intake));
    NamedCommands.registerCommand("Shoot", new Shoot(shooter));
    NamedCommands.registerCommand("ShooterOff", new ShooterOff(shooter));
    NamedCommands.registerCommand("Feed", new Feed(feeder));
    NamedCommands.registerCommand("FeederOff", new FeederOff(feeder));

    // TELEOP Setup
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.

    autoChooserPathPlan = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    sbCompTab.add("Choose Path Planner Auto", autoChooserPathPlan).withSize(2, 1).withPosition(0, 0);

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
              -MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband),
              -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
              -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband),
              DriveConstants.kFieldCentric, true, DriveConstants.rotPt),
            robotDrive));
  }

  private void configureBindings() {

    // DRIVER CONTROLS
    new JoystickButton(driverController, Button.kBack.value)
        .whileTrue(new RunCommand(() -> robotDrive.zeroHeading()));

    new JoystickButton(driverController, Button.kStart.value)
        .whileTrue(new RunCommand(() -> robotDrive.setX(), robotDrive));

    // Cardinal Direction Buttons
    new JoystickButton(driverController, Button.kY.value).whileTrue(new FaceForward(robotDrive));
    new JoystickButton(driverController, Button.kX.value).whileTrue(new FaceLeft(robotDrive));
    new JoystickButton(driverController, Button.kB.value).whileTrue(new FaceRight(robotDrive));
    new JoystickButton(driverController, Button.kA.value).whileTrue(new FaceBackwards(robotDrive));
    
    // Backup Cardinal Direction Buttons
    // new JoystickButton(driverController, Button.kY.value).whileTrue(repeatPointF);
    // new JoystickButton(driverController, Button.kX.value).whileTrue(repeatPointL);
    // new JoystickButton(driverController, Button.kA.value).whileTrue(repeatPointB);
    // new JoystickButton(driverController, Button.kB.value).whileTrue(repeatPointR);
 
    // Set speed modes
    new JoystickButton(driverController, Button.kLeftBumper.value).onTrue(new SetSlowMode(robotDrive));
    new JoystickButton(driverController, Button.kRightBumper.value).onFalse(new SetNormalMode(robotDrive));
    new JoystickButton(driverController, Button.kRightBumper.value).onTrue(new SetFastMode(robotDrive));
    new JoystickButton(driverController, Button.kLeftBumper.value).onFalse(new SetNormalMode(robotDrive));

    // Set point of rotation
    new POVButton(driverController, 45).whileTrue(new RunCommand(() -> robotDrive.setRotPoint(1, -1)));
    new POVButton(driverController, 135).whileTrue(new RunCommand(() -> robotDrive.setRotPoint(1, 1)));
    new POVButton(driverController, 225).whileTrue(new RunCommand(() -> robotDrive.setRotPoint(-1, 1)));
    new POVButton(driverController, 315).whileTrue(new RunCommand(() -> robotDrive.setRotPoint(-1, -1)));
    new JoystickButton(driverController, Button.kRightStick.value).whileTrue(new RunCommand(() -> robotDrive.setRotPoint(0, 0)));

    // Field Centric vs. Robot Centric
    // new POVButton(driverController, 90).toggleOnTrue(new RunCommand(() -> robotDrive.setFieldcentric(false)));
    // new POVButton(driverController, 270).toggleOnTrue(new RunCommand(() -> robotDrive.setFieldcentric(true)));



    //GUNNER CONTROLS
    new JoystickButton(gunnerController, Button.kRightBumper.value).whileTrue(new Intake(intake));
    new JoystickButton(gunnerController, Button.kLeftBumper.value).whileTrue(new Outtake(intake));

    new JoystickButton(gunnerController, Button.kStart.value).whileTrue(new FeedAndShoot(shooter, feeder));
    new JoystickButton(gunnerController, Button.kStart.value).onFalse(new ShooterOff(shooter));

    new JoystickButton(gunnerController, Button.kA.value).whileTrue(new Feed(feeder));
    new JoystickButton(gunnerController, Button.kB.value).whileTrue(new AutoIntakeNote(robotDrive, intakeLL, intake));

    new JoystickButton(gunnerController, Button.kX.value).onTrue(new RunCommand(() -> intakeLL.setPipeline(0), intakeLL));
    new JoystickButton(gunnerController, Button.kY.value).onTrue(new RunCommand(() -> intakeLL.setPipeline(1), intakeLL));
  }

  public int driverLTPressed() {
    if (Math.abs(driverController.getLeftTriggerAxis()) > 0.3) {
      return 1;
    } else {
      return 0;
    }
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
