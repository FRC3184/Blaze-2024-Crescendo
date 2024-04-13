package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Intake;

/**
 * Command to reorient the robot to face forward.
 */
public class OldLLIntake extends Command {
  private final DriveSubsystemSwerve robotDrive;
  private final IntakeLimelight intakeLL;
  private final Intake intake;
  private double thetaPower;
  private PIDController thetaController;
  private double target;
  private final CarriageBelt carriageBelt;

  /**
   * Add subsystem and configure PID controller.
   *
   * @param subsystem drive subsystem
   */
  public OldLLIntake(DriveSubsystemSwerve subsystem, IntakeLimelight LLsubsystem, Intake intakeSubsystem, CarriageBelt carriageSS) {
    robotDrive = subsystem;
    intakeLL = LLsubsystem;
    intake = intakeSubsystem;
    carriageBelt = carriageSS;
    addRequirements(subsystem, LLsubsystem, intakeSubsystem, carriageBelt);
    thetaController = new PIDController(0.005, CardinalConstants.CardinalI, CardinalConstants.CardinalD);
    target = 0;
  }

  @Override
  public void initialize() {
    // intake.setSpeed(0.5);
    // carriageBelt.setSpeed(0.5);
    thetaController.setSetpoint(target);
  }

  @Override
  public void execute() {
    // intake.runSpeed();
    // carriageBelt.runSpeed();
    double NoteX = intakeLL.getX();
    thetaPower = thetaController.calculate(NoteX);
    if(NoteX>target-5 && NoteX<target+5){
    robotDrive.drive(0, 0, thetaPower, false, true);
    } else{
    robotDrive.drive(0, 0, thetaPower, false, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    carriageBelt.stop();
  }
}
