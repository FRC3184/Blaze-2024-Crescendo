package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class StrafingNoteAlignment extends Command {

    private final IntakeLimelight intakeLL;

    private final DriveSubsystemSwerve robotDrive;
    int driverControllerPort = ConstJoysticks.kDriverControllerPort;
    private final XboxController driverController = new XboxController(driverControllerPort);
    private double target = 0;
    private PIDController XController;
    private double XPower = 0;

    public StrafingNoteAlignment(IntakeLimelight LLIntake, DriveSubsystemSwerve driveSS){
        intakeLL = LLIntake;
        robotDrive = driveSS;

        XController = new PIDController(0.01, 0, 0.0005);
        
        addRequirements(robotDrive);

    }

    public void initialize() {
        XController.setSetpoint(target);
    }

    @Override
    public void execute() {
        double NoteX = intakeLL.getX();
        XPower = XController.calculate(NoteX);

        if (intakeLL.getV()==1){
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), XPower, -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), false, true);
        } else {
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), ConstMaxSwerveDrive.DriveConstants.kFieldCentric, true);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
