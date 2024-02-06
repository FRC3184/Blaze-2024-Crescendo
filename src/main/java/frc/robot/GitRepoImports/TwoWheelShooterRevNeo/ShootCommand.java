package frc.robot.GitRepoImports.TwoWheelShooterRevNeo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_shooter; 
    private XboxController driverController = new XboxController(0);

    private double RTrigger = 0.0;
    private double RightShooterRPM = 0.0;
    private double LeftShooterRPM = 0.0;
    private double shooterSpeed = 0.0;
    private boolean readyToShoot = false;


    // RTrigger = driverController.getRightTriggerAxis();
    // private double RightTrigger driverController.getRightTriggerAxis();
    

    public ShootCommand(ShooterSubsystem subsystem){
        m_shooter = subsystem;
        RTrigger = driverController.getRightTriggerAxis();

        addRequirements(subsystem); 
    }

    public void initialize() {}



    @Override
    public void execute() {
        RightShooterRPM = m_shooter.getShooterRPM1();
        LeftShooterRPM = m_shooter.getShooterRPM2();

        SmartDashboard.putNumber("Right Trigger", RTrigger);
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putNumber("Front Shooter RPM", RightShooterRPM);
        SmartDashboard.putNumber("Back Shooter RPM", LeftShooterRPM);
        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot);

        if(m_shooter.getShooterRPM1()>=shooterSpeed-20 && m_shooter.getShooterRPM2()<=shooterSpeed+20){
            readyToShoot = true;
        }
        else readyToShoot = false;

        // press X: plus 500 rpm
        // press B: minus 500 rpm
        // press A: plus 100 rpm
        // press y: minus 100 rpm
        if (driverController.getXButtonPressed()){
            shooterSpeed = shooterSpeed+500;
        }
        else if (driverController.getBButtonPressed()){
            shooterSpeed = shooterSpeed-500;
        }
        else if (driverController.getAButtonPressed()){
            shooterSpeed = shooterSpeed+100;
        }
        else if (driverController.getYButtonPressed()){
            shooterSpeed = shooterSpeed-100;
        }

        if (driverController.getRightBumper()) {
            m_shooter.run(shooterSpeed, shooterSpeed);
        }
        else{
            m_shooter.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }
}