package frc.robot.GitRepoImports.TwoWheelShooterRevNeo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    
    // Define Motors and Encoders
    private final CANSparkMax m_Shooter1 = new CANSparkMax(ShooterConstants.kMotorPortWheel1, MotorType.kBrushless);
    private final CANSparkMax m_Shooter2 = new CANSparkMax(ShooterConstants.kMotorPortWheel2, MotorType.kBrushless);

    private final RelativeEncoder m_ShooterEncoder1 = m_Shooter1.getEncoder();
    private final RelativeEncoder m_shooterEncoder2 = m_Shooter2.getEncoder();

    // Define PID controller
    // PID values for each system is stored in a subsystem specific constants file 
    private SparkPIDController m_shootPID1, m_shootPID2;
    private double shootSpeed1, shootSpeed2;


    public ShooterSubsystem() {
        m_Shooter1.setInverted(ShooterConstants.invertedWheel1);
        m_Shooter2.setInverted(ShooterConstants.invertedWheel2);

        m_ShooterEncoder1.setVelocityConversionFactor(ShooterConstants.velFactorWheel1);
        m_shooterEncoder2.setVelocityConversionFactor(ShooterConstants.velFactorWheel2);

        resetEncoders();

        shootSpeed1 = 0;
        shootSpeed2 = 0;
        
        // ** SETTING UP PID FOR SHOOTER
        // PID init
        m_shootPID1 = m_Shooter1.getPIDController();
        m_shootPID2 = m_Shooter2.getPIDController();

        // set PID coefficients
        m_shootPID1.setP(ShooterConstants.kP);
        m_shootPID1.setI(ShooterConstants.kI);
        m_shootPID1.setD(ShooterConstants.kD);
        m_shootPID1.setIZone(ShooterConstants.kIz);
        m_shootPID1.setFF(ShooterConstants.kFF);
        m_shootPID1.setOutputRange(ShooterConstants.kMinOut, ShooterConstants.kMaxOut);

        m_shootPID2.setP(ShooterConstants.kP);
        m_shootPID2.setI(ShooterConstants.kI);
        m_shootPID2.setD(ShooterConstants.kD);
        m_shootPID2.setIZone(ShooterConstants.kIz);
        m_shootPID2.setFF(ShooterConstants.kFF);
        m_shootPID2.setOutputRange(ShooterConstants.kMinOut, ShooterConstants.kMaxOut);
    }
    
    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {

    }

    // Independent Velocity Setters and Getters
    public double getShooterRPM1(){
        return m_ShooterEncoder1.getVelocity();
    }
  
    public double getShooterRPM2(){
        return m_shooterEncoder2.getVelocity();
    }

    public void setShooterRPM1(double speed){
        // make sure speed doesn't surpass accepted values
        if (speed > ShooterConstants.maxRPM) {
            shootSpeed1 = ShooterConstants.maxRPM;
        } else if (speed < -ShooterConstants.maxRPM) {
            shootSpeed1 = -ShooterConstants.maxRPM;
        } else {
            shootSpeed1 = speed;
        }
    }

    public void setShooterRPM2(double speed){
        // make sure speed doesn't surpass accepted values
        if (speed > ShooterConstants.maxRPM) {
            shootSpeed2 = ShooterConstants.maxRPM;
        } else if (speed < -ShooterConstants.maxRPM) {
            shootSpeed2 = -ShooterConstants.maxRPM;
        } else {
            shootSpeed2 = speed;
        }
    }

    // Tandem Velocity Setters and Getters
    public void setShotSpeed(double speed) {
        setShooterRPM1(speed);
        setShooterRPM2(speed);
    }

    // SHOOTER OPERATION FUNCTIONS
    // set speed and run shooter
    public void run(double speed1, double speed2) {
        shootSpeed1 = speed1;
        shootSpeed2 = speed2;
        
        // make sure speed doesn't surpass accepted values
        if (shootSpeed1 > ShooterConstants.maxRPM) {
            shootSpeed1 = ShooterConstants.maxRPM;
        } else if (shootSpeed1 < -ShooterConstants.maxRPM) {
            shootSpeed1 = -ShooterConstants.maxRPM;
        }
        if (shootSpeed2 > ShooterConstants.maxRPM){
            shootSpeed2 = ShooterConstants.maxRPM;
        }else if (shootSpeed2 < -ShooterConstants.maxRPM) {
            shootSpeed2 = -ShooterConstants.maxRPM;
        }

        m_shootPID1.setReference(shootSpeed1, CANSparkMax.ControlType.kVelocity);
        m_shootPID2.setReference(shootSpeed2, CANSparkMax.ControlType.kVelocity);
    }
    // run shooter without setting a new speed first
    public void run() {
        m_shootPID1.setReference(shootSpeed1, CANSparkMax.ControlType.kVelocity);
        m_shootPID2.setReference(shootSpeed2, CANSparkMax.ControlType.kVelocity);
    }

    public void stop(){
        m_Shooter1.set(0);
        m_Shooter2.set(0);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_ShooterEncoder1.setPosition(0);
        m_shooterEncoder2.setPosition(0);
    }

    public void dashboardOut() {
        SmartDashboard.putNumber("Shoot Enc 1", m_ShooterEncoder1.getPosition());
        SmartDashboard.putNumber("Shoot Enc 2", m_shooterEncoder2.getPosition());
        SmartDashboard.putNumber("Shoot Vel 1", m_ShooterEncoder1.getVelocity());
        SmartDashboard.putNumber("Shoot Vel 2", m_shooterEncoder2.getVelocity());
    }
}