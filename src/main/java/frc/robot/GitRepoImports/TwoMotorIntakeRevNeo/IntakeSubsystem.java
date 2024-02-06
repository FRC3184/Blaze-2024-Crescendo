package frc.robot.GitRepoImports.TwoMotorIntakeRevNeo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    
    // Define Motors and Encoders
    private final CANSparkMax m_Intake1 = new CANSparkMax(RobotMap.constIntake.kIntakePort1, MotorType.kBrushless);
    private final CANSparkMax m_Intake2 = new CANSparkMax(RobotMap.constIntake.kIntakePort2, MotorType.kBrushless);

    private final RelativeEncoder m_IntakeEncoder1 = m_Intake1.getEncoder();
    private final RelativeEncoder m_IntakeEncoder2 = m_Intake2.getEncoder();

    private double intakePower1, intakePower2;


    public IntakeSubsystem() {
        m_Intake1.setInverted(RobotMap.constIntake.kIntakeInverted1);
        m_Intake2.setInverted(RobotMap.constIntake.kIntakeInverted2);

        m_IntakeEncoder1.setVelocityConversionFactor(RobotMap.constIntake.kIntakeVelFactor1);
        m_IntakeEncoder2.setVelocityConversionFactor(RobotMap.constIntake.kIntakeVelFactor2);

        resetEncoders();

        intakePower1 = 0;
        intakePower2 = 0;
    }
    
    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {

    }

    // Independent Velocity Setters and Getters
    public double getShooterPower1(){
        return m_IntakeEncoder1.getVelocity();
    }
  
    public double getShooterPower2(){
        return m_IntakeEncoder2.getVelocity();
    }

    public void setShooterPower1(double power){
        // make sure power doesn't surpass accepted values
        if (power > RobotMap.constIntake.kMaxPower) {
            intakePower1 = RobotMap.constIntake.kMaxPower;
        } else if (power < RobotMap.constIntake.kMinPower) {
            intakePower1 = RobotMap.constIntake.kMinPower;
        } else {
            intakePower1 = power;
        }
    }

    public void setShooterPower2(double power){
        // make sure power doesn't surpass accepted values
        if (power > RobotMap.constIntake.kMaxPower) {
            intakePower2 = RobotMap.constIntake.kMaxPower;
        } else if (power < RobotMap.constIntake.kMinPower) {
            intakePower2 = RobotMap.constIntake.kMinPower;
        } else {
            intakePower2 = power;
        }
    }

    // Tandem Velocity Setters and Getters
    public void setShotpower(double power) {
        setShooterPower1(power);
        setShooterPower2(power);
    }

    // SHOOTER OPERATION FUNCTIONS
    // set power and run shooter
    public void run(double power1, double power2) {
        intakePower1 = power1;
        intakePower2 = power2;
        
        // make sure power doesn't surpass accepted values
        if (intakePower1 > RobotMap.constIntake.kMaxPower) {
            intakePower1 = RobotMap.constIntake.kMaxPower;
        } else if (intakePower1 < RobotMap.constIntake.kMinPower) {
            intakePower1 = RobotMap.constIntake.kMinPower;
        }
        if (intakePower2 > RobotMap.constIntake.kMaxPower){
            intakePower2 = RobotMap.constIntake.kMaxPower;
        }else if (intakePower2 < RobotMap.constIntake.kMinPower) {
            intakePower2 = RobotMap.constIntake.kMinPower;
        }

        m_Intake1.set(intakePower1);
        m_Intake2.set(intakePower2);
    }
    // run shooter without setting a new power first
    public void run() {
        m_Intake1.set(intakePower1);
        m_Intake2.set(intakePower2);
    }

    public void stop(){
        m_Intake1.set(0);
        m_Intake2.set(0);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_IntakeEncoder1.setPosition(0);
        m_IntakeEncoder2.setPosition(0);
    }

    public void dashboardOut() {
        SmartDashboard.putNumber("Intake Enc 1", m_IntakeEncoder1.getPosition());
        SmartDashboard.putNumber("Intake Enc 2", m_IntakeEncoder2.getPosition());
        SmartDashboard.putNumber("Intake Vel 1", m_IntakeEncoder1.getVelocity());
        SmartDashboard.putNumber("Intake Vel 2", m_IntakeEncoder2.getVelocity());
    }
}