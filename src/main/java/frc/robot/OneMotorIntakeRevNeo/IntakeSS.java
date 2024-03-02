package frc.robot.OneMotorIntakeRevNeo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubmoduleSubsystemConstants.ConstIntake;

public class IntakeSS extends SubsystemBase {
    // motors
    public final CANSparkMax m_IntakeMotor = new CANSparkMax(ConstIntake.kIntakePort1, MotorType.kBrushless);

    // encoders
    public final RelativeEncoder m_IntakeMotorEncoder = m_IntakeMotor.getEncoder();

    // PID controllers
    private SparkPIDController m_intakePID = m_IntakeMotor.getPIDController();

    public double kP, kI, kD, kIz, kFF, kMaxOut, kMinOut, maxRPM;
    private int intakeSpeed;
  

    public IntakeSS() {
      m_IntakeMotor.setInverted(true);

      m_IntakeMotorEncoder.setVelocityConversionFactor(1.0);

      m_IntakeMotorEncoder.setPosition(0);

      intakeSpeed = 0;


      // set PID Constants
      kP = 6e-5;
      kI = 5e-9;
      kD = 1e-4;
      kIz = 0;
      kFF = 1.0/5700.0;
      kMaxOut = 1;
      kMinOut = -1;
      maxRPM = 5700;

      // set PID coefficients
      m_intakePID.setP(kP);
      m_intakePID.setI(kI);
      m_intakePID.setD(kD);
      m_intakePID.setIZone(kIz);
      m_intakePID.setFF(kFF);
      m_intakePID.setOutputRange(kMinOut, kMaxOut);

    }

    public void runIntake() {
      double intakeSetpoint = intakeSpeed;

      m_intakePID.setReference(-intakeSetpoint, CANSparkMax.ControlType.kVelocity);
    }
    

    public int getIntakeSpeed() {
      return intakeSpeed;
    }

    public void setIntakeSpeed(int speed) {
      intakeSpeed = speed;
    }

    public void dashboardOut() {
        SmartDashboard.putNumber("Left Shoot Enc", m_IntakeMotorEncoder.getPosition());
        SmartDashboard.putNumber("Left Shoot Vel", m_IntakeMotorEncoder.getVelocity());
    }

    public double getIntakeRPM(){
            return m_IntakeMotorEncoder.getVelocity();

    }

    @Override
    public void periodic() {

    }

    public void move(double speed){
      m_IntakeMotor.set(speed);
    }

    public void stop(){
      m_IntakeMotor.set(0);
    }

    @Override
    public void simulationPeriodic() {

    }
}
