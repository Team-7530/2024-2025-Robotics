package frc.robot.subsystems;
import static frc.robot.Constants.*;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_wristMotor = new TalonFX(ArmConstants.ARMMOTOR_ID);
  private final CANcoder m_canCoder = new CANcoder(ArmConstants.CANCODER_ID);

  // private final PositionDutyCycle m_wristrequest = new PositionDutyCycle(0).withSlot(0);
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withSlot(0);

  private double wristTargetPosition = 0;
  private boolean m_isTeleop = true;

  public ArmSubsystem() {

    TalonFXConfiguration configWrist = new TalonFXConfiguration();

    configWrist.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configWrist.Slot0.kS = ArmConstants.armMotorKS;
    configWrist.Slot0.kV = ArmConstants.armMotorKV;
    configWrist.Slot0.kA = ArmConstants.armMotorKA;
    configWrist.Slot0.kP = ArmConstants.armMotorKP;
    configWrist.Slot0.kI = ArmConstants.armMotorKI;
    configWrist.Slot0.kD = ArmConstants.armMotorKD;

    MotionMagicConfigs configMagic = configWrist.MotionMagic;
    configMagic.MotionMagicCruiseVelocity = ArmConstants.MMagicCruiseVelocity;
    configMagic.MotionMagicAcceleration = ArmConstants.MMagicAcceleration;
    configMagic.MotionMagicJerk = ArmConstants.MMagicJerk;

    m_wristMotor.getConfigurator().apply(configWrist);
    m_wristMotor.setPosition(
        m_canCoder.getAbsolutePosition().getValue());// * Angle.of(ArmConstants.kArmGearRatio, Units.Degrees));
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    // if (this.getPosition() <= 0.01) {
    //   m_wristMotor.stopMotor();
    // }
    updateSmartDashboard();
  }

  public void up() {
    this.setPosition(ArmConstants.kTargetArmHigh);
  }

  public void down() {
    this.setPosition(ArmConstants.kTargetArmLow);
  }

  public void setPosition(double pos) {
    m_isTeleop = false;
    wristTargetPosition = pos;

    // m_wristMotor.setControl(m_wristrequest.withPosition(wristTargetPosition));
    m_wristMotor.setControl(m_request.withPosition(wristTargetPosition));
  }

  public Angle getPosition() {
    return m_wristMotor.getPosition().getValue();
  }

  public boolean getOnTarget() {
    return Math.abs(this.getPosition() - wristTargetPosition) < 0.1;
    
  }

  public void setSpeed(double wspeed) {
    wristTargetPosition = 0;
    m_isTeleop = true;
    m_wristMotor.set(wspeed);
  }

  public void stop() {
    this.setSpeed(0.0);
  }

  public void teleop(double val) {
    val = MathUtil.applyDeadband(val, STICK_DEADBAND) * 0.1;

    if (m_isTeleop || (val != 0.0)) {
      this.setSpeed(val);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Wrist Postion", this.getPosition().in(Units.Degrees));
    SmartDashboard.putNumber("WristCanCoder Postion", m_canCoder.getPosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Wrist TargetPostion", wristTargetPosition);
  }
}