package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARMMOTOR_ID, "CANFD");
  private final CANcoder m_armEncoder = new CANcoder(ArmConstants.ARMENCODER_ID, "CANFD");

  private final PositionDutyCycle m_armrequest = new PositionDutyCycle(0).withSlot(0);
  // private final MotionMagicVoltage m_armrequest_mm = new MotionMagicVoltage(0).withSlot(0);

  private double armTargetPosition = 0;
  private boolean m_isTeleop = true;

  public ArmSubsystem() {

    initArmConfigs();
  }

  private void initArmConfigs() {
    TalonFXConfiguration configArm = new TalonFXConfiguration();
    configArm.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configArm.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configArm.Slot0.kS = ArmConstants.armMotorKS;
    configArm.Slot0.kV = ArmConstants.armMotorKV;
    configArm.Slot0.kA = ArmConstants.armMotorKA;
    configArm.Slot0.kP = ArmConstants.armMotorKP;
    configArm.Slot0.kI = ArmConstants.armMotorKI;
    configArm.Slot0.kD = ArmConstants.armMotorKD;
    configArm.Feedback.FeedbackRemoteSensorID = m_armEncoder.getDeviceID();
    configArm.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    configArm.Feedback.SensorToMechanismRatio = 1.0;
    configArm.Feedback.RotorToSensorRatio = ArmConstants.kArmGearRatio;

    MotionMagicConfigs configMagic = configArm.MotionMagic;
    configMagic.MotionMagicCruiseVelocity = ArmConstants.MMagicCruiseVelocity;
    configMagic.MotionMagicAcceleration = ArmConstants.MMagicAcceleration;
    configMagic.MotionMagicJerk = ArmConstants.MMagicJerk;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    status = m_armMotor.getConfigurator().apply(configArm);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void setArmPosition(double pos) {
    m_isTeleop = false;
    armTargetPosition = pos;

    // m_armMotor.setControl(m_armrequest.withPosition(armTargetPosition));
    m_armMotor.setControl(m_armrequest.withPosition(armTargetPosition));
  }

  public Angle getArmPosition() {
    return m_armMotor.getPosition().getValue();
  }

  public void setArmSpeed(double aspeed) {
    armTargetPosition = 0;
    m_isTeleop = true;
    m_armMotor.set(aspeed);
  }

  public void armStop() {
    this.setArmSpeed(0.0);
  }

  public void armUp() {
    this.setArmPosition(ArmConstants.kTargetArmHigh);
  }

  public void armDown() {
    this.setArmPosition(ArmConstants.kTargetArmLow);
  }

  public void teleop(double arm) {
    arm = MathUtil.applyDeadband(arm, STICK_DEADBAND) * 0.1;
    if (m_isTeleop && (arm != 0.0)) {
      this.setArmSpeed(arm);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm Postion", this.getArmPosition().in(Units.Degrees));
    SmartDashboard.putNumber(
        "ArmCanCoder Postion", m_armEncoder.getPosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Arm TargetPostion", armTargetPosition);
  }
}
