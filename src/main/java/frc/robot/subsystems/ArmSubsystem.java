package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARMMOTOR_ID, ArmConstants.CANBUS);
  private final CANcoder m_armEncoder =
      new CANcoder(ArmConstants.ARMENCODER_ID, ArmConstants.CANBUS);

  private final MotionMagicVoltage m_armRequest = new MotionMagicVoltage(0).withSlot(0);
  // private final MotionMagicExpoVoltage m_armRequest = new MotionMagicVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private double armTargetPosition = 0;
  private boolean m_isTeleop = true;

  public ArmSubsystem() {

    initEncoderConfigs();
    initArmConfigs();
  }

  private void initArmConfigs() {
    TalonFXConfiguration configArm = new TalonFXConfiguration();

    configArm.MotorOutput.Inverted = ArmConstants.kArmInverted;
    configArm.MotorOutput.NeutralMode = ArmConstants.kArmNeutralMode;
    configArm.Voltage.PeakForwardVoltage = ArmConstants.peakForwardVoltage;
    configArm.Voltage.PeakReverseVoltage = ArmConstants.peakReverseVoltage;

    configArm.Slot0.kG = ArmConstants.armMotorKG;
    configArm.Slot0.kS = ArmConstants.armMotorKS;
    configArm.Slot0.kV = ArmConstants.armMotorKV;
    configArm.Slot0.kA = ArmConstants.armMotorKA;
    configArm.Slot0.kP = ArmConstants.armMotorKP;
    configArm.Slot0.kI = ArmConstants.armMotorKI;
    configArm.Slot0.kD = ArmConstants.armMotorKD;
    configArm.Feedback.FeedbackRemoteSensorID = m_armEncoder.getDeviceID();
    configArm.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configArm.Feedback.SensorToMechanismRatio = 1.0;
    configArm.Feedback.RotorToSensorRatio = ArmConstants.kArmGearRatio;

    configArm.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MMagicCruiseVelocity;
    configArm.MotionMagic.MotionMagicAcceleration = ArmConstants.MMagicAcceleration;
    configArm.MotionMagic.MotionMagicJerk = ArmConstants.MMagicJerk;
    configArm.MotionMagic.MotionMagicExpo_kV = ArmConstants.MMagicExpo_kV;
    configArm.MotionMagic.MotionMagicExpo_kA = ArmConstants.MMagicExpo_kA;

    StatusCode status = m_armMotor.getConfigurator().apply(configArm);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(Units.Rotations.of(-0.406));

    StatusCode status = m_armEncoder.getConfigurator().apply(cc_cfg);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
    // set starting position to current absolute position
    m_armEncoder.setPosition(m_armEncoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void setArmPosition(double pos) {
    m_isTeleop = false;
    armTargetPosition =
        MathUtil.clamp(pos, ArmConstants.kArmPositionMin, ArmConstants.kArmPositionMax);

    m_armMotor.setControl(m_armRequest.withPosition(armTargetPosition));
  }

  public Angle getArmPosition() {
    return m_armEncoder.getPosition().getValue();
  }

  public void setArmSpeed(double aspeed) {
    armTargetPosition = 0;
    m_isTeleop = true;
    m_armMotor.set(aspeed);
  }

  public void armStop() {
    m_armMotor.setControl(m_brake);
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
