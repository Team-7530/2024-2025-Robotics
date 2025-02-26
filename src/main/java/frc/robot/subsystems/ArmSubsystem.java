package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARMMOTOR_ID, ArmConstants.CANBUS);
  private final CANcoder m_armEncoder =
      new CANcoder(ArmConstants.ARMENCODER_ID, ArmConstants.CANBUS);

  // private final MotionMagicVoltage m_armRequest = new MotionMagicVoltage(0).withSlot(0);
  private final MotionMagicExpoVoltage m_armRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private double armTargetPosition = 0;
  private boolean m_isTeleop = true;

  public ArmSubsystem() {

    initEncoderConfigs();
    initArmConfigs();
  }

  private void initArmConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = ArmConstants.kArmInverted;
    configs.MotorOutput.NeutralMode = ArmConstants.kArmNeutralMode;
    configs.Voltage.PeakForwardVoltage = ArmConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = ArmConstants.peakReverseVoltage;

    configs.Slot0.kG = ArmConstants.armMotorKG;
    configs.Slot0.kS = ArmConstants.armMotorKS;
    configs.Slot0.kV = ArmConstants.armMotorKV;
    configs.Slot0.kA = ArmConstants.armMotorKA;
    configs.Slot0.kP = ArmConstants.armMotorKP;
    configs.Slot0.kI = ArmConstants.armMotorKI;
    configs.Slot0.kD = ArmConstants.armMotorKD;
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Feedback.FeedbackRemoteSensorID = m_armEncoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configs.Feedback.SensorToMechanismRatio = 1.0;
    configs.Feedback.RotorToSensorRatio = ArmConstants.kArmGearRatio;

    configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = ArmConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = ArmConstants.MMagicJerk;
    configs.MotionMagic.MotionMagicExpo_kV = ArmConstants.MMagicExpo_kV;
    configs.MotionMagic.MotionMagicExpo_kA = ArmConstants.MMagicExpo_kA;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.kArmPositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.kArmPositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    StatusCode status = m_armMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = ArmConstants.kArmEncoderDirection;
    configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(ArmConstants.kArmEncoderOffset));

    StatusCode status = m_armEncoder.getConfigurator().apply(configs);
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

  public double getArmPosition() {
    return m_armEncoder.getPosition().getValueAsDouble();
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
    SmartDashboard.putNumber("Arm Postion", this.getArmPosition());
    SmartDashboard.putNumber("Arm TargetPostion", armTargetPosition);
  }
}
