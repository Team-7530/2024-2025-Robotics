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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

  private final TalonFX m_wristMotor =
      new TalonFX(WristConstants.WRISTMOTOR_ID, WristConstants.CANBUS);
  private final CANcoder m_wristEncoder =
      new CANcoder(WristConstants.WRISTENCODER_ID, WristConstants.CANBUS);

  private final MotionMagicVoltage m_wristRequest = new MotionMagicVoltage(0).withSlot(0);
  // private final MotionMagicExpoVoltage m_wristRequest = new
  // MotionMagicExpoVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private double wristTargetPosition = 0;
  private boolean m_isTeleop = true;

  public WristSubsystem() {

    initEncoderConfigs();
    initWristConfigs();
  }

  private void initWristConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = WristConstants.kWristInverted;
    configs.MotorOutput.NeutralMode = WristConstants.kWristNeutralMode;
    configs.Voltage.PeakForwardVoltage = WristConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = WristConstants.peakReverseVoltage;

    configs.Slot0.kG = WristConstants.wristMotorKG;
    configs.Slot0.kS = WristConstants.wristMotorKS;
    configs.Slot0.kV = WristConstants.wristMotorKV;
    configs.Slot0.kA = WristConstants.wristMotorKA;
    configs.Slot0.kP = WristConstants.wristMotorKP;
    configs.Slot0.kI = WristConstants.wristMotorKI;
    configs.Slot0.kD = WristConstants.wristMotorKD;
    configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Feedback.FeedbackRemoteSensorID = m_wristEncoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configs.Feedback.SensorToMechanismRatio = 1.0;
    configs.Feedback.RotorToSensorRatio = WristConstants.kWristGearRatio;

    configs.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = WristConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = WristConstants.MMagicJerk;
    configs.MotionMagic.MotionMagicExpo_kV = WristConstants.MMagicExpo_kV;
    configs.MotionMagic.MotionMagicExpo_kA = WristConstants.MMagicExpo_kA;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.kWristPositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.kWristPositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    StatusCode status = m_wristMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = WristConstants.kWristEncoderDirection;
    configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(WristConstants.kWristEncoderOffset));

    StatusCode status = m_wristEncoder.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
    // set starting position to current absolute position
    m_wristEncoder.setPosition(m_wristEncoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void setWristPosition(double pos) {
    m_isTeleop = false;
    wristTargetPosition =
        MathUtil.clamp(pos, WristConstants.kWristPositionMin, WristConstants.kWristPositionMax);

    m_wristMotor.setControl(m_wristRequest.withPosition(wristTargetPosition));
  }

  public double getWristPosition() {
    return m_wristEncoder.getPosition().getValueAsDouble();
  }

  public void setWristSpeed(double wspeed) {
    wristTargetPosition = 0;
    m_isTeleop = true;
    m_wristMotor.set(wspeed);
  }

  public void wristStop() {
    m_wristMotor.setControl(m_brake);
  }

  public void wristUp() {
    this.setWristPosition(WristConstants.kTargetWristHigh);
  }

  public void wristDown() {
    this.setWristPosition(WristConstants.kTargetWristLow);
  }

  public void teleop(double wrist) {
    wrist = MathUtil.applyDeadband(wrist, STICK_DEADBAND) * 0.1;

    if (m_isTeleop && (wrist != 0.0)) {
      this.setWristSpeed(wrist);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Wrist Postion", this.getWristPosition());
    SmartDashboard.putNumber("Wrist TargetPostion", wristTargetPosition);
  }
}
