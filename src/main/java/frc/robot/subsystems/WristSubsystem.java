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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

  private final TalonFX m_wristMotor =
      new TalonFX(WristConstants.WRISTMOTOR_ID, WristConstants.CANBUS);
  private final CANcoder m_wristEncoder =
      new CANcoder(WristConstants.WRISTENCODER_ID, WristConstants.CANBUS);

  private final MotionMagicVoltage m_wristRequest = new MotionMagicVoltage(0).withSlot(0);
  // private final MotionMagicExpoVoltage m_wristRequest = new MotionMagicVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private double wristTargetPosition = 0;
  private boolean m_isTeleop = true;

  public WristSubsystem() {

    initEncoderConfigs();
    initWristConfigs();
  }

  private void initWristConfigs() {
    TalonFXConfiguration configWrist = new TalonFXConfiguration();
    configWrist.MotorOutput.Inverted = WristConstants.kWristInverted;
    configWrist.MotorOutput.NeutralMode = WristConstants.kWristNeutralMode;
    configWrist.Voltage.PeakForwardVoltage = WristConstants.peakForwardVoltage;
    configWrist.Voltage.PeakReverseVoltage = WristConstants.peakReverseVoltage;

    configWrist.Slot0.kG = WristConstants.wristMotorKG;
    configWrist.Slot0.kS = WristConstants.wristMotorKS;
    configWrist.Slot0.kV = WristConstants.wristMotorKV;
    configWrist.Slot0.kA = WristConstants.wristMotorKA;
    configWrist.Slot0.kP = WristConstants.wristMotorKP;
    configWrist.Slot0.kI = WristConstants.wristMotorKI;
    configWrist.Slot0.kD = WristConstants.wristMotorKD;
    configWrist.Feedback.FeedbackRemoteSensorID = m_wristEncoder.getDeviceID();
    configWrist.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configWrist.Feedback.SensorToMechanismRatio = 1.0;
    configWrist.Feedback.RotorToSensorRatio = WristConstants.kWristGearRatio;

    configWrist.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MMagicCruiseVelocity;
    configWrist.MotionMagic.MotionMagicAcceleration = WristConstants.MMagicAcceleration;
    configWrist.MotionMagic.MotionMagicJerk = WristConstants.MMagicJerk;
    configWrist.MotionMagic.MotionMagicExpo_kV = WristConstants.MMagicExpo_kV;
    configWrist.MotionMagic.MotionMagicExpo_kA = WristConstants.MMagicExpo_kA;

    StatusCode status = m_wristMotor.getConfigurator().apply(configWrist);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(Units.Rotations.of(0.17728));

    StatusCode status = m_wristEncoder.getConfigurator().apply(cc_cfg);
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
