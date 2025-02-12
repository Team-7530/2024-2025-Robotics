package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
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

public class WristSubsystem extends SubsystemBase {

  private final TalonFX m_wristMotor = new TalonFX(WristConstants.WRISTMOTOR_ID, "rio");
  private final CANcoder m_wristEncoder = new CANcoder(WristConstants.WRISTENCODER_ID, "rio");

  private final PositionDutyCycle m_wristrequest = new PositionDutyCycle(0).withSlot(0);
  // private final MotionMagicVoltage m_wristrequest_mm = new MotionMagicVoltage(0).withSlot(0);

  private double wristTargetPosition = 0;
  private boolean m_isTeleop = true;

  public WristSubsystem() {

    initWristConfigs();
  }

  private void initWristConfigs() {
    TalonFXConfiguration configWrist = new TalonFXConfiguration();
    configWrist.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configWrist.Slot0.kS = WristConstants.wristMotorKS;
    configWrist.Slot0.kV = WristConstants.wristMotorKV;
    configWrist.Slot0.kA = WristConstants.wristMotorKA;
    configWrist.Slot0.kP = WristConstants.wristMotorKP;
    configWrist.Slot0.kI = WristConstants.wristMotorKI;
    configWrist.Slot0.kD = WristConstants.wristMotorKD;
    configWrist.Feedback.FeedbackRemoteSensorID = m_wristEncoder.getDeviceID();
    configWrist.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    configWrist.Feedback.SensorToMechanismRatio = 1.0;
    configWrist.Feedback.RotorToSensorRatio = WristConstants.kWristGearRatio;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    status = m_wristMotor.getConfigurator().apply(configWrist);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void setWristPosition(double pos) {
    m_isTeleop = false;
    wristTargetPosition = pos;

    // m_wristMotor.setControl(m_wristrequest.withPosition(wristTargetPosition));
    m_wristMotor.setControl(m_wristrequest.withPosition(wristTargetPosition));
  }

  public Angle getWristPosition() {
    return m_wristMotor.getPosition().getValue();
  }

  public void setWristSpeed(double wspeed) {
    wristTargetPosition = 0;
    m_isTeleop = true;
    m_wristMotor.set(wspeed);
  }

  public void wristStop() {
    this.setWristSpeed(0.0);
  }

  public void teleop(double wrist) {
    wrist = MathUtil.applyDeadband(wrist, STICK_DEADBAND) * 0.1;

    if (m_isTeleop && (wrist != 0.0)) {
      this.setWristSpeed(wrist);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Wrist Postion", this.getWristPosition().in(Units.Degrees));
    SmartDashboard.putNumber(
        "WristCanCoder Postion", m_wristEncoder.getPosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Wrist TargetPostion", wristTargetPosition);
  }
}
