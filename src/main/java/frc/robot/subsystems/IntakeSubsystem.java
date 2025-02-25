package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_LIntakeMotor =
      new TalonFX(IntakeConstants.LINTAKEMOTOR_ID, IntakeConstants.CANBUS);
  private final TalonFX m_RIntakeMotor =
      new TalonFX(IntakeConstants.RINTAKEMOTOR_ID, IntakeConstants.CANBUS);
  private final CANrange m_RangeSensor =
      new CANrange(IntakeConstants.RANGESENSOR_ID, IntakeConstants.CANBUS);

  private final VelocityTorqueCurrentFOC m_velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  // private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest =
  //    new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private double LintakeTargetVelocity = 0;
  private double RintakeTargetVelocity = 0;
  private boolean m_isTeleop = true;

  public IntakeSubsystem() {
    initIntakeConfigs();
    initRangeSensor();
  }

  private void initIntakeConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = IntakeConstants.kLIntakeInverted;
    configs.MotorOutput.NeutralMode = IntakeConstants.kIntakeNeutralMode;
    configs.Voltage.PeakForwardVoltage = IntakeConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = IntakeConstants.peakReverseVoltage;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = IntakeConstants.peakReverseTorqueCurrent;

    configs.Slot0.kS = IntakeConstants.intakeMotorTorqueKS;
    configs.Slot0.kP = IntakeConstants.intakeMotorTorqueKP;
    configs.Slot0.kI = IntakeConstants.intakeMotorTorqueKI;
    configs.Slot0.kD = IntakeConstants.intakeMotorTorqueKD;

    configs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = IntakeConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = IntakeConstants.MMagicJerk;

    configs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANrange;
    configs.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    configs.HardwareLimitSwitch.ReverseLimitRemoteSensorID = m_RangeSensor.getDeviceID();
    configs.HardwareLimitSwitch.ReverseLimitEnable = true;

    StatusCode status = m_LIntakeMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = IntakeConstants.kRIntakeInverted;
    status = m_RIntakeMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply bottom configs, error code: " + status.toString());
    }
  }

  private void initRangeSensor() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.FovParams.FOVCenterX = IntakeConstants.kRangeFOVCenterX;
    config.FovParams.FOVCenterY = IntakeConstants.kRangeFOVCenterY;
    config.FovParams.FOVRangeX = IntakeConstants.kRangeFOVRangeX;
    config.FovParams.FOVRangeY = IntakeConstants.kRangeFOVRangeY;
    config.ProximityParams.ProximityThreshold = IntakeConstants.kProxThreshold;
    config.ProximityParams.ProximityHysteresis = IntakeConstants.kProxHysteresis;
    config.ProximityParams.MinSignalStrengthForValidMeasurement = IntakeConstants.kMinSigStrength;

    /* User can change the configs if they want, or leave it empty for factory-default */
    StatusCode status = m_RangeSensor.getConfigurator().apply(config);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }

    /* Set the signal update rate */
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        m_RangeSensor.getDistance(),
        m_RangeSensor.getSignalStrength(),
        m_RangeSensor.getIsDetected());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void setIntakeVelocity(double Lvelocity, double Rvelocity) {
    LintakeTargetVelocity = Lvelocity * IntakeConstants.kIntakeGearRatio;
    RintakeTargetVelocity = Rvelocity * IntakeConstants.kIntakeGearRatio;

    m_LIntakeMotor.setControl(m_velocityRequest.withVelocity(LintakeTargetVelocity));
    m_RIntakeMotor.setControl(m_velocityRequest.withVelocity(RintakeTargetVelocity));
  }

  public void setIntakeSpeed(double speed) {
    LintakeTargetVelocity = 0;
    RintakeTargetVelocity = 0;

    m_LIntakeMotor.set(speed);
    m_RIntakeMotor.set(speed);
  }

  public void intakeStop() {
    LintakeTargetVelocity = 0;
    RintakeTargetVelocity = 0;

    m_LIntakeMotor.setControl(m_brake);
    m_RIntakeMotor.setControl(m_brake);
  }

  public void intakeIn() {
    this.setIntakeVelocity(IntakeConstants.intakeVelocity, IntakeConstants.intakeVelocity);
  }

  public void intakeOut() {
    this.setIntakeVelocity(IntakeConstants.outtakeVelocityR, IntakeConstants.outtakeVelocityR);
  }

  public void intakeOutSpin() {
    this.setIntakeVelocity(IntakeConstants.outtakeVelocityL, IntakeConstants.outtakeVelocityR);
  }

  public boolean hasCoralLoaded() {
    return m_RangeSensor.getIsDetected().getValue();
  }

  public void teleop(double intake) {
    intake = MathUtil.applyDeadband(intake, STICK_DEADBAND) * 0.1;

    if (m_isTeleop && (intake != 0.0)) {
      this.setIntakeSpeed(intake);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("LIntake Speed", m_LIntakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("RIntake Speed", m_RIntakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake TargetVelocity", LintakeTargetVelocity);

    SmartDashboard.putNumber("Distance", m_RangeSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Strength", m_RangeSensor.getSignalStrength().getValueAsDouble());
    SmartDashboard.putBoolean("Detected", m_RangeSensor.getIsDetected().getValue());
  }
}
