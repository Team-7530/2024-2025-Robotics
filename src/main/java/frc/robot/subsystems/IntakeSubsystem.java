package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_LIntakeMotor = new TalonFX(IntakeConstants.LINTAKEMOTOR_ID, "CANFD");
  private final TalonFX m_RIntakeMotor = new TalonFX(IntakeConstants.RINTAKEMOTOR_ID, "CANFD");
  private final CANrange m_RangeSensor = new CANrange(IntakeConstants.RANGESENSOR_ID, "CANFD");

  private final VelocityDutyCycle m_intakerequest = new VelocityDutyCycle(0).withSlot(0);

  private double intakeTargetVelocity = 0;
  private boolean m_isIntakeIn = false;
  private boolean m_isTeleop = true;

  public IntakeSubsystem() {
    initIntakeConfigs();
    initRangeSensor();
  }

  private void initIntakeConfigs() {
    TalonFXConfiguration configIntake = new TalonFXConfiguration();

    configIntake.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configIntake.Slot0.kS = IntakeConstants.KSConstant;
    configIntake.Slot0.kP = IntakeConstants.proportialPIDConstant;
    configIntake.Slot0.kI = IntakeConstants.integralPIDConstant;
    configIntake.Slot0.kD = IntakeConstants.derivativePIDConstant;
    configIntake.Slot0.kV = IntakeConstants.feedForwardPIDConstant;
    configIntake.Voltage.PeakForwardVoltage = IntakeConstants.peakForwardVoltage;
    configIntake.Voltage.PeakReverseVoltage = IntakeConstants.peakReverseVoltage;

    configIntake.Slot1.kP = IntakeConstants.proportialTorquePIDConstant;
    configIntake.Slot1.kI = IntakeConstants.integralTorquePIDConstant;
    configIntake.Slot1.kD = IntakeConstants.derivativeTorquePIDConstant;
    configIntake.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.peakForwardTorqueCurrent;
    configIntake.TorqueCurrent.PeakReverseTorqueCurrent = IntakeConstants.peakReverseTorqueCurrent;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    status = m_LIntakeMotor.getConfigurator().apply(configIntake);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
    status = m_RIntakeMotor.getConfigurator().apply(configIntake);
    if (!status.isOK()) {
      System.out.println("Could not apply bottom configs, error code: " + status.toString());
    }
  }

  private void initRangeSensor() {
    CANrangeConfiguration config = new CANrangeConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    status = m_RangeSensor.getConfigurator().apply(config);
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
    if (m_isIntakeIn && hasCoralLoaded()) {
      intakeStop();
    }

    updateSmartDashboard();
  }

  public void setIntakeVelocity(double Lvelocity, double Rvelocity) {
    intakeTargetVelocity = Lvelocity;
    m_isIntakeIn = Lvelocity > 0.0;

    m_LIntakeMotor.setControl(m_intakerequest.withVelocity(Lvelocity));
    m_RIntakeMotor.setControl(m_intakerequest.withVelocity(Rvelocity));
    // m_LIntakeMotor.setControl(m_torqueVelocity.withVelocity(Lvelocity).withFeedForward(1.0));
    // m_RIntakeMotor.setControl(m_torqueVelocity.withVelocity(Rvelocity).withFeedForward(1.0));
  }

  public void setIntakeSpeed(double speed) {
    intakeTargetVelocity = 0;
    m_isIntakeIn = speed > 0.0;

    m_LIntakeMotor.set(speed);
    m_RIntakeMotor.set(speed);
  }

  public void intakeStop() {
    intakeTargetVelocity = 0;
    m_isIntakeIn = false;
    this.setIntakeSpeed(0.0);
  }

  public void intakeIn() {
    this.setIntakeVelocity(IntakeConstants.intakeSpeed, IntakeConstants.intakeSpeed);
  }

  public void intakeOut() {
    this.setIntakeVelocity(IntakeConstants.outtakeSpeedL, IntakeConstants.outtakeSpeedL);
  }

  public void intakeOutSpin() {
    this.setIntakeVelocity(IntakeConstants.outtakeSpeedL, IntakeConstants.outtakeSpeedR);
  }

  public boolean hasCoralLoaded() {
    return m_RangeSensor.getIsDetected().getValue()
        && (m_RangeSensor.getDistance().getValue().lte(IntakeConstants.rangeThreshold));
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
    SmartDashboard.putNumber("Intake TargetVelocity", intakeTargetVelocity);

    SmartDashboard.putNumber("Distance", m_RangeSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Strength", m_RangeSensor.getSignalStrength().getValueAsDouble());
    SmartDashboard.putBoolean("Detected", m_RangeSensor.getIsDetected().getValue());
  }
}
