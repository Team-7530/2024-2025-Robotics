package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem implements Subsystem {
  private final TalonFX m_ClimbMotor =
      new TalonFX(ClimberConstants.CLIMBMOTOR_ID, ClimberConstants.CANBUS);
  private final DutyCycleEncoder m_ClimbEncoder =
      new DutyCycleEncoder(ClimberConstants.CLIMBENCODER_ID);
  private final Servo m_ClimberClampServo = new Servo(ClimberConstants.CLAMPSERVO_ID);

  private final MotionMagicTorqueCurrentFOC m_positionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(1);
  // private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();

  private double m_targetPosition = 0.0;
  private boolean m_isTeleop = false;
  private boolean m_isClamped = false;

  public ClimberSubsystem() {
    initClimberConfigs();
  }

  private void initClimberConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = ClimberConstants.kClimberInverted;
    configs.MotorOutput.NeutralMode = ClimberConstants.kClimberNeutralMode;
    configs.Voltage.PeakForwardVoltage = ClimberConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = ClimberConstants.peakReverseVoltage;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = ClimberConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = ClimberConstants.peakReverseTorqueCurrent;

    configs.Slot0.kG = ClimberConstants.climbMotorKG;
    configs.Slot0.kS = ClimberConstants.climbMotorKS;
    configs.Slot0.kV = ClimberConstants.climbMotorKV;
    configs.Slot0.kA = ClimberConstants.climbMotorKA;
    configs.Slot0.kP = ClimberConstants.climbMotorKP;
    configs.Slot0.kI = ClimberConstants.climbMotorKI;
    configs.Slot0.kD = ClimberConstants.climbMotorKD;
    configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Slot1.kP = ClimberConstants.climbMotorKP_Tor;
    configs.Slot1.kI = ClimberConstants.climbMotorKI_Tor;
    configs.Slot1.kD = ClimberConstants.climbMotorKD_Tor;
    configs.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    configs.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = ClimberConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = ClimberConstants.MMagicJerk;
    configs.MotionMagic.MotionMagicExpo_kV = ClimberConstants.MMagicExpo_kV;
    configs.MotionMagic.MotionMagicExpo_kA = ClimberConstants.MMagicExpo_kA;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kClimberPositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kClimberPositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    StatusCode status = m_ClimbMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    /* Make sure we start at 0 */
    this.resetMotorPostion();
    m_ClimberClampServo.set(ClimberConstants.kUnclampedPosition);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void restore() {
    this.setPosition(ClimberConstants.kTargetClimberUp);
  }

  public boolean isAtRestoredPosition() {
    return MathUtil.isNear(ClimberConstants.kTargetClimberUp, this.getPosition(), POSITION_TOLERANCE);
  }

  public void climb() {
    this.setClamp(true);
    this.setPosition(ClimberConstants.kTargetClimberDown);
  }

  public boolean isAtFullClimbPosition() {
    return MathUtil.isNear(ClimberConstants.kTargetClimberDown, this.getPosition(), POSITION_TOLERANCE);
  }

  public void setPosition(double pos) {
    m_isTeleop = false;
    m_targetPosition =
        MathUtil.clamp(
            pos, ClimberConstants.kClimberPositionMin, ClimberConstants.kClimberPositionMax);

    if (!m_isClamped || (m_targetPosition > this.getPosition())) { // is climbing or no ratchet
      m_ClimbMotor.setControl(m_positionRequest.withPosition(m_targetPosition));
    }
  }

  public double getPosition() {
    return m_ClimbMotor.getPosition().getValueAsDouble();
  }

  public boolean isAtPosition() {
    return MathUtil.isNear(m_targetPosition, this.getPosition(),  POSITION_TOLERANCE);
  }

  public void setSpeed(double speed) {
    m_targetPosition = 0.0;

    if (!m_isClamped || (speed > 0.0)) // is climbing or no ratchet
    m_ClimbMotor.set(speed);
  }

  public void stop() {
    m_ClimbMotor.setControl(m_brake);
  }

  public void teleopClimb(double val) {
    val = MathUtil.applyDeadband(val, STICK_DEADBAND);

    if (USE_POSITIONCONTROL) {
      if (val != 0.0)
        this.setPosition(this.getPosition() + (val * ClimberConstants.kClimbTeleopFactor));
    } else {
      if (m_isTeleop || (val != 0.0)) {
        m_isTeleop = true;
        this.setSpeed(val * ClimberConstants.kClimberSpeed);
      }
    }
  }

  public void setClamp(boolean clampOn) {
    m_isClamped = clampOn;
    this.stop();

    m_ClimberClampServo.set(
        m_isClamped ? ClimberConstants.kClampedPosition : ClimberConstants.kUnclampedPosition);
  }

  public boolean getClamp() {
    return m_isClamped;
  }

  public void resetMotorPostion() {
    m_ClimbMotor.setPosition(
        (m_ClimbEncoder.get() - ClimberConstants.kClimberEncoderMin)
            * ClimberConstants.kClimberGearRatio);
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber Postion", this.getPosition());
    SmartDashboard.putNumber("Climber Encoder Postion", m_ClimbEncoder.get());
    SmartDashboard.putNumber("Climber TargetPostion", m_targetPosition);
  }
}
