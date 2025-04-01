package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
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
  private final TalonFX m_ClimbMotorFollower =
      new TalonFX(ClimberConstants.CLIMBMOTORFOLLOWER_ID, ClimberConstants.CANBUS);
  private final DutyCycleEncoder m_ClimbEncoder = 
      new DutyCycleEncoder(ClimberConstants.CLIMBENCODER_ID);
  private final Servo m_ClimberClampServo = new Servo(ClimberConstants.CLAMPSERVO_ID);

  private final MotionMagicTorqueCurrentFOC m_positionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

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

    StatusCode status = m_ClimbMotorFollower.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.Slot0.kP = ClimberConstants.climbMotorTorqueKP;
    configs.Slot0.kI = ClimberConstants.climbMotorTorqueKI;
    configs.Slot0.kD = ClimberConstants.climbMotorTorqueKD;
    configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = ClimberConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = ClimberConstants.MMagicJerk;
    configs.MotionMagic.MotionMagicExpo_kV = ClimberConstants.MMagicExpo_kV;
    configs.MotionMagic.MotionMagicExpo_kA = ClimberConstants.MMagicExpo_kA;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kClimberPositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kClimberPositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    status = m_ClimbMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Make sure we start at 0 */
    this.resetMotorPostion();

    /* Follower is opposite, so we need to invert */
    m_ClimbMotorFollower.setControl(new Follower(m_ClimbMotor.getDeviceID(), true));    

    m_ClimberClampServo.set(ClimberConstants.kUnclampedPosition);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  /**
   * Returns climber to lowered position
   */
  public void restore() {
    this.setPosition(ClimberConstants.kTargetClimberDown);
  }

  /**
   * Returns true if climber is at lowered position
   */
  public boolean isAtRestoredPosition() {
    return MathUtil.isNear(ClimberConstants.kTargetClimberDown, this.getPosition(), POSITION_TOLERANCE);
  }

  /**
   * Closes the clamp and moves to climb position
   */
  public void climb() {
    this.setClamp(true);
    this.setPosition(ClimberConstants.kTargetClimberFull);
  }

  /**
   * Returns true if climber is at full raised position
   */
  public boolean isAtFullClimbPosition() {
    return MathUtil.isNear(ClimberConstants.kTargetClimberFull, this.getPosition(), POSITION_TOLERANCE);
  }

  /**
   * Sets the climber target position
   * @param pos double between 0 and 1
   */
  public void setPosition(double pos) {
    m_isTeleop = false;
    m_targetPosition =
        MathUtil.clamp(
            pos, ClimberConstants.kClimberPositionMin, ClimberConstants.kClimberPositionMax);

    if (!m_isClamped || (m_targetPosition > this.getPosition())) { // is climbing or no ratchet
      m_ClimbMotor.setControl(m_positionRequest.withPosition(m_targetPosition));
    }
  }

  /**
   * Returns the current climb motor position as a double
   */
  public double getPosition() {
    return m_ClimbMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns true if climber is at the target position or within the tolerance range
   */
  public boolean isAtPosition() {
    return MathUtil.isNear(m_targetPosition, this.getPosition(),  POSITION_TOLERANCE);
  }

  /**
   * sets the speed of the climber
   * @param speed target speed
   */
  public void setSpeed(double speed) {
    m_targetPosition = 0.0;

    if (!m_isClamped || (speed > 0.0)) // is climbing or no ratchet
      m_ClimbMotor.setControl(m_manualRequest.withOutput(speed));
  }

  /**
   * Stops the motor and activates the brake
   */
  public void stop() {
    m_ClimbMotor.setControl(m_brake);
  }

  /**
   * Handles climber controls during teleop
   * @param val controller deadband
   */
  public void teleopClimb(double val) {
    val = MathUtil.applyDeadband(val, STICK_DEADBAND);

    if (USE_POSITIONCONTROL) {
      if (val != 0.0) {
        this.setPosition(this.getPosition() + (val * ClimberConstants.kClimbTeleopFactor));
      }
    } else {
      if (m_isTeleop || (val != 0.0)) {
        m_isTeleop = true;
        this.setSpeed(val * ClimberConstants.kClimberSpeed);
      }
    }
  }

  /**
   * Opens or closes the clamp
   * @param clampOn bool
   */
  public void setClamp(boolean clampOn) {
    m_isClamped = clampOn;
    this.stop();

    m_ClimberClampServo.set(
        m_isClamped ? ClimberConstants.kClampedPosition : ClimberConstants.kUnclampedPosition);
  }

  /**
   * Returns true if clamp is closed, false if open
   */
  public boolean getClamp() {
    return m_isClamped;
  }

  /**
   * Returns the climb encoder position
   */
  public double encoderPosition() {
    double pos = m_ClimbEncoder.get() + ClimberConstants.kClimberEncoderOffset;
    return Math.abs(pos - (int)pos);
  }

  /**
   * Resets the motors internal position
   */
  public void resetMotorPostion() {
    m_ClimbMotor.setPosition(this.encoderPosition() * ClimberConstants.kClimberGearRatio);
    m_ClimbMotorFollower.setPosition(this.encoderPosition() * ClimberConstants.kClimberGearRatio);
  }

  /**
   * Updates the Smart Dashboard
   */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber Postion", this.getPosition());
    SmartDashboard.putNumber("Climber Encoder Postion", this.encoderPosition());
    SmartDashboard.putNumber("Climber TargetPostion", m_targetPosition);
  }
}
