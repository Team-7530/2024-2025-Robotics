package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberSubsystem implements Subsystem {
  private final TalonFX m_ClimbMotor = new TalonFX(ClimberConstants.CLIMBMOTOR_ID, ClimberConstants.CANBUS);
  private final VictorSPX m_RotateMotor = new VictorSPX(ClimberConstants.ROTATEMOTOR_ID);
  private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(ClimberConstants.ENCODER_ID);
  private final Servo m_ClimberClampServo = new Servo(ClimberConstants.CLAMPSERVO_ID);

  // private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  // private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  private final NeutralOut m_brake = new NeutralOut();

  private double m_targetPosition = 0.0;
  private boolean m_isTeleop = true;
  private boolean m_isClamped = false;

  public ClimberSubsystem() {

    initClimberConfigs();
  }

  private void initClimberConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configs.Slot0.kS = ClimberConstants.climbMotorKS;
    configs.Slot0.kV = ClimberConstants.climbMotorKV;
    configs.Slot0.kA = ClimberConstants.climbMotorKA;
    configs.Slot0.kP = ClimberConstants.climbMotorKP;
    configs.Slot0.kI = ClimberConstants.climbMotorKI;
    configs.Slot0.kD = ClimberConstants.climbMotorKD;

    configs.Voltage.withPeakForwardVoltage(Volts.of(ClimberConstants.peakForwardVoltage));
    configs.Voltage.withPeakReverseVoltage(Volts.of(ClimberConstants.peakReverseVoltage));

    configs.Slot1.kP = ClimberConstants.proportialTorquePIDConstant;
    configs.Slot1.kI = ClimberConstants.integralTorquePIDConstant;
    configs.Slot1.kD = ClimberConstants.derivativeTorquePIDConstant;
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(ClimberConstants.peakForwardTorqueCurrent));
    configs.TorqueCurrent.withPeakReverseTorqueCurrent(Amps.of(ClimberConstants.peakReverseTorqueCurrent));

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_ClimbMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    /* Make sure we start at 0 */
    m_ClimbMotor.setPosition(0);
    m_ClimberClampServo.set(ClimberConstants.kUnclampedPosition);
  }

  @Override
  public void periodic() {
    double pos = this.getPosition();
    double motorCycle = m_ClimbMotor.getDutyCycle().getValueAsDouble();

    if (((motorCycle > 0.0) && (pos >= ClimberConstants.kClimberMaxPosition))
        || ((motorCycle < 0.0) && (pos <= ClimberConstants.kClimberMinPosition))) {
      m_ClimbMotor.stopMotor();
    }

    m_ClimberClampServo.set(m_isClamped ? ClimberConstants.kClampedPosition : ClimberConstants.kUnclampedPosition);

    updateSmartDashboard();
  }

  public void restore() {
    this.setPosition(ClimberConstants.kClimberMinPosition);
  }

  public void climb() {
    this.setPosition(ClimberConstants.kClimberMaxPosition);
  }

  public void setPosition(double pos) {
    m_isTeleop = false;
    m_targetPosition =
        MathUtil.clamp(pos, ClimberConstants.kClimberMinPosition, ClimberConstants.kClimberMaxPosition);
    // m_ClimbMotor.setControl(m_positionVoltage.withPosition(m_targetPosition));

    double speed = 0.0; // default is 0 

    if ( m_targetPosition > this.getPosition()) // is climbing
      speed = m_isClamped ? ClimberConstants.kClimberSpeed2 : ClimberConstants.kClimberSpeed;
    else if (!m_isClamped)
      speed = -ClimberConstants.kClimberSpeed; // not climbing and not clamped
    m_ClimbMotor.set(speed);
  }

  public void setSpeed(double speed) {
    m_isTeleop = true;
    m_targetPosition = 0.0;
  
    if (!m_isClamped || (speed > 0.0))
      m_ClimbMotor.set(speed);
  }

  public void stop() {
    m_ClimbMotor.setControl(m_brake);
  }

  public double getPosition() {
    return m_Encoder.get();
  }

  public boolean getOnTarget() {
    // return Math.abs(this.getPosition() - m_targetPosition) < ClimberConstants.kOnTargetThreshold;
    return false;
  }

  public void rotateOpen() {
    this.setRotateSpeed(ClimberConstants.kRotateSpeed);
  }

  public void rotateClosed() {
    this.setRotateSpeed(-ClimberConstants.kRotateSpeed);
  }

  public void setRotateSpeed(double speed) {
    m_RotateMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopRotate() {
    m_RotateMotor.set(ControlMode.PercentOutput, 0);
  }

  // boolean toggles for specific subsystems are meant to be in the subsystem class, not robot container
  public void toggleClamp() {
      setClamp(!m_isClamped);
  }

  public void setClamp(boolean toggle) {
    m_isClamped = toggle;
    this.stop();
    m_ClimberClampServo.set(m_isClamped ? ClimberConstants.kClampedPosition : ClimberConstants.kUnclampedPosition);
  }


  public void teleop(double val, double rotate) {
    val = MathUtil.applyDeadband(val, 0.01);
    rotate = MathUtil.applyDeadband(rotate, 0.01);

    if (m_isTeleop || (val != 0.0)) {
      this.setSpeed(val * 0.45);     
    }
    if (m_isTeleop || (rotate != 0.0)) {
      this.setRotateSpeed(rotate);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber Encoder Postion", this.getPosition());
    SmartDashboard.putNumber("Climber TargetPostion", m_targetPosition);
  }
}
