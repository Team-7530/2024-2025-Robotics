package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem implements Subsystem {
  private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(new DigitalInput(ClimberConstants.encoderDI));
  private final TalonFX m_ClimbMotor = new TalonFX(ClimberConstants.MotorId);

  private static final TalonFXConfiguration configs = new TalonFXConfiguration();

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();


  private double m_targetPosition = 0.0;
  private boolean m_isTeleop = true;
  
  public ClimberSubsystem() {

    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Voltage.ofBaseUnits(8.0, Units.Volts))
      .withPeakReverseVoltage(Voltage.ofBaseUnits(-8.0, Units.Volts));

    configs.Slot1.kP = 60.0; // An error of 1 rotation results in 60 A output
    configs.Slot1.kI = 0.0; // No output for integrated error
    configs.Slot1.kD = 6.0; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Current.ofBaseUnits(120.0, Units.Amps))
      .withPeakReverseTorqueCurrent(Current.ofBaseUnits(-120.0, Units.Amps));

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
  }

  @Override
  public void periodic() {
    double pos = this.getPosition();
    double motorCycle = m_ClimbMotor.getDutyCycle().getValueAsDouble();

    if (((motorCycle > 0.0) && (pos >=  ClimberConstants.kClimberMaxPosition)) ||
        ((motorCycle < 0.0) && (pos <=  ClimberConstants.kClimberMinPosition))) {
      m_ClimbMotor.stopMotor();
    }

    updateSmartDashboard();
  }

  public void up() {
    this.setPosition(ClimberConstants.kTargetClimberUp);
  }

  public void down() {
    this.setPosition(ClimberConstants.kTargetClimberDown);
  }

  public void setPosition(double pos) {
    m_isTeleop = false;
    m_targetPosition = Math.max(Math.min(pos, ClimberConstants.kClimberMaxPosition), ClimberConstants.kClimberMinPosition) * ClimberConstants.kClimberGearRatio;
    m_ClimbMotor.setControl(m_positionVoltage.withPosition(m_targetPosition));
  }

  public void setSpeed(double speed) {
    m_isTeleop = true;
    m_targetPosition = 0.0;
    m_ClimbMotor.set(speed);
  }

  public void stop() {
    this.setSpeed(0);
  }

  public double getPosition() {
    return m_Encoder.get();
  }

  public boolean getOnTarget() {
    // return Math.abs(this.getPosition() - m_targetPosition) < ClimberConstants.kOnTargetThreshold;
    return false;
  }

  public void teleop(double val) {
    val = MathUtil.applyDeadband(val, 0.01) * 0.5;

    if (m_isTeleop || (val != 0.0)) {
      this.setSpeed(val);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber Encoder Postion", this.getPosition());
    SmartDashboard.putNumber("Climber TargetPostion", m_targetPosition);
  }

}