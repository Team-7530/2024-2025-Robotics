package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem implements Subsystem {
    private final TalonFX m_ClimbMotor = new TalonFX(ClimberConstants.MotorId);
    private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(new DigitalInput(ClimberConstants.encoderDI));
    private double m_targetPosition = 0.0;
    private boolean m_isTeleop = true;
  


    public ClimberSubsystem(){
        SmartDashboard.putNumber("ClimbEncoder", m_Encoder.get());


    }

      @Override
  public void periodic() {
    // Put code here to be run every loop
    // if (this.getPosition() <= 0.01) {
    //   m_leftElevatorMotor.stopMotor();
    // }

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
    m_targetPosition = Math.min(pos, ClimberConstants.kClimberMaxPosition);
    // m_ClimberController.setReference(m_targetPosition, CANSparkBase.ControlType.kPosition);
    // m_rightElevatorMotor.follow(m_leftElevatorMotor);
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
    SmartDashboard.putNumber("Climber Postion", this.getPosition());
    // SmartDashboard.putNumber(
    //     "ElevatorCanCoder Postion", m_canCoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("Climber TargetPostion", m_targetPosition);
    // SmartDashboard.putNumber("RClimber Postion", m_rightElevatorEncoder.getPosition());
  }

}