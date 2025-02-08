package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARMMOTOR_ID, "CANFD");
  private final CANcoder m_armEncoder = new CANcoder(ArmConstants.ARMENCODER_ID, "CANDFD");

  private final PositionDutyCycle m_armrequest = new PositionDutyCycle(0).withSlot(0);
  // private final MotionMagicVoltage m_armrequest_mm = new MotionMagicVoltage(0).withSlot(0);

  private final TalonFX m_wristMotor = new TalonFX(WristConstants.WRISTMOTOR_ID);
  private final CANcoder m_wristEncoder = new CANcoder(WristConstants.WRISTENCODER_ID);

  private final PositionDutyCycle m_wristrequest = new PositionDutyCycle(0).withSlot(0);
  // private final MotionMagicVoltage m_wristrequest_mm = new MotionMagicVoltage(0).withSlot(0);

  private final TalonFX m_LIntakeMotor = new TalonFX(IntakeConstants.LINTAKEMOTOR_ID, "CANFD");
  private final TalonFX m_RIntakeMotor = new TalonFX(IntakeConstants.RINTAKEMOTOR_ID, "CANFD");

  private final VelocityDutyCycle m_intakerequest = new VelocityDutyCycle(0).withSlot(0);

  private double armTargetPosition = 0;
  private double wristTargetPosition = 0;
  private double intakeTargetVelocity = 0;
  private boolean m_isTeleop = true;

  public ArmSubsystem() {

    initArmConfigs();
    initWristConfigs();
    initIntakeConfigs();
  }

  private void initArmConfigs() {
    TalonFXConfiguration configArm = new TalonFXConfiguration();
    configArm.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configArm.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configArm.Slot0.kS = ArmConstants.armMotorKS;
    configArm.Slot0.kV = ArmConstants.armMotorKV;
    configArm.Slot0.kA = ArmConstants.armMotorKA;
    configArm.Slot0.kP = ArmConstants.armMotorKP;
    configArm.Slot0.kI = ArmConstants.armMotorKI;
    configArm.Slot0.kD = ArmConstants.armMotorKD;
    configArm.Feedback.FeedbackRemoteSensorID = m_armEncoder.getDeviceID();
    configArm.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    configArm.Feedback.SensorToMechanismRatio = 1.0;
    configArm.Feedback.RotorToSensorRatio = ArmConstants.kArmGearRatio;

    MotionMagicConfigs configMagic = configArm.MotionMagic;
    configMagic.MotionMagicCruiseVelocity = ArmConstants.MMagicCruiseVelocity;
    configMagic.MotionMagicAcceleration = ArmConstants.MMagicAcceleration;
    configMagic.MotionMagicJerk = ArmConstants.MMagicJerk;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    status = m_armMotor.getConfigurator().apply(configArm);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
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

  @Override
  public void periodic() {
    // Put code here to be run every loop
    // if (this.getPosition() <= 0.01) {
    //   m_wristMotor.stopMotor();
    // }
    updateSmartDashboard();
  }

  public void setArmPosition(double pos) {
    m_isTeleop = false;
    armTargetPosition = pos;

    // m_armMotor.setControl(m_armrequest.withPosition(armTargetPosition));
    m_armMotor.setControl(m_armrequest.withPosition(armTargetPosition));
  }

  public Angle getArmPosition() {
    return m_armMotor.getPosition().getValue();
  }

  public void setArmSpeed(double aspeed) {
    armTargetPosition = 0;
    m_isTeleop = true;
    m_armMotor.set(aspeed);
  }

  public void armStop() {
    this.setArmSpeed(0.0);
  }

  public void armUp() {
    this.setArmPosition(ArmConstants.kTargetArmHigh);
  }

  public void armDown() {
    this.setArmPosition(ArmConstants.kTargetArmLow);
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

  public void setIntakeVelocity(double Lvelocity, double Rvelocity) {
    intakeTargetVelocity = Lvelocity;

    m_LIntakeMotor.setControl(m_intakerequest.withVelocity(Lvelocity));
    m_RIntakeMotor.setControl(m_intakerequest.withVelocity(Rvelocity));
    // m_LIntakeMotor.setControl(m_torqueVelocity.withVelocity(Lvelocity).withFeedForward(1.0));
    // m_RIntakeMotor.setControl(m_torqueVelocity.withVelocity(Rvelocity).withFeedForward(1.0));
  }

  public void setIntakeSpeed(double speed) {
    intakeTargetVelocity = 0;

    m_LIntakeMotor.set(speed);
    m_RIntakeMotor.set(speed);
  }

  public void intakeStop() {
    intakeTargetVelocity = 0;
    this.setIntakeSpeed(0.0);
  }

  public void intakeIn() {
    this.setIntakeVelocity(
        IntakeConstants.shooterReverseSpeed, IntakeConstants.shooterReverseSpeed);
  }

  public void intakeOut() {
    this.setIntakeVelocity(
        -IntakeConstants.shooterReverseSpeed, -IntakeConstants.shooterReverseSpeed);
  }

  public void intakeOutSpin() {
    this.setIntakeVelocity(
        -IntakeConstants.shooterReverseSpeed, -IntakeConstants.shooterReverseSpeed * 0.5);
  }

  public void teleop(double arm, double wrist, double intake) {
    arm = MathUtil.applyDeadband(arm, STICK_DEADBAND) * 0.1;
    wrist = MathUtil.applyDeadband(wrist, STICK_DEADBAND) * 0.1;
    intake = MathUtil.applyDeadband(intake, STICK_DEADBAND) * 0.1;

    if (m_isTeleop && ((arm != 0.0) || (wrist != 0.0) || (intake != 0.0))) {
      this.setArmSpeed(arm);
      this.setWristSpeed(wrist);
      this.setIntakeSpeed(intake);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm Postion", this.getArmPosition().in(Units.Degrees));
    SmartDashboard.putNumber(
        "ArmCanCoder Postion", m_armEncoder.getPosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Arm TargetPostion", armTargetPosition);

    SmartDashboard.putNumber("Wrist Postion", this.getWristPosition().in(Units.Degrees));
    SmartDashboard.putNumber(
        "WristCanCoder Postion", m_wristEncoder.getPosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Wrist TargetPostion", wristTargetPosition);

    SmartDashboard.putNumber("LIntake Speed", m_LIntakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("RIntake Speed", m_RIntakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake TargetVelocity", intakeTargetVelocity);
  }
}
