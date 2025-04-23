package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sim.Mechanisms;

public class ArmWristSubsystem extends SubsystemBase {

  public final ArmSubsystem arm = new ArmSubsystem();
  public final WristSubsystem wrist = new WristSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();

  private Mechanisms m_mechanism = new Mechanisms();

  public ArmWristSubsystem() {
  }
  
  @Override
  public void periodic() {
    m_mechanism.update(arm.getRotorPosition(), arm.getPosition(), wrist.getPosition());
  }
  
  /**
   * Moves the arm/wrist to a specific position
   *
   * @param pos position between 0 and 1
   */
  public void setPosition(double armpos, double wristpos) {
    arm.setPosition(armpos);
    wrist.setPosition(wristpos);
  }

  /**
   * Returns true if arm/wrist is at the target position or is within the error tolerance range
   */
  public boolean isAtPosition() {
    return arm.isAtPosition() && wrist.isAtPosition();
  }

  /**
   * Sets the arm rotation speed
   * 
   * @param aspeed requires a double
   */
  public void setArmSpeed(double aspeed) {
    arm.setSpeed(aspeed);
  }

  /**
   * Sets the wrist rotation speed
   * 
   * @param wspeed requires a double
   */
  public void setWristSpeed(double wspeed) {
    wrist.setSpeed(wspeed);
  }

  /**
   * Stops moter movement and activates the motor brake
   */
  public void stop() {
    arm.stop();
    wrist.stop();
  }

  /**
   * Attempts to hold the motor at its current position (can cause jitter)
   */
  public void hold() {
    arm.hold();
    wrist.hold();
  }

  /**
   * Teleop controls
   * @param aspeed a double that sets the arm speed during teleop
   */
  public void teleop(double aspeed, double wspeed) {
    arm.teleop(aspeed);
    wrist.teleop(wspeed);
  }

  public Command armWristToPositionCommand(double armPos, double wristPos) {
    return new SequentialCommandGroup(arm.armToPositionCommand(armPos), wrist.wristToPositionCommand(wristPos))
        .withName("ArmWristToPositionCommand")
        .withTimeout(5.0);
  }

  public Command cruisePositionCommand() {
    return armWristToPositionCommand(ScoringConstants.CruiseArmPosition, ScoringConstants.CruiseWristPosition)
        .withName("cruisePositionCommand")
        .withTimeout(5.0);
  }
  public Command getCoralPositionCommand() {
    return armWristToPositionCommand(ScoringConstants.LoadArmPosition, ScoringConstants.LoadWristPosition)
        .withName("getCoralPositionCommand")
        .withTimeout(5.0);
  }
  public Command l1ScoringPositionCommand() {
    return armWristToPositionCommand(ScoringConstants.L1ArmPosition, ScoringConstants.L1WristPosition)
        .withName("l1ScoringPositionCommand")
        .withTimeout(5.0);
  }
  public Command l2ScoringPositionCommand() {
    return armWristToPositionCommand(ScoringConstants.L2ArmPosition, ScoringConstants.L2WristPosition)
        .withName("l2ScoringPositionCommand")
        .withTimeout(5.0);
  }
  public Command climbPositionCommand() {
    return armWristToPositionCommand(ScoringConstants.ClimbArmPosition, ScoringConstants.ClimbWristPosition)
        .withName("climbPositionCommand")
        .withTimeout(5.0)
        .finallyDo(() -> {
          arm.stop();
          wrist.stop();
        });
  }

}
