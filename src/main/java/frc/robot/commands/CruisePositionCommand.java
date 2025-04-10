package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CruisePositionCommand extends ParallelCommandGroup {

  /**
   * Sets arm to defense position
   * @param arm Subsystem
   * @param wrist Subsystem
   */
  public CruisePositionCommand(ArmSubsystem arm, WristSubsystem wrist) {
    setName("CruisePositionCommand");
    addCommands(
        arm.armToPositionCommand(ScoringConstants.CruiseArmPosition),
        wrist.wristToPositionCommand(ScoringConstants.CruiseWristPosition));
  }
}
