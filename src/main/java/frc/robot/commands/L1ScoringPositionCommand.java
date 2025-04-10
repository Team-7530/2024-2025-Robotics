package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class L1ScoringPositionCommand extends ParallelCommandGroup {
  /**
   * Sets arm and wrist to L1 position
   * @param arm Subsystem
   * @param wrist Subsystem
   */
  public L1ScoringPositionCommand(ArmSubsystem arm, WristSubsystem wrist) {
    setName("L1ScoringPositionCommand");
    addCommands(
        arm.armToPositionCommand(ScoringConstants.L1ArmPosition),
        wrist.wristToPositionCommand(ScoringConstants.L1WristPosition));
  }
}
