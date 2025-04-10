package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class L2ScoringPositionCommand extends ParallelCommandGroup {
  
  /**
   * Sets arm and wrist to L1 position
   * @param arm Subsystem
   * @param wrist Subsystem
   */
  public L2ScoringPositionCommand(ArmSubsystem arm, WristSubsystem wrist) {
    setName("L2ScoringPositionCommand");
    addCommands(
        arm.armToPositionCommand(ScoringConstants.L2ArmPosition),
        wrist.wristToPositionCommand(ScoringConstants.L2WristPosition));
  }
}
