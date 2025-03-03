package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class L2ScoringPositionCommand extends ParallelCommandGroup {

  public L2ScoringPositionCommand(ArmSubsystem arm, WristSubsystem wrist) {
    addCommands(
        new ArmToPositionCommand(arm, ScoringConstants.L2ArmPosition).withName("ArmToL2Position"),
        new WristToPositionCommand(wrist, ScoringConstants.L2WristPosition, false)
            .withName("WristToL2Position"));
  }
}
