package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class L2ScoringCommand extends SequentialCommandGroup {

  public L2ScoringCommand(ArmSubsystem arm, WristSubsystem wrist) {
    addCommands(
        new ArmToPositionCommand(arm, ScoringConstants.L2ArmPosition).withName("ArmToL2Position"),
        new WristToPositionCommand(wrist, ScoringConstants.L2WristPosition)
            .withName("WristToL2Position"));
  }
}
