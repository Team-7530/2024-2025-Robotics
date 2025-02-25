package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class StowCommand extends SequentialCommandGroup {

  public StowCommand(ArmSubsystem arm, WristSubsystem wrist) {
    addCommands(
        new ArmToPositionCommand(arm, ScoringConstants.StowArmPosition)
            .withName("ArmToLoadingPosition"),
        new WristToPositionCommand(wrist, ScoringConstants.StowWristPosition)
            .withName("WristToLoadingPosition"));
  }
}
