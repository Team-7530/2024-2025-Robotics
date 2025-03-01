
package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CruisePositionCommand extends ParallelCommandGroup {

  public CruisePositionCommand(ArmSubsystem arm, WristSubsystem wrist) {
    addCommands(
        new ArmToPositionCommand(arm, ScoringConstants.CruiseArmPosition)
            .withName("ArmToLoadingPosition"),
        new WristToPositionCommand(wrist, ScoringConstants.CruiseWristPosition, true)
            .withName("WristToLoadingPosition"));
  }
}
