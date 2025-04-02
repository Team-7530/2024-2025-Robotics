package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class DoAllResetCommand extends ParallelCommandGroup {

  public DoAllResetCommand(ArmSubsystem arm, WristSubsystem wrist, ClimberSubsystem climb) {
    addCommands(
        new CruisePositionCommand(arm, wrist)
            .withName("CruisePositionCommand"),
        new ClimberStowCommand(climb)
            .withName("ClimberStowCommand"));
  }
}
