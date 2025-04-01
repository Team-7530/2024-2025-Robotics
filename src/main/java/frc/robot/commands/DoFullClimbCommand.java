package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class DoFullClimbCommand extends SequentialCommandGroup {

  public DoFullClimbCommand(ClimberSubsystem climb, ArmSubsystem arm, WristSubsystem wrist) {
    addCommands(
        new ClimbPositionCommand(arm, wrist)
            .withName("ClimbPositionCommand"),
        new ClimbCommand(climb)
            .withName("ClimbCommand"));
  }
}
