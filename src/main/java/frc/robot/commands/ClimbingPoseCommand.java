package frc.robot.commands;

import static frc.robot.Constants.*;



import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;


public class ClimbingPoseCommand extends Command {
 

 

  public ClimbingPoseCommand(ArmSubsystem arm, double position) {
    
    
    addRequirements(arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    
  }
}
