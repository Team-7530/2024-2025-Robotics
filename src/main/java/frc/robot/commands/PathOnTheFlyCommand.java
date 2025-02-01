package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathOnTheFlyCommand extends Command {
  
  private final CommandSwerveDrivetrain drivetrain;
  private PathConstraints pathConstraints;
  private Command pathCommand;

  /** Creates a new driveToPose. */
  public PathOnTheFlyCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.pathConstraints = new PathConstraints(1.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    this.pathCommand = AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0.0);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //  drivetrain.resetPose(Pose2d.kZero);
  }

  @Override
  public void execute() {
    pathCommand.schedule();
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return pathCommand.isFinished();
    }
  

}