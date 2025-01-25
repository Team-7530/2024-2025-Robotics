package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem; 

public class PhotonVisionCommand extends Command {
  private final VisionSubsystem vision;
  private final CommandSwerveDrivetrain drivetrain;

  public PhotonVisionCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
    this.vision = vision;
    this.drivetrain = drivetrain;

    addRequirements(vision, drivetrain);
  }

  @Override
  public void execute() {
    // Correct pose estimate with vision measurements
    var visionEst = vision.getEstimatedGlobalPose();
    
    visionEst.ifPresent(est -> {
      // Change our trust in the measurement based on the tags we can see

      var estStdDevs = vision.getEstimationStdDevs();

      drivetrain.addVisionMeasurement(
          
          est.estimatedPose.toPose2d(),
            Utils.fpgaToCurrentTime(est.timestampSeconds),
            estStdDevs);   
          });
  }

}