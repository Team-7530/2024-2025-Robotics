package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.*;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.commands.PathOnTheFlyCommand;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static RobotContainer instance;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveTrainConstants.maxSpeed * 0.1).withRotationalDeadband(DriveTrainConstants.maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final Telemetry logger = new Telemetry(DriveTrainConstants.maxSpeed);

    /* Operator Interface */
    public OperatorInterface oi = new OperatorInterface() {};

    /* Subsystems */
    public final PowerDistribution power = new PowerDistribution();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision = new VisionSubsystem();

    /* Path follower */
    private SendableChooser<Command> autoChooser;

    public static RobotContainer GetInstance() {
      return instance;
    }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    instance = this;

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();
    configureAutoPaths();
    configureAutoCommands();
    configureTelemetry();    
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // x-stance
    oi.getXStanceButton().whileTrue(drivetrain.applyRequest(() -> brake));

    oi.getRobotRelative().whileTrue(drivetrain.applyRequest(() -> 
      point.withModuleDirection(new Rotation2d(oi.getTranslateX(), oi.getTranslateY()))));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    oi.getStartButton().and(oi.getYButton()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    oi.getStartButton().and(oi.getXButton()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    oi.getBackButton().and(oi.getYButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    oi.getBackButton().and(oi.getXButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    oi.getStartButton().and(oi.getAButton()).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    oi.getStartButton().and(oi.getBButton()).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    oi.getXButton().whileTrue(new PathOnTheFlyCommand(drivetrain, new Pose2d(16.24, 0.8, Rotation2d.fromDegrees(-60))));    
    oi.getYButton().whileTrue(new PathOnTheFlyCommand(drivetrain, new Pose2d(13.85, 2.67, Rotation2d.fromDegrees(124))));    
    oi.getAButton().whileTrue(getAutonomousCommand());    
    // oi.getBButton().whileTrue(new PathOnTheFlyCommand(drivetrain, new Pose2d(1, 1, Rotation2d.fromDegrees(180))));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Add commands to Autonomous Sendable Chooser
    autoChooser = AutoBuilder.buildAutoChooser("Forward");

    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(oi.getTranslateX() * DriveTrainConstants.maxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(oi.getTranslateY() * DriveTrainConstants.maxSpeed) // Drive left with negative X (left)
              .withRotationalRate(oi.getRotate() * DriveTrainConstants.maxAngularRate) // Drive counterclockwise with negative X (left)
          )
    );
    vision.setDefaultCommand(new PhotonVisionCommand(vision, drivetrain));
  }

  private void configureAutoPaths() {
    // NamedCommands.registerCommand("Intake", new IntakeCommand(shooter));
    // NamedCommands.registerCommand(
    //     "ElevateUp", new ElevateCommand(elevator, ElevatorConstants.kTargetElevatorHigh));
    // NamedCommands.registerCommand(
    //     "ElevateDown", new ElevateCommand(elevator, ElevatorConstants.kTargetElevatorLow));
    // NamedCommands.registerCommand(
    //     "RotateUp", new RotateWristCommand(wrist, WristConstants.kTargetWristHigh));
    // NamedCommands.registerCommand(
    //     "RotateDown", new RotateWristCommand(wrist, WristConstants.kTargetWristLow));
    // NamedCommands.registerCommand("Shoot", new ShootCommand(shooter));
    // NamedCommands.registerCommand("ShootSlow", new ShootSlowCommand(shooter));
  }
    
  private void configureTelemetry() {
    drivetrain.registerTelemetry(logger::telemeterize);

    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    tab.add("AutoChooser", autoChooser).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
    tab.addNumber("DriveTrain/Drive Scaling", () -> oi.driveScalingValue());
}

  public void simulationInit() {}

  public void simulationPeriodic() {
    // vision.simulationPeriodic(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()));

    // var debugField = vision.getSimDebugField();
    // debugField.getObject("EstimatedRobot").setPose(drivetrain.getPose());
    // debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

    // // Calculate battery voltage sag due to current draw
    // var batteryVoltage =
    //         BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw());

    // // Using max(0.1, voltage) here isn't a *physically correct* solution,
    // // but it avoids problems with battery voltage measuring 0.
    // RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
  }

  public void testInit() {
  }

  public void testPeriodic() {
  }

  public void testExit() {
  }
}
