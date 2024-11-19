package frc.robot;

//import static frc.robot.Constants.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Operator Interface */
    public OperatorInterface oi = new OperatorInterface() {};

    /* Subsystems */
    public final PowerDistribution power = new PowerDistribution();
    // public final VisionSubsystem vision = new VisionSubsystem();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* Path follower */
    private SendableChooser<Command> autoChooser;

  /* Test System */
  //  private TestChecklist m_test;

  /* Cameras */
  // public UsbCamera cam0;

  // public static Map<String, Trajectory> trajectoryList = new HashMap<String, Trajectory>();
  // public static Map<String, List<PathPlannerTrajectory>> pptrajectoryList =
  //     new HashMap<String, List<PathPlannerTrajectory>>();
  // public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

  private static RobotContainer instance;

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

    // cam0 = CameraServer.startAutomaticCapture(0);
    // cam0.setConnectVerbose(0);

    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    tab.add(autoChooser).withSize(2, 1);
    // tab.addNumber("DriveTrain/Drive Scaling", () -> oi.getDriveScaling());
    // tab.addNumber("DriveTrain/Rotate Scaling", () -> oi.getRotateScaling());

    // m_test = new TestChecklist(driveTrain, arm);
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

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(oi.getTranslateX()) // Drive forward with negative Y (forward)
              .withVelocityY(oi.getTranslateY()) // Drive left with negative X (left)
              .withRotationalRate(oi.getRotate()) // Drive counterclockwise with negative X (left)
          )
    );
  // vision.setDefaultCommand(new VisionCommand(vision, driveTrain));
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

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    oi.getBackButton().and(oi.getBButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    oi.getBackButton().and(oi.getAButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
    autoChooser = AutoBuilder.buildAutoChooser();

    // SmartDashboard Buttons
    SmartDashboard.putData("Auto chooser", autoChooser);
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

  public void simulationInit() {}

  public void simulationPeriodic() {}

  public void testInit() {
    // m_test.initialize();
  }

  public void testPeriodic() {
    // m_test.periodic();
  }

  public void testExit() {
    // m_test.exit();
  }
}
