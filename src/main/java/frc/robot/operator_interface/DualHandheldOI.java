package frc.robot.operator_interface;

// import static frc.robot.Constants.*;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a dual Xbox controllers. */
public class DualHandheldOI extends SingleHandheldOI {
  private final XboxController operator;

  public DualHandheldOI(int port0, int port1) {
    super(port0);
    operator = new XboxController(port1);
  }

  @Override
  public boolean testResults(int mode) {
    boolean result = true;
    if (mode == DRIVER) {
      result = super.testResults(mode);
    } else if (mode == OPERATOR) {
      testController(operator, tests[mode]);

      for (boolean test : tests[mode]) {
        result = result && test;
      }
    }
    return result;
  }

  @Override
  public double getLeftThumbstickX() {
    return operator.getLeftX();
  }
  @Override
  public double getLeftThumbstickY() {
    return operator.getLeftY();
  }
  @Override
  public double getRightThumbstickX() {
    return operator.getRightX();
  }
  @Override
  public double getRightThumbstickY() {
    return operator.getRightY();
  }

  @Override
  public Trigger getPOVUp() {
    int pov = operator.getPOV();
    return new Trigger(() -> (pov == 0) || (pov == 45) || (pov == 315));
  }
  @Override
  public Trigger getPOVDown() {
    int pov = operator.getPOV();
    return new Trigger(() -> (pov == 180) || (pov == 225) || (pov == 135));
  }
  @Override
  public Trigger getPOVLeft() {
    int pov = operator.getPOV();
    return new Trigger(() -> (pov == 270) || (pov == 315) || (pov == 225));
  }
  @Override
  public Trigger getPOVRight() {
    int pov = operator.getPOV();
    return new Trigger(() -> (pov == 90) || (pov == 135) || (pov == 45));
  }

}
