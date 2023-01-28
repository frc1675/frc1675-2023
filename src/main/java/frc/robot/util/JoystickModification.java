package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class JoystickModification {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");
  private DoubleTopic topic = table.getDoubleTopic("Speed scaler");
  private DoublePublisher speedScalerPub = topic.publish();
  private DoubleSubscriber speedScalerSub = topic.subscribe(1);

  public JoystickModification() {
    speedScalerPub.set(1);
  }

  public double getSpeedScaler() {
    return speedScalerSub.get();
  }

  public double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public double modifyAxis(double value) {
    value = deadband(value, 0.05);
    value = Math.copySign(value * value, value);
    value = value * speedScalerSub.get();; // scale down value to slow robot

    return value;
  }
}
