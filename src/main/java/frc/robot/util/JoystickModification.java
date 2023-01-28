package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class JoystickModification {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");
  private static DoubleTopic topic = table.getDoubleTopic("Speed scaler");
  private static DoubleSubscriber speedScaler = topic.subscribe(1);

  public static double getSpeedScaler() {
    return speedScaler.get();
  }

  public static double deadband(double value, double deadband) {
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

  public static double modifyAxis(double value) {
    value = deadband(value, 0.05);
    value = Math.copySign(value * value, value);
    double scaler = speedScaler.get();
    value = value * scaler; // scale down value to slow robot

    return value;
  }
}
