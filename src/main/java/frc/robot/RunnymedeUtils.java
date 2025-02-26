package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Objects;

public class RunnymedeUtils {

  private static DriverStation.Alliance alliance = null;

  public static DriverStation.Alliance getRunnymedeAlliance() {
    if (alliance == null) {
      DriverStation.getAlliance().ifPresent(value -> alliance = value);
    }

    return Objects.requireNonNullElse(alliance, DriverStation.Alliance.Red);
  }
}
