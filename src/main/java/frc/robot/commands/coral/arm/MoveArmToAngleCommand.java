package frc.robot.commands.coral.arm;

import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class MoveArmToAngleCommand extends LoggingCommand {

  public MoveArmToAngleCommand(
      Constants.CoralConstants.ArmAngle armAngle, CoralSubsystem coralSubsystem) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
