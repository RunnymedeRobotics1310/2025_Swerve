package frc.robot.commands.coral;

import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class MoveToCoralPoseCommand extends LoggingCommand {

  public MoveToCoralPoseCommand(
      Constants.CoralConstants.CoralPose pose, CoralSubsystem coralSubsystem) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
