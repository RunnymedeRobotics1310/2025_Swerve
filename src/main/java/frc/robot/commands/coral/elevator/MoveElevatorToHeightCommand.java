package frc.robot.commands.coral.elevator;

import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class MoveElevatorToHeightCommand extends LoggingCommand {

  public MoveElevatorToHeightCommand(
      Constants.CoralConstants.ElevatorHeight elevatorHeight, CoralSubsystem coralSubsystem) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
