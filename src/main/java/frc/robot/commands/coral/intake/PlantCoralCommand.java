package frc.robot.commands.coral.intake;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class PlantCoralCommand extends LoggingCommand {

  public PlantCoralCommand(CoralSubsystem coralSubsystem) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
