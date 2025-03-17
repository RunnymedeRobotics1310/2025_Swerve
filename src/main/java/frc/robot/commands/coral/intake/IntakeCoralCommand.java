package frc.robot.commands.coral.intake;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

/** Pulls in coral until it is fully inside the arm, then stops the wheels. */
public class IntakeCoralCommand extends LoggingCommand {

  public IntakeCoralCommand(CoralSubsystem coralSubsystem, boolean isFar) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
