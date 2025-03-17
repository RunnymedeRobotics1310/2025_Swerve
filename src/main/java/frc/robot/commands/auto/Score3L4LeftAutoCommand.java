package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.FieldLocation.*;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.SetAllianceGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score3L4LeftAutoCommand extends BaseAutoCommand {

  public Score3L4LeftAutoCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision, double delay) {
    super(swerve, vision);

    addCommands(new SetAllianceGyroCommand(swerve, 180));
    addCommands(new WaitCommand(delay));

    goScoreL4Coral(PRE_SCORE_LEFT_4);

    autoIntake(blueLeftOuterStation);

    goScoreL4Coral(PRE_SCORE_LEFT_2);

    autoIntake(blueLeftOuterStation);

    goScoreL4Coral(PRE_SCORE_LEFT_3);

    autoIntake(blueLeftOuterStation);
  }
}
