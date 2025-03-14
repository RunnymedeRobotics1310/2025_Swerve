package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.DriveToVisibleTagCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score1CoralCenterAutoCommand extends SequentialCommandGroup {

  public Score1CoralCenterAutoCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision, double delay) {

    addCommands(new WaitCommand(delay));

    addCommands(
        new DriveToFieldLocationCommand(
            swerve, Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_6));
    addCommands(new DriveToVisibleTagCommand(swerve, vision, true));

    //        addCommands(new PlantCoralCommand(coral));
    //        addCommands(new SetCoralPoseCommand(coral, compact));

  }
}
