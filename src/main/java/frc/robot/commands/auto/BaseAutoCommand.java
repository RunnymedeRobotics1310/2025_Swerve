package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.DriveThroughFieldLocationCommand;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.DriveToVisibleTagCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class BaseAutoCommand extends SequentialCommandGroup {

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;
  private double allianceOffset = 0;

  public BaseAutoCommand(SwerveSubsystem swerve, LimelightVisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      allianceOffset = 180;
    }
  }

  protected Command driveToLocation(FieldLocation location) {
    return new DriveToFieldLocationCommand(swerve, location);
  }

  protected Command driveThroughLocation(FieldLocation location, double speed) {
    return new DriveThroughFieldLocationCommand(swerve, location, speed);
  }

  protected Command approachReef(boolean isLeftBranch) {
    return new DriveToVisibleTagCommand(swerve, vision, isLeftBranch);
  }

  public Command goScoreL4Coral(FieldLocation location) {
    double locationHeading = location.pose.getRotation().getDegrees() + allianceOffset;

    return driveThroughLocation(location, 2)
        .andThen(approachReef(location.isLeftSide))
        .andThen(new DriveRobotOrientedCommand(swerve, -0.2, 0, locationHeading).withTimeout(1));
  }

  public Command autoIntake(FieldLocation location) {
    double locationHeading = location.pose.getRotation().getDegrees() + allianceOffset;

    return driveThroughLocation(location, 2)
        .andThen(
            new WaitCommand(5)
                .deadlineFor(new DriveRobotOrientedCommand(swerve, 0.25, 0, locationHeading)));
  }
}
