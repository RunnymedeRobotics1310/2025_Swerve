package frc.robot.commands.swervedrive;

import static frc.robot.Constants.AutoConstants.*;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToFieldLocationCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final FieldLocation location;

  public DriveToFieldLocationCommand(SwerveSubsystem swerve, FieldLocation location) {
    this.swerve = swerve;
    this.location = location;
  }

  public DriveToFieldLocationCommand(
      SwerveSubsystem swerve, FieldLocation redLocation, FieldLocation blueLocation) {
    this.swerve = swerve;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
      this.location = blueLocation;
    } else {
      this.location = redLocation;
    }
  }

  @Override
  public void initialize() {
    logCommandStart();
    log("Pose: " + swerve.getPose());
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    double xDif = location.pose.getX() - currentPose.getX();
    double yDif = location.pose.getY() - currentPose.getY();
    double targetAngle = location.pose.getRotation().getDegrees();
    double angleDif = SwerveUtils.normalizeDegrees(location.pose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());

    log("Xdif: " + xDif + " Ydif: " + yDif + " ºdif: " + angleDif);

    swerve.driveFieldOriented(
        swerve.computeTranslateVelocity(xDif),
        swerve.computeTranslateVelocity(yDif),
        swerve.computeOmega(targetAngle));
    //        swerve.driveFieldOriented(Math.signum(xDif) * 0.5, Math.signum(yDif) * 0.5, 0);
  }

  @Override
  public boolean isFinished() {
    double angleDif = SwerveUtils.normalizeDegrees(location.pose.getRotation().getDegrees() - swerve.getPose().getRotation().getDegrees());
    return (SwerveUtils.isCloseEnough(
            swerve.getPose().getTranslation(), location.pose.getTranslation(), 0.05)
        && SwerveUtils.isCloseEnough(swerve.getYaw(), SwerveUtils.normalizeDegrees(location.pose.getRotation().getDegrees()), 10));
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
  }
}
