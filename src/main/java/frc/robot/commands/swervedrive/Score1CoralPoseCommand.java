package frc.robot.commands.swervedrive;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score1CoralPoseCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem visionSubsystem;
  private final Constants.AutoConstants.FieldLocation location;
  private final double targetHeadingDeg;
  private boolean doneDriving = false;
  private double vX = 0;
  private double vY = 0;

  public Score1CoralPoseCommand(
      SwerveSubsystem swerve,
      LimelightVisionSubsystem visionSubsystem,
      Constants.AutoConstants.FieldLocation location) {
    this.swerve = swerve;
    this.visionSubsystem = visionSubsystem;
    this.location = location;
    this.targetHeadingDeg = SwerveUtils.normalizeDegrees(location.pose.getRotation().getDegrees());

    addRequirements(swerve);
  }

  @Override
  public void initialize() {

    logCommandStart();
    visionSubsystem.setDoVisionUpdates(false);

    log("Pose: " + swerve.getPose());
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    double xDif = location.pose.getX() - currentPose.getX();
    double yDif = location.pose.getY() - currentPose.getY();

    System.out.println("Pose:" + currentPose + " xDif: " + xDif + " yDif: " + yDif);

    double angleDif =
        SwerveUtils.normalizeDegrees(targetHeadingDeg - currentPose.getRotation().getDegrees());
    //        log("Xdif: " + xDif + " Ydif: " + yDif + " ºdif: " + angleDif);

    //    swerve.driveFieldOriented(
    //            swerve.computeTranslateVelocity(xDif, 0.02, 0.5),
    //            swerve.computeTranslateVelocity(yDif, 0.02, 0.5),
    //            swerve.computeOmega(targetHeadingDeg));
    double xSpeed = calcSwerveSpeedX(xDif, 0.02, 0.5);
    double ySpeed = calcSwerveSpeedY(yDif, 0.02, 0.5);

    System.out.println("xSpeed: " + xSpeed + " ySpeed: " + ySpeed);

    swerve.driveFieldOriented(xSpeed, ySpeed, swerve.computeOmega(targetHeadingDeg));
    //    System.out.println("xSpeed: " + swerve.computeTranslateVelocity(xDif, 0.02, 0.5));
    //    System.out.println("ySpeed: " + swerve.computeTranslateVelocity(yDif, 0.02, 0.5));

  }

  private double calcSwerveSpeedX(double xDif, double tolerance, double maxSpeed) {
    double xSpeed = Math.max(0.2, Math.min(maxSpeed, Math.abs(xDif)));
    if (Math.abs(xDif) < tolerance) {
      xSpeed = 0;
    }
    return xSpeed * Math.signum(xDif);
  }

  private double calcSwerveSpeedY(double yDif, double tolerance, double maxSpeed) {
    double ySpeed = Math.max(0.2, Math.min(maxSpeed, Math.abs(yDif)));
    if (Math.abs(yDif) < tolerance) {
      ySpeed = 0;
    }
    return ySpeed * Math.signum(yDif);
  }

  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    visionSubsystem.setDoVisionUpdates(true);
    swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return doneDriving;
  }
}
