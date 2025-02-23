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

    public DriveToFieldLocationCommand(SwerveSubsystem swerve, FieldLocation redLocation, FieldLocation blueLocation) {
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

        double xDif = currentPose.getX() - location.pose.getX();
        double yDif = currentPose.getY() - location.pose.getY();
        double targetAngle = location.pose.getRotation().getDegrees();

        log("Xdif: " + xDif + " Ydif: " + yDif);

        swerve.driveFieldOriented(
            swerve.computeTranslateVelocity(xDif),
            swerve.computeTranslateVelocity(yDif),
            //            swerve.computeOmega(targetAngle)
            0
        );
        //        swerve.driveFieldOriented(Math.signum(xDif) * 0.5, Math.signum(yDif) * 0.5, 0);
    }

    @Override
    public boolean isFinished() {
        return (
            SwerveUtils.isCloseEnough(swerve.getPose().getTranslation(), location.pose.getTranslation(), 0.05) &&
            SwerveUtils.isCloseEnough(swerve.getYaw(), location.pose.getRotation().getDegrees(), 1)
        );
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        swerve.stop();
    }
}
