package frc.robot.subsystems.swerve;

import static ca.team1310.swerve.utils.SwerveUtils.normalizeRotation;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.gyro.GyroAwareSwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

    private final RunnymedeSwerveDrive drive;
    private final SwerveDriveSubsystemConfig config;
    private final SwerveTelemetry telemetry;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter omegaLimiter;
    private final PIDController headingPIDController;

    public SwerveSubsystem(SwerveDriveSubsystemConfig config) {
        this.drive = new GyroAwareSwerveDrive(config.coreConfig());
        //        this.drive = new FieldAwareSwerveDrive(config.coreConfig());
        //        this.drive                  = new VisionAwareSwerveDrive(config.coreConfig(), config.visionConfig());
        this.config = config;
        this.telemetry = config.coreConfig().telemetry();
        this.xLimiter = new SlewRateLimiter(this.config.translationConfig().maxAccelMPS2());
        this.yLimiter = new SlewRateLimiter(this.config.translationConfig().maxAccelMPS2());
        this.omegaLimiter = new SlewRateLimiter(config.rotationConfig().maxAccelerationRadPS2());
        headingPIDController = new PIDController(
            config.rotationConfig().headingP(),
            config.rotationConfig().headingI(),
            config.rotationConfig().headingD()
        );
    }

    public void periodic() {
        drive.updateTelemetry(telemetry);
    }

    /*
     * *********************************************************************************************
     * Core methods for controlling the drivebase
     */

    private void driveSafely(ChassisSpeeds robotOrientedVelocity) {
        double x = robotOrientedVelocity.vxMetersPerSecond;
        double y = robotOrientedVelocity.vyMetersPerSecond;
        double w = robotOrientedVelocity.omegaRadiansPerSecond;

        // Limit change in values. Note this may not scale
        // evenly - one may reach desired speed before another.

        // Use driveFieldOriented to avoid this.

        x = xLimiter.calculate(x);
        y = yLimiter.calculate(y);
        w = omegaLimiter.calculate(w);

        ChassisSpeeds safeVelocity = new ChassisSpeeds(x, y, w);

        if (this.config.enabled()) {
            this.drive.drive(x, y, w);
        }
    }

    /**
     * The primary method for controlling the drivebase. The provided {@link ChassisSpeeds}
     * specifies the robot-relative chassis speeds of the robot.
     * <p>
     * This method is responsible for applying safety code to prevent the robot from attempting to
     * exceed its physical limits both in terms of speed and acceleration.
     *
     * @param velocity The intended velocity of the robot chassis relative to itself.
     * @see ChassisSpeeds for how to construct a ChassisSpeeds object including
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     */
    public final void driveRobotOriented(ChassisSpeeds velocity) {
        this.telemetry.fieldOrientedVelocityX = 0;
        this.telemetry.fieldOrientedVelocityY = 0;
        this.telemetry.fieldOrientedVelocityOmega = 0;
        this.telemetry.fieldOrientedDeltaToPoseX = 0;
        this.telemetry.fieldOrientedDeltaToPoseY = 0;
        this.telemetry.fieldOrientedDeltaToPoseHeading = 0;

        driveSafely(velocity);
    }

    /**
     * Stop all motors as fast as possible
     */
    public void stop() {
        driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
     * field-oriented inputs into the required robot-oriented {@link ChassisSpeeds} object that can
     * be used by the robot.
     *
     * @param velocity the linear velocity of the robot in metres per second. Positive x is away
     * from the alliance wall, and positive y is toward the left wall when looking through the
     * driver station glass.
     * @param omega the rotation rate of the heading of the robot. CCW positive.
     */
    public final void driveFieldOriented(Translation2d velocity, Rotation2d omega) {
        this.telemetry.fieldOrientedDeltaToPoseX = 0;
        this.telemetry.fieldOrientedDeltaToPoseY = 0;
        this.telemetry.fieldOrientedDeltaToPoseHeading = 0;

        driveFieldOrientedInternal(velocity, omega);
    }

    private void driveFieldOrientedInternal(Translation2d velocity, Rotation2d omega) {
        this.telemetry.fieldOrientedVelocityX = velocity.getX();
        this.telemetry.fieldOrientedVelocityY = velocity.getY();
        this.telemetry.fieldOrientedVelocityOmega = omega.getRadians();

        double x = velocity.getX();
        double y = velocity.getY();
        double w = omega.getRadians();
        Rotation2d theta = Rotation2d.fromDegrees(drive.getYaw());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, w, theta);
        driveSafely(chassisSpeeds);
    }

    /**
     * Lock the swerve drive to prevent it from moving. This can only be called when the robot is
     * nearly stationary.
     *
     * @return true if successfully locked, false otherwise
     */
    public boolean lock() {
        return drive.lock();
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return new Pose2d();
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        drive.zeroGyro();
    }

    /**
     * Change the robot's internal understanding of its position and rotation. This
     * is not an incremental change or suggestion, it discontinuously re-sets the
     * pose to the specified pose.
     *
     * @param pose the new location and heading of the robot.
     */
    public void resetOdometry(Pose2d pose) {}

    /**
     * Set the desired module state for the named module. This should ONLY be used when testing
     * the serve drivebase in a controlled environment.
     *
     * This SHOULD NOT be called during normal operation - it is designed for TEST MODE ONLY!
     *
     * @param moduleName the module to activate
     * @param desiredState the state of the specified module.
     */
    public void setModuleState(String moduleName, SwerveModuleState desiredState) {
        //        drive.setModuleState(moduleName, desiredState);
    }

    @Override
    public String toString() {
        Pose2d pose = getPose();
        double x = pose.getX();
        double y = pose.getY();
        double theta = pose.getRotation().getDegrees();
        return String.format("SwerveDriveSubsystem Pose: %.2f,%.2f @ %.1f deg", x, y, theta);
    }

    /*
     * *********************************************************************************************
     * Convenience methods for subsystem users
     */

    /**
     * Utility function to compute the required rotation speed of the robot given the heading
     * provided.
     *
     * @param desiredHeading the desired heading of the robot
     * @return The required rotation speed of the robot
     */
    public Rotation2d computeOmega(Rotation2d desiredHeading) {
        // todo: replace with PID

        double targetRad = normalizeRotation(desiredHeading.getRadians());
        double currentRad = normalizeRotation(Math.toRadians(drive.getYaw()));
        SmartDashboard.putNumber("computeOmega/1-targetRad", targetRad);
        SmartDashboard.putNumber("computeOmega/2-currentRad", currentRad);

        double errorRad = targetRad - currentRad;
        errorRad = normalizeRotation(errorRad);
        double absErrRad = Math.abs(errorRad);
        double errSignum = Math.signum(errorRad);

        final double omegaRadPerSec;
        if (absErrRad < config.rotationConfig().toleranceRadians()) {
            omegaRadPerSec = 0;
        } else if (absErrRad < config.rotationConfig().slowZoneRadians()) {
            omegaRadPerSec = errSignum * config.rotationConfig().minRotVelocityRadPS();
        } else {
            omegaRadPerSec = errSignum * config.rotationConfig().maxJumpSpeedRadPS();
        }
        SmartDashboard.putNumber("computeOmega/3-errorRad", errorRad);
        SmartDashboard.putNumber("computeOmega/4-omegaRadPerSec", omegaRadPerSec);

        return Rotation2d.fromRadians(omegaRadPerSec);
    }
}
