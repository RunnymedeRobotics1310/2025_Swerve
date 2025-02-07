package frc.robot.subsystems.swerve;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.gyro.GyroAwareSwerveDrive;
import ca.team1310.swerve.odometry.FieldAwareSwerveDrive;
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
        //        this.drive = new GyroAwareSwerveDrive(config.coreConfig());
        this.drive = new FieldAwareSwerveDrive(config.coreConfig());
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

    /**
     * Add limiters to the change in drive values. Note this may not scale
     * evenly - one may reach desired speed before another.
     * @param x m/s
     * @param y m/s
     * @param omega rad/s
     */
    private void driveSafely(double x, double y, double omega) {
        x = xLimiter.calculate(x);
        y = yLimiter.calculate(y);
        omega = omegaLimiter.calculate(omega);

        if (this.config.enabled()) {
            this.drive.drive(x, y, omega);
        }
    }

    /**
     * The primary method for controlling the drivebase. The provided parameters
     * specify the robot-relative chassis speeds of the robot.
     * <p>
     * This method is responsible for applying safety code to prevent the robot from attempting to
     * exceed its physical limits both in terms of speed and acceleration.
     * <p>
     * @param x m/s
     * @param y m/s
     * @param omega rad/s
     */
    public final void driveRobotOriented(double x, double y, double omega) {
        this.telemetry.fieldOrientedVelocityX = 0;
        this.telemetry.fieldOrientedVelocityY = 0;
        this.telemetry.fieldOrientedVelocityOmega = 0;
        this.telemetry.fieldOrientedDeltaToPoseX = 0;
        this.telemetry.fieldOrientedDeltaToPoseY = 0;
        this.telemetry.fieldOrientedDeltaToPoseHeading = 0;

        driveSafely(x, y, omega);
    }

    /**
     * Stop all motors as fast as possible
     */
    public void stop() {
        driveRobotOriented(0, 0, 0);
    }

    /**
     * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
     * field-oriented inputs into the required robot-oriented inputs that can
     * be used by the robot.
     *
     * @param x the linear velocity of the robot in metres per second. Positive x is away from the blue alliance wall
     * @param y the linear velocity of the robot in metres per second. Positive y is to the left of the robot
     * @param omega the rotation rate of the heading of the robot in radians per second. CCW positive.
     */
    public final void driveFieldOriented(double x, double y, double omega) {
        this.telemetry.fieldOrientedDeltaToPoseX = 0;
        this.telemetry.fieldOrientedDeltaToPoseY = 0;
        this.telemetry.fieldOrientedDeltaToPoseHeading = 0;

        driveFieldOrientedInternal(x, y, omega);
    }

    /*
     * INTERNAL method for driving field-oriented. This should be called by another method that updates
     * telemetry values  fieldOrientedDeltaToPoseX, fieldOrientedDeltaToPoseY, fieldOrientedDeltaToPoseHeading.
     * @param x
     * @param y
     * @param omega
     */
    private void driveFieldOrientedInternal(double x, double y, double omega) {
        this.telemetry.fieldOrientedVelocityX = x;
        this.telemetry.fieldOrientedVelocityY = y;
        this.telemetry.fieldOrientedVelocityOmega = omega;

        Rotation2d theta = Rotation2d.fromDegrees(drive.getYaw());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, theta);
        driveSafely(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
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
        return new Pose2d(); // todo: fixme:
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
    public void resetOdometry(Pose2d pose) {
        // todo: fixme:
    }

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
        // todo: fixme:
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
     * Compute the required rotation speed of the robot given the desired heading.
     * Note the desired heading is specified in degrees, adn the returned value
     * is in radians per second.
     * @param desiredHeadingDegrees the desired heading of the robot
     * @return the required rotation speed of the robot (omega) in rad/s
     */
    public double computeOmega(double desiredHeadingDegrees) {
        return headingPIDController.calculate(drive.getYaw(), desiredHeadingDegrees);
    }
}
