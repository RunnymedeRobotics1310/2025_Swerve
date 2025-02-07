package frc.robot.commands.swervedrive;

import static frc.robot.Constants.OiConstants.GENERAL_SPEED_FACTOR;
import static frc.robot.Constants.OiConstants.MAX_SPEED_FACTOR;
import static frc.robot.Constants.OiConstants.SLOW_SPEED_FACTOR;
import static frc.robot.Constants.Swerve.ROTATION_CONFIG;
import static frc.robot.Constants.Swerve.TRANSLATION_CONFIG;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;
import static frc.robot.commands.operator.OperatorInput.Axis.X;
import static frc.robot.commands.operator.OperatorInput.Axis.Y;
import static frc.robot.commands.operator.OperatorInput.Stick.LEFT;
import static frc.robot.commands.operator.OperatorInput.Stick.RIGHT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TeleopDriveCommand extends BaseDriveCommand {

    private final OperatorInput oi;
    private Rotation2d headingSetpoint = null;
    private boolean fieldOriented = true;

    /**
     * Used to drive a swerve robot in full field-centric mode.
     */
    public TeleopDriveCommand(SwerveSubsystem swerve, OperatorInput operatorInput) {
        super(swerve);
        this.oi = operatorInput;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    //    @Override
    public void execute() {
        super.execute();

        // The FRC field-oriented coordinate system
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        final Alliance alliance = getRunnymedeAlliance();
        SmartDashboard.putString("1310/Alliance", alliance.toString());

        // The coordinate system defines (0,0) as the right side of the blue alliance wall. The
        // x-axis is positive toward the red alliance, and the y-axis is positive to the left.
        // When the robot is on the red alliance, we need to invert inputs for the stick values
        // to move the robot in the right direction.
        final boolean invert = alliance == Alliance.Red;

        // With the driver standing behind the driver station glass, "forward" on the left stick is
        // its y value, but that should convert into positive x movement on the field. The
        // Runnymede Controller inverts stick y-axis values, so "forward" on stick is positive.
        // Thus, positive y stick axis maps to positive x translation on the field.
        final double vX = oi.getDriverControllerAxis(LEFT, Y);

        // Left and right movement on the left stick (the stick's x-axis) maps to the y-axis on the
        // field. Left on the stick (negative x) maps to positive y on the field, and vice versa.
        // Thus, negative x stick axis maps to positive y translation on the field.
        final double vY = -oi.getDriverControllerAxis(LEFT, X);

        // Left and right on the right stick will change the direction the robot is facing - its
        // heading. Positive x values on the stick translate to clockwise motion, and vice versa.
        // The coordinate system has positive motion as CCW.
        // Therefore, negative x stick value maps to positive rotation on the field.
        final double ccwRotAngularVelPct = -oi.getDriverControllerAxis(RIGHT, X);

        // Compute boost factor
        final boolean isSlow = oi.isDriverLeftBumper();
        final boolean isFast = oi.isDriverRightBumper();
        final double boostFactor = isSlow ? SLOW_SPEED_FACTOR : (isFast ? MAX_SPEED_FACTOR : GENERAL_SPEED_FACTOR);

        Translation2d velocity = calculateTeleopVelocity(vX, vY, boostFactor, invert);

        final double omegaRadiansPerSecond;

        double correctedCcwRotAngularVelPct = ccwRotAngularVelPct;

        // User is steering!
        if (correctedCcwRotAngularVelPct != 0) {
            // Compute omega
            omegaRadiansPerSecond = Math.pow(correctedCcwRotAngularVelPct, 3) * ROTATION_CONFIG.maxRotVelocityRadPS();
            // Save previous heading for when we are finished steering.
            headingSetpoint = swerve.getPose().getRotation();
        } else {
            // Translating only. Just drive on the last heading we knew.
            if (headingSetpoint == null) {
                headingSetpoint = swerve.getPose().getRotation();
            }
            // todo: after yawRate() drops to zero, we should reset headingSetpoint to the current heading
            //            omegaRadiansPerSecond = swerve.computeOmega(headingSetpoint.getDegrees());
            omegaRadiansPerSecond = 0;
        }

        if (fieldOriented) {
            // Field-oriented mode
            swerve.driveFieldOriented(velocity.getX(), velocity.getY(), omegaRadiansPerSecond);
        } else {
            // Robot-oriented mode
            swerve.driveRobotOriented(velocity.getX(), velocity.getY(), omegaRadiansPerSecond);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        headingSetpoint = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        super.isFinished();
        return false;
    }

    private static Translation2d calculateTeleopVelocity(double vX, double vY, double boostFactor, boolean invert) {
        // invert
        if (invert) {
            vX = -vX;
            vY = -vY;
        }

        // handy utilities
        Translation2d input = new Translation2d(vX, vY);
        double magnitude = input.getNorm();
        Rotation2d angle = magnitude > 1e-6 ? input.getAngle() : new Rotation2d();

        // apply boost factor
        magnitude *= boostFactor;

        // handle case where in simulator, a value of 1,1 is possible whereas normally the
        // controller magnitude never exceeds 1
        magnitude = MathUtil.clamp(magnitude, -1, 1);

        // cube to allow more fine-grained control for user at low values
        magnitude = Math.pow(magnitude, 3);

        // convert from % to mps
        magnitude *= TRANSLATION_CONFIG.maxSpeedMPS();

        // convert to vector
        return new Translation2d(magnitude, angle);
    }
}
