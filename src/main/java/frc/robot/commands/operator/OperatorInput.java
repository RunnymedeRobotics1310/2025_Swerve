package frc.robot.commands.operator;

import static frc.robot.Constants.AutoConstants.AutoPattern.*;
import static frc.robot.Constants.AutoConstants.Delay.*;
import static frc.robot.Constants.UsefulPoses.BLUE_2_2_20;
import static frc.robot.Constants.UsefulPoses.RED_2_2_20;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.auto.ExitZoneAutoCommand;
import frc.robot.commands.auto.OptimisticAutoCommand;
import frc.robot.commands.swervedrive.DriveDistanceCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final SwerveSubsystem swerve;
    private final XboxController driverController;
    private final XboxController operatorController;
    private final SendableChooser<Constants.AutoConstants.AutoPattern> autoPatternChooser = new SendableChooser<>();
    private final SendableChooser<Constants.AutoConstants.Delay> delayChooser = new SendableChooser<>();

    public enum Stick {
        LEFT,
        RIGHT
    }

    public enum Axis {
        X,
        Y
    }

    /**
     * Construct an OperatorInput class that is fed by a DriverController and an
     * OperatorController.
     *
     * @param driverControllerPort on the driver station which the driver joystick
     * is plugged into
     * @param operatorControllerPort on the driver station which the aux joystick is
     * plugged into
     */
    public OperatorInput(int driverControllerPort, int operatorControllerPort, SwerveSubsystem swerve) {
        this.swerve = swerve;
        driverController = new RunnymedeGameController(driverControllerPort);
        operatorController = new RunnymedeGameController(operatorControllerPort);
    }

    public boolean isToggleTestMode() {
        return !DriverStation.isFMSAttached() && driverController.getBackButton() && driverController.getStartButton();
    }

    public XboxController getRawDriverController() {
        return driverController;
    }

    public boolean getRotate180Val() {
        return driverController.getLeftBumperButton();
    }

    public boolean isDriverRightBumper() {
        return driverController.getRightBumperButton();
    }

    public boolean isFaceSpeaker() {
        return driverController.getYButton();
    }

    public boolean isZeroGyro() {
        return driverController.getBackButton();
    }

    public boolean isCancel() {
        return driverController.getStartButton();
    }

    public int getPOV() {
        return driverController.getPOV();
    }

    public double getDriverControllerAxis(Stick stick, Axis axis) {
        switch (stick) {
            case LEFT:
                switch (axis) {
                    case X:
                        return driverController.getLeftX();
                    case Y:
                        return driverController.getLeftY();
                }
                break;
            case RIGHT:
                switch (axis) {
                    case X:
                        return driverController.getRightX();
                }
                break;
        }

        return 0;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings(SwerveSubsystem driveSubsystem) {
        // Enter Test Mode (Start and Back pressed at the same time)
        new Trigger(() -> (isToggleTestMode())).onTrue(new SystemTestCommand(this, driveSubsystem));

        new Trigger(() -> (isZeroGyro())).onTrue(new ZeroGyroCommand(driveSubsystem));
        new Trigger(this::isCancel).whileTrue(new CancelCommand(driveSubsystem));
        new Trigger(driverController::getXButton).whileTrue(
            new ResetOdometryCommand(driveSubsystem, new Pose2d(1.83, 0.40, Rotation2d.fromDegrees(0)))
        );
        // drive to position test
        //        Translation2d location = new Translation2d(2, 2);
        //        Rotation2d heading = Rotation2d.fromDegrees(-20);
        //        Pose2d desiredPose = new Pose2d(location, heading);
        //        DriveToPositionCommand dtpc = new DriveToPositionCommand(driveSubsystem, BLUE_2_2_20, RED_2_2_20);
        //        new Trigger(driverController::getBButton).onTrue(dtpc);
    }

    public Command getAutonomousCommand() {
        double delay =
            switch (delayChooser.getSelected()) {
                case WAIT_0_5_SECOND -> 0.5;
                case WAIT_1_SECOND -> 1;
                case WAIT_1_5_SECONDS -> 1.5;
                case WAIT_2_SECONDS -> 2;
                case WAIT_2_5_SECONDS -> 2.5;
                case WAIT_3_SECONDS -> 3;
                case WAIT_5_SECONDS -> 5;
                default -> 0;
            };

        return switch (autoPatternChooser.getSelected()) {
            // not used
            case OPTIMISTIC_AUTO -> new OptimisticAutoCommand(swerve, delay);
            // used in competition
            case EXIT_ZONE -> new ExitZoneAutoCommand(swerve, delay);
            default -> new InstantCommand();
        };
    }
}
