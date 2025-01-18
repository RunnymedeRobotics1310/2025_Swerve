package frc.robot.commands.auto;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class OptimisticAutoCommand extends SequentialCommandGroup {

    private final Pose2d blueOuterRightStation = new Pose2d(100, 70, Rotation2d.fromDegrees(234));
    private final Pose2d redOuterRightStation = new Pose2d(
        FIELD_EXTENT_METRES_X - blueOuterRightStation.getX(),
        FIELD_EXTENT_METRES_Y - blueOuterRightStation.getY(),
        Rotation2d.fromDegrees(blueOuterRightStation.getRotation().getDegrees() - 180)
    );

    private final Translation2d blueCloseTransit = new Translation2d(280, 170);
    private final Translation2d redCloseTransit = new Translation2d(
        FIELD_EXTENT_METRES_X - blueCloseTransit.getX(),
        FIELD_EXTENT_METRES_Y - blueCloseTransit.getY()
    );

    public OptimisticAutoCommand(SwerveSubsystem swerve, double delay) {
        super();
        addCommands(new WaitCommand(delay));

        addCommands(
            new DriveToPositionCommand(
                swerve,
                new Pose2d(510, 270, Rotation2d.fromDegrees(120)),
                new Pose2d(FIELD_EXTENT_METRES_X - 510, FIELD_EXTENT_METRES_Y - 270, Rotation2d.fromDegrees(300))
            )
        );
        // Score L4 - 1
        addCommands(
            new DriveToPositionCommand(
                swerve,
                new Translation2d(440, 270),
                new Translation2d(FIELD_EXTENT_METRES_X - 440, FIELD_EXTENT_METRES_Y - 90)
            )
        );
        addCommands(new DriveToPositionCommand(swerve, blueOuterRightStation, redOuterRightStation));
        // Intake Coral
        addCommands(new DriveToPositionCommand(swerve, blueCloseTransit, redCloseTransit));
        addCommands(
            new DriveToPositionCommand(
                swerve,
                new Pose2d(320, 380, new Rotation2d()),
                new Pose2d(FIELD_EXTENT_METRES_X - 320, FIELD_EXTENT_METRES_Y - 380, Rotation2d.fromDegrees(180))
            )
        );
        // Score L4 - 2
        addCommands(new DriveToPositionCommand(swerve, blueCloseTransit, redCloseTransit));
        addCommands(new DriveToPositionCommand(swerve, blueOuterRightStation, redOuterRightStation));
        // Intake Coral
        addCommands(
            new DriveToPositionCommand(
                swerve,
                new Pose2d(400, 310, Rotation2d.fromDegrees(60)),
                new Pose2d(FIELD_EXTENT_METRES_X - 400, FIELD_EXTENT_METRES_Y - 310, Rotation2d.fromDegrees(240))
            )
        );
        // Score L4 - 3
        addCommands(new DriveToPositionCommand(swerve, blueOuterRightStation, redOuterRightStation));
        // Intake Coral
        addCommands(
            new DriveToPositionCommand(
                swerve,
                new Pose2d(420, 280, Rotation2d.fromDegrees(60)),
                new Pose2d(FIELD_EXTENT_METRES_X - 420, FIELD_EXTENT_METRES_Y - 280, Rotation2d.fromDegrees(240))
            )
        );
        // Score L4 - 4

        // ------------------ IF TIME ------------------ //

        addCommands(new DriveToPositionCommand(swerve, blueOuterRightStation, redOuterRightStation));
        // Intake Coral
        addCommands(new DriveToPositionCommand(swerve, blueCloseTransit, redCloseTransit));
        addCommands(
            new DriveToPositionCommand(
                swerve,
                new Pose2d(320, 410, new Rotation2d()),
                new Pose2d(FIELD_EXTENT_METRES_X - 320, FIELD_EXTENT_METRES_Y - 410, Rotation2d.fromDegrees(180))
            )
        );
        // Score L4 - 5?
        addCommands(new DriveToPositionCommand(swerve, blueCloseTransit, redCloseTransit));
        addCommands(new DriveToPositionCommand(swerve, blueOuterRightStation, redOuterRightStation));
        // Intake Coral

    }
}
