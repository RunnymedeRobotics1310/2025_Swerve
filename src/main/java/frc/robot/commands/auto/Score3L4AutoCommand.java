package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.DriveThroughFieldLocationCommand;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.SetPoseCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.AutoConstants.FieldLocation.*;

public class Score3L4AutoCommand extends SequentialCommandGroup {

    double speed = 1;

    public Score3L4AutoCommand(SwerveSubsystem swerve, double delay) {
        super();

        addCommands(new SetPoseCommand(swerve, new Pose2d(7.6, 6.00, Rotation2d.fromDegrees(180))));
//        addCommands(new ZeroGyroCommand(swerve));

        addCommands(new WaitCommand(delay));

        addCommands(new DriveToFieldLocationCommand(swerve, blueJ));
        // Score l4 at left4
//        addCommands(new DriveThroughFieldLocationCommand(swerve, blueLeftExitTransit, speed));
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
        // Intake coral
//        addCommands(new DriveThroughFieldLocationCommand(swerve, blueLeftPickupTransit, speed));
        addCommands(new DriveToFieldLocationCommand(swerve, blueA));
        // Score left1
        addCommands(new DriveThroughFieldLocationCommand(swerve, blueLeftPickupTransit, speed));
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
        // Intake coral
        addCommands(new DriveToFieldLocationCommand(swerve, blueL));
        // Score coral
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
        // Intake coral

    }
}
