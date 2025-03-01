package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.SetPoseCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import static frc.robot.Constants.AutoConstants.FieldLocation.*;

public class Score3L4AutoCommand extends SequentialCommandGroup {

    public Score3L4AutoCommand(SwerveSubsystem swerve, double delay) {
        super();

        addCommands(new SetPoseCommand(swerve, new Pose2d(7.6, 6.00, Rotation2d.fromDegrees(180))));
//        addCommands(new ZeroGyroCommand(swerve));

        addCommands(new WaitCommand(delay));

        addCommands(new DriveToFieldLocationCommand(swerve, blueJ));
        // Score l4 at left4
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftExitTransit));
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
        // Intake coral
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftPickupTransit));
        addCommands(new DriveToFieldLocationCommand(swerve, blueA));
        // Score left1
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftPickupTransit));
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
        // Intake coral
        addCommands(new DriveToFieldLocationCommand(swerve, blueL));
        // Score coral
        addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
        // Intake coral

    }
}
