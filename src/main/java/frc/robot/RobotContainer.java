// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.auto.Score3L4LeftAutoCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.swervedrive.TeleopDriveCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerveDriveSubsystem =
      new SwerveSubsystem(Constants.Swerve.SUBSYSTEM_CONFIG);
  private final LimelightVisionSubsystem visionSubsystem =
      new LimelightVisionSubsystem(Constants.VISION_CONFIG, swerveDriveSubsystem);
  private final OperatorInput operatorInput =
      new OperatorInput(OiConstants.DRIVER_CONTROLLER_PORT, OiConstants.OPERATOR_CONTROLLER_PORT);
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize all Subsystem default commands
    swerveDriveSubsystem.setDefaultCommand(
        new TeleopDriveCommand(swerveDriveSubsystem, operatorInput));
    // Configure the trigger bindings
    operatorInput.configureBindings(swerveDriveSubsystem, visionSubsystem);
  }

  public Command getAutonomousCommand() {
    return new Score3L4LeftAutoCommand(swerveDriveSubsystem, coralSubsystem, visionSubsystem, 0);
    //    return new DriveToLeftCenterPointAutoCommand(swerveDriveSubsystem);
    //    return new Score1CoralCenterAutoCommand(swerveDriveSubsystem, visionSubsystem, 0);
  }
}
