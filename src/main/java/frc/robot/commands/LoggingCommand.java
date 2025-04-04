package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

/** The Logging Command Base implements command helpers to aid with logging and command timeout */
public class LoggingCommand extends Command {

  SimpleDateFormat START_TIMESTAMP_FMT = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");

  protected double initializeTime = 0;
  private String finishReason = null;

  List<Subsystem> subsystemList = new ArrayList<>();

  /** Default implementation automatically logs start of command with required subsystems */
  @Override
  public void initialize() {
    logCommandStart();
  }

  /**
   * Default implementation logs command end
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
  }

  /**
   * Check if the command has been running longer than the passed in timeout.
   *
   * <p>NOTE: The timeout is checked from the time that the logCommandStart method was called, not
   * from the time that the command was constructed. This method will only work if one of the
   * logCommandStart methods is called in the initialize() method of the command.
   *
   * @param timeout to check, in seconds
   * @return {@code true} if the timeout has been exceeded, {@code false} otherwise
   */
  public boolean hasElapsed(double timeout) {
    return Timer.getFPGATimestamp() - initializeTime > timeout;
  }

  /**
   * At command start, log the start time and the state of all subsystems required by this command.
   */
  public void logCommandStart() {
    logCommandStart(null);
  }

  /**
   * At command start, log the start time, the passed in string of command parms, and the state of
   * all subsystems required by this command.
   *
   * @param commandParms
   */
  public void logCommandStart(String commandParms) {
    this.subsystemList.clear();
    finishReason = null;
    initializeTime = 0;

    this.subsystemList.addAll(getRequirements());

    logCommandState("STARTING", commandParms, true);

    // Set the initialize time after logging of the start message.
    initializeTime = Timer.getFPGATimestamp();
  }

  /**
   * Set the text associated for the reason this command has finished.
   *
   * <p>This method should be called in the isFinished routine before returning {@code true}
   *
   * @param finishReason indicating the condition under which the commmand is determined to be
   *     finished
   */
  public void setFinishReason(String finishReason) {
    this.finishReason = finishReason;
  }

  /**
   * Log the command end state and the status of the interrupted flag.
   *
   * <p>This routine will also log the finish reason if a reason was set using the {@link
   * #setFinishReason(String)} method
   *
   * @param interrupted {@code true} if the command was interrupted, {@code false} otherwise.
   */
  public void logCommandEnd(boolean interrupted) {
    logCommandEnd(interrupted, null);
  }

  /**
   * Log the command end state and the status of the interrupted flag, and an optional end message
   *
   * <p>This routine will also log the finish reason if a reason was set using the {@link
   * #setFinishReason(String)} method
   *
   * @param interrupted {@code true} if the command was interrupted, {@code false} otherwise.
   */
  public void logCommandEnd(boolean interrupted, String endMsg) {
    String state = "ENDED";

    if (interrupted) {
      state = "INTERRUPTED";
    }

    logCommandState(state, endMsg, true);
  }

  /**
   * Log a transition to a new state, and the reason for the state transition
   *
   * @param newState text value
   * @param transitionReason text string
   */
  public void logStateTransition(String newState, String transitionReason) {
    logStateTransition(newState, transitionReason, false);
  }

  /**
   * /** Log a transition to a new state, and the reason for the state transition
   *
   * @param newState text value
   * @param transitionReason text string
   * @param logSubsystems {@code true} to log the state of all subsystems required by this command,
   *     {@code false} otherwise.
   */
  public void logStateTransition(String newState, String transitionReason, boolean logSubsystems) {
    logCommandState(newState, transitionReason, logSubsystems);
  }

  /**
   * Log a message.
   *
   * <p>This method can be used as debug to log a message to the console in the case of more complex
   * commands.
   *
   * @param msg
   */
  public void log(String msg) {
    logCommandState(null, msg, false);
  }

  /**
   * Log a message.
   *
   * <p>This method can be used as debug to log a message to the console in the case of more complex
   * commands.
   *
   * @param msg
   * @param logSubsystems {@code true} to log the state of all required subsystems
   */
  public void log(String msg, boolean logSubsystems) {
    logCommandState(null, msg, logSubsystems);
  }

  private void logCommandState(String state, String msg, boolean logSubsystems) {
    StringBuilder sb = new StringBuilder();

    sb.append(this.getClass().getSimpleName());

    if (state != null) {
      sb.append(" : ").append(state);
    }

    if (initializeTime == 0) {
      sb.append(" at ").append(START_TIMESTAMP_FMT.format(new Date()));
    } else {
      sb.append(" at ").append(Timer.getFPGATimestamp() - initializeTime).append("s");
    }

    if (finishReason != null) {
      sb.append(" : ").append(finishReason);
    }

    if (msg != null) {
      sb.append(" : ").append(msg);
    }

    if (logSubsystems) {
      // Print the subsystems as passed in on the command start
      for (Subsystem subsystem : subsystemList) {
        if (subsystem != null) {
          sb.append("\n   ").append(subsystem.toString());
        }
      }
    }

    System.out.println(sb.toString());
  }

  public static String format(double d) {
    return String.format("%.2f", d);
  }

  public static String format(Pose2d pose) {
    return String.format("%.2f", pose.getX())
        + ","
        + String.format("%.2f", pose.getY())
        + " @ "
        + format(pose.getRotation());
  }

  public static String format(Rotation2d rotation) {
    return String.format("%.1f", rotation.getDegrees()) + " deg";
  }

  public static String format(Translation2d vector) {
    return (String.format("%.2f", vector.getNorm())
        + " ("
        + String.format("%.2f", vector.getX())
        + ","
        + String.format("%.2f", vector.getY())
        + ")");
  }

  public static String format(Transform2d transform) {
    return format(transform.getTranslation()) + " @ " + format(transform.getRotation());
  }
}
