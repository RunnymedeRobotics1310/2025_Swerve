package frc.robot.subsystems.vision;

import ca.team1310.swerve.vision.PoseEstimate;
import ca.team1310.swerve.vision.VisionPoseCallback;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class LimelightVisionSubsystem extends SubsystemBase implements VisionPoseCallback {

  private final NetworkTable nikolaVision =
          NetworkTableInstance.getDefault().getTable("limelight-nikola");

  // inputs/configs
  private final NetworkTableEntry nikolaCamMode = nikolaVision.getEntry("camMode");
  private final NetworkTableEntry nikolaPipeline = nikolaVision.getEntry("pipeline");
  private final DoubleArrayPublisher nikolaRobotOrientation =
          nikolaVision.getDoubleArrayTopic("robot_orientation_set").publish();

  // output
  private final DoubleArraySubscriber nikolaMegaTag1 =
          nikolaVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
  private final DoubleArraySubscriber nikolaMegaTag2 =
          nikolaVision.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);

  public enum CamStreamType {
    SIDE_BY_SIDE(0),
    LIMELIGHT(1),
    WEBCAM(2);

    private final int index;

    // Constructor for the enum, which assigns the index to each constant.
    private CamStreamType(int index) {
      this.index = index;
    }

    // Getter method to retrieve the index of the enum constant.
    public int getIndex() {
      return index;
    }
  }

  public enum TagType {
    RED_SOURCE_LEFT(1),
    RED_SOURCE_RIGHT(2),
    RED_PROCESSOR(3),
    RED_REEF_1(6),
    RED_REEF_2(7),
    RED_REEF_3(8),
    RED_REEF_4(9),
    RED_REEF_5(10),
    RED_REEF_6(11),
    BLUE_SOURCE_LEFT(12),
    BLUE_SOURCE_RIGHT(13),
    BLUE_PROCESSOR(16),
    BLUE_REEF_1(17),
    BLUE_REEF_2(18),
    BLUE_REEF_3(19),
    BLUE_REEF_4(20),
    BLUE_REEF_5(21),
    BLUE_REEF_6(22);

    private int tag;

    // Constructor for the enum, which assigns the index to each constant.
    private TagType(int tag) {
      this.tag = tag;
    }

    // Getter method to retrieve the index of the enum constant.
    public int getIndex() {
      return tag;
    }
  }

  private static final Matrix<N3, N1> POSE_DEVIATION_MEGATAG1 = VecBuilder.fill(0.01, 0.01, 0.05);
  private static final Matrix<N3, N1> POSE_DEVIATION_MEGATAG2 =
          VecBuilder.fill(0.06, 0.06, 9999999);

  private final LimelightBotPose botPoseMegaTag1 = new LimelightBotPose(null, 0);
//  private final LimelightBotPose botPoseMegaTag2 = new LimelightBotPose(null, 0);

  private double[] orientationSet = new double[] {0, 0, 0, 0, 0, 0};

  private final double fieldExtentMetresX;
  private final double fieldExtentMetresY;
  private final double maxAmbiguity;
  private final double highQualityAmbiguity;
  private final double maxVisposDeltaDistanceMetres;

  private int targetTagId = 0;

  public LimelightVisionSubsystem(VisionConfig visionConfig) {
    this.fieldExtentMetresX = visionConfig.fieldExtentMetresX();
    this.fieldExtentMetresY = visionConfig.fieldExtentMetresY();
    this.maxAmbiguity = visionConfig.maxAmbiguity();
    this.highQualityAmbiguity = visionConfig.highQualityAmbiguity();
    this.maxVisposDeltaDistanceMetres = visionConfig.maxVisposeDeltaDistanceMetres();

    this.nikolaPipeline.setNumber(visionConfig.pipelineAprilTagDetect());
    this.nikolaCamMode.setNumber(visionConfig.camModeVision());

    Telemetry.vision.enabled = visionConfig.telemetryEnabled();
  }

  @Override
  public void periodic() {
    TimestampedDoubleArray botPoseBlueMegaTag1 = nikolaMegaTag1.getAtomic();
    botPoseMegaTag1.update(botPoseBlueMegaTag1.value, botPoseBlueMegaTag1.timestamp);

//    TimestampedDoubleArray botPoseBlueMegaTag2 = nikolaMegaTag2.getAtomic();
//    botPoseMegaTag2.update(botPoseBlueMegaTag2.value, botPoseBlueMegaTag2.timestamp);
  }

  /* Public API */

  public void setTargetTagId(TagType tag) {
    this.targetTagId = tag.getIndex();
  }

  public void clearTargetTagId() {
    this.targetTagId = 0;
  }

  public double getVisibleTargetTagId() {
    return botPoseMegaTag1.getTagId(0);
  }

  public double distanceToTarget() {
    int index = 0;
    if (targetTagId > 0) {
      index = botPoseMegaTag1.getTagIndex(targetTagId);
    }
    return botPoseMegaTag1.getTagDistToRobot(index);
  }

  public double angleToTarget() {
    int index = 0;
    if (targetTagId > 0) {
      index = botPoseMegaTag1.getTagIndex(targetTagId);
    }
    return botPoseMegaTag1.getTagTxnc(index);
  }

  public LimelightBotPose getBotPose() {
    return botPoseMegaTag1;
  }

  /**
   * Get the pose estimate from the vision system, for callback via VisionPoseCallback
   *
   * <p>Logic In This Function:
   *
   * <ol>
   *   <li>Set the orientation of the robot from odometry into the limelight for MegaTag2 to work
   *   <li>Ensure the latest vision pose data is valid (on field, >=1 tags visible, robot not
   *       spinning like crazy
   *   <li>Check tag ambiguity - if it's very low (<0.1) it means we've got a very very high
   *       confidence set of data, and we therefore use MegaTag1 data to update pose and heading
   *       data, which will keep MegaTag 2 honest.
   *   <li>If ambiguity is medium (0.1-0.7), we can trust MegaTag2 as a high confidence source
   *   <li>Otherwise, let's not use high ambiguity data at all and let Odometry do its thing
   * </ol>
   *
   * @param odometryPose the current odometry pose of the robot
   * @param yaw the current yaw of the robot
   * @param yawRate the current yaw rate of the robot
   * @return the pose estimate
   */
  public PoseEstimate getPoseEstimate(Pose2d odometryPose, double yaw, double yawRate) {
    // First, update the limelight and let it know our orientation
    orientationSet[0] = yaw;
    nikolaRobotOrientation.set(orientationSet);

    PoseEstimate returnVal = null;

    // Get the current pose delta
    double compareDistance = -1;
    double compareHeading = -1;
    double tagAmbiguity = botPoseMegaTag1.getTagAmbiguity(0);

    LimelightPoseEstimate.PoseConfidence poseConfidence = LimelightPoseEstimate.PoseConfidence.NONE;
    LimelightBotPose botPose = botPoseMegaTag1; // default to MT1 for telemetry

    // If pose is 0,0 or no tags in view, we don't actually have data - return null
    if (botPoseMegaTag1.getTagCount() > 0
            && botPoseMegaTag1.isPoseXInBounds(0, fieldExtentMetresX)
            && botPoseMegaTag1.isPoseYInBounds(0, fieldExtentMetresY)
            && yawRate <= 720) {

      // Do we have a decent signal?  i.e. Ambiguity < 0.7
      if (tagAmbiguity < maxAmbiguity) {
        // Check for super good signal - ambiguity < 0.1, or we're disabled (field setup)
        if (tagAmbiguity < highQualityAmbiguity || DriverStation.isDisabled()) {
          // use megatag1 as is, it's rock solid
          poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG1;
          returnVal =
                  new LimelightPoseEstimate(
                          botPoseMegaTag1.getPose(),
                          botPoseMegaTag1.getTimestampSeconds(),
                          POSE_DEVIATION_MEGATAG1);
//        } else {
//          // Use MegaTag 2
//          poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG2;
//          botPose = botPoseMegaTag2;
//          returnVal =
//                  new LimelightPoseEstimate(
//                          botPoseMegaTag2.getPose(),
//                          botPoseMegaTag2.getTimestampSeconds(),
//                          POSE_DEVIATION_MEGATAG2);
        }

        if (Telemetry.vision.enabled) {
          compareDistance =
                  botPose.getPose().getTranslation().getDistance(odometryPose.getTranslation());
          compareHeading = botPose.getPose().getRotation().getDegrees() - yaw;
        }
      }
    }

    // Telemetry Handling
    if (Telemetry.vision.enabled) {
      Telemetry.vision.tagAmbiguity = tagAmbiguity;
      Telemetry.vision.poseConfidence = poseConfidence;
      Telemetry.vision.poseDeltaMetres = compareDistance;
      Telemetry.vision.headingDeltaDegrees = compareHeading;
      Telemetry.vision.poseMetresX = odometryPose.getX();
      Telemetry.vision.poseMetresY = odometryPose.getY();
      Telemetry.vision.poseHeadingDegrees = odometryPose.getRotation().getDegrees();
      Telemetry.vision.visionPoseX = botPose.getPoseX();
      Telemetry.vision.visionPoseY = botPose.getPoseY();
      Telemetry.vision.visionPoseHeading = botPose.getPoseRotationYaw();
    }

    return returnVal;
  }

  @Override
  public String toString() {
    return "Swerve Vision Subsystem";
  }
}
