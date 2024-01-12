package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  ArrayList<CameraResultProcessingPackage> allCameraResultProcessingPackages =
      new ArrayList<CameraResultProcessingPackage>();

  AprilTagFieldLayout aprilTagLayout;

  boolean useVisionForPoseEstimation = true;
  boolean useMaxDistanceAwayFromExistingEstimate = true;

  private static LoggedTunableNumber thetaStdDevCoefficient =
      new LoggedTunableNumber("vision/thetaStdDevCoefficient", 0.075);
  private static LoggedTunableNumber xyStdDevCoefficient =
      new LoggedTunableNumber("vision/xyStdDevCoefficient", 0.075);

  private HashMap<Integer, Double> lastTagDetectionTimes = new HashMap<Integer, Double>();

  private List<Pose3d> actualPosesUsedInPoseEstimator = new ArrayList<>();
  private List<TimestampedVisionUpdate> allTimestampedVisionUpdates =
      new ArrayList<TimestampedVisionUpdate>();

  // private Alert noAprilTagLayoutAlert = new Alert("No AprilTag layout file found",
  // AlertType.ERROR);

  private final Drivetrain drivetrain;

  public Vision(
      AprilTagFieldLayout aprilTagLayout, Drivetrain drivetrain, VisionModule... VisionModules) {

    this.drivetrain = drivetrain;

    // FIXME use config for layout instead
    // try {
    //   aprilTagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    //   noAprilTagLayoutAlert.set(false);
    // } catch (Exception e) {
    //   aprilTagLayout =
    //       new AprilTagFieldLayout(
    //           Collections.emptyList(),
    //           //FIXME: probably need to pass in a april tag field layout
    //           0.1,
    //           0.1);
    //   noAprilTagLayoutAlert.set(true);
    // }

    this.aprilTagLayout = aprilTagLayout;

    for (VisionModule config : VisionModules) {
      allCameraResultProcessingPackages.add(
          new CameraResultProcessingPackage(config, aprilTagLayout));
    }

    // log all robotToCamera constants, useful for cameraOverride view mode in advantage scope
    // FIXME add something that takes robot pose and adds it to the camera pose
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      Logger.recordOutput(
          "Vision/" + cameraPackage.name + "CameraConstant",
          new Pose3d().transformBy(cameraPackage.RobotToCamera));
    }

    for (AprilTag tag : aprilTagLayout.getTags()) {
      Logger.recordOutput("Vision/AllAprilTags3D/" + tag.ID, tag.pose);
    }

    aprilTagLayout.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, -1.0));
  }

  @Override
  public void periodic() {
    // update all inputs
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      cameraPackage.visionIO.updateInputs(cameraPackage.visionIOInputs);
      Logger.processInputs("Vision/" + cameraPackage.name, cameraPackage.visionIOInputs);
    }

    boolean processVision = true;
    if (!useVisionForPoseEstimation) {
      processVision = false;
      setAllCameraPackageUnsuccessfulStatus(VisionProcessingStatus.NOT_PROCESSING_VISION);
    }

    if (processVision) {
      for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
        var fieldsToLog = processVision(cameraPackage);
        cameraPackage.loggedFields = fieldsToLog;
      }
    }

    // process vision fill the allTimestampedVisionUpdates list
    if (allTimestampedVisionUpdates.size() > 0) {
      drivetrain.addVisionEstimate(allTimestampedVisionUpdates);
    }
    allTimestampedVisionUpdates.clear();

    // logging for camera fields
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      logVisionProcessingStatusAndFields(cameraPackage);
    }

    // FIXME: you can optimize this
    List<Pose3d> allAtThisVeryMomentVisibleTags = new ArrayList<>();

    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (detectionEntry.getValue() == Timer.getFPGATimestamp()) {
        var tagPose = aprilTagLayout.getTagPose(detectionEntry.getKey());
        allAtThisVeryMomentVisibleTags.add(tagPose.get());
      }
    }

    Logger.recordOutput(
        "Vision/currentVisibleTags_EXACT_MOMENT",
        allAtThisVeryMomentVisibleTags.toArray(new Pose3d[allAtThisVeryMomentVisibleTags.size()]));

    Logger.recordOutput(
        "Vision/actual_poses_used_in_pose_estimator",
        actualPosesUsedInPoseEstimator.toArray(new Pose3d[actualPosesUsedInPoseEstimator.size()]));

    actualPosesUsedInPoseEstimator.clear();

    Logger.recordOutput("Vision/useVision", useVisionForPoseEstimation);
    Logger.recordOutput(
        "Vision/useMaxDistanceAwayFromExistingEstimate", useMaxDistanceAwayFromExistingEstimate);
  }

  // FIXME: comb over this 1 more time
  public VisionProcessingLoggedFields processVision(CameraResultProcessingPackage cameraPackage) {
    PhotonPipelineResult cameraResult;
    double currentResultTimeStamp;

    Pose2d prevEstimatedRobotPose = drivetrain.getPose();

    Pose3d newCalculatedRobotPose;

    double xyStandardDeviation;
    double thetaStandardDeviation;

    // storing fields to log
    double tagAmbiguity;
    double distanceFromTag;
    Pose3d cameraPose;

    synchronized (cameraPackage.visionIOInputs) {
      cameraResult = cameraPackage.visionIOInputs.lastResult;
      currentResultTimeStamp = cameraPackage.visionIOInputs.lastTimestamp;
    }

    // FIXME: maybe changed to using a - b < epsilon?
    if (cameraPackage.lastProcessedResultTimeStamp >= currentResultTimeStamp) {
      return VisionProcessingLoggedFields.unsuccessfulStatus(
          VisionProcessingStatus.NOT_A_NEW_RESULT);
    }

    cameraPackage.lastProcessedResultTimeStamp = currentResultTimeStamp;

    ArrayList<PhotonTrackedTarget> cleanTargets = new ArrayList<PhotonTrackedTarget>();

    for (PhotonTrackedTarget target : cameraResult.getTargets()) {
      // FIXME: might me a way to optimize this
      if (aprilTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
        cleanTargets.add(target);
      }
    }

    cleanTargets.forEach(
        (PhotonTrackedTarget tag) ->
            lastTagDetectionTimes.put(tag.getFiducialId(), Timer.getFPGATimestamp()));

    var numTargetsSeen = cleanTargets.size();

    if (numTargetsSeen == 0) {
      return VisionProcessingLoggedFields.unsuccessfulStatus(
          VisionProcessingStatus.NO_TARGETS_VISIBLE);
    }

    // if only 1 tag there is different logic for which status to return
    if (numTargetsSeen == 1) {
      PhotonTrackedTarget singularTag = cameraResult.getTargets().get(0);

      if (!isValidTarget(singularTag)) {
        return (singularTag.getPoseAmbiguity() >= VisionConstants.MAXIMUM_AMBIGUITY)
            ? VisionProcessingLoggedFields.unsuccessfulStatus(
                VisionProcessingStatus.INVALID_TAG_AMBIGUITY_TOO_HIGH)
            : VisionProcessingLoggedFields.unsuccessfulStatus(VisionProcessingStatus.INVALID_TAG);
      }
    }

    Optional<EstimatedRobotPose> photonPoseEstimatorOptionalResult;

    // set reference pose in case multi-tag needs a fall back
    cameraPackage.photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    photonPoseEstimatorOptionalResult = cameraPackage.photonPoseEstimator.update(cameraResult);

    if (photonPoseEstimatorOptionalResult.isEmpty()) {
      return VisionProcessingLoggedFields.unsuccessfulStatus(
          VisionProcessingStatus.PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY);
    }

    newCalculatedRobotPose = photonPoseEstimatorOptionalResult.get().estimatedPose;

    // logged fields
    tagAmbiguity = 0.0;
    cameraPose = newCalculatedRobotPose.transformBy(cameraPackage.RobotToCamera);

    double totalDistance = 0.0;
    for (PhotonTrackedTarget tag : cleanTargets) {
      totalDistance +=
          aprilTagLayout
              .getTagPose(tag.getFiducialId())
              .get()
              .getTranslation()
              .getDistance(newCalculatedRobotPose.getTranslation());
    }

    // FIXME: rename this to average distance from tags?
    distanceFromTag = totalDistance / (double) cleanTargets.size();

    var distanceFromExistingPoseEstimate =
        prevEstimatedRobotPose
            .getTranslation()
            .getDistance(
                new Translation2d(newCalculatedRobotPose.getX(), newCalculatedRobotPose.getY()));

    if (useMaxDistanceAwayFromExistingEstimate
        && (distanceFromExistingPoseEstimate
            > (VisionConstants.MAX_VALID_DISTANCE_AWAY_METERS * numTargetsSeen))) {
      return VisionProcessingLoggedFields.unsuccessfulStatus(
          VisionProcessingStatus.TOO_FAR_FROM_EXISTING_ESTIMATE);
    }

    xyStandardDeviation =
        (xyStdDevCoefficient.get() * Math.pow(distanceFromTag, 2)) / ((double) numTargetsSeen);
    thetaStandardDeviation =
        (thetaStdDevCoefficient.get() * Math.pow(distanceFromTag, 2)) / ((double) numTargetsSeen);

    // drivetrain.addVisionEstimate(visionData);

    var timestampedVisionUpdate =
        new PoseEstimator.TimestampedVisionUpdate(
            currentResultTimeStamp,
            newCalculatedRobotPose.toPose2d(),
            VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation));

    allTimestampedVisionUpdates.add(timestampedVisionUpdate);
    // FIXME: add a pose2d version of this
    actualPosesUsedInPoseEstimator.add(newCalculatedRobotPose);

    return new VisionProcessingLoggedFields(
        VisionProcessingStatus.SUCCESSFUL,
        numTargetsSeen,
        tagAmbiguity,
        distanceFromTag,
        distanceFromExistingPoseEstimate,
        xyStandardDeviation,
        thetaStandardDeviation,
        currentResultTimeStamp,
        cameraPose,
        newCalculatedRobotPose);
  }

  public void setAllCameraPackageUnsuccessfulStatus(VisionProcessingStatus status) {
    for (CameraResultProcessingPackage cameraPackage : allCameraResultProcessingPackages) {
      cameraPackage.loggedFields = VisionProcessingLoggedFields.unsuccessfulStatus(status);
    }
  }

  public void logVisionProcessingStatusAndFields(CameraResultProcessingPackage cameraPackage) {

    String ROOT_TABLE_PATH = "Vision/" + cameraPackage.name + "/";

    var fieldsToLog = cameraPackage.loggedFields;

    Logger.recordOutput(
        ROOT_TABLE_PATH + "*STATUS",
        fieldsToLog.status().name() + ": " + fieldsToLog.status().logOutput);

    Logger.recordOutput(ROOT_TABLE_PATH + "calculated_robotPose_3d", fieldsToLog.robotPose3d());
    Logger.recordOutput(
        ROOT_TABLE_PATH + "calculated_robotPose_2d", fieldsToLog.robotPose3d().toPose2d());
    Logger.recordOutput(ROOT_TABLE_PATH + "camera_pose_3d", fieldsToLog.cameraPose());
    Logger.recordOutput(ROOT_TABLE_PATH + "num_seen_targets", fieldsToLog.numSeenTargets());
    Logger.recordOutput(ROOT_TABLE_PATH + "processed_timestamp", fieldsToLog.processedTimeStamp());
    Logger.recordOutput(
        ROOT_TABLE_PATH + "distance_from_existing_pose_estimate",
        fieldsToLog.distanceFromExistingPoseEstimate());
    Logger.recordOutput(ROOT_TABLE_PATH + "distance_from_tag", fieldsToLog.distanceFromTag());
    Logger.recordOutput(ROOT_TABLE_PATH + "tag_ambiguity", fieldsToLog.tagAmbiguity());
    Logger.recordOutput(
        ROOT_TABLE_PATH + "xy_standard_deviation", fieldsToLog.xyStandardDeviation());
    Logger.recordOutput(
        ROOT_TABLE_PATH + "theta_standard_deviation", fieldsToLog.thetaStandardDeviation());

    boolean addedVisionEstimateToPoseEstimator =
        (fieldsToLog.status().equals(VisionProcessingStatus.SUCCESSFUL)) ? true : false;

    Logger.recordOutput(
        ROOT_TABLE_PATH + "added_vision_measurement_to_pose_estimator(AKA SUCCESSFUL?)",
        addedVisionEstimateToPoseEstimator);
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && aprilTagLayout.getTagPose(target.getFiducialId()).isPresent();
  }

  public void useMaxDistanceAwayFromExistingEstimate(boolean value) {
    useMaxDistanceAwayFromExistingEstimate = value;
  }

  private class CameraResultProcessingPackage {
    final VisionIO visionIO;
    final VisionIOInputs visionIOInputs;
    final PhotonPoseEstimator photonPoseEstimator;
    final Transform3d RobotToCamera;
    final String name;

    public double lastProcessedResultTimeStamp;
    public VisionProcessingLoggedFields loggedFields;

    public CameraResultProcessingPackage(
        VisionModule config, AprilTagFieldLayout aprilTagFieldLayout) {
      this.visionIO = config.visionIO;
      this.visionIOInputs = new VisionIOInputs();

      this.RobotToCamera = config.robotToCamera;

      this.name = config.logName;

      this.photonPoseEstimator =
          new PhotonPoseEstimator(
              aprilTagFieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              visionIO.getCamera(),
              RobotToCamera);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

      lastProcessedResultTimeStamp = 0.0;
    }
  }

  // NOTE: the record keyword below requires using Java Version 17 or later. change in build.gradle
  private record VisionProcessingLoggedFields(
      VisionProcessingStatus status,
      double numSeenTargets,
      double tagAmbiguity,
      double distanceFromTag,
      double distanceFromExistingPoseEstimate,
      double xyStandardDeviation,
      double thetaStandardDeviation,
      double processedTimeStamp,
      Pose3d cameraPose,
      Pose3d robotPose3d) {

    private VisionProcessingLoggedFields {}

    private VisionProcessingLoggedFields(VisionProcessingStatus status) {
      this(status, -1, -1, -1, -1, -1, -1, -1, new Pose3d(), new Pose3d());
    }

    public static VisionProcessingLoggedFields DEFAULT_LOG_VALUES =
        new VisionProcessingLoggedFields(
            // probably make the poses like -10, -10, 0 to move them off screen and make it obvious
            // its bad results
            VisionProcessingStatus.UNKNOWN, -1, -1, -1, -1, -1, -1, -1, new Pose3d(), new Pose3d());

    public static VisionProcessingLoggedFields unsuccessfulStatus(VisionProcessingStatus status) {
      return new VisionProcessingLoggedFields(status);
    }
  }

  private enum VisionProcessingStatus {
    NOT_PROCESSING_VISION(""),
    GYRO_ANGLE_NOT_VALID(""),
    NOT_A_NEW_RESULT(""),
    PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY(""),
    NO_TARGETS_VISIBLE(""),

    LOGIC_ERROR_EXPECTED_1_TARGET(""),

    TOO_FAR_FROM_EXISTING_ESTIMATE(""),

    INVALID_TAG_AMBIGUITY_TOO_HIGH(""),
    INVALID_TAG(""),

    TAG_NOT_IN_LAYOUT(""),

    SUCCESSFUL(""),
    UNKNOWN("UNKNOWN");

    public final String logOutput;

    private VisionProcessingStatus(String logOutput) {
      this.logOutput = logOutput;
    }
  }
}
