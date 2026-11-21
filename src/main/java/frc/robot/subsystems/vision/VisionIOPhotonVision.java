package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final String name;
  protected final double linearStddev, angularStddev;

  public VisionIOPhotonVision(
      String name, Transform3d robotToCamera, double linearStddev, double angularStddev) {
    this.name = name;
    this.camera = new PhotonCamera(this.name);
    this.robotToCamera = robotToCamera;
    this.linearStddev = linearStddev;
    this.angularStddev = angularStddev;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = this.camera.isConnected();
    inputs.name = this.name;
    inputs.linearStddev = this.linearStddev;
    inputs.angularStddev = this.angularStddev;

    List<PoseObservation> observations = new LinkedList<>();
    Set<Short> tagIds = new HashSet<>();
    for (var result : camera.getAllUnreadResults()) {
      if (result.multitagResult.isPresent()) {
        MultiTargetPNPResult mtResult = result.multitagResult.get();
        Transform3d fieldToCamera = mtResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(this.robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double avgTagDistance =
            result.targets.stream().mapToDouble(this::distanceFromTarget).average().orElse(0.0);

        mtResult.fiducialIDsUsed.forEach(id -> tagIds.add(id));
        observations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                mtResult.estimatedPose.ambiguity,
                mtResult.fiducialIDsUsed.size(),
                avgTagDistance,
                PoseObservationType.PHOTONVISION));
      } else if (!result.targets.isEmpty()) {
        var target = result.targets.get(0);
        Constants.aprilTagLayout
            .getTagPose(target.fiducialId)
            .ifPresent(
                pose -> {
                  Transform3d fieldToTarget =
                      new Transform3d(pose.getTranslation(), pose.getRotation());
                  Transform3d cameraToTarget = target.bestCameraToTarget;
                  Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                  Transform3d fieldToRobot = fieldToCamera.plus(this.robotToCamera.inverse());
                  Pose3d robotPose =
                      new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                  tagIds.add((short) target.fiducialId);
                  observations.add(
                      new PoseObservation(
                          result.getTimestampSeconds(),
                          robotPose,
                          target.poseAmbiguity,
                          1,
                          distanceFromTarget(target),
                          PoseObservationType.PHOTONVISION));
                });
      }
    }

    inputs.poseObservations = new PoseObservation[observations.size()];
    for (int i = 0; i < observations.size(); i++) inputs.poseObservations[i] = observations.get(i);

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (Short tagId : tagIds) {
      inputs.tagIds[i++] = tagId;
    }
  }

  private double distanceFromTarget(PhotonTrackedTarget value) {
    return value.bestCameraToTarget.getTranslation().getNorm();
  }

  public static VisionIO fromConfig(VisionConfig config) {
    return new VisionIOPhotonVision(
        config.name(), config.robotToCamera(), config.linearStddev(), config.angularStddev());
  }
}
