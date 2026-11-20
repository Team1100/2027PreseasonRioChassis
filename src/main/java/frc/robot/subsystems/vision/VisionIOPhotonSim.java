package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonSim(
      String name,
      Transform3d robotToCamera,
      Supplier<Pose2d> poseSupplier,
      double linearStddev,
      double angularStddev) {
    super(name, robotToCamera, linearStddev, angularStddev);
    this.poseSupplier = poseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Constants.aprilTagLayout);
    }

    var cameraProps = new SimCameraProperties();
    cameraProps.setAvgLatencyMs(35);
    cameraProps.setCalibError(0.25, 0.08);
    cameraProps.setFPS(30);
    cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(90));
    this.cameraSim = new PhotonCameraSim(this.camera, cameraProps, Constants.aprilTagLayout);
    visionSim.addCamera(this.cameraSim, this.robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(this.poseSupplier.get());
    super.updateInputs(inputs);
  }

  public static VisionIO fromConfig(VisionConfig config, Supplier<Pose2d> poseSupplier) {
    return new VisionIOPhotonSim(
        config.name(),
        config.robotToCamera(),
        poseSupplier,
        config.linearStddev(),
        config.angularStddev());
  }
}
