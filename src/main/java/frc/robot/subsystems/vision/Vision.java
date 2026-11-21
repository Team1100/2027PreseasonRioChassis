package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionSystem.SystemResult;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Map<String, VisionSystem> systems;

  public Vision(VisionConsumer consumer, VisionIO... ios) {
    this.consumer = consumer;
    this.systems = new HashMap<>();
    for (var io : ios) {
      var inputs = new VisionIOInputs();
      io.updateInputs(inputs);
      systems.put(inputs.name, new VisionSystem(io));
    }
  }

  @Override
  public void periodic() {
    List<Pose3d> poses = new LinkedList<>();
    List<Pose3d> posesAccepted = new LinkedList<>();
    List<Pose3d> posesFiltered = new LinkedList<>();
    for (var entry : this.systems.entrySet()) {
      var system = entry.getValue();
      system.process();
      system
          .getLatestResults()
          .forEach(
              result -> {
                poses.add(result.pose());
                if (result.filtered()) {
                  posesFiltered.add(result.pose());
                } else {
                  consumer.accept(result.pose().toPose2d(), result.timestamp(), result.stddevs());
                  posesAccepted.add(result.pose());
                }
              });
    }
    Logger.recordOutput("Vision/Summary/RobotPoses", poses.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/RobotPosesAccepted", posesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/RobotPosesFiltered", posesFiltered.toArray(Pose3d[]::new));
  }

  /* Returns the latest *valid* result of the specified camera, or empty. */
  public Optional<SystemResult> getLatestResultOf(String cameraName) {
    if (this.systems.containsKey(cameraName)) {
      SystemResult result = this.systems.get(cameraName).getLatestResult();
      if (result != null) return Optional.of(result);
    }
    return Optional.empty();
  }

  public boolean cameraSeesAnyOf(String cameraName, int... ids) {
    if (this.systems.containsKey(cameraName)) {
      return this.systems.get(cameraName).seesAnyOf(ids);
    }
    return false;
  }

  public boolean cameraSeesAllOf(String cameraName, int... ids) {
    if (this.systems.containsKey(cameraName)) {
      return this.systems.get(cameraName).seesAllOf(ids);
    }
    return false;
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
