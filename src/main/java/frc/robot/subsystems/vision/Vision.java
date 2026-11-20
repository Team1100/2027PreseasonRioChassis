package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionSystem[] systems;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.systems = new VisionSystem[io.length];
    for (int i = 0; i < io.length; i++) {
      this.systems[i] = new VisionSystem(io[i]);
    }
  }

  @Override
  public void periodic() {
    List<Pose3d> poses = new LinkedList<>();
    List<Pose3d> posesAccepted = new LinkedList<>();
    List<Pose3d> posesFiltered = new LinkedList<>();
    for (var system : this.systems) {
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

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
