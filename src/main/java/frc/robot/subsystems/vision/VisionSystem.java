package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionSystem {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs;
  private final Alert disconnectedAlert;
  private final List<SystemResult> results;
  private SystemResult latestResult;

  private final double AMBIGUITY_THRESHOLD = 0.5;
  private final double MAX_Z = 1.0;
  private final double LINEAR_STDDEV_BASELINE = 0.02;
  private final double ANGULAR_STDDEV_BASELINE = 0.06;

  public VisionSystem(VisionIO io) {
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged();
    this.disconnectedAlert =
        new Alert("Vision Camera '<unnamed>' is disconnected.", AlertType.kWarning);
    this.results = new LinkedList<>();
    this.latestResult = null;
  }

  public void process() {
    this.io.updateInputs(this.inputs);
    Logger.processInputs("Vision/" + this.inputs.name, this.inputs);

    this.disconnectedAlert.setText("Vision Camera '" + this.inputs.name + "' is disconnected.");
    this.disconnectedAlert.set(!inputs.connected);

    for (var i : this.inputs.poseObservations) {
      Matrix<N3, N1> stddevs = stddevsOf(i);

      boolean filter =
          (i.tagCount() <= 0
              || i.tagCount() == 1 && i.ambiguity() > AMBIGUITY_THRESHOLD
              || Math.abs(i.pose().getZ()) > MAX_Z
              || i.pose().getX() < 0
              || i.pose().getX() > Constants.aprilTagLayout.getFieldLength()
              || i.pose().getY() < 0
              || i.pose().getY() > Constants.aprilTagLayout.getFieldWidth());
      SystemResult result = new SystemResult(i.pose(), i.timestamp(), stddevs, filter);
      this.results.add(result);
      if (!filter) this.latestResult = result;
    }
  }

  private Matrix<N3, N1> stddevsOf(PoseObservation observation) {
    double stddevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStddev = LINEAR_STDDEV_BASELINE * stddevFactor * this.inputs.linearStddev;
    double angularStddev = ANGULAR_STDDEV_BASELINE * stddevFactor * this.inputs.angularStddev;

    return VecBuilder.fill(linearStddev, linearStddev, angularStddev);
  }

  public List<SystemResult> getLatestResults() {
    List<SystemResult> retResults = new LinkedList<>(this.results);
    this.results.clear();
    return retResults;
  }

  public SystemResult getLatestResult() {
    return this.latestResult;
  }

  public boolean seesAnyOf(int... ids) {
    for (int id : ids) {
      for (int viewedId : this.inputs.tagIds) {
        if (id == viewedId) return true;
      }
    }
    return false;
  }

  public boolean seesAllOf(int... ids) {
    for (int id : ids) {
      boolean found = false;
      for (int viewedId : this.inputs.tagIds) {
        if (id == viewedId) found = true;
      }
      if (!found) return false;
    }
    return true;
  }

  public static record SystemResult(
      Pose3d pose, double timestamp, Matrix<N3, N1> stddevs, boolean filtered) {}
}
