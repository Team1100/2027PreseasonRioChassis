package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public record VisionConfig(
    String name, Transform3d robotToCamera, double linearStddev, double angularStddev) {}
