package frc.robot.bobot_state2.varc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state2.BobotState;
import frc.robot.field.FieldUtils;

public class ProcessorTagTracker extends TargetAngleTracker {
  private Rotation2d rotationTarget = Rotation2d.kZero;
  private double distanceMeters = 0;

  public void update() {
    Pose2d closestPose = FieldUtils.getProcessorFace().processor.getPose();
    rotationTarget = closestPose.getRotation().plus(Rotation2d.kPi);
    distanceMeters =
        closestPose.getTranslation().getDistance(BobotState.getGlobalPose().getTranslation());
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public double getDistanceMeters() {
    return distanceMeters;
  }
}
