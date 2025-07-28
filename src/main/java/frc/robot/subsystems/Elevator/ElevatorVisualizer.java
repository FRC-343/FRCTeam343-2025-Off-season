package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d elevator;
  private final String key;

  private final double elevatorLength = Units.inchesToMeters(78);
  private final Translation2d elevatorOrigin = new Translation2d(0, 0);

  public ElevatorVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("elevator", 1.0, 0.4);
    elevator =
        new LoggedMechanismLigament2d("elevator", elevatorLength, 90.0, 6, new Color8Bit(color));

    elevator.append(
        new LoggedMechanismLigament2d(
            "elevator_stage", Units.inchesToMeters(6), 90, 6, new Color8Bit(color)));
    root.append(elevator);
  }

  /** Update elevator visualizer with current elevator height */
  public void update(double positionInches) {
    elevator.setLength(Units.inchesToMeters(positionInches));
    Logger.recordOutput("Elevator/Mechanisms/" + key + "/Mechanism2d", mechanism);

    // Log 3d poses
    Pose3d elevator =
        new Pose3d(
            elevatorOrigin.getX(),
            elevatorOrigin.getY() + this.elevator.getLength(),
            0.0,
            new Rotation3d());
    Logger.recordOutput("Elevator/Mechanisms/" + key + "/Pose3d", elevator);
  }
}
