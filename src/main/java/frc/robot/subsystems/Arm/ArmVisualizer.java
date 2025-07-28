package frc.robot.subsystems.Arm;

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

public class ArmVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d Arm;
  private final String key;

  private final double ArmLength = Units.inchesToMeters(78);
  private final Translation2d ArmOrigin = new Translation2d(0, 0);

  public ArmVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("Arm", 1.0, 0.4);
    Arm =
        new LoggedMechanismLigament2d("Arm", ArmLength, 90.0, 6, new Color8Bit(color));

    Arm.append(
        new LoggedMechanismLigament2d(
            "Arm_stage", Units.inchesToMeters(6), 90, 6, new Color8Bit(color)));
    root.append(Arm);
  }

  /** Update Arm visualizer with current Arm height */
  public void update(double positionInches) {
    Arm.setLength(Units.inchesToMeters(positionInches));
    Logger.recordOutput("Arm/Mechanisms/" + key + "/Mechanism2d", mechanism);

    // Log 3d poses
    Pose3d Arm =
        new Pose3d(
            ArmOrigin.getX(),
            ArmOrigin.getY() + this.Arm.getLength(),
            0.0,
            new Rotation3d());
    Logger.recordOutput("Arm/Mechanisms/" + key + "/Pose3d", Arm);
  }
}
