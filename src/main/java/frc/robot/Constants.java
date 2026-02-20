package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
@@ -13,9 +18,42 @@
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double MAX_SPEED = Units.feetToMeters(10);


  public static boolean isBlue = DriverStation.getAlliance().get() == Alliance.Blue;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double triggerDeadband = 0.1;
    public static final double DEADBAND = 0.0875; 
  }

  public static class Vision {
        
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }


  public static class Targets {

    public static final double[] hub;
    public static final double[] leftShot;
    public static final double[] rightShot;
    public static final double[] farShot;

    static {
      if (isBlue) {
        hub = new double[] {4.035,4.03,1.1938};
        leftShot = new double[] {2,2,0};
        rightShot = new double[] {6,2,0};
        farShot = new double[] {4.035,2,0};
      } else {
        hub = new double[] {4.035,4.03,1.1938};
        leftShot = new double[] {2,2,0};
        rightShot = new double[] {6,2,0};
        farShot = new double[] {4.035,2,0};
      }

    }
    
    
  }
}