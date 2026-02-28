/*
* MIT License
*
* Copyright (c) PhotonVision
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Rotation3d;

public class Vision extends SubsystemBase {

    public boolean visionEnabled = true;

    private final Transform3d robotToCam1 = new Transform3d(
        new Translation3d(0.22225, -0.1635125, 0.254),
        new Rotation3d(0, 0, 0)
    );

    private final Transform3d robotToCam2 = new Transform3d(
        new Translation3d(-0.2032, 0.295275, 0.254),
        new Rotation3d(0, 0, Math.PI)
    );

    private final PhotonCamera cam1 = new PhotonCamera("cam1");
    private final PhotonCamera cam2 = new PhotonCamera("cam2");

    private final PhotonPoseEstimator photonEstimator1 =
        new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam1);

    private final PhotonPoseEstimator photonEstimator2 =
        new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam2);

    private final PhotonCamera[] cams = { cam2, cam1 };
    private final PhotonPoseEstimator[] photonEstimators = { photonEstimator2, photonEstimator1 };

    private Matrix<N3, N1> curStdDevs;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    private final SwerveDrive swerveDrive;

    public Vision(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        photonEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(kTagLayout);

            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);

            cameraSim = new PhotonCameraSim(cam1, cameraProp);
            visionSim.addCamera(cameraSim, robotToCam1);
            cameraSim.enableDrawWireframe(true);
        }
    }

    public void addVisionMeasurement() {
        swerveDrive.updateOdometry();
        EstimatedRobotPose[] visionEsts = GetVisionEstimates();

        for (var est : visionEsts) {
            var estStdDevs = getEstimationStdDevs();
            Pose2d estPose = est.estimatedPose.toPose2d();
            double[] poseEstArray = { estPose.getX(), estPose.getY() };

            swerveDrive.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
            SmartDashboard.putNumberArray("Robot Pose Est:", poseEstArray);
        }
    }

    public EstimatedRobotPose[] GetVisionEstimates() {
        List<EstimatedRobotPose> visionEsts = new ArrayList<>();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (int i = 0; i < cams.length; i++) {
            for (var change : cams[i].getAllUnreadResults()) {
                visionEst = photonEstimators[i].update(change);
                updateEstimationStdDevs(visionEst, change.getTargets());
                visionEst.ifPresent(visionEsts::add);

                if (Robot.isSimulation()) {
                    visionEst.ifPresentOrElse(
                        est -> getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                        () -> getSimDebugField().getObject("VisionEstimation").setPoses()
                    );
                }
            }
        }

        return visionEsts.toArray(new EstimatedRobotPose[0]);
    }

    /**
     * Safe updateEstimationStdDevs — fully crash-proof
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty() || targets.isEmpty()) {
            curStdDevs = kSingleTagStdDevs;
            return;
        }

        int[] visibleTags = {0};
        double[] totalDist = {0};

        for (var tgt : targets) {
            var tagPose = photonEstimator1.getFieldTags().getTagPose(tgt.getFiducialId());
            tagPose.ifPresent(pose -> {
                estimatedPose.ifPresent(est -> {
                    totalDist[0] += pose.toPose2d()
                                        .getTranslation()
                                        .getDistance(est.estimatedPose.toPose2d().getTranslation());
                    visibleTags[0]++;
                });
            });
        }

        if (visibleTags[0] == 0) {
            curStdDevs = kSingleTagStdDevs;
            return;
        }

        double avgDist = totalDist[0] / visibleTags[0];
        var estStdDevs = kSingleTagStdDevs;

        if (visibleTags[0] > 1) {
            estStdDevs = kMultiTagStdDevs;
        }

        if (visibleTags[0] == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        curStdDevs = estStdDevs;
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}