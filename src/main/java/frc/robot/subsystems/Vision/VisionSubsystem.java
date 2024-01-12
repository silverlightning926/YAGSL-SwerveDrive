package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    // --- Constants ---

    private final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    private final String LEFT_CAMERA_NAME = "leftCamera";
    private final String RIGHT_CAMERA_NAME = "rightCamera";

    private final Transform3d LEFT_CAMERA_ROBOT_TO_CAM_TRANSFORM_METERS = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));

    private final Transform3d RIGHT_CAMERA_ROBOT_TO_CAM_TRANSFORM_METERS = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));

    // --- Constants ---

    private final PhotonVisionCamera leftCamera;
    private final PhotonVisionCamera rightCamera;

    private Optional<EstimatedRobotPose> leftVisionEstimate;
    private Optional<EstimatedRobotPose> rightVisionEstimate;

    private double leftVisionEstimateTimestamp;
    private double rightVisionEstimateTimestamp;

    public VisionSubsystem() {
        try {
            APRIL_TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("Could not load AprilTag field layout");
        }

        leftCamera = new PhotonVisionCamera(
                LEFT_CAMERA_NAME,
                APRIL_TAG_FIELD_LAYOUT,
                LEFT_CAMERA_ROBOT_TO_CAM_TRANSFORM_METERS);

        rightCamera = new PhotonVisionCamera(
                RIGHT_CAMERA_NAME,
                APRIL_TAG_FIELD_LAYOUT,
                RIGHT_CAMERA_ROBOT_TO_CAM_TRANSFORM_METERS);
    }

    private boolean getVisionEstimatePresent(Optional<EstimatedRobotPose> visionEstimate) {
        return visionEstimate.isPresent();
    }

    public boolean getLeftVisionEstimatePresent() {
        return getVisionEstimatePresent(leftVisionEstimate);
    }

    public boolean getRightVisionEstimatePresent() {
        return getVisionEstimatePresent(rightVisionEstimate);
    }

    private Pose2d getVisionEstimatePose2d(Optional<EstimatedRobotPose> visionEstimate) {
        return visionEstimate.get().estimatedPose.toPose2d();
    }

    public Pose2d getLeftVisionEstimatePose2d() {
        return getVisionEstimatePose2d(leftVisionEstimate);
    }

    public Pose2d getRightVisionEstimatePose2d() {
        return getVisionEstimatePose2d(rightVisionEstimate);
    }

    public double getLeftVisionEstimateTimestamp() {
        return leftVisionEstimateTimestamp;
    }

    public double getRightVisionEstimateTimestamp() {
        return rightVisionEstimateTimestamp;
    }

    @Override
    public void periodic() {
        leftVisionEstimate = leftCamera.getEstimatedGlobalPose();
        rightVisionEstimate = rightCamera.getEstimatedGlobalPose();

        leftVisionEstimateTimestamp = leftVisionEstimate.get().timestampSeconds;
        rightVisionEstimateTimestamp = rightVisionEstimate.get().timestampSeconds;
    }
}