package frc.robot.subsystems;

import java.io.File;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    public final double maximumSpeed = Units.feetToMeters(14.5);

    private final double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(((150.0 / 7.0) / 1.0),
            1);
    private final double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4.0),
            (6.75 / 1.0), 1);

    private final SwerveDriveTelemetry.TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;

    private final File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    private final SwerveDrive swerveDrive;

    private final VisionSubsystem visionSubsystem;

    public SwerveSubsystem(VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;

        SwerveDriveTelemetry.verbosity = telemetryVerbosity;

        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, angleConversionFactor,
                    driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException("Could Not Build Swerve Drive");
        }

        swerveDrive.setHeadingCorrection(false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(
                translation,
                rotation,
                fieldRelative,
                false);
    }

    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    @Override
    public void periodic() {
        if(visionSubsystem.getLeftVisionEstimatePresent())
        {
            addVisionMeasurment(
                visionSubsystem.getLeftVisionEstimatePose2d(), 
                visionSubsystem.getLeftVisionEstimateTimestamp()
            );    
        }

        if(visionSubsystem.getRightVisionEstimatePresent())
        {
            addVisionMeasurment(
                visionSubsystem.getRightVisionEstimatePose2d(), 
                visionSubsystem.getRightVisionEstimateTimestamp()
            );    
        }
    }

    public void addVisionMeasurment(Pose2d robotPose2d, double timestampSeconds)
    {
        swerveDrive.addVisionMeasurement(robotPose2d, timestampSeconds);
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
                getHeading().getRadians(), maximumSpeed);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maximumSpeed);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }
}
