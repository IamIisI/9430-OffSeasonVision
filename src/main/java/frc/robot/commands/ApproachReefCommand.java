package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ApproachReefCommand extends Command {
    private final DriveSubsystem drive;
    private final double desiredDistance;
    private final PIDController distanceController;
    private final PIDController lateralController;
    private final PIDController rotationController;

    // Tolerances and speed limits
    private static final double DISTANCE_TOLERANCE_METERS = 0.02; // 1.5cm tolerance (example)
    private static final double LATERAL_TOLERANCE_METERS = 0.015; // 1.5cm
    private static final double ROTATION_TOLERANCE_DEG = 2.5; // degrees tolerance
    private static final double MAX_FORWARD_SPEED = 1.5; // m/s
    private static final double MAX_LATERAL_SPEED = 1.0; // m/s
    private static final double MAX_ROTATION_SPEED = 0.5; // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds

    private double lastTagTimestamp = 0;
    // Variables to lock in the initial lateral offset when the tag is first seens
    private double desiredLateralOffset = 0;
    private boolean isIntake = false;

    // Camera selection for both intake and non-intake modes.
    private PhotonCamera selectedCamera = null; // Default to no specific camera

    public ApproachReefCommand(DriveSubsystem drive, double desiredDistance, double desiredOffset, double angle, PhotonCamera aligningCamera) {
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        this.desiredLateralOffset = desiredOffset;
        this.isIntake = false;
        this.selectedCamera = aligningCamera;
        addRequirements(drive);

        // PID for forward (distance) control
        distanceController = new PIDController(3.0, 0.0, 0.00);
        distanceController.setTolerance(0.0);

        // PID for lateral offset correction
        lateralController = new PIDController(3.0, 0.0, 0.05);
        lateralController.setTolerance(0.0);

        // PID for rotation to face the desired offset position
        rotationController = new PIDController(0.1, 0.005, 0.005);
        rotationController.setTolerance(0.0);
        rotationController.enableContinuousInput(-180, 180); // angle wrap-around
    }

    @Override
    public void initialize() {
        distanceController.reset();
        lateralController.reset();
        rotationController.reset();

        System.out.printf(
                "ApproachTagCommand initialized - Target distance: %.2f m, Lateral offset: %.2f m, Intake mode: %b%n",
                desiredDistance, desiredLateralOffset, isIntake);
    }

    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        boolean validTagDetection = false;

        // Use selected camera if one is set (applies to both intake and non-intake
        // modes)
        if (selectedCamera != null) {
            int detectedTag = poseEstimator.getLastTagDetectedByCamera(selectedCamera);
            double lastDetectionTime = poseEstimator.getLastCameraDetectionTimestamp(selectedCamera);
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

            if (detectedTag != -1 && (currentTime - lastDetectionTime) < LOST_TAG_TIMEOUT) {
                validTagDetection = true;
                lastTagTimestamp = lastDetectionTime;
            }
        } else {
            // Otherwise, use any camera detection available.
            if (poseEstimator.getLastDetectedTagId() != -1) {
                validTagDetection = true;
                lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();
            }
        }

        if (validTagDetection) {
            double currentDistance = poseEstimator.getXOffsetToTag(selectedCamera);
            double currentLateralOffset = poseEstimator.getYOffsetToTag(selectedCamera);
            double currentRotation = poseEstimator.getBearingToTagDeg(selectedCamera);

            // Compute corrections using PID controllers
            double forwardSpeed = -distanceController.calculate(currentDistance, desiredDistance);
            double lateralSpeed = -lateralController.calculate(currentLateralOffset, desiredLateralOffset);
            double rotationSpeed = -rotationController.calculate(currentRotation, 180);

            // Clamp speeds to maximum limits
            forwardSpeed = Math.min(Math.max(forwardSpeed, -MAX_FORWARD_SPEED), MAX_FORWARD_SPEED);
            lateralSpeed = Math.min(Math.max(lateralSpeed, -MAX_LATERAL_SPEED), MAX_LATERAL_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);

            // Drive the robot with computed speeds
            drive.driveRobotRelative(new ChassisSpeeds(forwardSpeed, lateralSpeed, rotationSpeed));
        } else {
            // No valid tag detected; stop the robot
            drive.driveRobotRelative(new ChassisSpeeds());
        }
    }

    @Override
    public boolean isFinished() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (currentTime - lastTagTimestamp > LOST_TAG_TIMEOUT) {
            System.out.println("ApproachTagCommand timeout - tag lost");
            return true;
        }

        boolean validTagDetection = false;
        double currentDistance = 0;
        double currentLateralOffset = 0;
        double currentRotation = 0;
    
        if (selectedCamera != null) {
            int detectedTag = poseEstimator.getLastTagDetectedByCamera(selectedCamera);
            double lastDetectionTime = poseEstimator.getLastCameraDetectionTimestamp(selectedCamera);

            if (detectedTag != -1 && (currentTime - lastDetectionTime) < LOST_TAG_TIMEOUT) {
                validTagDetection = true;
                currentDistance = poseEstimator.getXOffsetToTag(selectedCamera);
                currentLateralOffset = poseEstimator.getYOffsetToTag(selectedCamera);
                currentRotation = poseEstimator.getBearingToTagDeg(selectedCamera);
            }
        } else {
            if (poseEstimator.getLastDetectedTagId() != -1) {
                validTagDetection = true;
                currentDistance = poseEstimator.getDistanceToTag(selectedCamera);
                currentLateralOffset = poseEstimator.getLateralOffsetToTag(selectedCamera);
                currentRotation = poseEstimator.getBearingToTagDeg(selectedCamera);
            }
        }

        if (validTagDetection) {
            // Check if the distance, lateral offset, and rotation are within tolerance.
            boolean distanceOk = Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS;
            boolean lateralOk = Math.abs(currentLateralOffset - desiredLateralOffset) < LATERAL_TOLERANCE_METERS;
            double rotationError = (currentRotation - 180);
            boolean rotationOk = Math.abs(rotationError) < ROTATION_TOLERANCE_DEG;

            return distanceOk && lateralOk && rotationOk;
        }
        

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("ApproachTagCommand ended. Interrupted: " + interrupted);
    }
}