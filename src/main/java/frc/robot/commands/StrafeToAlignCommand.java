package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;


public class StrafeToAlignCommand extends Command {
    private final DriveSubsystem drive;
    private final PIDController distanceController;
    private final PIDController lateralController;
    private final PIDController rotationController;

    // Tolerances and speed limits
    private static final double DISTANCE_TOLERANCE_METERS = 0.015; // 1.5cm tolerance (example)
    private static final double LATERAL_TOLERANCE_METERS = 0.02; // 2cm
    private static final double ROTATION_TOLERANCE_DEG = 5.0; // degrees tolerance
    private static final double MAX_FORWARD_SPEED = 1.5; // m/s
    private static final double MAX_LATERAL_SPEED = 1.0; // m/s
    private static final double MAX_ROTATION_SPEED = 5.0; // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds

    // Camera indices matching those in PoseEstimatorSubsystem
    private static final int LEFT_CAMERA_INDEX = 2;  // for intake mode when target is right-side
    private static final int RIGHT_CAMERA_INDEX = 1; // for intake mode when target is left-side

    private double lastTagTimestamp = 0;
    // Variables to lock in the initial lateral offset when the tag is first seen
    private boolean lateralOffsetInitialized = false;
    private double desiredLateralOffset = 0;
    private boolean isIntake = false;
    
    // Camera selection for both intake and non-intake modes.
    private PhotonCamera selectedCamera = null; // Default to no specific camera

    public StrafeToAlignCommand(DriveSubsystem drive, double desiredLateralOffset, boolean isIntake) {
        this.drive = drive;
        this.desiredLateralOffset = desiredLateralOffset;
        this.isIntake = isIntake;
        addRequirements(drive);

        // PID for forward (distance) control
        distanceController = new PIDController(3.0, 0.0, 0.00);
        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);

        // PID for lateral offset correction
        lateralController = new PIDController(2.5, 0.0, 0.5);
        lateralController.setTolerance(LATERAL_TOLERANCE_METERS);

        // PID for rotation to face the desired offset position
        rotationController = new PIDController(0.125, 0.0, 0.005);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180); // angle wrap-around
    }

    @Override
    public void initialize() {
        distanceController.reset();
        lateralController.reset();
        rotationController.reset();
        lateralOffsetInitialized = false;
        
        // Select the camera based on mode and desired lateral offset
        if (isIntake) {
            // In intake mode: use the dedicated intake camera selection.
            // Positive offset means target is to the right so use left intake camera.
            if (desiredLateralOffset > 0) {
                selectedCamera = VisionConstants.BACK_LEFT_CAMERA;
                System.out.println("Intake mode: Using left camera (index " + LEFT_CAMERA_INDEX + ") for right-side approach");
            } else {
                selectedCamera = VisionConstants.BACK_RIGHT_CAMERA;
                System.out.println("Intake mode: Using right camera (index " + RIGHT_CAMERA_INDEX + ") for left-side approach");
            }
            
        } else {
            // In non-intake mode: if there is a lateral offset, use a front camera.
            if (desiredLateralOffset > 0) {
                selectedCamera = VisionConstants.FRONT_LEFT_CAMERA;  // Front camera for right-side approach
                System.out.println("Non-intake mode: Using front camera index 0 for right-side approach");
            } else if (desiredLateralOffset < 0) {
                selectedCamera = VisionConstants.FRONT_RIGHT_CAMERA;  // Front camera for left-side approach
                System.out.println("Non-intake mode: Using front camera index 3 for left-side approach");
            } else {
                selectedCamera = null; // No specific camera selected; use any available detection
                System.out.println("Non-intake mode: No lateral offset specified, using any available camera");
            }
        }

    }

    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        boolean validTagDetection = false;
        
        // Use selected camera if one is set (applies to both intake and non-intake modes)
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
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag(selectedCamera);
            double currentRotation = poseEstimator.getTagOrientationErrorDeg(selectedCamera);

            // Compute corrections using PID controllers
            double lateralSpeed = -lateralController.calculate(currentLateralOffset, desiredLateralOffset);
            double rotationSpeed = -rotationController.calculate(currentRotation, 180);

            // Clamp speeds to maximum limits
            lateralSpeed = Math.min(Math.max(lateralSpeed, -MAX_LATERAL_SPEED), MAX_LATERAL_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            
            // Drive the robot with computed speeds
            drive.driveRobotRelative(new ChassisSpeeds(0.0, lateralSpeed, rotationSpeed));
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
        double currentLateralOffset = 0;
        double currentRotation = 0;
        
        if (selectedCamera != null) {
            int detectedTag = poseEstimator.getLastTagDetectedByCamera(selectedCamera);
            double lastDetectionTime = poseEstimator.getLastCameraDetectionTimestamp(selectedCamera);
            
            if (detectedTag != -1 && (currentTime - lastDetectionTime) < LOST_TAG_TIMEOUT) {
                validTagDetection = true;
                currentLateralOffset = poseEstimator.getLateralOffsetToTag(selectedCamera);
                currentRotation = poseEstimator.getTagOrientationErrorDeg(selectedCamera);
            }
        } else {
            if (poseEstimator.getLastDetectedTagId() != -1) {
                validTagDetection = true;
                currentLateralOffset = poseEstimator.getLateralOffsetToTag(selectedCamera);
                currentRotation = poseEstimator.getTagOrientationErrorDeg(selectedCamera);
            }
        }

        if (validTagDetection) {
            // Check if the distance, lateral offset, and rotation are within tolerance.
            boolean lateralOk = Math.abs(currentLateralOffset - desiredLateralOffset) < LATERAL_TOLERANCE_METERS;
            double rotationError = (currentRotation - 180);
            boolean rotationOk = Math.abs(rotationError) < ROTATION_TOLERANCE_DEG;

            return lateralOk && rotationOk;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("ApproachTagCommand ended. Interrupted: " + interrupted);
    }
}
