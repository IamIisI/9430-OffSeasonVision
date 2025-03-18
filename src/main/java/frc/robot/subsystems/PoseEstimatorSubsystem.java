// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private CameraDetection FRONT_LEFT_DETECTION;
  private CameraDetection FRONT_RIGHT_DETECTION;
  private CameraDetection BACK_LEFT_DETECTION;
  private CameraDetection BACK_RIGHT_DETECTION;

  private final SwerveDriveOdometry poseEstimator;
  private Rotation2d currentHeading;
  
    /** Creates a new PoseEstimatorSubsystem. */
    public PoseEstimatorSubsystem(Pose2d initialPose) {
      poseEstimator = new SwerveDriveOdometry(
                Constants.DriveConstants.kDriveKinematics,
                initialPose.getRotation(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                });

        
        /*
         * FRONT LEFT CAMERA
         */

         // Create and store detection info for this camera.
         CameraDetection FL_detection = new CameraDetection(
           0,
           0,
           new Rotation2d(),
           0,
           0,
           0,
           0,
           0,
           0,
           VisionConstants.FRONT_LEFT_CAMERA);
 
         FRONT_LEFT_DETECTION = FL_detection;
         logCameraDetection(FRONT_LEFT_DETECTION);
 
         /*
          * FRONT RIGHT CAMERA
          */

         // Create and store detection info for this camera.
         CameraDetection FR_detection = new CameraDetection(
           0,
           0,
           new Rotation2d(),
           0,
           0,
           0,
           0,
           0,
           0,
           VisionConstants.FRONT_RIGHT_CAMERA);
 
         FRONT_RIGHT_DETECTION = FR_detection;
         logCameraDetection(FRONT_RIGHT_DETECTION);
         /*
          * BACK LEFT CAMERA
          */

         // Create and store detection info for this camera.
         CameraDetection BL_detection = new CameraDetection(
           0,
           0,
           new Rotation2d(),
           0,
           0,
           0,
           0,
           0,
           0,
           VisionConstants.BACK_LEFT_CAMERA);
 
         BACK_LEFT_DETECTION = BL_detection;
         logCameraDetection(BACK_LEFT_DETECTION);
 
         /*
          * BACK RIGHT CAMERA
          */
 
         // Create and store detection info for this camera.
         CameraDetection BR_detection = new CameraDetection(
           0,
           0,
           new Rotation2d(),
           0,
           0,
           0,
           0,
           0,
           0,
           VisionConstants.BACK_RIGHT_CAMERA);
 
         BACK_RIGHT_DETECTION = BR_detection;
         logCameraDetection(BACK_RIGHT_DETECTION);
    }
  
    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        // Store current heading for use in coordinate transformations
        this.currentHeading = gyroRotation;

        // Update pose estimator with odometry
        poseEstimator.update(gyroRotation, modulePositions);

        /*
         * FRONT LEFT CAMERA
         */

        PhotonPipelineResult FL_result = VisionConstants.FRONT_LEFT_CAMERA.getLatestResult();
        PhotonTrackedTarget FL_bestTarget = FL_result.getBestTarget();
        Transform3d FL_cameraToTarget = (FL_bestTarget != null)? FL_bestTarget.getBestCameraToTarget() : new Transform3d();
        Transform3d FL_robotToTarget = VisionConstants.FRONT_LEFT_CAMERA_LOCATION.plus(FL_cameraToTarget);
        Translation3d FL_translation = FL_robotToTarget.getTranslation();
        Rotation3d FL_rotation = FL_robotToTarget.getRotation();

        // Create and store detection info for this camera.
        CameraDetection FL_detection = new CameraDetection(
          (FL_bestTarget != null)? FL_bestTarget.getFiducialId() : -1,
          FL_result.getTimestampSeconds(),
          gyroRotation,
          FL_translation.getMeasureX().in(Meter),
          FL_rotation.getMeasureZ().in(Degree),
          FL_translation.getMeasureY().in(Meter),
          FL_cameraToTarget.getTranslation().getX(),
          FL_cameraToTarget.getTranslation().getY(),
          FL_rotation.getMeasureZ().in(Degree),
          VisionConstants.FRONT_LEFT_CAMERA);

        FRONT_LEFT_DETECTION = FL_detection;
        logCameraDetection(FRONT_LEFT_DETECTION);

        /*
         * FRONT RIGHT CAMERA
         */

        PhotonPipelineResult FR_result = VisionConstants.FRONT_RIGHT_CAMERA.getLatestResult();
        PhotonTrackedTarget FR_bestTarget = FR_result.getBestTarget();
        Transform3d FR_cameraToTarget = (FR_bestTarget != null)? FR_bestTarget.getBestCameraToTarget() : new Transform3d();
        Transform3d FR_robotToTarget = VisionConstants.FRONT_RIGHT_CAMERA_LOCATION.plus(FR_cameraToTarget);
        Translation3d FR_translation = FR_robotToTarget.getTranslation();
        Rotation3d FR_rotation = FR_robotToTarget.getRotation();

        // Create and store detection info for this camera.
        CameraDetection FR_detection = new CameraDetection(
          (FR_bestTarget != null)? FR_bestTarget.getFiducialId() : -1,
          FR_result.getTimestampSeconds(),
          gyroRotation,
          FR_translation.getMeasureX().in(Meter),
          FR_rotation.getMeasureZ().in(Degree),
          FR_translation.getMeasureY().in(Meter),
          FR_cameraToTarget.getTranslation().getX(),
          FR_cameraToTarget.getTranslation().getY(),
          FR_rotation.getMeasureZ().in(Degree),
          VisionConstants.FRONT_RIGHT_CAMERA);

        FRONT_RIGHT_DETECTION = FR_detection;
        logCameraDetection(FRONT_RIGHT_DETECTION);
        
        /*
         * BACK LEFT CAMERA
         */

        PhotonPipelineResult BL_result = VisionConstants.BACK_LEFT_CAMERA.getLatestResult();
        PhotonTrackedTarget BL_bestTarget = BL_result.getBestTarget();
        Transform3d BL_cameraToTarget = (BL_bestTarget != null)? BL_bestTarget.getBestCameraToTarget() : new Transform3d();
        Transform3d BL_robotToTarget = VisionConstants.BACK_LEFT_CAMERA_LOCATION.plus(BL_cameraToTarget);
        Translation3d BL_translation = BL_robotToTarget.getTranslation();
        Rotation3d BL_rotation = BL_robotToTarget.getRotation();

        // Create and store detection info for this camera.
        CameraDetection BL_detection = new CameraDetection(
          (BL_bestTarget != null)? BL_bestTarget.getFiducialId() : -1,
          BL_result.getTimestampSeconds(),
          gyroRotation,
          BL_translation.getMeasureX().in(Meter),
          BL_rotation.getMeasureZ().in(Degree),
          BL_translation.getMeasureY().in(Meter),
          BL_cameraToTarget.getTranslation().getX(),
          BL_cameraToTarget.getTranslation().getY(),
          BL_rotation.getMeasureZ().in(Degree),
          VisionConstants.BACK_LEFT_CAMERA);

        BACK_LEFT_DETECTION = BL_detection;
        logCameraDetection(BACK_LEFT_DETECTION);

        /*
         * BACK RIGHT CAMERA
         */

        PhotonPipelineResult BR_result = VisionConstants.BACK_RIGHT_CAMERA.getLatestResult();
        PhotonTrackedTarget BR_bestTarget = BR_result.getBestTarget();
        Transform3d BR_cameraToTarget = (BR_bestTarget != null)? BR_bestTarget.getBestCameraToTarget() : new Transform3d();
        Transform3d BR_robotToTarget = VisionConstants.BACK_RIGHT_CAMERA_LOCATION.plus(BR_cameraToTarget);
        Translation3d BR_translation = BR_robotToTarget.getTranslation();
        Rotation3d BR_rotation = BR_robotToTarget.getRotation();

        // Create and store detection info for this camera.
        CameraDetection BR_detection = new CameraDetection(
          (BR_bestTarget != null)? BR_bestTarget.getFiducialId() : -1,
          BR_result.getTimestampSeconds(),
          gyroRotation,
          BR_translation.getMeasureX().in(Meter),
          BR_rotation.getMeasureZ().in(Degree),
          BR_translation.getMeasureY().in(Meter),
          BR_cameraToTarget.getTranslation().getX(),
          BR_cameraToTarget.getTranslation().getY(),
          BR_rotation.getMeasureZ().in(Degree),
          VisionConstants.BACK_RIGHT_CAMERA);

        BACK_RIGHT_DETECTION = BR_detection;
        logCameraDetection(BACK_RIGHT_DETECTION);
    }

    private void logCameraDetection(CameraDetection detection) {
      String cameraName = detection.camera.getName();
      // Update dashboard with camera-specific info.
      SmartDashboard.putBoolean("Camera " + cameraName + " Has Detection", true);
      SmartDashboard.putNumber("Camera " + cameraName + " Last Detection Time", detection.detectionTimestamp);
      SmartDashboard.putNumber("Camera " + cameraName + " Last Tag ID", detection.tagId);
      SmartDashboard.putNumber("Camera " + cameraName + " DistanceToTag", detection.distanceToTag);
      SmartDashboard.putNumber("Camera " + cameraName + " Tag Bearing (Deg)", detection.bearingToTagDeg);
      SmartDashboard.putNumber("Camera " + cameraName + " Tag Orientation Error (Deg)", detection.tagOrientationErrorDeg);
      SmartDashboard.putNumber("Camera " + cameraName + " LateralOffsetToTag", detection.lateralOffsetToTag);
      SmartDashboard.putNumber("Camera " + cameraName + " xOffsetToTag", detection.xOffsetToTag);
      SmartDashboard.putNumber("Camera " + cameraName + " yOffsetToTag", detection.yOffsetToTag);
    }

    // Helper method to get the most recent detection across all cameras.
    private CameraDetection getMostRecentDetection() {
      CameraDetection[] detections = {
        FRONT_LEFT_DETECTION,
        FRONT_RIGHT_DETECTION,
        BACK_LEFT_DETECTION,
        BACK_RIGHT_DETECTION
      };
      CameraDetection mostRecent = FRONT_LEFT_DETECTION;
      for (CameraDetection detection : detections) {
          if (detection != null) {
              if (mostRecent == null || detection.detectionTimestamp > mostRecent.detectionTimestamp) {
                  mostRecent = detection;
              }
          }
      }
      return mostRecent;
  }

  private CameraDetection getDetectionOf(PhotonCamera camera) {

    switch (camera.getName()) {
        case VisionConstants.FL_CAMERA_NAME:
            return FRONT_LEFT_DETECTION;
    
        case VisionConstants.FR_CAMERA_NAME:
            return FRONT_RIGHT_DETECTION;
    
        case VisionConstants.BL_CAMERA_NAME:
            return BACK_LEFT_DETECTION;

        case VisionConstants.BR_CAMERA_NAME:
            return BACK_RIGHT_DETECTION;

        default:
            return null;
            
    }
        
  }

    // Global getters (aggregated using the most recent detection)
    public int getLastDetectedTagId() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.tagId : -1;
    }

    public PhotonCamera getLastDetectionCamera() {
        CameraDetection detection = getMostRecentDetection();
        return detection.camera;
    }

    public double getLastDetectionTimestamp() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.detectionTimestamp : -1.0;
    }

    /**
     * Gets the distance to the tag for a specified camera index.
     * 
     * @param cameraIndex The index of the camera.
     * @return The distance to the tag in meters, or NaN if no detection exists.
     */
    public double getDistanceToTag(PhotonCamera camera) {
        CameraDetection detection = getDetectionOf(camera);
        return detection != null ? detection.distanceToTag : Double.NaN;
    }

    /**
     * Gets the lateral offset to the tag for a specified camera index.
     * 
     * @param cameraIndex The index of the camera.
     * @return The lateral offset in meters, or NaN if no detection exists.
     */
    public double getLateralOffsetToTag(PhotonCamera camera) {
        CameraDetection detection = getDetectionOf(camera);
        return detection != null ? detection.lateralOffsetToTag : Double.NaN;
    }

    /**
     * Angle from robot's forward axis to the tag's position. Positive means tag is
     * to the left.
     */
    public double getBearingToTagDeg() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.bearingToTagDeg : Double.NaN;
    }

    public double getBearingToTagDeg(PhotonCamera camera) {
        CameraDetection detection = getDetectionOf(camera);
        return detection != null ? detection.bearingToTagDeg : Double.NaN;
    }

    /**
     * The tag's orientation relative to the robot. 0° means tag and robot are
     * parallel.
     */
    public double getTagOrientationErrorDeg() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.tagOrientationErrorDeg : Double.NaN;
    }

    public double getTagOrientationErrorDeg(PhotonCamera camera) {
        CameraDetection detection = getDetectionOf(camera);
        return detection != null ? detection.tagOrientationErrorDeg : Double.NaN;
    }

    /**
     * Gets the X offset to the tag from the most recent detection.
     * 
     * @return The X offset in meters.
     */
    public double getXOffsetToTag() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.xOffsetToTag : Double.NaN;
    }

    public double getXOffsetToTag(PhotonCamera camera) {
        CameraDetection detection = getDetectionOf(camera);
        return detection != null ? detection.xOffsetToTag : Double.NaN;
    }

    /**
     * Gets the X offset to the tag from the most recent detection.
     * 
     * @return The X offset in meters.
     */
    public double getYOffsetToTag() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.yOffsetToTag : Double.NaN;
    }

    public double getYOffsetToTag(PhotonCamera camera) {
        CameraDetection detection = getDetectionOf(camera);
        return detection != null ? detection.yOffsetToTag : Double.NaN;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getPoseMeters();
    }

    public void resetPose(Pose2d newPose, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroRotation, modulePositions, newPose);
    }

    /**
     * Checks if any side camera (left or right) has detected a tag for intake
     * operations.
     * 
     * @return true if either side camera has a recent detection.
     */
    public boolean hasSideCameraDetection() {
        return 
            hasCameraDetectedTag(VisionConstants.BACK_LEFT_CAMERA) || 
            hasCameraDetectedTag(VisionConstants.BACK_RIGHT_CAMERA);
    }

    /**
     * Checks if any front camera (indices 0 or 3) has detected a tag.
     * 
     * @return true if either front camera has a recent detection.
     */
    public boolean hasFrontCameraDetection() {
        return 
            hasCameraDetectedTag(VisionConstants.FRONT_LEFT_CAMERA) || 
            hasCameraDetectedTag(VisionConstants.FRONT_RIGHT_CAMERA);
    }

    /**
     * Gets the most recent side camera detection information.
     * 
     * @return The index of the side camera with the most recent detection, or -1 if
     *         no detection.
     */
    public CameraDetection getLastSideCameraDetection() {
        CameraDetection leftDetection = BACK_LEFT_DETECTION;
        CameraDetection rightDetection = BACK_RIGHT_DETECTION;

        if (leftDetection == null && rightDetection == null) {
            return null;
        }

        if (leftDetection != null && rightDetection != null) {
            return (leftDetection.detectionTimestamp > rightDetection.detectionTimestamp) ? BACK_LEFT_DETECTION
                    : BACK_RIGHT_DETECTION;
        }

        return leftDetection != null ? BACK_LEFT_DETECTION : BACK_RIGHT_DETECTION;
    }

    /**
     * Checks whether a specific camera has detected a tag recently.
     * 
     * @param camera The camera to check.
     * @return true if the camera has a detection within the last 0.6 seconds.
     */
    public boolean hasCameraDetectedTag(PhotonCamera camera) {
        double currentTime = Timer.getFPGATimestamp();
        CameraDetection detection = getDetectionOf(camera);
        return detection != null && (currentTime - detection.detectionTimestamp) < 0.6;
    }

    /**
     * Gets the timestamp of the last detection for a specific camera.
     * 
     * @param cameraIndex The index of the camera.
     * @return The timestamp of the last detection, or -1 if none.
     */
    public double getLastCameraDetectionTimestamp(PhotonCamera camera) {
        CameraDetection detection = getDetectionOf(camera);
        return detection != null ? detection.detectionTimestamp : -1.0;
    }

    /**
     * Gets the ID of the last tag detected by a specific camera.
     * 
     * @param cameraIndex The index of the camera.
     * @return The tag ID, or -1 if none.
     */
    public int getLastTagDetectedByCamera(PhotonCamera camera) {
        CameraDetection detection = getDetectionOf(camera);
        return detection != null ? detection.tagId : -1;
    }

    /**
     * Transforms robot-relative coordinates to field-relative coordinates.
     * 
     * @param robotRelative The point in robot-relative coordinates.
     * @return The point in field-relative coordinates.
     */
    public Translation2d robotToFieldCoordinates(Translation2d robotRelative) {
        return robotRelative.rotateBy(currentHeading);
    }

    /**
     * Transforms field-relative coordinates to robot-relative coordinates.
     * 
     * @param fieldRelative The point in field-relative coordinates.
     * @return The point in robot-relative coordinates.
     */
    public Translation2d fieldToRobotCoordinates(Translation2d fieldRelative) {
        return fieldRelative.rotateBy(currentHeading.unaryMinus());
    }

    /**
     * Gets the lateral offset to the tag, adjusted for the robot's current
     * orientation.
     * This ensures the lateral offset is consistent regardless of the robot's
     * heading.
     * 
     * @return The adjusted lateral offset in meters.
     */
    public double getAdjustedLateralOffsetToTag() {
        CameraDetection detection = getMostRecentDetection();
        if (detection == null) {
            return Double.NaN;
        }

        // Calculate the heading change since the detection was recorded.
        Rotation2d headingChange = currentHeading.minus(detection.detectionHeading);

        // Create a position vector for the tag as seen at the time of detection.
        Translation2d tagPositionAtDetection = new Translation2d(detection.xOffsetToTag, detection.yOffsetToTag);

        // Rotate the vector by the heading change.
        Translation2d adjustedTagPosition = tagPositionAtDetection.rotateBy(headingChange);

        // Return the Y component (lateral offset).
        return adjustedTagPosition.getY();
    }

    public static double normalizeDegrees(double degrees) {
        degrees = degrees % 360;
        return degrees < 0 ? degrees + 360 : degrees;
    }



  // A helper class to hold detection information per camera.
  private static class CameraDetection {
      int tagId;
      double detectionTimestamp;
      Rotation2d detectionHeading;
      double distanceToTag;
      double bearingToTagDeg;
      double lateralOffsetToTag;
      double xOffsetToTag;
      double yOffsetToTag;
      double tagOrientationErrorDeg;
      PhotonCamera camera;

      public CameraDetection(int tagId, double detectionTimestamp, Rotation2d detectionHeading,
              double distanceToTag, double bearingToTagDeg, double lateralOffsetToTag,
              double xOffsetToTag, double yOffsetToTag, double tagOrientationErrorDeg,
              PhotonCamera camera) {
          this.tagId = tagId;
          this.detectionTimestamp = detectionTimestamp;
          this.detectionHeading = detectionHeading;
          this.distanceToTag = distanceToTag;
          this.bearingToTagDeg = bearingToTagDeg;
          this.lateralOffsetToTag = lateralOffsetToTag;
          this.xOffsetToTag = xOffsetToTag;
          this.yOffsetToTag = yOffsetToTag;
          this.tagOrientationErrorDeg = tagOrientationErrorDeg;
          this.camera = camera;
      }
  }
}
