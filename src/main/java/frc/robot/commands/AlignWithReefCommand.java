// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithReefCommand extends Command {

  private final DriveSubsystem drive;
  private final PoseEstimatorSubsystem pose;
  private final PhotonCamera aligningCamera;

  private PIDController yController = new PIDController(2, 0, 0);
  private PIDController xController = new PIDController(3, 0, 0);
  private PIDController rController = new PIDController(0.001, 0, 0.01);
  

  /** Creates a new AlignWithReefCommand. */
  public AlignWithReefCommand(DriveSubsystem drive, PhotonCamera aligningCamera) {
    addRequirements(drive);
    this.drive = drive;
    this.pose = drive.getPoseEstimatorSubsystem();
    this.aligningCamera = aligningCamera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    rController.setTolerance(5);

    xController.setSetpoint(0.5);
    yController.setSetpoint(0);
    rController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yError = pose.getYOffsetToTag(aligningCamera);
    System.out.println(yError);

    SmartDashboard.putNumber("yError", yError);

    double xError = pose.getXOffsetToTag(aligningCamera);
    System.out.println(xError);
    
    SmartDashboard.putNumber("xError", xError);

    double rError = pose.getBearingToTagDeg(aligningCamera);
    System.out.println(rError);
    
    SmartDashboard.putNumber("rError", rError);


    double ySpeed = yController.calculate(yError);
    double xSpeed = xController.calculate(xError);
    double rSpeed = rController.calculate(rError);

    drive.driveRobotRelative(new ChassisSpeeds(-xSpeed, -ySpeed, rSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && Math.abs(pose.getBearingToTagDeg(aligningCamera)) > 179;
  }
}
