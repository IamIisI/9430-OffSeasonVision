// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.ControllerUtils.POV;
import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final SendableChooser<Command> autoChooser;

        // The driver's controller
        public static CommandXboxController c_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        // The operator's controller
        public static CommandXboxController c_operatorController = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // Configure the button bindings
                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);
                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(c_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(c_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(c_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));
        }

        /** Configures NamedCommands for pathplanner */


        /** Represents modes for different controls */
        enum ControlMode {
                /**
                 * Elevator and coral manipulator move but robot does not
                 * approach automatically after d-pad mapping
                 */
                Manual,
                /**
                 * Robot automatically scores / intakes coral after
                 * d-pad mapping
                 */
                SemiAuto;

                /**
                 * Returns if object is Manual
                 * 
                 * @return boolean
                 */
                boolean manual() {
                        return this.equals(Manual);
                }

                /**
                 * Returns if object is SemiAuto
                 * 
                 * @return boolean
                 */
                boolean semiAuto() {
                        return this.equals(SemiAuto);
                }
        }

        Double driverPOVRecency = null;
        POV driverLatestPOVButton = POV.None;

        Double operatorPOVRecency = null;
        POV operatorLatestPOVButton = POV.None;

        ControlMode activeMode = ControlMode.SemiAuto;

        double operatorStartButtonTimestamp = Double.NEGATIVE_INFINITY;

        boolean climbingMode = false;

        /**
         * ~~Use this method to define your button->command mappings. Buttons can be
         * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID}
         * or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
         * {@link XboxController}), and then calling passing it to a
         * {@link JoystickButton}.~~
         * 
         * <p>
         * Binds Commands to Xbox controller buttons using
         * {@link CommandXboxController} methods
         * <p>
         * This method should only be run once by the constructer
         */
        private void configureButtonBindings() {

                /*
                 * * * * * * * * * * * * * *\
                 * *
                 * OPERATOR BUTTON MAPPING *
                 * *
                 * \* * * * * * * * * * * * *
                 */

                 

                // Y button - Toggle Coral Mode
                c_operatorController.y()
                                .onTrue(new InstantCommand(() -> {
                                        
                                }));

                // X button - Algae Reef Clear Mode
                c_operatorController.x()
                                .onTrue(new InstantCommand(() -> {
                                        
                                }));

                // B button - Algae intake mode
                c_operatorController.b()
                                .onTrue(new InstantCommand(() -> {
                                        
                                }));

                // A button -
                c_operatorController.a()
                                .onTrue(new InstantCommand(() -> {
                                        
                                }));

                /*
                 * * * * * * * * * * * * *\
                 * *
                 * DRIVER BUTTON MAPPING *
                 * *
                 * \* * * * * * * * * * * *
                 */

                // Y button - Climbing mode
                c_driverController.y()
                                .onTrue(new InstantCommand(() -> {
                                        
                                }));

                // X button -
                c_driverController.x()
                                .onTrue(new InstantCommand());

                // B button -
                c_driverController.b()
                                .onTrue(new InstantCommand());

                // A button -
                c_driverController.a()
                                .onTrue(new InstantCommand());
                        
                }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
