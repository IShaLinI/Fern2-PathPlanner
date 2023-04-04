// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

/** Add your docs here. */
public class AutoCommands {

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Pivot pivot;

    private final RamseteAutoBuilder autoBuilder;
    private final SendableChooser<Command> autoChooser;
    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    public AutoCommands(Drivetrain _drivetrain, Intake _intake, Pivot _pivot) {
        drivetrain = _drivetrain;
        intake = _intake;
        pivot = _pivot;

        eventMap.put("FireL1", 
            new SequentialCommandGroup(
                pivot.changeState(PivotConstants.State.L1),
                new WaitCommand(0.5),
                intake.changeState(IntakeConstants.State.L1RELEASE)
            )
        );

        eventMap.put("FireL2", 
            new SequentialCommandGroup(
                pivot.changeState(PivotConstants.State.L2),
                new WaitCommand(0.5),
                intake.changeState(IntakeConstants.State.L2RELEASE)
            )
        );

        autoBuilder = new RamseteAutoBuilder(
            drivetrain::getRobotPosition,
            drivetrain::resetPoseAndGyro,
            new RamseteController(),
            drivetrain.getKinematics(),
            drivetrain.getFeedforward(),
            drivetrain::getWheelVelocities,
            DriveConstants.mTrajConstants,
            drivetrain::setVoltages,
            eventMap,
            drivetrain
        );

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Nothing", new PrintCommand("No auto selected"));
        autoChooser.addOption("Test", makeAuto("Test"));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAuto(){
        return autoChooser.getSelected();
    }

    private Command makeAuto(String path) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, DriveConstants.kTrajectoryMaxSpeed, DriveConstants.kTrajectoryMaxAccel));
    }

}