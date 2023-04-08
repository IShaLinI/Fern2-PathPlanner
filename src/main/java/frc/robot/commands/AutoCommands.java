// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants.StartingPose;
import frc.robot.subsystems.Drivetrain;
import frc.util.field.AllianceTransform;

/** Add your docs here. */
public class AutoCommands {

    private final SendableChooser<Command> autoChooser;
    private final CommandFactory commandFactory;

    public AutoCommands(Drivetrain _drivetrain, CommandFactory _commandFactory) {
        commandFactory = _commandFactory;
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Nothing", new PrintCommand("No auto selected"));

        autoChooser.addOption("Sub-L3-Taxi",
            new SequentialCommandGroup(
                commandFactory.setDrivetrainStartingPose(StartingPose.BLUE_SUB, StartingPose.RED_SUB),
                commandFactory.getFireL3(),
                commandFactory.getCarry(),
                commandFactory.getRotateAbsolute(0),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(3.75,-3.75)),
                commandFactory.getRotateAbsolute(AllianceTransform.allianceBasedDouble(45,-45)),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(0.85, -0.85)),
                commandFactory.getFloorPickup(),
                commandFactory.getCarry()
            )
        );

        autoChooser.addOption("Mid-L3-Charge", 
            new SequentialCommandGroup(
                commandFactory.setDrivetrainStartingPose(StartingPose.BLUE_MID, StartingPose.RED_MID),
                commandFactory.getFireL3(),
                commandFactory.getCarry(),
                commandFactory.getRotateAbsolute(AllianceTransform.allianceBasedDouble(-180,180)),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(-4,4), 2),
                commandFactory.getDriveMeters(AllianceTransform.allianceBasedDouble(2.5,-2.5),1),
                commandFactory.getCharge()
            )
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAuto(){
        return autoChooser.getSelected();
    }

}