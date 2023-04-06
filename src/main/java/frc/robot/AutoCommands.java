// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

/** Add your docs here. */
public class AutoCommands {

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Pivot pivot;

    private final SendableChooser<Command> autoChooser;
    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    public AutoCommands(Drivetrain _drivetrain, Intake _intake, Pivot _pivot) {
        drivetrain = _drivetrain;
        intake = _intake;
        pivot = _pivot;

        eventMap.put("Carry", 
            new ParallelCommandGroup(
                pivot.changeState(PivotConstants.State.CARRY),
                intake.changeState(IntakeConstants.State.STOP)
            )
        );

        eventMap.put("FireL1", 
            new SequentialCommandGroup(
                pivot.changeState(PivotConstants.State.L1),
                new WaitUntilCommand(() -> pivot.atTarget()),
                intake.changeState(IntakeConstants.State.L1RELEASE),
                new WaitCommand(0.25)
            )
        );

        eventMap.put("FireL2", 
            new SequentialCommandGroup(
                pivot.changeState(PivotConstants.State.L2),
                new WaitUntilCommand(() -> pivot.atTarget()),
                intake.changeState(IntakeConstants.State.L2RELEASE),
                new WaitCommand(0.25)
            )
        );

        eventMap.put("FireL3", 
            new SequentialCommandGroup(
                pivot.changeState(PivotConstants.State.L3),
                new WaitUntilCommand(() -> pivot.atTarget()),
                intake.changeState(IntakeConstants.State.L3RELEASE),
                new WaitCommand(0.25)
            )
        );

        eventMap.put("FloorPickup", 
            new SequentialCommandGroup(
                intake.changeState(IntakeConstants.State.GRAB),
                pivot.changeState(PivotConstants.State.FLOOR),
                new WaitUntilCommand(() -> pivot.atTarget())
            )
        );

        eventMap.put("SubPickup",
            new SequentialCommandGroup(
                intake.changeState(IntakeConstants.State.GRAB),
                pivot.changeState(PivotConstants.State.SUBSTATION)
            )
        );

        autoChooser = new SendableChooser<>();
        
        autoChooser.addOption("Nothing", new PrintCommand("No auto selected"));

        autoChooser.addOption("L3-Taxi-Sub",
            new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.resetPoseAndGyro(RobotConstants.StartingPose.SUB.pose)),
                eventMap.get("FireL3"),
                eventMap.get("Carry"),
                new RunCommand(() -> drivetrain.setVoltages(1, 1)),
                drivetrain.RotateTo(0)
            )
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getEvent(String event){
        return eventMap.get(event);
    }

    public Command getAuto(){
        return autoChooser.getSelected();
    }

}