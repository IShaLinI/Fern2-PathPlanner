package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class CommandFactory {
    
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Pivot pivot;

    public CommandFactory(Drivetrain _drivetrain, Intake _intake, Pivot _pivot) {
        
        drivetrain = _drivetrain;
        intake = _intake;
        pivot = _pivot;

    }

    public Command getCarry(){
        return new SequentialCommandGroup(
            pivot.changeState(PivotConstants.State.CARRY),
            intake.changeState(IntakeConstants.State.STOP)
        );
    }

    public Command getFireL1(){
        return new SequentialCommandGroup(
            pivot.changeState(PivotConstants.State.L1),
            new WaitUntilCommand(() -> pivot.atTarget()),
            intake.changeState(IntakeConstants.State.L1RELEASE),
            new WaitCommand(0.25)
        );
    }

    public Command getFireL2(){
        return new SequentialCommandGroup(
            pivot.changeState(PivotConstants.State.L2),
            new WaitUntilCommand(() -> pivot.atTarget()),
            intake.changeState(IntakeConstants.State.L2RELEASE),
            new WaitCommand(0.25)
        );
    }

    public Command getFireL3(){
        return new SequentialCommandGroup(
            pivot.changeState(PivotConstants.State.L3),
            new WaitUntilCommand(() -> pivot.atTarget()),
            intake.changeState(IntakeConstants.State.L3RELEASE),
            new WaitCommand(0.25)
        );
    }

    public Command getFloorPickup(){
        return new SequentialCommandGroup(
            intake.changeState(IntakeConstants.State.GRAB),
            pivot.changeState(PivotConstants.State.FLOOR),
            new WaitUntilCommand(() -> pivot.atTarget())
        );
    }

    public Command getSubPickup(){
        return new SequentialCommandGroup(
            intake.changeState(IntakeConstants.State.GRAB),
            pivot.changeState(PivotConstants.State.SUBSTATION)
        );
    }

    public Command getRotateRelative(DoubleSupplier angleSupplier){
        return drivetrain.new RotateRelative(angleSupplier);
    }

    public Command getRotateRelative(double angle){
        return drivetrain.new RotateRelative(angle);
    }

    public Command getRotateAbsolute(DoubleSupplier angleSupplier){
        return drivetrain.new RotateAbsolute(angleSupplier);
    }

    public Command getRotateAbsolute(double angle){
        return drivetrain.new RotateAbsolute(angle);
    }

    public Command getDriveMeters(DoubleSupplier distanceSupplier){
        return drivetrain.new DriveMeters(distanceSupplier);
    }

    public Command getDriveMeters(double distance){
        return drivetrain.new DriveMeters(distance);
    }

    public Command getDriveMeters(DoubleSupplier distanceSupplier, double maxSpeed){
        return drivetrain.new DriveMeters(distanceSupplier, maxSpeed);
    }

    public Command getDriveMeters(double distance, double maxSpeed){
        return drivetrain.new DriveMeters(distance, maxSpeed);
    }

    public Command setDrivetrainStartingPose(RobotConstants.StartingPose bluePose, RobotConstants.StartingPose redPose){
        return new InstantCommand(
            () -> drivetrain.resetPoseAndGyro(
                DriverStation.getAlliance() == Alliance.Blue ? bluePose.pose : redPose.pose
            ),
            drivetrain
        );
    }

    public Command getCharge(){
        return drivetrain.new Charge();
    }

}
