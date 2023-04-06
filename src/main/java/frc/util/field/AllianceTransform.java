package frc.util.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AllianceTransform {
 
    public static Pose2d flipAlliance(Pose2d pose){

        double xComp = pose.getX();
        double yComp = pose.getY();
        double rComp = pose.getRotation().getDegrees();

        double outXComp = FieldConstants.fieldLength - xComp;
        double outYComp = yComp;
        double outRComp = rComp + 180;

        return new Pose2d(
            new Translation2d(outXComp, outYComp),
            Rotation2d.fromDegrees(outRComp)
        );

    }

}
