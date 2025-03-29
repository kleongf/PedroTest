package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "left_front";
        FollowerConstants.leftRearMotorName = "left_back";
        FollowerConstants.rightFrontMotorName = "right_front";
        FollowerConstants.rightRearMotorName = "right_back";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        // get mass in kg (REQUIRED)
        // increased by 2.5 lbs (1.1kg) because of counterweight (was 8)
        FollowerConstants.mass = 9.2;

        FollowerConstants.xMovement = 78;// 74.3369; // 78
        FollowerConstants.yMovement = 63;// 53.2528; // 63

        FollowerConstants.forwardZeroPowerAcceleration = -65; //-54.7083; // -65
        FollowerConstants.lateralZeroPowerAcceleration = -83; //-70.7978; // -83

        // WE ARE HERE


        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.07,0,0.005,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        // p0.9, d0.2
        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0.04,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.00001,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        // higher: more oscillation, faster breaking: default was 4
        FollowerConstants.zeroPowerAccelerationMultiplier = 2;
        FollowerConstants.centripetalScaling = 0.001;

        // was 500 but im tryna reduce oscillation at end
        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
