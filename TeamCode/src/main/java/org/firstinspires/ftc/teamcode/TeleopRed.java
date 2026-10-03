package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;



@TeleOp(name = "JazzTimeRed", group = "LinearOpMode")
public class TeleopRed extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor intakeMotor, transferMotor;
    DcMotorEx shooterMotor1, shooterMotor2;
    Servo hood, flowerIntake1, flowerIntake2, funnel1;
    CRServo turret1, turret2;
    DcMotorEx turretEncoder;

    double highVelocity = 2000;
    double mediumVelocity = 1500;
    double lowVelocity = 1000;

    Boolean manualTargeting = true;
    Pose2D goal;

    Pose2D hpRed = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    Pose2D redGoal1 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    Pose2D redGoal2 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);



    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        funnel1 = hardwareMap.get(Servo.class, "funnel1");
        hood = hardwareMap.get(Servo.class, "hood");

        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        flowerIntake1 = hardwareMap.get(Servo.class, "flowerIntake1");
        flowerIntake2 = hardwareMap.get(Servo.class, "flowerIntake2");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0, 0, 0, 0);
        //shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);



        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {

            pinpoint.update();
            drive();
            targetSelect();
            aimTurret();
            //firingSolution();
            manualShooter();

        }

    }



    private void drive() {
        double lfPower = Range.clip(-1 * gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x, -1, 1);
        double rfPower = Range.clip(-1 * gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x, -1, 1);
        double lbPower = Range.clip(-1 * gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x, -1, 1);
        double rbPower = Range.clip(-1 * gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x, -1, 1);

        double precision = 1;
        if (gamepad1.left_bumper) {
            precision = 0.25;
        }

        if (gamepad1.right_bumper) {
            precision = 1;
        }

        leftFront.setPower(lfPower * precision);
        leftBack.setPower(lbPower * precision);
        rightFront.setPower(rfPower * precision);
        rightBack.setPower(rbPower * precision);
    }



    private void manualShooter() {
        if (manualTargeting) {

            if (gamepad2.a) {
                shooterMotor1.setVelocity(lowVelocity);
                shooterMotor2.setVelocity(lowVelocity);
            }

            if (gamepad2.b) {
                shooterMotor1.setVelocity(mediumVelocity);
                shooterMotor2.setVelocity(mediumVelocity);
            }

            if (gamepad2.x) {
                shooterMotor1.setVelocity(highVelocity);
                shooterMotor2.setVelocity(highVelocity);
            }

            if (gamepad2.y) {
                shooterMotor1.setVelocity(0);
                shooterMotor2.setVelocity(0);
            }
        }
    }



private void targetSelect() {

    if (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.x) {
        pinpoint.setPosition(hpRed);
        goal = redGoal1;
    }

    if (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.y) {
        pinpoint.setPosition(hpRed);
        goal = redGoal2;

    }
}



private void aimTurret() {

    if (gamepad2.rightStickButtonWasPressed() && manualTargeting == false) {
        manualTargeting = true;
    }

    if (gamepad2.leftStickButtonWasPressed() && manualTargeting == true) {
        manualTargeting = false;
    }


    if (manualTargeting) {
    int turretPos = turretEncoder.getCurrentPosition();

    if(turretPos%4000 > 0){
        turret1.setPower(-0.25);
        turret2.setPower(-0.25);
    }

    if(turretPos%4000 < 0){
        turret1.setPower(0.25);
        turret2.setPower(0.25);
    }


    }

    if (!manualTargeting) {
        if (goal != null) {
            double deltaY = (goal.getY(DistanceUnit.INCH) - pinpoint.getPosY(DistanceUnit.INCH));
            double deltaX = (goal.getX(DistanceUnit.INCH) - pinpoint.getPosX(DistanceUnit.INCH));

            if (deltaX != 0) {
                double goalHeadingFromXAxis = Math.toDegrees(Math.atan2(deltaY, deltaX));
                double robotHeading = pinpoint.getHeading(AngleUnit.DEGREES);
                double desiredTurretPos = AngleUnit.normalizeDegrees(goalHeadingFromXAxis - robotHeading - 180);

                int turretPosWrapped = ((turretEncoder.getCurrentPosition() % 4000) + 4000) % 4000;
                double turretPosDegrees = turretPosWrapped * (360.0/4000.0);

                if(turretPosDegrees > desiredTurretPos){
                    turret1.setPower(-0.25);
                    turret2.setPower(-0.25);
                }

                if(turretPosDegrees < desiredTurretPos){
                    turret1.setPower(0.25);
                    turret2.setPower(0.25);
                }


            }



        }

    }
}
}
