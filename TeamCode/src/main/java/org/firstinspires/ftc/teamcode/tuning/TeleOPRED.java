package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "RED TeleOP", group = "Drive")
public class TeleOPRED extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake1, intake2;

    // ===== LAUNCHER =====
    private DcMotorEx launcher, launcher2;
    private Servo stopper, ramp2;

    // ===== TURRET =====
    private Servo tur1, tur2;
    private GoBildaPinpointDriver pinpoint;
    LimeLightHelper limelight;

    public static double V1 = 1315;
    public static double V2 = 400;
    public static double V1Y = 1340;
    public static double V2Y = 400;
    public static double V1X = 1365;
    public static double V2X = 400;
    public static double V1A = 1390;
    public static double V2A = 400;

    // ===== LAUNCHER PID 1 =====
    public static double kP1 = 0.007;
    public static double kI1 = 0.0012;
    public static double kD1 = 0.000000001;

    private double integral1 = 0;
    private double lastError1 = 0;
    private long lastTime1 = 0;

    // ===== LAUNCHER PID 2 =====
    public static double kP2 = 0.0009;
    public static double kI2 = 0.00075;
    public static double kD2 = 0.0000001;

    private double integral2 = 0;
    private double lastError2 = 0;
    private long lastTime2 = 0;

    private double targetVelocity1 = 1000;
    private double targetVelocity2 = 1000;
    private boolean rampage;
    public static int LEFT_TAG = 32;
    public static int RIGHT_TAG = 35;
    public void moveRobot() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;

        double rotate = gamepad1.right_stick_x*0.7;

        double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double globalForward = forward * cos - strafe * sin;
        double globalStrafe = forward * sin + strafe * cos;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        normalizew(newWheelSpeeds);

        frontLeft.setPower(newWheelSpeeds[0]);
        frontRight.setPower(newWheelSpeeds[1]);
        backLeft.setPower(newWheelSpeeds[2]);
        backRight.setPower(newWheelSpeeds[3]);
    }

    private void normalizew(double[] speeds) {

        double max = Math.abs(speeds[0]);

        for (double speed : speeds) {
            if (Math.abs(speed) > max)
                max = Math.abs(speed);
        }

        if (max > 1.0) {
            for (int i = 0; i < speeds.length; i++)
                speeds[i] /= max;
        }
    }

    public void closeTrack(int tagId) {

        if (!limelight.hasTag(tagId)) {
            return;
        }

        double tx = limelight.getTxForTag(tagId);

        double deadzone = 0.5;

        if (Math.abs(tx) < deadzone)
            return;

        double k = 0.0006;

        double currentPos = tur1.getPosition();

        double newPos = currentPos - tx * k;

        newPos = Range.clip(newPos, 0.0, 1.0);

        tur1.setPosition(newPos);
        tur2.setPosition(newPos);
    }

    @Override
    public void runOpMode() {

        limelight = new LimeLightHelper(hardwareMap);

        tur1 = hardwareMap.get(Servo.class, "tur1");
        tur2 = hardwareMap.get(Servo.class, "tur2");

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        stopper = hardwareMap.get(Servo.class, "stopper");
        ramp2 = hardwareMap.get(Servo.class, "ramp2");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint.setOffsets(0, 0, DistanceUnit.INCH);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        Pose2D startPose = new Pose2D(
                DistanceUnit.INCH,
                0,
                0,
                AngleUnit.DEGREES,
                0
        );

        pinpoint.setPosition(startPose);

        waitForStart();

        while (opModeIsActive()) {

            pinpoint.update();


            // ===== LAUNCHER 1 PID =====

            double currentVelocity1 = launcher.getVelocity();
            double error1 = targetVelocity1 - currentVelocity1;

            long now1 = System.nanoTime();

            double dt1 = (lastTime1 == 0)
                    ? 0
                    : (now1 - lastTime1) / 1e9;

            lastTime1 = now1;

            integral1 += error1 * dt1;
            integral1 = Range.clip(integral1, -500, 500);

            double derivative1 = (dt1 > 0)
                    ? (error1 - lastError1) / dt1
                    : 0;

            lastError1 = error1;

            double output1 =
                    kP1 * error1 +
                            kI1 * integral1 +
                            kD1 * derivative1;

            output1 = Range.clip(output1, -1.0, 1.0);


            double currentVelocity2 = launcher2.getVelocity();
            double error2 = targetVelocity2 - currentVelocity2;

            long now2 = System.nanoTime();

            double dt2 = (lastTime2 == 0)
                    ? 0
                    : (now2 - lastTime2) / 1e9;

            lastTime2 = now2;

            integral2 += error2 * dt2;
            integral2 = Range.clip(integral2, -500, 500);

            double derivative2 = (dt2 > 0)
                    ? (error2 - lastError2) / dt2
                    : 0;

            lastError2 = error2;

            double output2 =
                    kP2 * error2 +
                            kI2 * integral2 +
                            kD2 * derivative2;

            output2 = Range.clip(output2, -1.0, 1.0);



            launcher.setPower(-output1);
            launcher2.setPower(output2);

            moveRobot();

            if (gamepad1.x) {
                Pose2D reset1 = new Pose2D(
                        DistanceUnit.INCH,
                        0,
                        0,
                        AngleUnit.DEGREES,
                        0
                );

                pinpoint.setPosition(reset1);
            }

            if (gamepad2.right_bumper) {
                rampage = true;
                ramp2.setPosition(0.5);

                if (limelight.hasTarget()) {
                    closeTrack(RIGHT_TAG);
                }

            } else if(gamepad2.left_bumper) {
                rampage = true;
                ramp2.setPosition(0.5);
                if (limelight.hasTarget()) {
                    closeTrack(LEFT_TAG);
                }
            } else if(gamepad2.right_stick_button) {
                ramp2.setPosition(0.7);
            }else {
                rampage = false;
                if(gamepad2.dpad_up){
                    tur1.setPosition(0.5);
                    tur2.setPosition(0.5);
                } else if(gamepad2.dpad_left){
                    tur1.setPosition(0.675);
                    tur2.setPosition(0.675);
                } else if(gamepad2.dpad_right){
                    tur1.setPosition(0.325);
                    tur2.setPosition(0.325);
                }
            }

            if (gamepad1.right_bumper) {
                ramp2.setPosition(0.075);
            } else if (gamepad1.left_bumper) {
                ramp2.setPosition(0.7);
            }

            if (gamepad2.y) {
                targetVelocity1 = V1Y;
                targetVelocity2 = V2Y;
            } else if (gamepad2.b) {
                targetVelocity1 = V1;
                targetVelocity2 = V2;
            } else if (gamepad2.x) {
                targetVelocity1 = V1X;
                targetVelocity2 = V2X;
            } else if (gamepad2.a) {
                targetVelocity1 = V1A;
                targetVelocity2 = V2A;
            }



            if (Math.abs(gamepad2.left_trigger - gamepad2.right_trigger) > 0.1) {

                double p = (gamepad2.left_trigger - gamepad2.right_trigger);

                intake1.setPower(p * 0.8);
                intake2.setPower(-p * 0.8);

                stopper.setPosition(0.8);

            } else {
                double p = (gamepad1.left_trigger - gamepad1.right_trigger);
                intake1.setPower(p);
                intake2.setPower(-p);
                stopper.setPosition(0.98);
            }



            telemetry.addData("Launcher 1 Error", error1);
            telemetry.addData("Launcher 1 Vel", currentVelocity1);
            telemetry.addData("Launcher 1 Target", targetVelocity1);
            telemetry.addData("Launcher 2 Error", error2);
            telemetry.addData("Launcher 2 Vel", currentVelocity2);
            telemetry.addData("Launcher 2 Target", targetVelocity2);
            telemetry.update();
        }
    }
}