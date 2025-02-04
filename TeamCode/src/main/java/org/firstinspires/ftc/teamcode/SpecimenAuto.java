package org.firstinspires.ftc.teamcode;

import android.text.format.Time;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

@Config
@Autonomous(name = "SpecimenAuto", group = "Autonomous")
public class SpecimenAuto extends LinearOpMode {
    //@Config
    // i = intake, iC = intake claw, iW = intake wrist, iA = intake arm
    // d = delivery, dC = delivery claw, dCR = delivery claw right, dCL = delivery claw left, dW = delivery wrist
    public static double iCOpen = 0.7;
    public static double iCClose = 0.55;
    public static double iCAlign = 0.57;
    public static double iWTransferPos = 0.84;
    public static double iWAlteredPos = 0;
    public static double iWAlignmentPos = 0;
    public static double iAUp = 0;
    public static double iADown = 0.64;
    public static double iAReady = 0.5;
    public static double dCOpen = 0.5;
    public static double dCClose = 0.35;
    public static double dWTransfer = 0.95;
    public static double dWDeliverBucket = 0.2;
    public static double dWDeliverSpecimen = 0;
    public static double dWStartPos = 0.5;
    public static int verticalSlidePos;
    public static int horizontalSlidePos;
    public static double BucketDeliverWait = 1.8;
    public static double BarDeliverWait = 1.5;
    public static double PickupWait = 1.7;

    public static double TransferWait = 1;
    public static double RRInitPosX = 5;
    public static double RRInitPosY = 62;
    public static double RRInitPosHeading = 90;
    public static double lineToY1 = 33;
    public static double lineToY2 = 37;
    public static double lineToY3 = 50;
    public static double lineToY4 = 33;
    public static double lineToY5 = 40;
    public static double strafeTo1X = -40;
    public static double strafeTo1Y = 37;
    public static double strafeTo2X = -40;
    public static double strafeTo2Y = 23;
    public static double strafeTo3X = -55;
    public static double strafeTo3Y = 23;
    public static double strafeTo4X = -55;
    public static double strafeTo4Y = 55;
    public static double strafeTo5X = -55;
    public static double strafeTo5Y = 62;
    public static double turnTo1 = 270;
    public static double setTangent2 = 270;
    public static double splineTo1X = -40;
    public static double splineTo1Y = 40;
    public static double splineTo1Heading = 90;
    public static double splineTo1Tangent = 270;
//    public static double splineTo2X = -35;
//    public static double splineTo2Y = 58;
//    public static double splineTo2Heading = 270;
//    public static double splineTo2Tangent = 90;
//    public static double splineTo3X = 2;
//    public static double splineTo3Y = 32;
//    public static double splineTo3Heading = 90;
//    public static double splineTo3Tangent = 270;
    public static int HorizontalExtensionTicks = 210;
    public static int HorizontalRetractionTicks = 2;
    public static double HorizontalMotorSpeed = 400;
    public static double VerticalMotorSpeed = 1200;
    public static int VerticalExtensionTicks = 1400;
    public static int VerticalScoringTicks = 1500;
    public static int VerticalRetractionTicks = 5;
    public static int TransferDelay1 = 250;
    public static int TransferDelay2 = 500;
    public static int PickUpDelay1 = 300;
    public static int PickUpDelay2 = 800;
    public static boolean PickupWithContact = false;
    public static int ForwardTimeForPickup = 400;
    public ProfileAccelConstraint fastMode = new ProfileAccelConstraint(-70,100);
    public ProfileAccelConstraint slowMode = new ProfileAccelConstraint(-20,20);


    // public static int target;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor horizontalSlideMotor;
    private DcMotor verticalSlideMotor;
    private Servo intakeClaw;
    private Servo intakeWrist;
    private Servo intakeArm;
    private Servo deliveryWrist;
    private Servo deliveryClaw;
    public Pose2d RRPos;

    public class VerticalSlide {
        private DcMotorEx verticalSlideMotor;

        public VerticalSlide(HardwareMap hardwareMap) {
            verticalSlideMotor = hardwareMap.get(DcMotorEx.class, "verticalSlideMotor");
            verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            verticalSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class SlideUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    verticalSlideMotor.setTargetPosition(VerticalExtensionTicks);
                    verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    verticalSlideMotor.setVelocity(VerticalMotorSpeed);
                }

                // checks lift's current position
                double vertpos = verticalSlideMotor.getCurrentPosition();
                telemetry.addData("vertliftPos", vertpos);
                telemetry.update();
                if (vertpos < VerticalExtensionTicks) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    /*new Timer().schedule(
                            new java.util.TimerTask() {
                                @Override
                                public void run() {
                                    verticalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);                                }
                            }, 800
                    );*/
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }

        public Action slideUp() {
            return new VerticalSlide.SlideUp();
        }

        public class SlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    verticalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    verticalSlideMotor.setPower(-0.7);
                    initialized = true;
                }

                // checks lift's current position
                double vertpos = verticalSlideMotor.getCurrentPosition();
                telemetry.addData("vertliftPos", vertpos);
                telemetry.update();
                if (vertpos > VerticalRetractionTicks) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    new Timer().schedule(
                            new java.util.TimerTask() {
                                @Override
                                public void run() {
                                    verticalSlideMotor.setPower(0);
                                }
                            }, 300
                    );
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }

        public Action slideDown() {
            return new VerticalSlide.SlideDown();
        }
    }
    public class HorizontalSlide {
        private DcMotorEx horizontalSlideMotor;

        public HorizontalSlide(HardwareMap hardwareMap) {
            horizontalSlideMotor = hardwareMap.get(DcMotorEx.class, "horizontalSlideMotor");
            horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            horizontalSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SlideForward implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    horizontalSlideMotor.setTargetPosition(HorizontalExtensionTicks);
                    horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horizontalSlideMotor.setVelocity(HorizontalMotorSpeed);

                }

                // checks lift's current position
                double horizpos = horizontalSlideMotor.getCurrentPosition();
                telemetry.addData("horizliftPos", horizpos);
                telemetry.update();
                if (horizpos < HorizontalExtensionTicks) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }
        public Action slideForward() {
            return new SlideForward();
        }
        public class SlideBack implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    horizontalSlideMotor.setTargetPosition(HorizontalRetractionTicks);
                    horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horizontalSlideMotor.setVelocity(-HorizontalMotorSpeed);
                    initialized = true;
                }

                // checks lift's current position
                double horizpos = horizontalSlideMotor.getCurrentPosition();
                telemetry.addData("horizliftPos", horizpos);
                telemetry.update();
                if (horizpos > HorizontalRetractionTicks) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    new Timer().schedule(
                            new java.util.TimerTask() {
                                @Override
                                public void run() {
                                    horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);                                }
                            },1000
                    );
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }
        public Action slideBack() {
            return new SlideBack();
        }
    }
    public class DeliverySystem {
        private Servo dC;
        private Servo dW;
        private DcMotorEx verticalSlideMotor;


        public DeliverySystem(HardwareMap hardwareMap) {
            dC = hardwareMap.get(Servo.class, "deliveryClawR");
            dW = hardwareMap.get(Servo.class, "deliveryWrist");
            verticalSlideMotor = hardwareMap.get(DcMotorEx.class, "verticalSlideMotor");
        }
        public class CloseDeliveryClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dC.setPosition(dCClose);
                return false;
            }
        }
        public Action closeDeliveryClaw() {
            return new CloseDeliveryClaw();
        }
        public class OpenDeliveryClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dC.setPosition(dCOpen);
                return false;
            }
        }
        public Action openDeliveryClaw() {
            return new OpenDeliveryClaw();
        }
        public class DeliveryWristBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWDeliverSpecimen);
                return false;
            }
        }
        public Action deliveryWristBack() {
            return new DeliveryWristBack();
        }
        public class DeliveryWristTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWTransfer);
                return false;
            }
        }
        public Action deliveryWristTransfer() {
            return new DeliveryWristTransfer();
        }
        public class DeliveryWristInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWStartPos);
                return false;
            }
        }
        public Action deliveryWristInit() {
            return new DeliveryWristInit();
        }
        public class DeliverToBucket implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWDeliverSpecimen);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                dC.setPosition(dCOpen);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                dW.setPosition(dWTransfer);
                                                new Timer().schedule(
                                                        new java.util.TimerTask() {
                                                            @Override
                                                            public void run() {
                                                                dC.setPosition(dCClose);
                                                                /*new Timer().schedule(
                                                                        new java.util.TimerTask() {
                                                                            @Override
                                                                            public void run() {

                                                                            }
                                                                        },1500
                                                                );*/
                                                            }
                                                        },500
                                                );
                                            }
                                        },500
                                );
                            }
                        },300
                );
                return false;
            }
        }
        public Action deliverToBucket() {
            return new DeliverToBucket();
        }
        public class DeliverToBar implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWDeliverSpecimen);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                verticalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                verticalSlideMotor.setPower(-0.7);
                            }
                        },1100
                );
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                dC.setPosition(dCOpen);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                //dW.setPosition(dWTransfer);
                                                //verticalSlideMotor.setPower(0);
                                                new Timer().schedule(
                                                        new java.util.TimerTask() {
                                                            @Override
                                                            public void run() {
                                                                //dC.setPosition(dCClose);
                                                            }
                                                        },150
                                                );
                                            }
                                        },50
                                );
                            }
                        },1200
                );
                return false;

            }
        }
        public Action deliverToBar() {
            return new DeliverToBar();
        }
    }
    public class IntakeSystem {
        private Servo iC;
        private Servo iW;
        private Servo iA;
        private Servo dC;
        private Servo dW;
        public IntakeSystem(HardwareMap hardwareMap) {
            iC = hardwareMap.get(Servo.class, "intakeClaw");
            iW = hardwareMap.get(Servo.class, "intakeWrist");
            iA = hardwareMap.get(Servo.class, "intakeArm");
            dC = hardwareMap.get(Servo.class, "deliveryClawR");
            dW = hardwareMap.get(Servo.class, "deliveryWrist");

        }
        public class CloseIntakeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iC.setPosition(iCClose);
                return false;
            }
        }
        public Action closeIntakeClaw() {
            return new CloseIntakeClaw();
        }
        public class OpenIntakeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iC.setPosition(iCOpen);
                return false;
            }
        }
        public Action openIntakeClaw() {
            return new OpenIntakeClaw();
        }
        public class IntakeWristCenter implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iW.setPosition(iWTransferPos);
                return false;
            }
        }
        public Action intakeWristCenter() {
            return new IntakeWristCenter();
        }
        public class IntakeWristAlignTask implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iW.setPosition(iWAlignmentPos);
                iC.setPosition(iCClose);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                iC.setPosition(iCAlign);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                iC.setPosition(iCClose);
                                                new Timer().schedule(
                                                        new java.util.TimerTask() {
                                                            @Override
                                                            public void run() {
                                                                iC.setPosition(iCAlign);
                                                                new Timer().schedule(
                                                                        new java.util.TimerTask() {
                                                                            @Override
                                                                            public void run() {
                                                                                iC.setPosition(iCClose);
                                                                                new Timer().schedule(
                                                                                        new java.util.TimerTask() {
                                                                                            @Override
                                                                                            public void run() {
                                                                                                iW.setPosition(iWTransferPos);
                                                                                            }
                                                                                        },100
                                                                                );
                                                                            }
                                                                        },300
                                                                );
                                                            }
                                                        },100
                                                );
                                            }
                                        },300
                                );
                            }
                        },500
                );
                return false;
            }
        }
        public Action intakeWristAlignTask() {
            return new IntakeWristAlignTask();
        }
        public class IntakeArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iA.setPosition(iAUp);
                return false;
            }
        }
        public Action intakeArmUp() {
            return new IntakeArmUp();
        }
        public class IntakeArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iA.setPosition(iADown);
                return false;
            }
        }
        public Action intakeArmDown() {
            return new IntakeArmDown();
        }

        public class TransferToDelivery implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dW.setPosition(dWTransfer);
                dC.setPosition(dCOpen);
                iW.setPosition(iWTransferPos);
                iA.setPosition(iAUp);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                dC.setPosition(dCClose);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                iC.setPosition(iCOpen);
                                            }
                                        },TransferDelay1
                                );
                            }
                        },TransferDelay2
                );
                return false;
            }
        }
        public Action transferToDelivery() {
            return new TransferToDelivery();
        }
        public class PickUpSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                iC.setPosition(iCOpen);
                iA.setPosition(iADown);
                new Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                iC.setPosition(iCClose);
                                new Timer().schedule(
                                        new java.util.TimerTask() {
                                            @Override
                                            public void run() {
                                                if (!PickupWithContact) {
                                                    iA.setPosition(iAUp);
                                                }
                                                if (PickupWithContact) {
                                                    horizontalSlideMotor.setPower(0.3);
                                                    iA.setPosition(iAReady);
                                                    new Timer().schedule(
                                                            new java.util.TimerTask() {
                                                                @Override
                                                                public void run() {
                                                                    horizontalSlideMotor.setPower(0);
                                                                    iA.setPosition(iAUp);
                                                                    new Timer().schedule(
                                                                            new java.util.TimerTask() {
                                                                                @Override
                                                                                public void run() {
                                                                                    /*
                                                                                    iC.setPosition(iCClose);
                                                                                    new Timer().schedule(
                                                                                            new java.util.TimerTask() {
                                                                                                @Override
                                                                                                public void run() {
                                                                                                    iC.setPosition(iCClose);
                                                                                                }
                                                                                            }, 100
                                                                                    );*/
                                                                                }
                                                                            }, 500
                                                                    );
                                                                }
                                                            }, ForwardTimeForPickup
                                                    );
                                                }
                                            }
                                        },PickUpDelay1
                                );
                            }
                        },PickUpDelay2
                );
                return false;
            }
        }
        public Action pickUpSample() {
            return new PickUpSample();
        }

    }
    public class Roadrunner {

        private Pose2d currentPos;

        public class FindPose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                RRPos = currentPos;
                return false;
            }
        }
        public Action findPose(Pose2d CurrentPos) {
            currentPos = CurrentPos;
            return new FindPose();
        }

    }

    public SpecimenAuto() {
    }


    @Override

    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(RRInitPosX,RRInitPosY, Math.toRadians(RRInitPosHeading));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        VerticalSlide verticalSlide = new VerticalSlide(hardwareMap);
        HorizontalSlide horizontalSlide = new HorizontalSlide(hardwareMap);
        DeliverySystem deliverySystem = new DeliverySystem(hardwareMap);
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);
        Roadrunner roadrunner = new Roadrunner();

        initMotors();
        initServos();
        //Initialization Code

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            //Deliver first specimen and back up for reset

            Actions.runBlocking(
                    new SequentialAction (
                            new ParallelAction (
                                    drive.actionBuilder(drive.pose)
                                            .lineToY(lineToY1)
                                            .stopAndAdd(roadrunner.findPose(drive.pose))
                                            .build(),
                                    verticalSlide.slideUp()
                            ),
                            deliverySystem.deliverToBar(),
                            new SleepAction(BarDeliverWait),
                            verticalSlide.slideDown(),
                            drive.actionBuilder(RRPos)
                                    .lineToY(lineToY2)
                                    .stopAndAdd(roadrunner.findPose(drive.pose))
                                    .build(),

                            //Push in sample to human player and pick up second

                            drive.actionBuilder(RRPos)
                                    .strafeToLinearHeading(new Vector2d(strafeTo1X,strafeTo1Y),Math.toRadians(90),null,fastMode)
                                    .stopAndAdd(roadrunner.findPose(drive.pose))
                                    .build(),
                            drive.actionBuilder(RRPos)
                                    .turnTo(Math.toRadians(turnTo1))
                                    .stopAndAdd(roadrunner.findPose(drive.pose))
                                    .build(),
                            drive.actionBuilder(RRPos)
                                    .strafeToLinearHeading(new Vector2d(strafeTo2X,strafeTo2Y),Math.toRadians(270),null,fastMode)
                                    .strafeToLinearHeading(new Vector2d(strafeTo3X,strafeTo3Y),Math.toRadians(270),null,fastMode)
                                    .strafeToLinearHeading(new Vector2d(strafeTo4X,strafeTo4Y),Math.toRadians(270),null,fastMode)
                                    .stopAndAdd(roadrunner.findPose(drive.pose))
                                    .build(),
                            drive.actionBuilder(RRPos)
                                    .strafeTo(new Vector2d(strafeTo5X,strafeTo5Y))
                                    .stopAndAdd(roadrunner.findPose(drive.pose))
                                    .build(),
                            new SleepAction(1)/*,
                            drive.actionBuilder(drive.pose)
                                    .turnTo(Math.toRadians(270))
                                    .lineToY(lineToY3,null,slowMode)
                                    .build(),
                            deliverySystem.closeDeliveryClaw(),

                            //Score second specimen

                            new ParallelAction(
                                    verticalSlide.slideUp(),
                                    drive.actionBuilder(drive.pose)
                                            .setTangent(setTangent2)
                                            .splineToLinearHeading(new Pose2d(splineTo1X,splineTo1Y,Math.toRadians(splineTo1Heading)),Math.toRadians(splineTo1Tangent))
                                            .build()
                            ),
                            drive.actionBuilder(drive.pose)
                                    .lineToY(lineToY4)
                                    .build(),
                            deliverySystem.deliverToBar(),
                            new SleepAction(BarDeliverWait),
                            new ParallelAction (
                                    drive.actionBuilder(drive.pose)
                                            .lineToY(lineToY5)
                                            .build(),
                                    verticalSlide.slideDown()
                            )*/
                    ));

            telemetry.addData("posestimate",drive.pose);
            telemetry.update();
        }
    }

    private void initServos() {
        intakeClaw.setPosition(iCOpen);
        intakeWrist.setPosition(iWTransferPos);
        intakeArm.setPosition(iAUp);
        deliveryWrist.setPosition(dWStartPos);
        deliveryClaw.setPosition(dCClose);
    }
    private void initMotors() {
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        horizontalSlideMotor = hardwareMap.get(DcMotor.class, "horizontalSlideMotor");
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "verticalSlideMotor");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        deliveryWrist = hardwareMap.get(Servo.class, "deliveryWrist");
        deliveryClaw = hardwareMap.get(Servo.class, "deliveryClawR");
        //Reset any nonzero encoder values
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Go to run_without_encoder, to be swapped out later (?)
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Setting other modes: direction and motor braking
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //verticalSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
    }
}
