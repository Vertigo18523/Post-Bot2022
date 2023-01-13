package org.firstinspires.ftc.teamcode.Components;

import com.vuforia.State;

import org.firstinspires.ftc.teamcode.Base.Component;
import org.firstinspires.ftc.teamcode.Utils.PursuitPoint;
import org.firstinspires.ftc.teamcode.RMath.Circle;
import org.firstinspires.ftc.teamcode.RMath.Point;
import org.firstinspires.ftc.teamcode.RMath.Segment;
import org.firstinspires.ftc.teamcode.RMath.Util;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.List;

public class PurePursuit implements Component {

    private PIDMecanum drivetrain;
    private Odometry odometry;
    public List<PursuitPoint> path = new ArrayList<>();
    private State state = State.IDLE;
    private double lookahead = 5;

    public double minStraightDist = 20;

    public double rotationTolerance = 5;
    public double rotationDeadZone = 4;

    private String telemetry = "";

    public boolean isBusy = false;

    public PurePursuit(PIDMecanum drivetrain, Odometry odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
    }

    //    There is going to be math here to get the 2D point that the robot has to follow
    public FollowPoint getFollowPoint() {

        PursuitPoint current = path.get(0);
        PursuitPoint next = path.get(path.size() > 1 ? 1 : 0);

        telemetry += "\nlength of path: " + path.size();

        if (path.size() < 2) state = State.MAINTAINING;

        switch (state) {
            case MOVING: {
                isBusy = true;
                telemetry += "\nstate: moving";
                telemetry += "\nnext point: (" + next.x + ", " + next.y + ")";
                telemetry += "\nposition: " + odometry.getPosition().toString();

                boolean withinLookahead = Util.dist(odometry.getPosition(), next.getPosition()) < lookahead;
                boolean noActions = next.actions.isEmpty();
                boolean withinRotationTolerance = next.rotation == null || Math.abs(getAngleDiff(odometry.getRotation(), next.rotation)) < rotationTolerance;

//                telemetry += "\nwithinLookahead: " + withinLookahead;
//                telemetry += "\nnoActions: " + noActions;
//                telemetry += "\nwithinRotationTolerance: " + withinRotationTolerance;

                telemetry += "\nlookahead: " + lookahead;

                if (withinLookahead && noActions && withinRotationTolerance) {
                    path.remove(0);
                    telemetry += "\nrecursively returned";
                    return getFollowPoint();
                }

//                This means that there is still stuff that the robot needs to do,
//                whether it is completing an action or rotating to the right heading
                if (withinLookahead) {
                    state = State.MAINTAINING;
                    telemetry += "\nrecursively returned";
                    return getFollowPoint();
                }

                double rotation;

                double distToNextRotPoint = Util.dist(odometry.getPosition(), next.getPosition());
                Double nextRot = null;
                boolean upcomingRotPoint = false;

                for (int i = 1; i < path.size() - 1; i++) {

                    if (path.get(i).rotation != null) {
                        upcomingRotPoint = true;
                        nextRot = path.get(i).rotation;
                        break;
                    }

                    distToNextRotPoint += Util.dist(path.get(i).getPosition(), path.get(i + 1).getPosition());
                }

//                telemetry += "\ndist to next rot point: " + distToNextRotPoint;
//                telemetry += "\nupcoming rot point? " + upcomingRotPoint;

//                if(path.get(path.size() - 1).rotation != null) {
//                    upcomingRotPoint = true;
//                    nextRot = path.get(path.size() - 1).rotation;
//                }

                Segment segment = new Segment(current.getPosition(), next.getPosition());
                Point[] intersection = Util.getIntersection(new Circle(odometry.getPosition(), lookahead), segment);
                Point closest = odometry.getPosition().closestPoint(segment);

                Point followPoint;
                if (Util.dist(closest, odometry.getPosition()) > lookahead) {
                    followPoint = closest;
                } else if (intersection.length == 1 || Util.dist(intersection[0], next.getPosition()) < Util.dist(intersection[1], next.getPosition())) {
                    followPoint = intersection[0];
                } else {
                    followPoint = intersection[1];
                }

//                If the robot needs to be at a certain rotation for an upcoming point, and it is close to that point,
//                rotated to its rotation
                if (distToNextRotPoint < minStraightDist && upcomingRotPoint) {
                    rotation = nextRot;
                } else {
                    final double followRot;
                    if(Util.dist(odometry.getPosition(), followPoint) < rotationDeadZone){
                        followRot = Math.toDegrees(Util.angle(current.getPosition(), next.getPosition()));
                    }else{
                        followRot = Math.toDegrees((Util.angle(odometry.getPosition(), followPoint)));
                    }

                    final double forward = followRot - 90;
                    final double backward = followRot + 90;

                    rotation = Math.abs(getAngleDiff(odometry.getRotation(), forward)) < 90 ? forward : backward;
                }

                telemetry += "\ncurrent rotation: " + current;
                telemetry += "\ntarget rotation: " + rotation;

                return new FollowPoint(followPoint.x, followPoint.y, rotation);
            }

            case MAINTAINING: {
                double rotation;

                telemetry += "\nstate: maintaining";

                if (next.rotation != null) {
                    rotation = next.rotation;
                } else {
                    if (Util.dist(odometry.getPosition(), next.getPosition()) > rotationDeadZone) {
                        rotation = Math.toDegrees(Util.angle(odometry.getPosition(), next.getPosition()));
                    } else {
                        rotation = odometry.getRotation();
                    }
                }

                telemetry += "\ntarget pos: " + next.getPosition().toString();
                telemetry += "\npos: " + odometry.getPosition().toString();
                telemetry += "\ntarget rot: " + rotation;
                telemetry += "\nrot: " + odometry.getLoopedRotation();

                boolean withinLookahead = Util.dist(odometry.getPosition(), next.getPosition()) < lookahead;
                boolean noActions = next.actions.isEmpty();
                boolean withinRotationTolerance = next.rotation == null || Math.abs(getAngleDiff(odometry.getRotation(), next.rotation)) < rotationTolerance;

//                If the robot is finished with everything at this point, and there is still left to go,
//                finish the path
                if (withinLookahead && noActions && withinRotationTolerance) {
                    if (path.size() > 1) {
                        state = State.MOVING;
                    } else {
                        isBusy = false;
                    }
                }

                return new FollowPoint(next.x, next.y, rotation);
            }
        }

        return null;
    }

    @Override

    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

        telemetry = "";

        try {

            if (state != State.IDLE) {
                PursuitPoint nextPoint = path.get(path.size() > 1 ? 1 : 0);

//                telemetry += "\n next point actions: " + nextPoint.actions.size();

                if (nextPoint.lookahead != null) lookahead = nextPoint.lookahead;
                if (nextPoint.rotationTolerance != null)
                    rotationTolerance = nextPoint.rotationTolerance;
                if(path.get(0).movementSpeed != null) drivetrain.maxSpeed = path.get(0).movementSpeed;

                boolean withinLookahead = Util.dist(odometry.getPosition(), nextPoint.getPosition()) < lookahead;
                boolean withinRotation = nextPoint.rotation == null || Math.abs(getAngleDiff(odometry.getRotation(), nextPoint.rotation)) < rotationTolerance;
                if (!nextPoint.actions.isEmpty() && withinLookahead && withinRotation) {

                    PursuitPoint.Action action = nextPoint.actions.get(0);
                    double dist = Util.dist(odometry.getPosition(), nextPoint.getPosition());

                    if (action.isThread()) {
//                    If the action is a thread, start it and continue on
                        new Thread(action::run).start();
                        nextPoint.actions.remove(0);
                    } else {
//                        If its not a thread, start it but don't remove it until its done
                        if (!action.hasStarted()) {
                            boolean withinDistance = action.tolerance == null || dist < action.tolerance;
                            if (withinDistance)
                                new Thread(action::run).start();
                        } else if (action.hasStarted() && !action.isRunning()) {
//                        If the action is finished, remove it from the list
                            nextPoint.actions.remove(0);
                        }
                    }
                }


//           Get the follow point
                FollowPoint follow = getFollowPoint();
                followPoint(follow);
            }else{
                isBusy = false;
            }
        } catch (Exception e) {
            StringWriter sw = new StringWriter();
            e.printStackTrace(new PrintWriter(sw));
            telemetry = "Exception occured:\n" + sw.toString();
        }
    }

    public void followPoint(FollowPoint point) {
//        Speed will depend on the state
//        If it is maintain, use pid to efficiently approach the point
//        If it is move, the the speed will constant, however in the future, the robot
//        could brace for sharp turns by slowing down

//        if (state == State.MAINTAINING) {
//            drivetrain.moveToPosition(point.getPosition());
//
//        } else {
//            double dist = Util.dist(odometry.getPosition(), path.get(1).getPosition());
//
////            Look for actions and rotation points in the path so the robot can preemptively slow down
//
//            double tempRotationTolerance = rotationTolerance;
//            for (int i = 1; i < path.size() - 1; i++) {
//                dist += Util.dist(path.get(i).getPosition(), path.get(i + 1).getPosition());
//                if (path.get(i).rotationTolerance != null)
//                    tempRotationTolerance = path.get(i).rotationTolerance;
//
//                boolean noActions = path.get(i).actions.isEmpty();
//                boolean withinRotationTolerance = path.get(i).rotation == null || (Math.abs(getAngleDiff(odometry.getRotation(), path.get(i).rotation)) < tempRotationTolerance);
//
//                if (!(noActions && withinRotationTolerance)) break;
//            }
//////            Calculate local direction
//////            The robot needs a local direction because it wont always be facing forwards
////            final double globalDirection = Util.angle(odometry.getPosition(), point.getPosition());
////            double localDirection = globalDirection - Math.toRadians(odometry.getRotation());
////
//////            Gets a ratio of forward and strafe kP's depending on what direction the robot is driving in
//////            This is so the robot has a generally accurate speed value if its coming to a stop
////            double combinedP = Util.dist(drivetrain.forwardPID.kP * Math.abs(Math.sin(localDirection)), drivetrain.strafePID.kP * Math.abs(Math.cos(localDirection)));
////
//////            Rotational velocity is irrelevant here because its overwritten on turnToRotation
////            drivetrain.polarDrive(dist * combinedP, localDirection, 0);
//
//            Vector error = new Vector(point.getPosition()).subtract(new Vector(odometry.getPosition()));
//            error.setMagnitude(dist);
//            drivetrain.moveToPosition(error.toPoint());
//        }


        telemetry += "\nmove power: (" + drivetrain.xPower + ", " + drivetrain.yPower + ")";
        telemetry += "\nrot power: " + drivetrain.turnPower;

        drivetrain.moveToPosition(point.getPosition());
        drivetrain.turnToRotation(point.rotation);
        drivetrain.heldPosition = odometry.getPosition();
        drivetrain.heldRotation = odometry.getLoopedRotation();

//        For debugging
//        drivetrain.move(0,0,0,0);
    }

    public PursuitPoint addPoint(double x, double y) {
        PursuitPoint p = new PursuitPoint(x, y);
        path.add(p);
        return p;
    }

    public PursuitPoint addPoint(double x, double y, double rotation) {
        PursuitPoint p = new PursuitPoint(x, y, rotation);
        path.add(p);
        return p;
    }

    @Deprecated
    public void addLookahead(double lookahead) {
        path.get(path.size() - 1).lookahead = lookahead;
    }

    @Deprecated
    public void addRotationTolerance(double tolerance) {
        path.get(path.size() - 1).rotationTolerance = tolerance;
    }

    @Deprecated
    public void addAction(Runnable action, boolean isThread) {
        path.get(path.size() - 1).addAction(action, isThread);
    }

    @Deprecated
    public void addAction(Runnable action, boolean isThread, double tolerance) {
        path.get(path.size() - 1).addAction(action, isThread, tolerance);
    }

    public void maintainPosition() {
        state = State.MAINTAINING;
        path.clear();
        path.add(new PursuitPoint(odometry.getX(), odometry.getY(), odometry.getRotation()));
    }

    public void idle() {
        path.clear();
        state = State.IDLE;
    }

    public void move() {
        state = State.MOVING;
        path.add(0, new PursuitPoint(odometry.getX(), odometry.getY()));
    }

    @Override
    public String getTelemetry() {
        return telemetry;
    }

    /**
     * @param a1 the first angle
     * @param a2 the second angle
     * @return the shortest interval between the two angles
     */
    private double getAngleDiff(double a1, double a2) {
        a1 = Util.loop(a1, 0, 360);
        a2 = Util.loop(a2, 0, 360);

        double dist = a1 - a2;
        double shortest;
        if (Math.abs(dist) < 180)
            shortest = dist;
        else {
            if (dist > 0) shortest = dist - 360;
            else shortest = dist + 360;
        }

        return shortest;
    }


    public enum State {
        IDLE,
        MAINTAINING,
        MOVING
    }

    private class FollowPoint {

        double targetX, targetY;
        Double rotation = null;

        public FollowPoint(double targetX, double targetY) {
            this.targetX = targetX;
            this.targetY = targetY;
        }

        public FollowPoint(double targetX, double targetY, double rotation) {
            this.targetX = targetX;
            this.targetY = targetY;
            this.rotation = rotation;
        }

        public Point getPosition() {
            return new Point(targetX, targetY);
        }

    }
}