package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.RMath.Point;

import java.util.ArrayList;
import java.util.List;

public class PursuitPoint {

    public double x, y;
    public Double rotation = null;
    public Double lookahead = null;
    public Double rotationTolerance = null;
    public Double movementSpeed = null;
    public List<Action> actions = new ArrayList<>();

    public PursuitPoint(double x, double y){
        this.x = x;
        this.y = y;
    }

    public PursuitPoint(double x, double y, double rotation){
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    public PursuitPoint addAction(Runnable action, boolean isThread){
        actions.add(new PursuitPoint.Action(action, isThread));
        return this;
    }

    public PursuitPoint addAction(Runnable action, boolean isThread, double tolerance){
        actions.add(new PursuitPoint.Action(action, isThread, tolerance));
        return this;
    }

    public boolean hasInterruptingActions(){
        for(Action a : actions) if (!a.thread) return true;
        return false;
    }

    public PursuitPoint setLookahead(Double lookahead) {
        this.lookahead = lookahead;
        return this;
    }

    public PursuitPoint setRotationTolerance(Double rotationTolerance) {
        this.rotationTolerance = rotationTolerance;
        return this;
    }

    public PursuitPoint setMovementSpeed(Double movementSpeed) {
        this.movementSpeed = movementSpeed;
        return this;
    }

    public class Action{
        public Runnable action;
        private boolean running = false;
        private boolean started = false;

        private boolean thread;

        public Double tolerance = null;

        public Action(Runnable action, boolean isThread){
            this.action = action;
            thread = isThread;
        }

        public Action(Runnable action, boolean isThread, double tolerance){
            this.action = action;
            thread = isThread;
            this.tolerance = tolerance;
        }

        public void run(){
            running = true;
            started = true;
            action.run();
            running = false;
        }

        public boolean isRunning(){
            return running;
        }

        public boolean hasStarted(){
            return started;
        }

        public boolean isThread(){
            return thread;
        }

    }

    public Point getPosition(){
        return new Point(x, y);
    }

}