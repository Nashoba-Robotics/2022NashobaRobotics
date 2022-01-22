package frc.robot.lib;

public class AccelerationControl {
    private double lastMove = 0;
    private double lastTurn = 0;
    private long lastMillis;

    private double maxAccelMove;
    private double maxDecelMove;
    private double maxAccelTurn;
    private double maxDecelTurn;

    public AccelerationControl(double maxAccelMove, double maxDecelMove, double maxAccelTurn, double maxDecelTurn) {
        this.maxAccelMove = maxAccelMove;
        this.maxDecelMove = maxDecelMove;
        this.maxAccelTurn = maxAccelTurn;
        this.maxDecelTurn = maxDecelTurn;
        lastMillis = System.currentTimeMillis();
    }

    public JoystickValues next(JoystickValues input) {   
        long elapsed = System.currentTimeMillis() - lastMillis;
        lastMillis += elapsed;

        double moveChange = input.movement - lastMove;
        double maxMoveChange = getMaxChange(lastMove, input.movement, elapsed, maxAccelMove, maxDecelMove);

        double maxMove = Math.min(Math.abs(moveChange), maxMoveChange);
        if(moveChange < 0) maxMove *= -1;
        double newMove = lastMove + maxMove;
        lastMove = newMove;

        double turnChange = input.turning - lastTurn;
        double maxTurnChange = getMaxChange(lastTurn, input.turning, elapsed, maxAccelTurn, maxDecelTurn);

        double maxTurn = Math.min(Math.abs(turnChange), maxTurnChange);
        if(turnChange < 0) maxTurn *= -1;
        double newTurn = lastTurn + maxTurn;
        lastTurn = newTurn;

        return new JoystickValues(newMove, newTurn);
    }

    
    private static double getMaxChange(double lastValue, double newValue, long elapsed, double maxAccel, double maxDecel) {
        if( (lastValue < 0 && newValue > 0)
        || (newValue < 0 && lastValue > 0)
         || Math.abs(newValue) < Math.abs(lastValue) ) {
            return maxDecel * elapsed;
        }
        return maxAccel * elapsed;
    }
}
