package frc.robot.lib;

import java.util.function.Consumer;

public class StateChange<T> {
    private T state = null;

    public StateChange() {}

    public StateChange(T initial) {
        state = initial;
    }

    public StateChangeResult<T> update(T newState) {
        T last = state;
        state = newState;
        return new StateChangeResult<T>(last, last.equals(state));
    }

    public T getState() {
        return state;
    }

    public static class StateChangeResult<T> {
        private boolean changed;
        private T last;

        private StateChangeResult(T last, boolean changed) {
            this.last = last;
            this.changed = changed;
        }

        public StateChangeResult<T> onChange(Consumer<T> fn) {
            if(changed) {
                fn.accept(last);
            }
            return this;
        }

        public StateChangeResult<T> onNoChange(Runnable fn) {
            if(!changed) {
                fn.run();
            }
            return this;
        }
    }
}
