package raidzero.robot.submodules;

public class Superstructure extends Submodule {

    private static Superstructure instance = null;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private Superstructure() {}

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {}

    @Override
    public void stop() {}
}