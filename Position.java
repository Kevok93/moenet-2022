public class Position{
    int x; DoubleSubscriber xsub;
    int y; DoubleSubscriber ysub;
    int z; DoubleSubscriber zsub;

    //compare with z -axis, rotated around y axis
    int yaw; DoubleSubscriber yawsub;
    //compare with y - axis, rotated around x axis
    int pitch; DoubleSubscriber pitchsub;
    //compare with x - axis, rotated around z axis
    int roll; DoubleSubscriber rollsub;

    NetworkTableInstance inst;
    NetworkTable table;

    void init(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("SmartDashboard");
        
        xsub = table.getDoubleTopic("x").subscribe(0.0);
        ysub = table.getDoubleTopic("y").subscribe(0.0);
        zsub = table.getDoubleTopic("z").subscribe(0.0);

        yawsub = table.getDoubleTopic("yaw").subscribe(0.0);
        rollsub = table.getDoubleTopic("roll").subscribe(0.0);
        pitchsub = table.getDoubleTopic("pitch").subscribe(0.0);
    }

    void update(){
        x = xsub.get();
        y = ysub.get();
        z = zsub.get();

        yaw = yawsub.get();
        pitch = pitchsub.get();
        roll = rollsub.get();
    }
}