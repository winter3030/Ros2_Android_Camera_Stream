package com.example.ros2videostream;

import android.util.Log;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.timer.WallTimer;

import java.util.concurrent.TimeUnit;

public class Ros2Node extends BaseComposableNode {
    private static String logtag = Ros2Node.class.getName();
    private final String topic;
    private Publisher<std_msgs.msg.String> publishertext;
    private WallTimer timer;
    private int count=0;

    public Ros2Node(final String name, final String topic) {
        super(name);
        this.topic = topic;
        this.publishertext = this.node.createPublisher(std_msgs.msg.String.class, this.topic);
    }

    public void start() {
        Log.d(logtag, "TalkerNode::start()");
        if (timer != null) {
            timer.cancel();
        }
        //count = 0;
        timer = node.createWallTimer(500, TimeUnit.MILLISECONDS, this::onTimer);
    }

    public void stop() {
        Log.d(logtag, "TalkerNode::stop()");
        if (timer != null) {
            timer.cancel();
        }
    }

    private void onTimer() {
        std_msgs.msg.String msg_text = new std_msgs.msg.String();
        msg_text.setData("Hello ROS2 from Android: " + count);
        count++;
        publishertext.publish(msg_text);
    }

}
