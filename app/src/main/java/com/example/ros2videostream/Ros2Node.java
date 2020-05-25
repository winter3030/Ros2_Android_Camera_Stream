package com.example.ros2videostream;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.Image;
import android.util.Log;

import androidx.camera.core.ImageProxy;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.timer.WallTimer;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Ros2Node extends BaseComposableNode {
    private static String logtag = Ros2Node.class.getName();
    private static final int TEXTTYPE = 1;
    private static final int IMAGETYPE = 2;
    private final String topic;
    private Publisher<std_msgs.msg.String> publishertext;
    private Publisher<sensor_msgs.msg.Image> publisherimage;
    private sensor_msgs.msg.Image msg_image;
    private WallTimer timer;
    private int count=0;

    public Ros2Node(final String name, final String topic,final int type) {
        super(name);
        this.topic = topic;
        switch (type){
            case TEXTTYPE:
                this.publishertext = this.node.createPublisher(std_msgs.msg.String.class, this.topic);
            case IMAGETYPE:
                this.publisherimage = this.node.createPublisher(sensor_msgs.msg.Image.class, this.topic,QoSProfile.SENSOR_DATA);
                msg_image=new sensor_msgs.msg.Image();
        }
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

    public void start_stream(ImageProxy image, File tmp){
        //image Width and Height
        int w = image.getWidth();
        int h = image.getHeight();
        //image Rotation
        int IRotation=image.getImageInfo().getRotationDegrees();
        //image Rotation buffer
        ByteBuffer IRbuffer=ByteBuffer.allocate(4).putInt(IRotation);
        //YUV buffer
        ByteBuffer Ybuffer=image.getPlanes()[0].getBuffer();
        ByteBuffer Ubuffer=image.getPlanes()[1].getBuffer();
        ByteBuffer Vbuffer=image.getPlanes()[2].getBuffer();
        //buffer remaining
        int Wr=IRbuffer.rewind().remaining();
        int Yr=Ybuffer.remaining();
        int Ur=Ubuffer.remaining();
        int Vr=Vbuffer.remaining();
        //Log
        /*byte[] aa=IRbuffer.array();
        for (byte b : aa) {
            Log.d("byte",String.format("value = %x", b));
        }
        Log.d("w",String.valueOf(w));
        Log.d("h",String.valueOf(h));
        Log.d("ImageProxyRotation",String.valueOf(IRotation));
        Log.d("YStride",String.valueOf(image.getPlanes()[0].getPixelStride()));
        Log.d("UStride",String.valueOf(image.getPlanes()[1].getPixelStride()));
        Log.d("VStride",String.valueOf(image.getPlanes()[2].getPixelStride()));
        Log.d("Wbuffer",String.valueOf(Wr));
        Log.d("Hbuffer",String.valueOf(Hr));
        Log.d("Ybuffer",String.valueOf(Yr));
        Log.d("Ubuffer",String.valueOf(Ur));
        Log.d("Vbuffer",String.valueOf(Vr));*/
        //Image format YUV_420 Semiplanar
        //Y Y Y Y
        //Y Y Y Y
        //Y Y Y Y
        //Y Y Y Y
        //U V U V
        //U V U V <== U & V交叉儲存
        //Flatten
        //Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y U V U V U V U V
        //buffer->array
        byte[] nv21 = new byte[Yr + Ur + Vr];
        //Copy the buffer into a byte array
        //U and V are swapped
        Ybuffer.get(nv21, 0, Yr);
        Vbuffer.get(nv21, Yr, Vr);
        Ubuffer.get(nv21, Yr + Vr, Ur);
        //buffer->array
        byte[] IRarray=IRbuffer.array();
        //test
        /*byte[] nv21 = new byte[Wr+Hr+Yr + Ur + Vr];
        Wbuffer.get(nv21,0,Wr);
        Hbuffer.get(nv21,Wr,Hr);
        Ybuffer.get(nv21, Wr+Hr, Yr);
        Ubuffer.get(nv21, Wr+Hr+Yr, Ur);
        Vbuffer.get(nv21, Wr+Hr+Yr+Ur, Vr);*/

        //YuvImage
        YuvImage yuvImage = new YuvImage(nv21, ImageFormat.NV21, w, h, null);
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, yuvImage.getWidth(), yuvImage.getHeight()), 80, out);
        byte[] imageBytes = out.toByteArray();
        //Bitmap
        /*Bitmap img= BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        img.compress(Bitmap.CompressFormat.JPEG, 70 , baos);
        byte[] b = baos.toByteArray();*/
        //write file
        /*try {
            FileOutputStream fOut=new FileOutputStream(tmp);
            img.compress(Bitmap.CompressFormat.JPEG, 70, fOut);
            try {
                fOut.flush();
                fOut.close();
            } catch (IOException e) {
                e.printStackTrace();
            }

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }*/
        List<Byte> img_list = new ArrayList<Byte>();
        for (byte item : IRarray) {
            img_list.add(item);
        }
        for (byte item : imageBytes) {
            img_list.add(item);
        }
        msg_image.setData(img_list);
        publisherimage.publish(msg_image);
        Log.d("===","===");
        image.close();
    }
}
