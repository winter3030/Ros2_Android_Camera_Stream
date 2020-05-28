package com.example.ros2videostream.ros2;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.ScriptIntrinsicYuvToRGB;
import android.renderscript.Type;
import android.util.Log;

import androidx.camera.core.ImageProxy;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.timer.WallTimer;

import java.io.ByteArrayOutputStream;
import java.io.File;
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
    private RenderScript rs;
    private Allocation yuvAllocation;
    private Allocation rgbAllocation;
    private ScriptIntrinsicYuvToRGB scriptYuvToRgb;
    private QoSProfile qoSProfile;
    private Bitmap.CompressFormat compressFormat;

    public Ros2Node(final String name, final String topic, final int type,final String QOSfile) {
        super(name);
        this.topic = topic;
        switch (QOSfile){
            case "Default":
                qoSProfile=QoSProfile.DEFAULT;
                break;
            case "Services":
                qoSProfile=QoSProfile.SERVICES_DEFAULT;
                break;
            case "Parameters":
                qoSProfile=QoSProfile.PARAMETERS;
                break;
            case "Sensor":
                qoSProfile=QoSProfile.SENSOR_DATA;
                break;
        }
        switch (type){
            case TEXTTYPE:
                this.publishertext = this.node.createPublisher(std_msgs.msg.String.class, this.topic,qoSProfile);
                break;
            case IMAGETYPE:
                this.publisherimage = this.node.createPublisher(sensor_msgs.msg.Image.class, this.topic,qoSProfile);
                msg_image=new sensor_msgs.msg.Image();
                break;
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
        //ByteBuffer IRbuffer=ByteBuffer.allocate(4).putInt(IRotation);
        //YUV buffer
        ByteBuffer Ybuffer=image.getPlanes()[0].getBuffer();
        ByteBuffer Ubuffer=image.getPlanes()[1].getBuffer();
        ByteBuffer Vbuffer=image.getPlanes()[2].getBuffer();
        //buffer remaining
        //int Wr=IRbuffer.rewind().remaining();
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
        //byte[] IRarray=IRbuffer.array();
        //test
        /*byte[] nv21 = new byte[Wr+Hr+Yr + Ur + Vr];
        Wbuffer.get(nv21,0,Wr);
        Hbuffer.get(nv21,Wr,Hr);
        Ybuffer.get(nv21, Wr+Hr, Yr);
        Ubuffer.get(nv21, Wr+Hr+Yr, Ur);
        Vbuffer.get(nv21, Wr+Hr+Yr+Ur, Vr);*/

        //method1 YuvImage
        /*YuvImage yuvImage = new YuvImage(nv21, ImageFormat.NV21, w, h, null);
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, yuvImage.getWidth(), yuvImage.getHeight()), 90, out);
        byte[] imageBytes = out.toByteArray();
        //Bitmap
        Bitmap img= BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
        //img=rotate_bitmap(img,w,h,IRotation);
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        img.compress(Bitmap.CompressFormat.JPEG, 90 , baos);
        byte[] b = baos.toByteArray();*/

        //method2 ScriptIntrinsicYuvToRGB
        yuvAllocation.copyFrom(nv21);
        scriptYuvToRgb.setInput(yuvAllocation);
        scriptYuvToRgb.forEach(rgbAllocation);
        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.ARGB_8888);
        rgbAllocation.copyTo(bitmap);
        bitmap=rotate_bitmap(bitmap,w,h,IRotation);
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        bitmap.compress(compressFormat, 70 , baos);
        byte[] b = baos.toByteArray();
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
        for (byte item : b) {
            img_list.add(item);
        }
        msg_image.setData(img_list);
        publisherimage.publish(msg_image);
        Log.d("===","===");
        bitmap.recycle();
        image.close();
    }

    private Bitmap rotate_bitmap(Bitmap img,int w,int h,int rotate){
        Matrix matrix = new Matrix();
        matrix.postRotate(rotate);
        return Bitmap.createBitmap(img, 0, 0, w, h, matrix, true);
    }

    //ScriptIntrinsicYuvToRGB
    public void setup_script(Context context,int w,int h){
        rs = RenderScript.create(context);
        int len=(w*h)+(w*h/2-1)+(w*h/2-1);
        Type.Builder yuvType = new Type.Builder(rs, Element.U8(rs))
                .setX(len);
        Type.Builder rgbaType = new Type.Builder(rs, Element.RGBA_8888(rs))
                .setX(w)
                .setY(h);
        yuvAllocation = Allocation.createTyped(rs, yuvType.create(), Allocation.USAGE_SCRIPT);
        rgbAllocation = Allocation.createTyped(rs, rgbaType.create(), Allocation.USAGE_SCRIPT);
        scriptYuvToRgb = ScriptIntrinsicYuvToRGB.create(rs, Element.YUV(rs));
    }

    public void release_script(){
        yuvAllocation.destroy();
        rgbAllocation.destroy();
        rs.destroy();
    }

    public void setup_imageformat(String imageformat){
        switch (imageformat){
            case "PNG":
                compressFormat=Bitmap.CompressFormat.PNG;
                break;
            case "JPG":
                compressFormat=Bitmap.CompressFormat.JPEG;
                break;
        }
    }
}
