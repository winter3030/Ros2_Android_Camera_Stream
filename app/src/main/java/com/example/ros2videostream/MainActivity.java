package com.example.ros2videostream;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.FragmentManager;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.FrameLayout;

import org.ros2.android.activity.ROSActivity;
import org.ros2.rcljava.RCLJava;

public class MainActivity extends ROSActivity {
    private static String logtag = MainActivity.class.getName();
    private FragmentManager manager=getSupportFragmentManager();
    private MainFragment mainFragment;
    private TextFragment textFragment;
    private FrameLayout container;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        container=findViewById(R.id.fragment_container);
        //fragment
        if(savedInstanceState!=null){
            mainFragment= (MainFragment) manager.getFragment(savedInstanceState,"mainFragment");
            manager.beginTransaction().show(mainFragment).commit();
            Log.d(logtag,"reload mainFragment");
        }
        else{
            mainFragment=new MainFragment();
            manager.beginTransaction().add(R.id.fragment_container, mainFragment,"MFragment").commit();
        }
        //RCLJava
        RCLJava.rclJavaInit();
    }

    @Override
    protected void onSaveInstanceState(@NonNull Bundle outState) {
        super.onSaveInstanceState(outState);
        manager.putFragment(outState,"mainFragment",mainFragment);
    }

    @Override
    protected void onResume() {
        super.onResume();
        container.postDelayed(new Runnable() {
            @Override
            public void run() {
                container.setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN);
            }
        },500L);
    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
    }

    public void show_fragment_text(){
        Log.d(logtag,"now show fragment_text");
        textFragment =new TextFragment();
        manager.beginTransaction().addToBackStack("M_to_T").replace(R.id.fragment_container,textFragment,"TFragment").commit();
    }

}
