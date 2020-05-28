package com.example.ros2videostream.ui;

import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;

import com.example.ros2videostream.MainActivity;
import com.example.ros2videostream.R;
import com.example.ros2videostream.ros2.Ros2Node;
import com.example.ros2videostream.setting.MapItem;
import com.example.ros2videostream.viewmodel.SettingViewModel;

public class TextFragment extends Fragment {
    private static String logtag = TextFragment.class.getName();
    private View view;
    private Ros2Node talkerNode;
    private Button start;
    private Button stop;
    private boolean isWorking;
    private String qosfile;

    /*@Override
    public void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setRetainInstance(true);
    }*/

    @Override
    public void onSaveInstanceState(@NonNull Bundle outState) {
        super.onSaveInstanceState(outState);
        outState.putBoolean("isWorking", isWorking);
    }

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        if(view==null){
            view = inflater.inflate(R.layout.fragment_text, null);
            Log.d("view","load fragment_text");
        }
        //return super.onCreateView(inflater, container, savedInstanceState);
        return view;
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        Toolbar toolbar =view.findViewById(R.id.toolbar_text);
        ((AppCompatActivity) getActivity()).setSupportActionBar(toolbar);
        toolbar.setTitle(R.string.toolbar_title);
        toolbar.setNavigationIcon(R.drawable.ic_arrow_back_black_24dp);
        toolbar.setNavigationOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                requireActivity().onBackPressed();
            }
        });
        if (savedInstanceState != null) {
            Log.d("savedInstanceState","load isWorking");
            isWorking = savedInstanceState.getBoolean("isWorking");
        }
        //ViewModel
        SettingViewModel settingViewModel = new ViewModelProvider(requireActivity()).get(SettingViewModel.class);

        MapItem mapItem=settingViewModel.getGroupC().getValue();
        if(mapItem!=null){
            qosfile=mapItem.getSettingcontent();
        }
        if(talkerNode==null){
            talkerNode = new Ros2Node("android_talker_node", "chatter",1,qosfile);
            Log.d("talkerNode","new talkerNode");
        }
        start=view.findViewById(R.id.start);
        start.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(logtag, "onClick() called - start button");
                Toast.makeText(getContext(), "The Start button was clicked.", Toast.LENGTH_LONG).show();
                Log.d(logtag, "onClick() ended - start button");
                changeState(true);
            }
        });
        stop=view.findViewById(R.id.stop);
        stop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(logtag, "onClick() called - stop button");
                Toast.makeText(getContext(), "The Stop button was clicked.", Toast.LENGTH_LONG).show();
                Log.d(logtag, "onClick() ended - stop button");
                changeState(false);
            }
        });
    }

    @Override
    public void onResume() {
        super.onResume();
        if(isWorking){
            changeState(true);
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        if(isWorking){
            talkerNode.stop();
            ((MainActivity) requireActivity()).getExecutor().removeNode(talkerNode);
        }
    }

    private void changeState(boolean isWorking) {
        this.isWorking = isWorking;
        start.setEnabled(!isWorking);
        stop.setEnabled(isWorking);
        if (isWorking){
            ((MainActivity) requireActivity()).getExecutor().addNode(talkerNode);
            talkerNode.start();
        } else {
            talkerNode.stop();
            ((MainActivity) requireActivity()).getExecutor().removeNode(talkerNode);
        }
    }
}
