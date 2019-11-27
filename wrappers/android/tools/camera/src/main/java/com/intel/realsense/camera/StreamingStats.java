package com.intel.realsense.camera;

import android.util.Log;

import com.intel.realsense.librealsense.Extension;
import com.intel.realsense.librealsense.Frame;
import com.intel.realsense.librealsense.FrameCallback;
import com.intel.realsense.librealsense.FrameMetadata;
import com.intel.realsense.librealsense.FrameSet;
import com.intel.realsense.librealsense.StreamProfile;
import com.intel.realsense.librealsense.VideoStreamProfile;

import java.util.HashMap;
import java.util.Map;

public class StreamingStats {
    private static final String TAG = "librs camera streamer";
    private static final String DATA_LOG_TAG = "FRAME_DATA_LOG";

    private Map<Integer, Statistics> mStreamsMap = new HashMap<>();
    private Map<Integer, Statistics> mLastFrames = new HashMap<>();

    private Boolean isLogStatisticsEnabled = true;

    public void setIsLogStatisticsEnabled(boolean b) { isLogStatisticsEnabled = b; }

    private void initStream(StreamProfile profile){
        String resolution = "";
        if(profile.is(Extension.VIDEO_PROFILE)){
            VideoStreamProfile vsp = profile.as(Extension.VIDEO_PROFILE);
            resolution = vsp.getWidth() + "x" + vsp.getHeight();
        }

        String streamType = profile.getType().name();
        String format = profile.getFormat().name();
        String res = resolution;
        String fps = String.valueOf(profile.getFrameRate());

        String name = streamType + " | " +
                format + " | " +
                res + " | " +
                fps;

        Statistics stats = new Statistics(name);
        stats.mStreamType = streamType;
        stats.mFormat = format;
        stats.mResolution = res;
        stats.mRequestedFps = fps;

        mStreamsMap.put(profile.getUniqueId(), stats);
        mLastFrames.put(profile.getUniqueId(), new Statistics(name));
    }

    private FrameCallback mFrameCallback = new FrameCallback() {
        @Override
        public void onFrame(Frame f) {
            try(StreamProfile profile = f.getProfile()) {
                int fn = f.getNumber();
                int uid = f.getProfile().getUniqueId();
                if (!mLastFrames.containsKey(f.getProfile().getUniqueId()))
                    initStream(profile);
                if (mLastFrames.get(uid).mFrameNumber != fn) {
                    if(f.supportsMetadata(FrameMetadata.FRAME_EMITTER_MODE))
                        mStreamsMap.get(uid).mEmitter = String.valueOf(f.getMetadata(FrameMetadata.FRAME_EMITTER_MODE));
                    if(f.supportsMetadata(FrameMetadata.ACTUAL_EXPOSURE))
                        mStreamsMap.get(uid).mExposure = String.valueOf(f.getMetadata(FrameMetadata.ACTUAL_EXPOSURE));
                    if(f.supportsMetadata(FrameMetadata.AUTO_EXPOSURE))
                        mStreamsMap.get(uid).mAutoExposureMode = String.valueOf(f.getMetadata(FrameMetadata.AUTO_EXPOSURE));
                    if(f.supportsMetadata(FrameMetadata.GAIN_LEVEL))
                        mStreamsMap.get(uid).mGain = String.valueOf(f.getMetadata(FrameMetadata.GAIN_LEVEL));
                    if(f.supportsMetadata(FrameMetadata.FRAME_LASER_POWER))
                        mStreamsMap.get(uid).mLaserPower = String.valueOf(f.getMetadata(FrameMetadata.FRAME_LASER_POWER));
                    if(f.supportsMetadata(FrameMetadata.FRAME_LED_POWER))
                        mStreamsMap.get(uid).mLedPower = String.valueOf(f.getMetadata(FrameMetadata.FRAME_LED_POWER));
                    mStreamsMap.get(uid).mFrameNumber = fn;
                    mStreamsMap.get(uid).mHWTimestamp = f.getTimestamp();
                    mStreamsMap.get(uid).mSWTimestamp = System.currentTimeMillis();
                    mStreamsMap.get(uid).onFrame(f);
                    mLastFrames.put(uid, new Statistics(mStreamsMap.get(uid)));
                }
                else
                    mStreamsMap.get(uid).kick();
            }
        }
    };

    public void onFrameset(FrameSet frames){
        frames.foreach(mFrameCallback);
    }

    public String prettyPrint(){
        String rv = "";
        for(Map.Entry e : mStreamsMap.entrySet()){
            rv += ((Statistics)e.getValue()).prettyPrint() + "\n\n";
        }
        return rv;
    }

    private class Statistics{
        private final String mName;
        private String mStreamType;
        private String mFormat;
        private String mResolution;
        private String mRequestedFps;
        private long mStartTime = 0;
        private long mBaseTime = 0;
        private float mFps = 0;
        private long mFrameCount = 0;
        private long mFrameLoss = 0;
        private long mHWTimestamp = 0;
        private long mHWTimestampDiff = 0;
        private long mSWTimestamp = 0;
        private long mSWTimestampDiff = 0;
        private int mFrameNumber = 0;
        private long mTotalFrameCount = 0;
        private long mFirstFrameLatency = 0;
        private String mEmitter = "No data";
        private String mExposure = "No data";
        private String mGain = "No data";
        private String mAutoExposureMode = "No data";
        private String mLaserPower = "No data";
        private String mLedPower = "No data";

        public Statistics(String name) {
            mName = name;
            reset();
        }

        public Statistics(Statistics other) {
            mName = other.mName;
            mStreamType = other.mStreamType;
            mFormat = other.mFormat;
            mResolution = other.mResolution;
            mRequestedFps = other.mRequestedFps;
            mStartTime = other.mStartTime;
            mBaseTime = other.mBaseTime;
            mFps = other.mFps;
            mFrameCount = other.mFrameCount;
            mFrameLoss = other.mFrameLoss;
            mHWTimestamp = other.mHWTimestamp;
            mHWTimestampDiff = other.mHWTimestampDiff;
            mSWTimestamp = other.mSWTimestamp;
            mSWTimestampDiff = other.mSWTimestampDiff;
            mFrameNumber = other.mFrameNumber;
            mTotalFrameCount = other.mTotalFrameCount;
            mFirstFrameLatency = other.mFirstFrameLatency;
            mEmitter = other.mEmitter;
            mExposure = other.mExposure;
            mGain = other.mGain;
            mAutoExposureMode = other.mAutoExposureMode;
            mLaserPower = other.mLaserPower;
            mLedPower = other.mLedPower;
        }

        public void reset(){
            mStartTime = mBaseTime = System.currentTimeMillis();
            mFirstFrameLatency = 0;
        }

        public float getFps(){
            return mFps;
        }

        public float getFrameCount(){
            return mTotalFrameCount;
        }

        public String prettyPrint(){
            long curr = System.currentTimeMillis();
            int diffInSeconds = (int) ((curr - mStartTime) * 0.001);
            return mName +
                    "\nFrame Rate: " + mFps +
                    "\nFrame Count: " + mTotalFrameCount +
                    "\nFrame Number: " + mFrameNumber +
                    "\nFrame Loss: " + mFrameLoss +
                    "\nHW timestamp: " + mHWTimestamp +
                    "\nSW timestamp: " + mSWTimestamp +
                    "\nRun Time: " + diffInSeconds + " [sec]" +
                    "\nEmitter Mode: " + mEmitter +
                    "\nExposure: " + mExposure;
        }

        public synchronized void kick(){
            long curr = System.currentTimeMillis();
            float diffInSeconds = (float) ((curr - mBaseTime) * 0.001);
            if(diffInSeconds > 2){
                mFps = mFrameCount / diffInSeconds;
                mFrameCount = 0;
                mBaseTime = curr;
            }
        }

        public synchronized void onFrame(Frame f){
            mFrameCount++;
            mTotalFrameCount++;
            long curr = System.currentTimeMillis();
            float diffInSeconds = (float) ((curr - mBaseTime) * 0.001);
            if(mFirstFrameLatency == 0){
                mFirstFrameLatency = curr - mBaseTime;
            }

            if(diffInSeconds > 2){
                mFps = mFrameCount / diffInSeconds;
                mFrameCount = 0;
                mBaseTime = curr;
            }
            mFrameLoss = mFrameNumber - mTotalFrameCount;

            Statistics mLastFrame = mLastFrames.get(f.getProfile().getUniqueId());
            mHWTimestampDiff = mHWTimestamp - mLastFrame.mHWTimestamp;
            mSWTimestampDiff = mSWTimestamp - mLastFrame.mSWTimestamp;

            if(isLogStatisticsEnabled)
                logFrameData();
        }

        private void logFrameData() {
            String data =
                    mStreamType + ", " +
                    mFormat + ", " +
                    mResolution + ", " +
                    mRequestedFps + ", " +
                    mFrameNumber + ", " +
                    mHWTimestamp + ", " +
                    mHWTimestampDiff + ", " +
                    mSWTimestamp + ", " +
                    mSWTimestampDiff + ", " +
                    mAutoExposureMode + ", " +
                    mExposure + ", " +
                    mGain + ", " +
                    mLaserPower + ", " +
                    mEmitter + ", " +
                    mLedPower;
            Log.i(DATA_LOG_TAG, data);
        }
    }
}
