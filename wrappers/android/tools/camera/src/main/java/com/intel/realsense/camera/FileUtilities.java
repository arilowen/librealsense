package com.intel.realsense.camera;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;

public class FileUtilities {
    private static final String TAG = "file utilities";

    public static final int PERMISSIONS_REQUEST_CAMERA = 0;
    public static final int PERMISSIONS_REQUEST_READ = 1;
    public static final int PERMISSIONS_REQUEST_WRITE = 2;

    public static boolean isExternalStorageWritable() {
        return Environment.getExternalStorageState() == Environment.MEDIA_MOUNTED;
    }

    public static void saveFileToExternalDir(String fileName, byte[] data) {
        try {
            File file = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + fileName);
            FileOutputStream fos = new FileOutputStream(file);
            fos.write(data);
            Log.i(TAG, "saveFileToExternalDir: file " + fileName + "saved successfully");
        } catch (Exception e) {
            Log.e(TAG, "saveFileToExternalDir: failed to create a file " + fileName, e);
        }
    }
}
