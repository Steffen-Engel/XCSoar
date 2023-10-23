// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

package org.xcsoar;

import java.util.Map;
import java.util.HashMap;
import java.io.File;
import android.media.MediaPlayer;
import android.content.Context;
import android.net.Uri;
import java.io.IOException;

public class SoundUtil {
  private static Map<String, Integer> resources = new HashMap<String, Integer>();

  static {
    resources.put("IDR_FAIL", R.raw.fail);
    resources.put("IDR_INSERT", R.raw.insert);
    resources.put("IDR_REMOVE", R.raw.remove);
    resources.put("IDR_WAV_BEEPBWEEP", R.raw.beep_bweep);
    resources.put("IDR_WAV_CLEAR", R.raw.beep_clear);
    resources.put("IDR_WAV_DRIP", R.raw.beep_drip);
  }

  public static boolean play(Context context, String name) {
    Integer id = resources.get(name);
    if (id == null)
    {
      // no internal resource, try to load external file
      try {
        MediaPlayer mp = new MediaPlayer();
        mp.setDataSource(name);
        mp.prepare();
        mp.start();
        return true;
      }
      catch (IllegalArgumentException e) {
        // dummy, we actually do not care
      }
      catch (IllegalStateException e) {
        // dummy, we actually do not care
      }
      catch (IOException e) {
        // dummy, we actually do not care
      }
      return false;
    }

    return run(MediaPlayer.create(context, id));
  }

  public static boolean playExternal(Context context, String path) {
    final File file = new File(path);
    if (!file.exists()) {
      return false;
    }

    return run(MediaPlayer.create(context, Uri.fromFile(file)));
  }

  private static boolean run(final MediaPlayer mp) {
    if (mp == null) {
      return false;
    }

    mp.setOnCompletionListener(new MediaPlayer.OnCompletionListener() {
      @Override
      public void onCompletion(MediaPlayer mediaPlayer) {
        mp.release();
      }
    });

    mp.start();
    return true;
  }
}
