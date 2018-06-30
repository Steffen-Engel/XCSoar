/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "PCMResourcePlayer.hpp"

#include "PCMPlayerFactory.hpp"

#include "LogFile.hpp"
#include "ResourceLoader.hpp"
#include <windef.h> // for MAX_PATH
#include "Util/StringFormat.hpp"
#include "OS/FileUtil.hpp"
#include "LocalPath.hpp"

#include <utility>

PCMResourcePlayer::PCMResourcePlayer() :
    player(PCMPlayerFactory::CreateInstance())
{
}

bool
PCMResourcePlayer::PlayResource(const TCHAR *resource_name)
{
  PCMBufferDataSource::PCMData pcm_data =
      PCMBufferDataSource::PCMData::FromVoid(
          ResourceLoader::Load(resource_name, _T("WAVE")));
  if (pcm_data.IsNull()) {
    // load external file if existent
    // check local path for file
    AllocatedPath sndfile = LocalPath(resource_name);
    if (File::Exists(sndfile)) {
      TCHAR command[MAX_PATH];
      StringFormat(command, MAX_PATH, "aplay %s &", sndfile.c_str());
      system(command);
      return true;
    }
    LogFormat(_T("PCM resource \"%s\" not found!"), resource_name);
    return false;
  }

  const ScopeLock protect(lock);

  if (1 == buffer_data_source.Add(std::move(pcm_data))) {
    if (!player->Start(buffer_data_source)) {
      buffer_data_source.Clear();
      return false;
    }
  }

  return true;
}
