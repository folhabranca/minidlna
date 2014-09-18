/* Video thumbnail functions
 *
 * Project : minidlna
 * Website : http://sourceforge.net/projects/minidlna/
 *
 * MiniDLNA media server
 * Copyright (C) 2009  Justin Maggard
 *
 * This file is part of MiniDLNA.
 *
 * MiniDLNA is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * MiniDLNA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MiniDLNA. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __VIDEOTHUMB_H__
#define __VIDEOTHUMB_H__

int
video_thumb_generate_tofile(const char *moviefname, const char* thumbfname, int seek, int width);

int
video_thumb_generate_tobuff(const char *moviefname, void* imgbuffer, int seek, int width, enum AVPixelFormat pixfmt);

char*
video_thumb_generate_mta_file(const char *moviefname, int duration, int allblack);

#endif

