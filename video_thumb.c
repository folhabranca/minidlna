/* MiniDLNA project
 *
 * http://sourceforge.net/projects/minidlna/
 *
 * MiniDLNA media server
 * Copyright (C) 2008-2009  Justin Maggard
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
 *
 * Portions of the code from the MiniUPnP project:
 *
 * Copyright (c) 2006-2007, Thomas Bernard
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The name of the author may not be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "config.h"

#include <unistd.h>
#include <libgen.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef ENABLE_VIDEO_THUMB
#if HAVE_FFMPEG_LIBSWSCALE_SWSCALE_H
#include <ffmpeg/libswscale/swscale.h>
#elif HAVE_LIBAV_LIBSWSCALE_SWSCALE_H
#include <libav/libswscale/swscale.h>
#elif HAVE_LIBSWSCALE_SWSCALE_H
#include <libswscale/swscale.h>
#elif HAVE_FFMPEG_SWSCALE_H
#include <ffmpeg/swscale.h>
#elif HAVE_LIBAV_SWSCALE_H
#include <libav/swscale.h>
#elif HAVE_SWSCALE_H
#include <swscale.h>
#endif
#endif

#include "upnpglobalvars.h"
#include "image_utils.h"
#include "libav.h"
#include "log.h"
#include "video_thumb.h"
#include "utils.h"


#ifdef ENABLE_VIDEO_THUMB
static int
get_video_packet(AVFormatContext *ctx, AVCodecContext *vctx,
		 AVPacket *pkt, AVFrame *frame, int vstream)
{
	int moreframes, decoded, finished;
	int avret, i, j, l;

	av_free_packet(pkt);

	finished = 0;
	for (l = 0; !finished && l < 500; l++)
	{
		moreframes = 1;
		decoded = 0;
		for (i = 0; moreframes && ! decoded && i < 2000; i++)
		{
			avret = av_read_frame(ctx, pkt);
			if ((moreframes = (avret >= 0)))
			{
				if (!(decoded = (pkt->stream_index == vstream)))
				{
					av_free_packet(pkt);
					continue;
				}

				for (j = 0; !finished && j < 50; j++)
				{
					lav_frame_unref(frame);
					avret = lav_avcodec_decode_video(vctx, frame, &finished, pkt);
					if (avret < 0)
						return 0;
				}


			}
		}
	}

	return (finished > 0);
}

static int
video_seek(int seconds, AVFormatContext *ctx, AVCodecContext *vctx,
		 AVPacket *pkt, AVFrame *frame, int vstream)
{
	int64_t tstamp = AV_TIME_BASE * (int64_t) seconds;

	if (tstamp < 0)
		tstamp = 0;

	if ((av_seek_frame(ctx, -1, tstamp, 0) >=0))
		avcodec_flush_buffers(ctx->streams[vstream]->codec);

	return get_video_packet(ctx, vctx, pkt, frame, vstream);
}
#endif

int
video_thumb_generate_tofile(const char *moviefname, const char *thumbfname, int seek, int width)
{
#ifdef ENABLE_VIDEO_THUMB
	image_s img;
	int ret = 0;
	clock_t start, end;

	start = clock();
	memset(&img, 0, sizeof(image_s));

	if ((video_thumb_generate_tobuff(moviefname, &img, seek, width, PIX_FMT_RGB32_1) < 0))
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tofile: unable to generate thumbnail to buffer! \n");
		return -1;
	}

	if (!image_save_to_jpeg_file(&img, (char*) thumbfname))
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tofile: unable to save jpeg file : %s \n", thumbfname);
		ret = -1;
	}

	free(img.buf);

	end = clock();
	DPRINTF(E_DEBUG, L_METADATA, "Generated thumbnail for (%s) in %ldms.\n", moviefname, (end-start) * 1000 / CLOCKS_PER_SEC);

	return ret;
#else
	return -1;
#endif
}

#ifdef ENABLE_VIDEO_THUMB
static int
video_thumb_generate_ctx_tobuff(AVFormatContext *fctx, void* imgbuffer, int seek, int width, enum AVPixelFormat pixfmt)
{
	AVFrame *frame = NULL, *scframe = NULL;
	AVPacket packet;
	AVCodecContext *vcctx = NULL;
	AVCodec *vcodec = NULL;
	struct SwsContext *scctx = NULL;
	int avret, i, vs, ret = -1;
	int dwidth, dheight;
	image_s* buffer = (image_s*) imgbuffer;

	if (!fctx)
		return -1;

	if (!buffer)
		return -1;

	if (seek < 0)
		seek = 0;
	else if (seek > 90)
		seek = 90;

	if (width < 64)
		width = 64;
	else if (width > 480)
		width = 480;

	av_init_packet(&packet);

	vs = -1;
	for (i =0; i<fctx->nb_streams; i++)
	{
		if (fctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
		{
			vs = i;
			vcctx = fctx->streams[vs]->codec;
			break;
		}
	}

	if( vs < 0)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: unable to find a video stream \n");
		return ret;
	}

	vcodec = avcodec_find_decoder(vcctx->codec_id);
	if (!vcodec)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: unable to find a video decoder \n");
		goto thumb_generate_error;
	}

	vcctx->workaround_bugs = 1;

	avret =  lav_avcodec_open(vcctx, vcodec, NULL);
	if(avret < 0)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: unable to open a decoder \n");
		goto thumb_generate_error;
	}

	frame = lav_frame_alloc();
	if (!frame)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: malloc error!! Unable to alloc memory for a new frame \n");
		goto thumb_generate_error;
	}

	if (!video_seek(seek*(fctx->duration/AV_TIME_BASE)/100,
		fctx, vcctx, &packet, frame, vs))
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: unable to seek video for %d%% position \n", seek);
		goto thumb_generate_error;
	}

	if (frame->interlaced_frame)
	{
		DPRINTF(E_DEBUG, L_METADATA, "video_thumb_generate_tobuff: got an interlaced video \n");
		avpicture_deinterlace((AVPicture*) frame, (AVPicture*) frame,
					vcctx->pix_fmt, vcctx->width, vcctx->height);
	}

	dwidth = width;
	dheight = (int) ((float) (width * vcctx->height) / vcctx->width );

	scframe = lav_frame_alloc();
	if (!scframe)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: malloc error!! Unable to alloc memory for the scaled frame \n");
		goto thumb_generate_error;
	}

	avret = avpicture_alloc((AVPicture*)scframe, pixfmt, dwidth, dheight);
	if (avret < 0)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: malloc error!! Unable to alloc memory for the scaled frame buffer \n");
		goto thumb_generate_error;
	}

	scctx = sws_getCachedContext(scctx, vcctx->width, vcctx->height, vcctx->pix_fmt,
					dwidth, dheight, pixfmt, SWS_BICUBIC, NULL, NULL, NULL);
	if (!scctx)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: Unable to get a scale context! \n");
		goto thumb_generate_error;
	}

	avret = sws_scale(scctx, (const uint8_t * const*)frame->data, frame->linesize, 0, vcctx->height,
			scframe->data, scframe->linesize);
	if (avret <= 0)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: Unable to scale thumbnail! \n");
		goto thumb_generate_error;
	}

	free(buffer->buf);
	buffer->buf = (pix*) malloc(sizeof(uint8_t) * scframe->linesize[0] * dheight);
	if(!buffer->buf)
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: malloc error!! Unable to alloc memory to buffer the picture \n");
		goto thumb_generate_error;
	}

	buffer->width = dwidth;
	buffer->height = dheight;
	memcpy(buffer->buf, scframe->data[0], scframe->linesize[0] * buffer->height);

	ret = 0;

thumb_generate_error:
	sws_freeContext(scctx);
	avpicture_free((struct AVPicture*)scframe);
	av_free(scframe);
	av_free_packet(&packet);
	av_free(frame);
	avcodec_close(vcctx);

	return ret;
}
#endif

int
video_thumb_generate_tobuff(const char *moviefname, void* imgbuffer, int seek, int width, enum AVPixelFormat pixfmt)
{
#ifdef ENABLE_VIDEO_THUMB
	AVFormatContext *fctx = NULL;
	int ret = -1;

	if(!moviefname)
		return -1;

	if (lav_open(&fctx, moviefname))
	{
		DPRINTF(E_WARN, L_METADATA, "video_thumb_generate_tobuff: unable to open movie file (%s) \n", moviefname);
		return -1;
	}

	ret = video_thumb_generate_ctx_tobuff(fctx, imgbuffer, seek, width, pixfmt);

	lav_close(fctx);
	return ret;
#else
	return -1;
#endif
}

#define MTA_WIDTH 96

char*
video_thumb_generate_mta_file(const char *moviefname, int duration, int allblack)
{
	static const char mta_header[] =
		"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
		"<SEC:SECMeta xsi:schemaLocation=\"urn:samsung:metadata:2009 Video_Metadata_v1.0.xsd\""
		" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:SEC=\"urn:samsung:metadata:2009\">\r\n"
		"<SpecVersion>1.0</SpecVersion>\r\n"
		"<MediaInformation>\r\n"
		"<VideoLocator>\r\n"
		"<MediaUri>file://samsung_content.con</MediaUri>\r\n"
		"</VideoLocator>\r\n"
		"</MediaInformation>\r\n"
		"<ContentInformation>\r\n"
		"<Chaptering>\r\n";

	static const char mta_tail[] =
		"</Chaptering>\r\n"
		"</ContentInformation>\r\n"
		"</SEC:SECMeta>\r\n";

	static const char mta_chapter_header[] =
		"<ChapterSegment>\r\n"
		"<KeyFrame>\r\n"
		"<InlineMedia>";

	static const char mta_chapter_tail[] =
		"</InlineMedia>\r\n"
		"</KeyFrame>\r\n"
		"<MediaPosition>\r\n"
		"<MediaTime timePoint=\"%d\"/>\r\n"
		"</MediaPosition>\r\n"
		"</ChapterSegment>\r\n";

	/* 128x72 Black JPEG base 64 encoded */
	static const char bjpg64[] =
		"/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAEBAQEBAQEBAQEBAQECAgMCAgICAgQDAwIDBQQFBQUEBAQFB"
		"gcGBQUHBgQEBgkGBwgICAgIBQYJCgkICgcICAj/2wBDAQEBAQICAgQCAgQIBQQFCAgICAgICAgICAgICAg"
		"ICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAj/wAARCABIAIADASIAAhEBAxEB/8QAHwAAA"
		"QUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1F"
		"hByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZ"
		"WZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NX"
		"W19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAt"
		"REAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8Rc"
		"YGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXm"
		"JmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAM"
		"BAAIRAxEAPwD/AD/6KKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKK"
		"KKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKK"
		"ACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKAP//Z";

#ifdef ENABLE_VIDEO_THUMB
	AVFormatContext *fctx = NULL;
	unsigned char *jpeg;
	int jpegsize;
#endif
	image_s img;
	FILE *fp = NULL;
	char *mta_path, *mta_dir;
	char *img64 = NULL;
	size_t sizei64;
	int i, res, ret = -1;
	int per[] = { 15, 35, 50, 65, 85 };
	clock_t start, end;

	if (!moviefname)
		return 0;

	if (duration <= 0)
		return 0;

	if (!is_video(moviefname))
		return 0;

	memset(&img, 0, sizeof(image_s));

	start = clock();

	res = xasprintf(&mta_path, "%s/art_cache%s.mta", db_path, moviefname);
	if ( res < 0 )
		return 0;

	if ( !access(mta_path, F_OK) )
	{
		DPRINTF(E_INFO, L_METADATA, "The MTA file (%s) already exists.", mta_path);
		return mta_path;
	}

	res = xasprintf(&mta_dir, "%s/art_cache%s.mta", db_path, moviefname);
	if ( res < 0 )
	{
		mta_dir = NULL;
		goto mta_error;
	}

	if ( make_dir(dirname(mta_dir), S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH) )
		goto mta_error;

	if ( !(fp = fopen(mta_path, "w")) )
		goto mta_error;


#ifdef ENABLE_VIDEO_THUMB
	if(!allblack)
		lav_open(&fctx, moviefname);
#endif

	res = fwrite(mta_header, 1, sizeof(mta_header) -1, fp);
	if(res != sizeof(mta_header) - 1)
		goto mta_error;

	for (i = 0; i < sizeof(per)/sizeof(int); i++)
	{
		res = fwrite(mta_chapter_header, 1, sizeof(mta_chapter_header) - 1, fp);
		if(res != sizeof(mta_chapter_header) - 1)
			goto mta_error;

#ifdef ENABLE_VIDEO_THUMB
		if ( !allblack && fctx)
		{
			res = video_thumb_generate_ctx_tobuff(fctx, &img, per[i], MTA_WIDTH, PIX_FMT_RGB32_1);
			if (img.buf)
			{
				jpeg = image_save_to_jpeg_buf(&img, &jpegsize);
				if(jpeg)
				{
					img64 = base64_encode(jpeg, jpegsize, &sizei64);
					free(jpeg);
					jpeg = NULL;
				}
			}
		}
#endif

		if (!allblack && img64)
		{
			res = fwrite(img64, 1, sizei64, fp);
			free(img64);
			if(res != sizei64)
				goto mta_error;
		}
		else
		{
			res = fwrite(bjpg64, 1, sizeof(bjpg64) -1, fp);
			if(res != sizeof(bjpg64) -1)
				goto mta_error;
		}
		fprintf(fp, mta_chapter_tail, (duration * per[i])/100);
	}
	res = fwrite(mta_tail, 1, sizeof(mta_tail) - 1, fp);
	if( res != sizeof(mta_tail) -1 )
		goto mta_error;

	ret = 0;

	end = clock();
	DPRINTF(E_DEBUG, L_METADATA, "Generated MTA file for (%s) in %ldms.\n", moviefname, (end-start) * 1000 / CLOCKS_PER_SEC);

mta_error:
	free(img.buf);
#ifdef ENABLE_VIDEO_THUMB
	if (fctx)
		lav_close(fctx);
#endif
	fclose(fp);
	free(mta_dir);
	if (ret)
	{
		remove(mta_path);
		free(mta_path);
		mta_path = NULL;
	}

	return mta_path;
}
