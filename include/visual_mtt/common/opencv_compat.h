#pragma once

// Backward compatibility definitions for Opencv 3 as of Opencv 4
#if CV_MAJOR_VERSION > 3

#ifndef CV_TERMCRIT_ITER
#define CV_TERMCRIT_ITER cv::TermCriteria::MAX_ITER
#endif

#ifndef CV_WINDOW_NORMAL
#define CV_WINDOW_NORMAL cv::WINDOW_NORMAL
#endif

#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY
#endif

#ifndef CV_BGR2RGB
#define CV_BGR2RGB cv::COLOR_BGR2RGB
#endif

#ifndef CV_RGB2GRAY
#define CV_RGB2GRAY cv::COLOR_RGB2GRAY
#endif

#ifndef CV_BGRA2GRAY
#define CV_BGRA2GRAY cv::COLOR_BGRA2GRAY
#endif

#ifndef CV_GRAY2RGBA
#define CV_GRAY2RGBA cv::COLOR_GRAY2RGBA
#endif

#ifndef CV_GRAY2RGB
#define CV_GRAY2RGB cv::COLOR_GRAY2RGB
#endif

#ifndef CV_GRAY2BGRA
#define CV_GRAY2BGRA cv::COLOR_GRAY2BGRA
#endif

#ifndef CV_GRAY2BGR565
#define CV_GRAY2BGR565 cv::COLOR_GRAY2BGR565
#endif

#ifndef CV_RGBA2BGR565
#define CV_RGBA2BGR565 cv::COLOR_RGBA2BGR565
#endif

#ifndef CV_RGBA2RGB
#define CV_RGBA2RGB cv::COLOR_RGBA2RGB
#endif

#ifndef CV_RGB2RGBA
#define CV_RGB2RGBA cv::COLOR_RGB2RGBA
#endif

#ifndef CV_RGB2BGR565
#define CV_RGB2BGR565 cv::COLOR_RGB2BGR565
#endif

#ifndef CV_RGBA2BGR
#define CV_RGBA2BGR cv::COLOR_RGBA2BGR
#endif

#ifndef CV_RGBA2GRAY
#define CV_RGBA2GRAY cv::COLOR_RGBA2GRAY
#endif

#ifndef CV_ADAPTIVE_THRESH_GAUSSIAN_C
#define CV_ADAPTIVE_THRESH_GAUSSIAN_C cv::ADAPTIVE_THRESH_GAUSSIAN_C
#endif

#ifndef CV_THRESH_BINARY
#define CV_THRESH_BINARY cv::THRESH_BINARY
#endif

#ifndef CV_BGR2HSV
#define CV_BGR2HSV cv::COLOR_BGR2HSV
#endif

#ifndef CV_BGR2Lab
#define CV_BGR2Lab cv::COLOR_BGR2Lab
#endif

#ifndef CV_RGB2HSV
#define CV_RGB2HSV cv::COLOR_RGB2HSV
#endif

#ifndef CV_HSV2RGB
#define CV_HSV2RGB cv::COLOR_HSV2RGB
#endif

#ifndef CV_THRESH_OTSU
#define CV_THRESH_OTSU cv::THRESH_OTSU
#endif

#ifndef CV_THRESH_BINARY
#define CV_THRESH_BINARY THRESH_BINARY
#endif

#ifndef CV_MOP_CLOSE
#define CV_MOP_CLOSE cv::MORPH_CLOSE
#endif

#ifndef CV_RETR_CCOMP
#define CV_RETR_CCOMP cv::RETR_CCOMP
#endif

#ifndef CV_RETR_LIST
#define CV_RETR_LIST cv::RETR_LIST
#endif

#ifndef CV_RETR_TREE
#define CV_RETR_TREE cv::RETR_TREE
#endif

#ifndef CV_RETR_FLOODFILL
#define CV_RETR_FLOODFILL cv::RETR_FLOODFILL
#endif

#ifndef CV_RETR_EXTERNAL
#define CV_RETR_EXTERNAL cv::RETR_EXTERNAL
#endif

#ifndef CV_CHAIN_APPROX_SIMPLE
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE
#endif

#ifndef CV_CHAIN_APPROX_TC89_KCOS
#define CV_CHAIN_APPROX_TC89_KCOS cv::CHAIN_APPROX_TC89_KCOS
#endif

#ifndef CV_FILLED
#define CV_FILLED cv::FILLED
#endif

#ifndef CV_REDUCE_SUM
#define CV_REDUCE_SUM cv::REDUCE_SUM
#endif

#ifndef CV_REDUCE_AVG
#define CV_REDUCE_AVG cv::REDUCE_AVG
#endif

#ifndef CV_AA
#define CV_AA cv::LINE_AA
#endif

#ifndef CV_RANSAC
#define CV_RANSAC cv::RANSAC
#endif

#ifndef CV_StsBadArg
#define CV_StsBadArg cv::Error::StsBadArg
#endif

#ifndef cvFastArctan
#define cvFastArctan( Y, X ) ( std::atan2( (Y), (X) ) )
#endif

#ifndef CV_IMWRITE_JPEG_QUALITY
#define CV_IMWRITE_JPEG_QUALITY cv::IMWRITE_JPEG_QUALITY
#endif

#ifndef CV_IMWRITE_PNG_QUALITY
#define CV_IMWRITE_PNG_QUALITY cv::IMWRITE_PNG_QUALITY
#endif

#ifndef CV_LOAD_IMAGE_COLOR
#define CV_LOAD_IMAGE_COLOR cv::IMREAD_COLOR
#endif

#ifndef CV_COMP_CORREL
#define CV_COMP_CORREL cv::HISTCMP_CORREL
#endif

#ifndef CV_WINDOW_AUTOSIZE
#define CV_WINDOW_AUTOSIZE cv::WINDOW_AUTOSIZE
#endif

#ifndef CV_CAP_PROP_FRAME_HEIGHT
#define CV_CAP_PROP_FRAME_HEIGHT cv::CAP_PROP_FRAME_HEIGHT
#endif

#ifndef CV_CAP_PROP_FRAME_COUNT
#define CV_CAP_PROP_FRAME_COUNT cv::CAP_PROP_FRAME_COUNT
#endif

#ifndef CV_CAP_PROP_POS_FRAMES
#define CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#endif

#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY cv::COLOR_RGB2GRAY
#endif

#ifndef CV_FOURCC
#define CV_FOURCC(c1, c2, c3, c4) ( cv::VideoWriter::fourcc(c1, c2, c3, c4) )
#endif

#endif //endif CV_MAJOR_VERSION