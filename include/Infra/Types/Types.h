//
//  "$Id: Types.h 16720 2010-12-01 09:51:53Z wang_haifeng $"
//
//  Copyright (c)1992-2007, ZheJiang Dahua Technology Stock CO.LTD.
//  All Rights Reserved.
//
//	Description:
//	Revisions:		Year-Month-Day  SVN-Author  Modification
//


#ifndef __DAHUA3_TYPES_H__
#define __DAHUA3_TYPES_H__

#include "IntTypes.h"
////////////////////////////////////////////////////////////////////////////////

// WIN32 Dynamic Link Library
#ifdef _MSC_VER

// 废弃INTERFACE_API的使用，每个模块DLL应该定义自己的XXX_API，请参考INFRA_API的定义
// Discard the use of INTERFACE_API. Each module DLL should define its own XXX_API. Please refer to the definition of INFRA_API
#ifdef _DLL_BUILD
	#define  INTERFACE_API _declspec(dllexport)
#elif defined _DLL_USE
	#define  INTERFACE_API _declspec(dllimport)
#else
	#define INTERFACE_API
#endif

#else
	#define INTERFACE_API
#endif

#if (defined (WIN32) || defined(WIN64))
#   define CALLMETHOD __stdcall
#else
#	define CALLMETHOD
#endif // end #if (defined (WIN32) || defined(WIN64))

#ifdef __GNUC__
#define INFINITE            0xFFFFFFFF  // Infinite timeout
#endif

#define MAX_POLYGON_PEAK_NUM        128     // 最大多边形顶点个数 // Maximum number of polygon vertices


// IP 地址结构
// IP address structure
typedef union tagMvSIpAddress
{
	uint8_t		c[4];
	uint16_t	s[2];
	uint32_t	l;
}IpAddress;

//////////////////////////////////////////////////////////////////////////
// 系统时间定义
// System time definition
typedef struct tagMvSSystemTime
{
	int32_t  year;		//< 年。	|year
	int32_t  month;		///< 月，	| month   eg:January = 1, February = 2, and so on.
	int32_t  day;		///< 日。
	int32_t  wday;		///< 星期 	| weekday eg:Sunday = 0, Monday = 1, and so on
	int32_t  hour;		///< 时。	| hour
	int32_t  minute;	///< 分。	| minute
	int32_t  second;	///< 秒。   | seconds
	int32_t  isdst;		///< 夏令时标识。 | Daylight saving time sign
}MvSSystemTime;	



// 2D点结构体定义
// Definition of 2D point structure
typedef struct tagMvSPoint
{
	int32_t     x;
	int32_t     y;
} MvSPoint;

typedef struct tagMvSPoint2Di32
{
	int32_t		x;
	int32_t		y;
} MvSPoint2Di32;

typedef struct tagMvSPoint2Di16
{
	int16_t		x;
	int16_t		y;
} MvSPoint2Di16;

typedef struct tagMvSPoint2Df32
{
	float		x;
	float		y;
} MvSPoint2Df32;

typedef struct tagMvSPoint2Df64
{
	double		x;
	double		y;
} MvSPoint2Df64;



// 3D点结构体定义
// Definition of 3D point structure
typedef struct tagMvSPoint3Di32
{
	int32_t		x;
	int32_t		y;
	int32_t		z;
} MvSPoint3Di32;

typedef struct tagMvSPoint3Di16
{
	int16_t		x;
	int16_t		y;
	int16_t		z;
} MvSPoint3Di16;

typedef struct tagMvSPoint3Df32
{
	float		x;
	float		y;
	float		z;
} MvSPoint3Df32;

typedef struct tagMvSPoint3Df64
{
	double		x;
	double		y;
	double		z;
} MvSPoint3Df64;



// 矩形
// rectangle
typedef struct tagMvSRect
{
	int32_t left;
	int32_t top;
	int32_t right;
	int32_t bottom;
} MvSRect;

// 尺寸
// size
typedef struct tagMvSSize
{
	int32_t w;
	int32_t h;
} MvSSize;

// 颜色
// color
typedef struct tagMvSColor
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
} MvSColor;

// 直线
// Line
typedef struct tagMvSLine
{
	MvSPoint start;
	MvSPoint end;
} MvSLine;

typedef struct tagMvSLinePA
{
	MvSPoint2Df32   point;
	float           angle;
}MvSLinePA;

// 时间段结构
// Time period structure
typedef struct tagMvSTimeSection
{
	int enable;			///< 使能 				| enable
	int startHour;		///< 开始时间:小时		| start time: hour
	int	startMinute;	///< 开始时间:分钟		| start time: minute
	int	startSecond;	///< 开始时间:秒钟		| start time: seconds
	int	endHour;		///< 结束时间:小时		| end time: hour
	int	endMinute;		///< 结束时间:分钟		| end time: minute
	int	endSecond;		///< 结束时间:秒钟		| end time: seconds
}MvSTimeSection;




// 二维线段线段定义
// 2D segment segment definition
//////////////////////////////////////////////////////////////////////////
typedef struct tagMvSLineSeg2Di32
{
	MvSPoint2Di32		pt1;
	MvSPoint2Di32    	pt2;
} MvSLineSeg2Di32;

typedef struct tagMvSLineSeg2Df32
{
	MvSPoint2Df32		pt1;
	MvSPoint2Df32		pt2;
} MvSLineSeg2Df32;

typedef struct tagMvSLineSegPtSlope2Df32
{
	float				angle;
	float				length;
	MvSPoint2Df32		pt;
} MvSLineSegPtSlope2Df32;

typedef union tagMvULineSeg
{
	MvSLineSeg2Df32		    twoPt;				///< 两点式 | Type of two point 
    MvSLineSegPtSlope2Df32	ptSlope;			///< 点斜式 | Type of point slope
	int32_t reserved[32];
}MvULineSeg;

typedef enum TypeLineSeg
{
    LINESEG_TOW_POINT = 0,                      ///< 两点式 | Type of two point 
    LINESEG_POINT_SLOPE,                        ///< 点斜式 | Type of point slope
}TypeLineSeg;

typedef struct tagMvSLineSeg
{
    TypeLineSeg         type;                   ///< 线段类型 | Type of line
    MvULineSeg          lineSeg;				///< 线段     | line
    int32_t             reserved[31];           ///< 预留字段 | reserved line
} MvSLineSeg;


// 尺寸定义
// define of size
/////////////////////////////////////////////////////////////////////////////////
typedef struct tagMvSSizei32
{
	int32_t		width;							///< 宽 | width
	int32_t		height;							///< 高 | height
} MvSSizei32;

typedef struct tagMvSSizef32
{
	float		width;							///< 浮点宽高 |float of width and height
	float		height;
} MvSSizef32;
/////////////////////////////////////////////////////////////////////////////////////

// 矩形框结构体
// rectangle of struct
//////////////////////////////////////////////////////////////////////////////////
typedef struct tagMvSRect2Di32
{
	MvSPoint2Di32		ul;	//左上 | left upper
	MvSPoint2Di32		lr;	//右下 | right down
} MvSRect2Di32;

typedef struct tagMvSRect2Df32
{
	MvSPoint2Df32	ul;	//左上	| left upper
	MvSPoint2Df32	lr;	//右下	| right down
} MvSRect2Df32;

//矩形
//rectangle
typedef struct tagMvSRectangle 
{
    float                   cx;         ///< 中心x 		| center of X
    float                   cy;         ///< 中心y 		| center of Y
    float                   lx;         ///< x方向长	| X direction length
    float                   ly;         ///< y方向长	| Y direction length
    float                   ra;         ///< 旋转角度	| Rotation angle
    float                   ska;        ///< 斜切角度	| Bevel angle
}MvSRectangle;

//圆
//circle
typedef struct tagMvSCircle
{
    float                   cx;         ///< 中心x		| center of X
    float                   cy;         ///< 中心y		| center of Y
    float                   radius;     ///< 半径		| radius
} MvSCircle;

// 圆弧 
// circular arc
typedef struct tagMvSArc
{
    float                   cx;         ///< 圆弧中心X	| Arc center x
    float                   cy;         ///< 圆弧中心Y	| Arc center Y
    float                   radius;     ///< 圆弧半径	| Arc radius
    float                   angleStart; ///< 起始角度	| Starting angle
    float                   angleSpan;  ///< 角度范围 	| range of angle
}MvSArc;


// 椭圆
// ellipse
typedef struct tagMvSEllipse
{
    float                   cx;         ///< 中心x		| center of X
    float                   cy;         ///< 中心y		| center of y
    float                   rx;         ///< x轴系数	| X axis coefficient
    float                   ry;         ///< y轴系数	| y axis coefficient
    float                   ra;         ///< 旋转角度  	| Rotation angle
    float                   ska;        ///< 斜切角度	| Bevel angle
}MvSEllipse;

// 圆环段
// Circular section
typedef struct tagMvSAnnulusSection
{
    float               cx;             ///< 中心x		| center of X
    float               cy;             ///< 中心y		| center of y
    float               innerR;         ///< 内圆半径 	| Radius of inner circle
    float               outerR;         ///< 外圆半径	| Radius of outner circle
    float               startAngle;     ///< 起始角度	| Starting angle
    float               endAngle;       ///< 终止角度	| Termination angle
    int32_t             reserved[26];   ///< 预留字段	| Reserved
} MvSAnnulusSection;

// 多边形
// polygon
typedef struct tagMvSPolygonf32
{
    int32_t             num;                            ///< 顶点数量 | Number of vertices
    MvSPoint2Df32       pts[MAX_POLYGON_PEAK_NUM];      ///< 顶点数据 | Data of vertices
    int32_t             reserved[31];                   ///< 预留字段 | Reserved
} MvSPolygonf32;

// 32位程序128个字节
// 128 bytes of 32-bit program
typedef union tagMvUGraphObj
{
	MvSRectangle		rect;						           ///< type == 0
	MvSCircle			circle;								   ///< type == 1
	MvSEllipse			ellipse;							   ///< type == 2
    MvSPolygonf32       polygon;
    MvSAnnulusSection   annulusSection;
	int32_t reserved[32];
}MvUGraphObj;

// 任意目标
// Arbitrary target
typedef struct tagMvSGraphBase
{
	int32_t				type;															///< 目标类型 | type of target
	MvUGraphObj			graph;															///< 目标数据 | data of target
}MvSGraphBase;

//////////////////////////////////////////////////////////////
// 图像结构体定义
// define of image structure
typedef struct tagMvSImage
{
    int32_t                 type;       // 图像格式，见PublicDefine.h中MvsImgType  		| Image format, see MvsImgtype in publicdefine.h
    int32_t                 dataType;   // 存储类型，见PublicDefine.h中MvsImgDataType	| Storage type, see MvsImgtype in publicdefine.h
    int32_t                 width;      // 传入图像数据宽度								| recieve image data width
    int32_t                 height;     // 传入图像数据高度								| recieve image data height
    MvSRect2Di32            roi;        // 有效数据区域									| Valid data area
    uint8_t                 *imageData; // 图像数据指针									| Image data pointer
    uint8_t                 *mask;      // 图像掩膜，0 (MVS_NULL)表示没有掩膜 			|Image mask, 0 (MVS_NULL) means no mask
    int32_t                 *reserved[22];  // 预留字段1 								| reserved
}MvSImage;


typedef struct tagMvSLineCoef
{
	float		    a;
	float		    b;
	float		    c;
}MvSLineCoef;


// ROI
typedef struct tagMvSROI
{
    int32_t							num;							//ROI个数  			| Number of ROI 
    MvSRect2Di32					*rect;							//ROI外接矩形		| rectangle of ROI
    MvSImage						*img;							//ROI图像			| image of ROI
}MvSROI;


/*coordinate struct*/
typedef struct tagMvSCoordinate
{
	float x;               // 平移x轴坐标				| Translate X coordinate
	float y;               // 平移y轴坐标				| Translate Y coordinate
	float scale;           // 缩放尺度					| Scaling scale
	float ratio;           // y轴单位与x轴单位的比率	| Ratio of y-axis units to X-axis units
	float ra;              // 旋转角					| Rotation angle
	float ska;             // 倾斜角					| Inclination angle
	int firstProc;        //模板匹配之后的第一次执行	| First execution after template matching
	int32_t *reserved[25]; // 预留字段1					| reserved
}MvSCoordinate;

// 图像类型定义（类型_扫描方式_图像类型_类型说明）
// Image type definition (type scan method ,image type description)
typedef enum tagMvSImgType
{
	//逐行扫描图像 | line by line scan image
	// Y平面保存格式 
	// Y plane save format
	MVS_IMGTP_UITL_Y			= 0x0100,
	// RGB平面保存格式
	// RGB plane save format
	MVS_IMGTP_UITL_RGB			= 0x0200,
	// RGB 24平面保存格式 - R在一个平面，G在一平面，B在一个平面。
	// RGB 24 plane save format - R in one plane, G in one plane, B in one plane.
	MVS_IMGTP_UITL_RGBP_24		= 0x0201,
	// HSI 平面保存格式 
	// HSI plane save format
	MVS_IMGTP_UITL_HSI			= 0x0400,
	// LAB 平面保存格式 
	// LAB plane save format
	MVS_IMGTP_UITL_LAB			= 0x0500,
	// LAB 平面保存格式 
	// LAB plane save format 
	MVS_IMGTP_UITL_DIF			= 0x0600,
	// XYZ 平面保存格式 
	// XYZ plane save format
	MVS_IMGTP_UITL_XYZ			= 0x0700,
	/* UITL_YUV*/
	MVS_IMGTP_UITL_YUV			= 0x1000,
	// YUV 420平面保存格式 - YUV是一个平面。
	// YUV 420 plane save format - YUV is a plane.
	MVS_IMGTP_UITL_YUV_420,
	// YUV 420平面保存格式 - Y是一个平面，U是一个平面，V也是一个平面。
	// YUV 420 plane save format - y is a plane, u is a plane, V is a plane.
	MVS_IMGTP_UITL_YUVP_420,
	// YUV 420半平面保存格式 - Y在一个平面，UV在另一平面（交错保存）。
	// YUV 420 plane save format - y is a plane, uv is a plane(interleaved preservation).
	MVS_IMGTP_UITL_YUVSP_420,
	// YUV 420半平面保存格式 - Y在一个平面，VU在另一平面（交错保存）。
	// YUV 420 plane save format - y is a plane, vu is a plane(interleaved preservation).
	MVS_IMGTP_UITL_YVUSP_420,
	// YUV 422平面保存格式 - YUV是一个平面。
	// YUV 420 plane save format - yUV is a plane.
	MVS_IMGTP_UITL_YUV_422,
	// YUV 422平面保存格式 - Y是一个平面，V是一个平面，U也是一个平面。
	// YUV 422 plane save format - y is a plane, V is a plane, u is a plane.
	MVS_IMGTP_UITL_YUVP_422,
	// YUV 422半平面保存格式 - Y是一个平面，UV是另一平面（交错保存）。
	// YUV 422 halfplane save format - y is one plane and UV is another (interleaved save).
	MVS_IMGTP_UITL_YUVSP_422,
	// YUV 422半平面保存格式 - Y在一个平面，VU在另一平面（交错保存）。
	// YUV 422 halfplane save format - y is one plane and VU is another (interleaved save).
	MVS_IMGTP_UITL_YVUSP_422,

	// 隔行扫描图像
	// Interlaced scan image
	// ITL_YUV
	MVS_IMGTP_ITL_YUV			= 0x3000,
	// YUV 420平面保存格式 - Y是一个平面，U是一个平面，V也是一个平面。
	// YUV 422 plane save format - y is a plane, U is a plane, V is a plane.
	MVS_IMGTP_ITL_YUVP_420,
	// YUV 420半平面保存格式 - Y在一个平面，UV在另一平面（交错保存）。
	// YUV 420 plane save format - y is a plane, uv is a plane(interleaved preservation).
	MVS_IMGTP_ITL_YUVSP_420,
	// YUV 420半平面保存格式 - Y在一个平面，VU在另一平面（交错保存）。
	// YUV 420 plane save format - y is a plane, vu is a plane(interleaved preservation).
	MVS_IMGTP_ITL_YVUSP_420,
	// YUV 422平面保存格式 - Y是一个平面，V是一个平面，U也是一个平面。
	// YUV 422 plane save format - y is a plane, V is a plane, U is a plane.
	MVS_IMGTP_ITL_YUVP_422,
	// YUV 422半平面保存格式 - Y是一个平面，UV是另一平面（交错保存）。
	// YUV 420 plane save format - X is a plane, uv is a plane(interleaved preservation).
	MVS_IMGTP_ITL_YUVSP_422,
	// YUV 422半平面保存格式 - Y在一个平面，VU在另一平面（交错保存）。
	// YUV 420 plane save format - y is a plane, vu is a plane(interleaved preservation).
	MVS_IMGTP_ITL_YVUSP_422,
	// 内格式定义
	// Internal format definition
}MvSImgType;


// 图像数据格式
// image datea format
typedef enum tagMvSImgDataType
{
	// 无符号8位 	| unsigned 8 bytes
	MVS_IMGDTP_U8			= 0,								
	// 有符号8位 	| signed 8 bytes
	MVS_IMGDTP_S8,
	// 有符号32位 	| signed 32 bytes
	MVS_IMGDTP_S32,			
	// 无符号32位  	| unsigned 32 bytes
	MVS_IMGDTP_U32,			
	// 有符号16位	| signed 16 bytes
	MVS_IMGDTP_S16,			
	// 无符号16位	| unsigned 16 bytes
	MVS_IMGDTP_U16,			
	// 浮点32位 	| float 32 bytes
	MVS_IMGDTP_F32,		
	// 浮点64位	| float 64 bytes
	MVS_IMGDTP_F64,
}MvSImgDataType;

// 目标类型
// type of object
typedef enum tagMvSObjectType
{
	// 点 			| point
	MVS_OBJTP_POINT,
	// 线段 		| line
	MVS_OBJTP_LINE_SEG,
	// 线 			| line
	MVS_OBJTP_LINE,
	// 圆			| circle
	MVS_OBJTP_CIRCLE,
	// 椭圆	   		| ellipse
	MVS_OBJTP_ELLIPSE,
	// 图像			| image
	MVS_OBJTP_IMG,
}MvSObjectType;

// 单应性矩阵	
// Homography matrix
typedef struct tagMvSHomMat2D
{
    float mat[9];
}MvSHomMat2D;

// 矩阵
// rectangle
typedef struct tagMvSMat2Di32
{
	int32_t				rows;		// 矩阵的行数 | Number of rows of matrix
	int32_t				cols;		// 矩阵的列数 | Number of columns of matrix
	int32_t				*data;		// 矩阵的数据 | Number of data of matrix
} MvSMat2Di32;

typedef struct tagMvSMat2Df32
{
	int32_t				rows;		// 矩阵的行数 | Number of rows of matrix
	int32_t				cols;		// 矩阵的列数 | Number of columns of matrix
	float				*data;		// 矩阵的数据 | Number of data of matrix
}MvSMat2Df32;

typedef struct tagMvSMat2Df64
{
	int32_t				rows;		// 矩阵的行数 | Number of rows of matrix
	int32_t				cols;		// 矩阵的列数 | Number of columns of matrix
	double				*data;		// 矩阵的数据 | Number of data of matrix
}MvSMat2Df64;

typedef struct tagMvSRotateRect
{
	float		centerX;		// 中心点		 | center point
	float		centerY;
	float		hWidth;			// 矩形半宽 高	 | Rectangular half width and height
	float		hHeight;
	float		angle;			// 旋转角度		 | Rotation angle
	int32_t     reserved[27];   // 预留字段		 | reserved
}MvSRotateRect;

typedef struct tagMvSRegion
{
	int32_t			imgWidth;
	int32_t			imgHeight;		// region所在图像宽高 | Image width and height of region
	uint32_t		area;           // 面积				  | area
	MvSPoint2Df32	center;         // 中心				  | center
	float			circularity;	// 圆度				  | circularity
	float			rectangularity;	// 矩形度			  | rectangularity
	float			convexity;		// 凸度				  | convexity
	float			contlength;		// 周长				  | contlength
	MvSRotateRect	minRect;		// 最小外接矩形		  | Minimum circumscribed rectangle
	int32_t			boxTop;			// 包围盒信息X1		  | Bounding box information x1
	int32_t			boxLeft;		// 包围盒信息Y1		  | Bounding box information y1
	int32_t			boxRight;		// 包围盒信息X2		  | Bounding box information x2 
	int32_t			boxDown;		// 包围盒信息Y2		  | Bounding box information y2

	int32_t			conFlag;		// 1 八联通 2 四联通  | 1 Eight Unicom 2 four Unicom
	uint32_t		conSize;		// 轮廓大小 		  | Outline size
	uint16_t		*conAdrX;		// 轮廓地址序例，数据类型为jU16		|  profile address sequence, data type is ju16
	uint16_t		*conAdrY;		// 轮廓地址序例，数据类型为jU16		|  profile address sequence, data type is ju16

	int32_t			posFlag;
	uint32_t		posSize;		// 区域大小 						| region size
	uint16_t		*areaAdrX;		// 区域地址序例，数据类型为jU16		|  region address sequence, data type is ju16
	uint16_t		*areaAdrY;		// 区域地址序例，数据类型为jU16		|  region address sequence, data type is ju16

	int32_t			runFlag;		// 1 八联通 2 四联通 | 1 Eight Unicom 2 four Unicom
	uint32_t		runNum;         // 行程数量			 | Number of route
	uint16_t		*runRow;        // 行程所在行		 | rows of route
	uint16_t		*runStart;      // 行程起始列		 | columns of route
	uint16_t		*runEnd;        // 行程终止列		 | terminate columns of route
	int32_t			reserved[6];    // 预留字段			 | reserved
}MvSRegion;

typedef struct tagMvSMultiRegion
{
	uint32_t areaNum;       // 区域个数 | number of regions
	MvSRegion *regions;		// 区域		| regions 
}MvSMultiRegion;

#endif// __DAHUA_TYPES_H__