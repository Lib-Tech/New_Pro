// ------------------  汉字字模的数据结构定义 ------------------------ //
typedef struct typFNT_GB                // 汉字字模数据结构
{
       signed char Index[2];              // 汉字内码索引
       char Msk[72];                      // 点阵码数据
}typFNT_GB;

/////////////////////////////////////////////////////////////////////////
// 汉字字模表                                                          //
// 汉字库: 宋体24.dot,横向取模左高位,数据排列:从左到右从上到下         //
/////////////////////////////////////////////////////////////////////////
const unsigned char GB_24[] =        // 数据表
{
	0x00, 0x00, 0x00, 0x00, 0x07, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x24, 0x3c, 0x1c, 0x04, 0x04, 
	0x04, 0x04, 0x04, 0x04, 0x0c, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x02, 0x0c, 0xf0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //广 
	0x00, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x07, 0x3c, 0x34, 0x24, 0x04, 0x04, 0x04, 
	0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x1c, 0x38, 0x69, 
	0x8b, 0x09, 0x08, 0x08, 0xff, 0x48, 0x08, 0x0a, 0x09, 0x08, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x80, 0x04, 0x06, 0x06, 0xfe, 0x00, 0x00, 0x00, 
	0x00, 0x80, 0xc0, 0x60, 0x38, 0x18, 0x00, 0x00, //东
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x3f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x0c, 0x38, 0xe0, 
	0x40, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x80, 0x40, 0x20, 0x30, 0x1c, 0x0f, 0x03, 0x00, 0x00, 
	0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x06, 0x0e, 0xfc, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, //小
	0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0f, 0x08, 0x08, 0x08, 0x08, 
	0x08, 0x08, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
	0x20, 0x23, 0x3e, 0xf0, 0x30, 0x2c, 0x23, 0x20, 0x20, 0x20, 0x20, 0x20, 0x60, 0x20, 0x00, 0x00, 
	0x00, 0x00, 0x02, 0x04, 0x04, 0x08, 0x10, 0x20, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 
	0x60, 0x30, 0x18, 0x0c, 0x0c, 0x0c, 0x00, 0x00, //天
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x3f, 0x20, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x81, 
	0x83, 0x86, 0x98, 0xb0, 0xe0, 0x80, 0xff, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x08, 0x10, 0x20, 0x40, 0xc0, 0x80, 0x00, 0x00, 0x04, 0x04, 0x06, 0x06, 0xfc, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //才
	0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x1f, 0x10, 0x30, 0x30, 0x00, 0x00, 0x00, 0x04, 0x07, 0x00, 
	0x00, 0x00, 0x3f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x41, 0x46, 0x5c, 0xff, 0x4f, 
	0x48, 0x4c, 0x86, 0x00, 0x20, 0x10, 0x0c, 0x01, 0x01, 0x01, 0xff, 0x01, 0x02, 0x06, 0x02, 0x00, 
	0x00, 0x20, 0xc0, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 
	0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, //科
	0x00, 0x00, 0x01, 0x01, 0x01, 0x7f, 0x3f, 0x01, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x3f, 
	0x21, 0x01, 0x01, 0x01, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x06, 0x04, 0x04, 0xff, 0xff, 0x08, 
	0x10, 0x10, 0x10, 0x10, 0x1c, 0x13, 0x10, 0xf0, 0x10, 0x11, 0x17, 0x1c, 0x10, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x04, 0x06, 0xfe, 0xfc, 0x00, 0x02, 0x02, 0x02, 0x04, 0x04, 0x08, 0xd0, 0x30, 
	0x70, 0xc8, 0x0c, 0x04, 0x06, 0x06, 0x00, 0x00, //技
	0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x07, 0x1c, 0x34, 0x24, 0x04, 0x04, 0x04, 
	0x04, 0x04, 0x04, 0x04, 0x0c, 0x0c, 0x04, 0x00, 0x00, 0x02, 0x02, 0x04, 0x08, 0x10, 0x20, 0x7f, 
	0xc4, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0xff, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x44, 0x44, 
	0x46, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //有
	0x00, 0x00, 0x00, 0x1f, 0x10, 0x10, 0x11, 0x1e, 0x38, 0x00, 0x00, 0x1f, 0x11, 0x11, 0x11, 0x11, 
	0x11, 0x11, 0x11, 0x3f, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x40, 0xa0, 0x10, 
	0x0f, 0x03, 0x00, 0xff, 0x10, 0x10, 0x1e, 0x11, 0x10, 0x11, 0x1b, 0xf6, 0x06, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xfe, 0x00, 0x80, 0xc0, 0xc0, 0x80, 0x00, 0x00, 0xfe, 0x04, 0x08, 0x08, 0xd0, 
	0xe0, 0x30, 0x18, 0x0c, 0x0c, 0x04, 0x00, 0x00, //限
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0e, 0x18, 0x10, 0x00, 0x00, 0x00, 0x3c, 0x03, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x08, 0x10, 0x20, 0xc0, 0x80, 
	0x00, 0x03, 0x0e, 0x3c, 0x10, 0x00, 0x00, 0x00, 0xc0, 0x60, 0x30, 0x18, 0x18, 0x08, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x08, 0x0c, 0x1c, 0x6c, 0xc8, 0x08, 0x08, 0x08, 0x08, 0x08, 0x88, 0x48, 
	0x28, 0x1c, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, //公
	0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 
	0x10, 0x10, 0x10, 0x1f, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x9f, 0x90, 0x90, 
	0x90, 0x90, 0x90, 0x90, 0x90, 0x9f, 0x10, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xe0, 0x00, 0x00, 
	0x04, 0x04, 0x0e, 0xfc, 0x00, 0x00, 0x00, 0x00, //司	
};

//struct typFNT_GB GB_24x[] =
//{
//"长", 	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x20, 0x00, 0x00, 0x00, 
//		0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
//		0x80, 0x80, 0x8f, 0xfc, 0xb0, 0x8c, 0x83, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 
//		0x00, 0x00, 0x02, 0x02, 0x04, 0x04, 0x08, 0x10, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 
//		0x60, 0x30, 0x18, 0x0c, 0x0e, 0x04, 0x00, 0x00,
//
//};

// 汉字表：
// 请说

