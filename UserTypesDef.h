/*****************************************************************************************************
* FileName:                    UserTypesDef.h
*
* Description:                 �����������Ͷ���
*
* Author:                      YanDengxue, Fiberhome-Fuhua
*
* Rev History:
*       <Author>        <Data>        <Hardware>     <Version>        <Description>
*     YanDengxue   2011-03-29 15:30       --           1.00             Create
*****************************************************************************************************/
#ifndef _User_Types_Define_H
#define _User_Types_Define_H

#ifdef __cplusplus
extern "C" {
#endif

//====================================================================================================
// ���Ͷ���
//====================================================================================================
typedef char                     int8;
typedef unsigned char            Uint8;
typedef short                    int16;
typedef unsigned short           Uint16;
typedef long                     int32;
typedef unsigned long            Uint32;
typedef long long                int64;
typedef unsigned long long       Uint64;
typedef volatile unsigned long   VUint32;
typedef volatile unsigned short  VUint16;
typedef volatile unsigned char   VUint8;
typedef volatile long            Vint32;
typedef volatile short           Vint16;
typedef volatile char            Vint8;
typedef volatile float           Vfloat;
typedef volatile long long          Vint64;
typedef volatile unsigned long long VUint64;

//====================================================================================================
// �궨��
//====================================================================================================
// NULL����
#ifndef NULL
#define NULL (void *)0
#endif

#ifndef far
#define __FAR__       far
#endif

#ifndef near
#define __NEAR__      near
#endif

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#define LLSB(x)     ((x) & 0xffu)// 32bit word byte/word swap macros
#define LHSB(x)    (((x) >> 8u) & 0xffu)
#define HLSB(x)    (((x) >> 16u) & 0xffu)
#define HHSB(x)    (((x) >> 24u) & 0xffu)

#define LSHW(x)     ((x) & 0xffffu)// 32bit word byte/word swap macros
#define HSHW(x)    (((x) >> 16u) & 0xffffu)

//====================================================================================================
// ���򷵻ؽ��
//====================================================================================================
#define NORMAL_ERROR       -1// ����ִ��ʧ��
#define NORMAL_SUCCESS      0// ����ִ�гɹ�
#define PTR_ERROR           NULL// ָ�뷵��ʧ��

#ifdef __cplusplus
}
#endif

#endif

