/*****************************************************************************************************
* FileName:                    PeripheralDownload.c
*
* Description:                 DSP FPGA等程序下载驱动
*
* Author:                      YanDengxue, Fiberhome-Fuhua
*
* Rev History:
*       <Author>        <Data>        <Hardware>     <Version>        <Description>
*     YanDengxue   2011-03-29 15:30       --           1.00             Create
*****************************************************************************************************/
//====================================================================================================
// 本文件使用的头文件
//====================================================================================================
// 库头文件
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

// 自定义头文件
#include "UserTypesDef.h"
#include "OmapL138Register.h"
#include "SystemBase.h"

//====================================================================================================
// 宏定义
//====================================================================================================
#define PERIPHERAL_DOWNLOAD_MINOR 20

#define DSP_RESET_VECTOR_ADDR     (SHAREDDR_PHYS + SHAREDDR_SIZE - 0x400u)
#define pDSP_RESET_VECTOR_VA      ((Uint32 *)(SHAREDDR_ADDRESS(DSP_RESET_VECTOR_ADDR)))

#define FPGA_INIT_B    (pGPIO_VA->IN_DATA67  &  (GPIO_GP6P12))
#define FPGA_DONE      (pGPIO_VA->IN_DATA67  &  (GPIO_GP6P8))
#define FPGA_CCLK_H    (pGPIO_VA->OUT_DATA67 |= (GPIO_GP6P11))
#define FPGA_CCLK_L    (pGPIO_VA->OUT_DATA67 &= (~(GPIO_GP6P11)))
#define FPGA_DIN_H     (pGPIO_VA->OUT_DATA67 |= (GPIO_GP6P10))
#define FPGA_DIN_L     (pGPIO_VA->OUT_DATA67 &= (~(GPIO_GP6P10)))
#define FPGA_PROG_B_H  (pGPIO_VA->OUT_DATA67 |= (GPIO_GP6P9))
#define FPGA_PROG_B_L  (pGPIO_VA->OUT_DATA67 &= (~(GPIO_GP6P9)))
#define FPGA_OPERATE_OVERTIME_CNT_MAX 1000000u

#define TRACE(fmt, args...)  printk("%s:%s:%d: " fmt "\r\n", __FILE__, __FUNCTION__, __LINE__, ##args)

//====================================================================================================
// 本地函数声明,此处声明的函数不与外部接口
//====================================================================================================
static int __init PeripheralDownloadInit(void);
static void __exit PeripheralDownloadExit(void);
static int PeripheralDownloadOpen(struct inode *inode, struct file *file);
static int PeripheralDownloadRelease(struct inode *inode, struct file *file);
static ssize_t PeripheralDownloadWrite(struct file *file, const char __user *usr_buf, size_t count, loff_t *ppos);
static int32 FpgaDownload(const char __user *usr_buf, size_t file_size);
static int32 DspAisFileDownload(const char __user *usr_buf, size_t file_size);
static int32 PscTransition(Uint8 psc_index, Uint8 module, Uint8 domain, Uint8 state);
static void DelayLoop(Uint32 loopcnt);

//====================================================================================================
// 本地变量声明,此处声明的变量不与外部接口
//====================================================================================================
static Uint32 const local_dsp_reset_vector[] =
{
    0x0000002A, 0x0000006A, 0x00000362,  0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

static const struct file_operations PeripheralDownloadFops =
{
    .owner    = THIS_MODULE,
    .open     = PeripheralDownloadOpen,
    .release  = PeripheralDownloadRelease,
    .write    = PeripheralDownloadWrite,
};

static struct miscdevice PeripheralDownloadDev =
{
    PERIPHERAL_DOWNLOAD_MINOR,
    "PeripheralDownload",
    &PeripheralDownloadFops
};

static struct semaphore peripheral_download_lock;

//====================================================================================================
// 函数实现
//====================================================================================================
//----------------------------------------------------------------------------------------------------
// 接口函数
//----------------------------------------------------------------------------------------------------
module_init(PeripheralDownloadInit);
module_exit(PeripheralDownloadExit);
MODULE_LICENSE("GPL");

//----------------------------------------------------------------------------------------------------
// 本地函数
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//   Function: PeripheralDownloadInit
//      Input: void
//     Output: void
//     Return: int : 函数执行情况
//Description: 驱动初始化函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static int __init PeripheralDownloadInit(void)
{
    if (0 != misc_register(&PeripheralDownloadDev))
    {
        TRACE("register \"%s\" failed", PeripheralDownloadDev.name);
        return NORMAL_ERROR;
    }

    sema_init(&peripheral_download_lock, 1);

    return NORMAL_SUCCESS;
}

//----------------------------------------------------------------------------------------------------
//   Function: PeripheralDownloadExit
//      Input: void
//     Output: void
//     Return: int : 函数执行情况
//Description: 驱动注销函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static void __exit PeripheralDownloadExit(void)
{
    misc_deregister(&PeripheralDownloadDev);
}

//----------------------------------------------------------------------------------------------------
//   Function: PeripheralDownloadOpen
//      Input: struct inode *inode
//             struct file *file
//     Output: void
//     Return: int : 函数执行情况
//Description: 设备打开函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static int PeripheralDownloadOpen(struct inode *inode, struct file *file)
{
    if (0 != down_trylock(&peripheral_download_lock))
    {
        printk("dev has been open by other process, please try later!\r\n");
        return NORMAL_ERROR;
    }

    return NORMAL_SUCCESS;
}

//----------------------------------------------------------------------------------------------------
//   Function: PeripheralDownloadRelease
//      Input: struct inode *inode
//             struct file *file
//     Output: void
//     Return: int : 函数执行情况
//Description: 设备关闭函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static int PeripheralDownloadRelease(struct inode *inode, struct file *file)
{
    up(&peripheral_download_lock);

    return NORMAL_SUCCESS;
}

//----------------------------------------------------------------------------------------------------
//   Function: PeripheralDownloadWrite
//      Input: struct file *file
//             const char __user *usr_buf
//             size_t count
//             loff_t *ppos
//     Output: void
//     Return: int32 : 函数执行情况
//Description: 外围程序下载主函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static ssize_t PeripheralDownloadWrite(struct file *file, const char __user *usr_buf, size_t count, loff_t *ppos)
{
    int32  lv_index;
    Uint32 lv_file_type;
    int32  lv_file_size;
    int32  lv_dsp_entry_point;
    Uint8  lv_buffer[4];
    const char __user *lv_p_usr_buf;
    int32 (*lv_p_download_func)(const char __user *, size_t);

    if (count <= 4)
    {
        TRACE("buffer size must great than 4, but here is %d", count);
        return NORMAL_ERROR;
    }

    lv_p_usr_buf = usr_buf;
    lv_file_size = count - 4;

    if (0 != copy_from_user(lv_buffer, lv_p_usr_buf, 4))
    {
        TRACE("copy_from_user error");
        return NORMAL_ERROR;
    }

    lv_file_type =  lv_buffer[0]
                 | (lv_buffer[1] << 8u)
                 | (lv_buffer[2] << 16u)
                 | (lv_buffer[3] << 24u);

    if (FILE_TYPE_FPGA == lv_file_type)
    {
        lv_p_download_func = FpgaDownload;
        lv_file_type = 1;
    }
    else if (FILE_TYPE_AIS == lv_file_type)
    {
        lv_file_type = 2;
        lv_p_download_func = DspAisFileDownload;
    }
    else
    {
        TRACE("unknow file type \"0x%08lX\"", lv_file_type);
        return NORMAL_ERROR;
    }

    if (1 == lv_file_type)
    {
        if (NORMAL_SUCCESS != lv_p_download_func(lv_p_usr_buf + 4, lv_file_size))
        {
            TRACE("fpga download failed!");
            return NORMAL_ERROR;
        }
    }
    else if (2 == lv_file_type)
    {
        pPSC0_VA->MDCTL[15u] &= ~PSC_LOCALRST;
        PscTransition(0, 15u, 1u, PSC_ENABLE);// 必须上电,否则不能访问L2RAM

        DelayLoop(5000);

        lv_dsp_entry_point = lv_p_download_func(lv_p_usr_buf + 4, lv_file_size);
        if (NORMAL_ERROR == lv_dsp_entry_point)
        {
            TRACE("dsp download failed!");
            return NORMAL_ERROR;
        }

        if (0 != (lv_dsp_entry_point & 0x000003FFu))
        {        
            for (lv_index = 0; lv_index < (sizeof(local_dsp_reset_vector) / sizeof(local_dsp_reset_vector[0])); lv_index++)
            {
                pDSP_RESET_VECTOR_VA[lv_index] =  local_dsp_reset_vector[lv_index];
            }
            pDSP_RESET_VECTOR_VA[0] |= (LSHW(lv_dsp_entry_point) << 7) ;
            pDSP_RESET_VECTOR_VA[1] |= (HSHW(lv_dsp_entry_point) << 7) ;

            lv_dsp_entry_point = DSP_RESET_VECTOR_ADDR;
        }

        pSYSCFG0_VA->KICKR[0] = SYSCFG_UNLOCK0;
        pSYSCFG0_VA->KICKR[1] = SYSCFG_UNLOCK1;
        
        pSYSCFG0_VA->HOSTCFG[1] = lv_dsp_entry_point;
        
        pSYSCFG0_VA->KICKR[0] = SYSCFG_LOCK0;
        pSYSCFG0_VA->KICKR[1] = SYSCFG_LOCK1;

        pPSC0_VA->MDCTL[15u] |= PSC_LOCALRST;
    }
    else
    {
        TRACE("unknow local file type \"0x%08lX\"", lv_file_type);
        return NORMAL_ERROR;
    }

    return NORMAL_SUCCESS;
}

//----------------------------------------------------------------------------------------------------
//   Function: FpgaDownload
//      Input: const char __user *usr_buf
//             size_t file_size
//     Output: void
//     Return: int32 : 函数执行情况
//Description: FPGA程序下载函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static int32 FpgaDownload(const char __user *usr_buf, size_t file_size)
{
    Uint32 i;
    int32  lv_file_size;     
    Uint32 lv_delay_count;
    Uint32 lv_byte_index;
    Uint32 lv_bit_index;
    Uint32 lv_extra_cclk;
    Uint8  lv_uchar_temp;
    const char __user *lv_p_usr_buf;

    lv_p_usr_buf = usr_buf;
    lv_file_size = file_size;

    pSYSCFG0_VA->KICKR[0] = SYSCFG_UNLOCK0;
    pSYSCFG0_VA->KICKR[1] = SYSCFG_UNLOCK1;
    
    pSYSCFG0_VA->PINMUX[13] &= 0x00000FFFu;
    pSYSCFG0_VA->PINMUX[13] |= 0x88888000u;
    
    pSYSCFG0_VA->KICKR[0] = SYSCFG_LOCK0;
    pSYSCFG0_VA->KICKR[1] = SYSCFG_LOCK1;

    pGPIO_VA->DIR67 |= 0x00001100u;
    pGPIO_VA->DIR67 &= (~0x00000E00u);

    FPGA_CCLK_H;

    FPGA_PROG_B_L;

    lv_delay_count = FPGA_OPERATE_OVERTIME_CNT_MAX;
    while ((0 != FPGA_INIT_B) || (0 != FPGA_DONE))// wait STATUS low
    {
        lv_delay_count--;
        if (0 == lv_delay_count)
        {
            TRACE("fpga INIT_B or DONE low detect overtime!");
            return NORMAL_ERROR;
        }
    }

    FPGA_PROG_B_H;

    DelayLoop(1000);

    lv_delay_count = FPGA_OPERATE_OVERTIME_CNT_MAX;
    while (0 == FPGA_INIT_B)// wait STATUS hight
    {
        lv_delay_count--;
        if (0 == lv_delay_count)
        {
            TRACE("INIT_B high detect overtime!");
            return NORMAL_ERROR;
        }
    }

    DelayLoop(1000);

    for (lv_byte_index = 0; lv_byte_index < lv_file_size; lv_byte_index++)
    {
        if (0 != (__get_user(lv_uchar_temp, &lv_p_usr_buf[lv_byte_index])))
        {
            TRACE("__get_user fpga data at addr %ld failed!", lv_byte_index);
            return NORMAL_ERROR;
        }

        for (lv_bit_index = 0; lv_bit_index < 8; lv_bit_index++)
        {
            DelayLoop(1);
            FPGA_CCLK_L;
            if (0 != ((lv_uchar_temp >> (7 - lv_bit_index)) & 0x01))
            {
                FPGA_DIN_H;
            }
            else
            {
                FPGA_DIN_L;
            }

            DelayLoop(1);
            FPGA_CCLK_H;

            if (0 != FPGA_DONE)
            {
                break;
            }

            if (0 == FPGA_INIT_B)
            {
                TRACE("Fpga byte[%lu] bit[%lu] download error, INIT_B goto low!", lv_byte_index, lv_bit_index);
                return NORMAL_ERROR;
            }
        }
    }

    FPGA_DIN_H;
    lv_delay_count = FPGA_OPERATE_OVERTIME_CNT_MAX;
    while (0 == FPGA_DONE)// wait CONFIG_DONE high
    {
        if (0 == FPGA_INIT_B)
        {
            TRACE("Fpga download error, INIT_B goto low!");
            return NORMAL_ERROR;
        }

        DelayLoop(1);
        FPGA_CCLK_L;

        DelayLoop(1);
        FPGA_CCLK_H;

        lv_delay_count--;
        if (0 == lv_delay_count)
        {
            TRACE("FPGA_DONE high detect overtime!");
            return NORMAL_ERROR;
        }
    }

    lv_extra_cclk = (lv_file_size - lv_byte_index) << 3u;
    if (lv_extra_cclk < 16u)
    {
        lv_extra_cclk = 16u;
    }

    for (i = 0; i < lv_extra_cclk; i++)
    {
        DelayLoop(1);
        FPGA_CCLK_L;

        DelayLoop(1);
        FPGA_CCLK_H;
    }

    return NORMAL_SUCCESS;
}

//----------------------------------------------------------------------------------------------------
//   Function: DspAisFileDownload
//      Input: const char __user *usr_buf
//             size_t file_size
//     Output: void
//     Return: int32 : 函数执行情况
//Description: AIS形式的DSP程序下载函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static int32 DspAisFileDownload(const char __user *usr_buf, size_t file_size)
{
    Uint8  lv_buffer[16];
    int32  lv_file_size;
    int32  lv_byte_index;
    int32  lv_dsp_entry_point;
    Uint32 lv_ulong_temp;
    Uint32 lv_dsp_section_addr;
    Uint32 lv_dsp_section_length;
    Uint8  *lv_p_dsp_section_virtual_addr;
    const char __user *lv_p_usr_buf;

    lv_p_usr_buf = usr_buf;
    lv_file_size = file_size;

    lv_byte_index = 0;
    lv_dsp_entry_point = 0;
    while (lv_byte_index < lv_file_size)
    {
        if ((lv_file_size - lv_byte_index) < 4)
        {
            TRACE("read ais command at addr %ld failed, command is less than 4!", lv_byte_index);
            return NORMAL_ERROR;
        }

        if (0 != copy_from_user(lv_buffer, &lv_p_usr_buf[lv_byte_index], 4u))
        {
            TRACE("copy_from_user error");
            return NORMAL_ERROR;
        }
        lv_ulong_temp =   lv_buffer[0]
                       | (lv_buffer[1] << 8u)
                       | (lv_buffer[2] << 16u)
                       | (lv_buffer[3] << 24u);
        lv_byte_index += 4;
        if (COMMAND_CLOSE_JUMP == lv_ulong_temp)
        {
            if ((lv_file_size - lv_byte_index) < 4)
            {
                TRACE("read ais COMMAND_CLOSE_JUMP at addr %ld failed, argvs is less than 4!", lv_byte_index);
                return NORMAL_ERROR;
            }

            if (0 != copy_from_user(lv_buffer, &lv_p_usr_buf[lv_byte_index], 4u))
            {
                TRACE("copy_from_user error");
                return NORMAL_ERROR;
            }

            lv_dsp_entry_point  =   lv_buffer[0]
                                 | (lv_buffer[1] << 8u)
                                 | (lv_buffer[2] << 16u)
                                 | (lv_buffer[3] << 24u);
            lv_byte_index += 4;
            break;
        }
        else if (COMMAND_SECTION_LOAD == lv_ulong_temp)
        {
            if ((lv_file_size - lv_byte_index) < 8)
            {
                TRACE("read ais COMMAND_SECTION_LOAD at addr %ld failed, argvs is less than 8!", lv_byte_index);
                return NORMAL_ERROR;
            }

            if (0 != copy_from_user(lv_buffer, &lv_p_usr_buf[lv_byte_index], 8u))
            {
                TRACE("copy_from_user error");
                return NORMAL_ERROR;
            }
            lv_dsp_section_addr  =  lv_buffer[0]
                                 | (lv_buffer[1] << 8u)
                                 | (lv_buffer[2] << 16u)
                                 | (lv_buffer[3] << 24u);
            lv_dsp_section_length = lv_buffer[4]
                                 | (lv_buffer[5] << 8u)
                                 | (lv_buffer[6] << 16u)
                                 | (lv_buffer[7] << 24u);
            lv_byte_index += 8u;

            if ((lv_file_size - lv_byte_index) < lv_dsp_section_length)
            {
                TRACE("read ais COMMAND_SECTION_LOAD data at addr %ld failed, data is less than %ld!", lv_byte_index, lv_dsp_section_length);
                return NORMAL_ERROR;
            }

            if ((lv_dsp_section_addr >= DSPL2RAM_PHYS) && (lv_dsp_section_addr < (DSPL2RAM_PHYS + DSPL2RAM_SIZE)))
            {
                if ((lv_dsp_section_addr + lv_dsp_section_length) >= (DSPL2RAM_PHYS + DSPL2RAM_SIZE))
                {
                    TRACE("end addr=0x%08lX exceed dsp l2 ram range", (lv_dsp_section_addr + lv_dsp_section_length));
                    return NORMAL_ERROR;
                }
                lv_p_dsp_section_virtual_addr = (Uint8 *)DSPL2RAM_ADDRESS(lv_dsp_section_addr);
            }
            else if ((lv_dsp_section_addr >= SHARERAM_PHYS) && (lv_dsp_section_addr < (SHARERAM_PHYS + SHARERAM_SIZE)))
            {
                if ((lv_dsp_section_addr + lv_dsp_section_length) >= (SHARERAM_PHYS + SHARERAM_SIZE))
                {
                    TRACE("end addr=0x%08lX exceed share ram range", (lv_dsp_section_addr + lv_dsp_section_length));
                    return NORMAL_ERROR;
                }
                lv_p_dsp_section_virtual_addr = (Uint8 *)SHARERAM_ADDRESS(lv_dsp_section_addr);
            }
            else if ((lv_dsp_section_addr >= SHAREDDR_PHYS) && (lv_dsp_section_addr < (SHAREDDR_PHYS + SHAREDDR_SIZE)))
            {
                if ((lv_dsp_section_addr + lv_dsp_section_length) >= DSP_RESET_VECTOR_ADDR)
                {
                    TRACE("end addr=0x%08lX exceed share ddr range", DSP_RESET_VECTOR_ADDR);
                    return NORMAL_ERROR;
                }
                lv_p_dsp_section_virtual_addr = (Uint8 *)SHAREDDR_ADDRESS(lv_dsp_section_addr);
            }
            else
            {
                TRACE("addr=0x%08lX exceed dsp ram range", lv_dsp_section_addr);
                return NORMAL_ERROR;
            }

            if (0 != copy_from_user(lv_p_dsp_section_virtual_addr, &lv_p_usr_buf[lv_byte_index], lv_dsp_section_length))
            {
                TRACE("copy_from_user error");
                return NORMAL_ERROR;
            }

            lv_byte_index += lv_dsp_section_length;
        }
        else
        {
            TRACE("unknown AIS command=0x%08lX", lv_ulong_temp);
            return NORMAL_ERROR;
        }
    }
    
    return lv_dsp_entry_point;
}

//----------------------------------------------------------------------------------------------------
//   Function: PscTransition
//      Input: Uint8 psc_index
//             Uint8 module
//             Uint8 domain
//             Uint8 state
//     Output: void
//     Return: int32 : 函数执行情况
//Description: 模块电源管理函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static int32 PscTransition(Uint8 psc_index, Uint8 module, Uint8 domain, Uint8 state)
{
    PSC_REGS *lv_p_psc;

    if (0 == psc_index)
    {
        lv_p_psc = pPSC0_VA;
    }
    else if(1 == psc_index)
    {
        lv_p_psc = pPSC1_VA;
    }
    else
    {
        return NORMAL_ERROR;
    }

    // Wait for any outstanding transition to complete
    while (0 != ((lv_p_psc->PTSTAT) & (0x00000001 << domain)));

    // If we are already in that state, just return
    if (((lv_p_psc->MDSTAT[module]) & 0x1F) == state)
    {
        return NORMAL_SUCCESS;
    }

    // Perform transition
    lv_p_psc->MDCTL[module] = ((lv_p_psc->MDCTL[module]) & (0xFFFFFFE0)) | (state);
    lv_p_psc->PTCMD |= (0x00000001 << domain);

    // Wait for transition to complete
    while (0 != ((lv_p_psc->PTSTAT) & (0x00000001 << domain)));

    // Wait and verify the state
    while (((lv_p_psc->MDSTAT[module]) & 0x1F) != state);
    
    return NORMAL_SUCCESS;
}

//----------------------------------------------------------------------------------------------------
//   Function: DelayLoop
//      Input: Uint32 loopcnt
//     Output: void
//     Return: void
//Description: 延时函数
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   YanDengxue     2011-03-21 16:30           Create
//----------------------------------------------------------------------------------------------------
static void DelayLoop(Uint32 loopcnt)
{
    Uint32 i;

    for (i = 0; i < loopcnt; i++)
    {
        asm("   NOP");
    }
}


