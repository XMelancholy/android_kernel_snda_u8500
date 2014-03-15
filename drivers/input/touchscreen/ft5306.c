#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/ft5306.h>
#define FT5x0x_TX_NUM	18
#define FT5x0x_RX_NUM   10

struct ft5306_device {
	struct i2c_client *client;
	int irq;
	struct work_struct work;
    struct workqueue_struct *workqueue;
	struct regulator *regulator;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct ft5306_platform_data *pdata;
	int sleeping;
};


static struct ft5306_device *ft5306_data;
static struct input_dev *in_dev;

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= ft5306_data->client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(ft5306_data->client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5306_read_reg(struct ft5306_device *chip, u8 reg, u8 buf[], int count)
{
	int ret;

	ret = i2c_master_send(chip->client, &reg, 1);
	if ( ret < 0 ) {
		printk(KERN_ERR "%s: send command failed!\n", __func__);
		return -EIO;
	}

	ret = i2c_master_recv(chip->client, buf, count);
	if (ret < 0) {
		printk(KERN_ERR "%s: read failed!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int ft5306_write_reg(struct ft5306_device *chip, u8 reg, u8 val)
{
	int ret;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;

	ret = i2c_master_send(chip->client, buf, 2);
	if ( ret < 0 ) {
		printk(KERN_ERR "%s: send command failed!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}

static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msgs[2];

    //
	buf[0] = addr;    //register address
	
	msgs[0].addr = ft5306_data->client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;
	msgs[1].addr = ft5306_data->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(ft5306_data->client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
  
}

static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char fw_id;
	ft5306_read_reg(ft5306_data, ID_G_FIRMWARE_ID, &fw_id, 1);
	return(fw_id);
}

#if 1 //upgrade related
typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

typedef struct _FTS_CTP_PROJECT_SETTING_T
{
    unsigned char uc_i2C_addr;             //I2C slave address (8 bit address)
    unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
    unsigned char uc_panel_factory_id;     //TP panel factory ID
}FTS_CTP_PROJECT_SETTING_T;

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}


/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(ft5306_data->client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[FTS]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(ft5306_data->client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(ft5306_data->client->addr, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(ft5306_data->client->addr, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(ft5306_data->client->addr, pbt_buf, bt_len);
}

#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
	#include "ft_app.i"
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
	ft5306_write_reg(ft5306_data, 0xfc, 0xaa);
    delay_qt_ms(50);
     /*write 0x55 to register 0xfc*/
	ft5306_write_reg(ft5306_data, 0xfc, 0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");
   
    delay_qt_ms(30);   


    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }

    cmd_write(0xcd,0x0,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[FTS] bootloader version = 0x%x\n", reg_val[0]);

     /*********Step 4:erase app and panel paramenter area ********************/
    cmd_write(0x61,0x00,0x00,0x00,1);  //erase app area
    delay_qt_ms(1500); 
    cmd_write(0x63,0x00,0x00,0x00,1);  //erase panel parameter area
    delay_qt_ms(100);
    printk("[FTS] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[FTS] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[FTS] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        delay_qt_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        delay_qt_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[FTS] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    msleep(300);  //make sure CTP startup normally
    
    return ERR_OK;
}

int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
	ft5306_write_reg(ft5306_data, 0, 0x40);
    delay_qt_ms(100);   //make sure already enter factory mode
   	ft5306_write_reg(ft5306_data, 2, 0x4);//write command to start calibration
    delay_qt_ms(300);
    for(i=0;i<100;i++)
    {
        ft5x0x_read_reg(0,&uc_temp);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        delay_qt_ms(200);
        printk("[FTS] waiting calibration %d\n",i);
        
    }
    printk("[FTS] calibration OK.\n");
    
    msleep(300);
//    ft5x0x_write_reg(0, 0x40);  //goto factory mode
   	ft5306_write_reg(ft5306_data, 0, 0x40);
    delay_qt_ms(100);   //make sure already enter factory mode
//    ft5x0x_write_reg(2, 0x5);  //store CLB result
   	ft5306_write_reg(ft5306_data, 2, 0x5);
    delay_qt_ms(300);
//    ft5x0x_write_reg(0, 0x0); //return to normal mode 
   	ft5306_write_reg(ft5306_data, 0, 0x0);
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
    
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
       //error handling ...
       //TBD
   }
   else
   {
       printk("[FTS] upgrade successfully.\n");
       fts_ctpm_auto_clb();  //start auto CLB
   }

   return i_ret;
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

#define    FTS_SETTING_BUF_LEN        128

//update project setting
//only update these settings for COB project, or for some special case
int fts_ctpm_update_project_setting(void)
{
    unsigned char uc_i2c_addr;             //I2C slave address (8 bit address)
    unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
    unsigned char uc_panel_factory_id;     //TP panel factory ID

    unsigned char buf[FTS_SETTING_BUF_LEN];
    FTS_BYTE reg_val[2] = {0};
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE  packet_buf[FTS_SETTING_BUF_LEN + 6];
    FTS_DWRD i = 0;
    int      i_ret;

    uc_i2c_addr = 0x70;
    uc_io_voltage = 0x0;
    uc_panel_factory_id = 0x5a;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    ft5x0x_write_reg(0xfc,0xaa);
    delay_qt_ms(50);
     /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(0xfc,0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");
   
    delay_qt_ms(30);   

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
    }

    cmd_write(0xcd,0x0,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("bootloader version = 0x%x\n", reg_val[0]);


    /* --------- read current project setting  ---------- */
    //set read start address
    buf[0] = 0x3;
    buf[1] = 0x0;
    buf[2] = 0x78;
    buf[3] = 0x0;
    byte_write(buf, 4);
    byte_read(buf, FTS_SETTING_BUF_LEN);
    
    printk("[FTS] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
        buf[0],  buf[2], buf[4]);
    for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
    {
        if (i % 16 == 0)     printk("\n");
        printk("0x%x, ", buf[i]);
        
    }
    printk("\n");

     /*--------- Step 4:erase project setting --------------*/
    cmd_write(0x62,0x00,0x00,0x00,1);
    delay_qt_ms(100);
   
    /*----------  Set new settings ---------------*/
    buf[0] = uc_i2c_addr;
    buf[1] = ~uc_i2c_addr;
    buf[2] = uc_io_voltage;
    buf[3] = ~uc_io_voltage;
    buf[4] = uc_panel_factory_id;
    buf[5] = ~uc_panel_factory_id;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    packet_buf[2] = 0x78;
    packet_buf[3] = 0x0;
    packet_buf[4] = 0;
    packet_buf[5] = FTS_SETTING_BUF_LEN;
    for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
    {
        packet_buf[6 + i] = buf[i];
        if (i % 16 == 0)     printk("\n");
        printk("0x%x, ", buf[i]);
    }
    printk("\n");
    byte_write(&packet_buf[0],FTS_SETTING_BUF_LEN + 6);
    delay_qt_ms(100);

    /********* reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    msleep(200);

    return 0;
    
}

//#if CFG_SUPPORT_AUTO_UPG

int fts_ctpm_auto_upg(void)
{
    unsigned char uc_host_fm_ver;
    unsigned char uc_tp_fm_ver;
    int           i_ret;

    uc_tp_fm_ver = ft5x0x_read_fw_ver();
    uc_host_fm_ver = fts_ctpm_get_i_file_ver();
    if ( uc_tp_fm_ver == 0xa6  ||   //the firmware in touch panel maybe corrupted
         uc_tp_fm_ver < uc_host_fm_ver //the firmware in host flash is new, need upgrade
        )
    {
        msleep(100);
        printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
            uc_tp_fm_ver, uc_host_fm_ver);
        i_ret = fts_ctpm_fw_upgrade_with_i_file();    
        if (i_ret == 0)
        {
            msleep(300);
            uc_host_fm_ver = fts_ctpm_get_i_file_ver();
            printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
        }
        else
        {
            printk("[FTS] upgrade failed ret=%d.\n", i_ret);
        }
    }

    return 0;
}

//#endif

#endif

static void mutitouch_ts_work(struct work_struct *work)
{
	int ret;
	u8 i, buf_position;
	u16 position_x, position_y;
	u8 buf[DATA_LENGTH], fingers_count;

	if (ft5306_data->sleeping) {
		goto out;
	}

	ret = ft5306_read_reg(ft5306_data, 0, buf, DATA_LENGTH);
	if (ret)
		goto out;

	fingers_count = buf[2] & 0xf;
	
	//printk("fingers_count: %d\n", fingers_count);

	if (fingers_count > DATA_FINGERS_MAX)
		fingers_count = DATA_FINGERS_MAX;

	//printk("later fingers_count: %d\n", fingers_count);

	if (fingers_count) {
		for (i = 0, buf_position = DATA_XY_START; i < fingers_count; i++) {
			position_x = (s16)(buf[buf_position] & 0x0F) << 8 |
												(s16)buf[buf_position + 1];
			position_y = (s16)(buf[buf_position + 2] & 0x0F) << 8 |
												(s16)buf[buf_position + 3];
		/*	pr_alert("%s: x[%d] y[%d] %d\n", __func__, position_x, position_y, (buf[buf_position + 2] >> 4) + 1);     */
			if (position_y == 0)
				position_y = 1;
			input_report_abs(in_dev, ABS_MT_TOUCH_MAJOR, 200);
#if defined(CONFIG_PAD_C7)
			input_report_abs(in_dev, ABS_MT_POSITION_X, position_y);
			input_report_abs(in_dev, ABS_MT_POSITION_Y, position_x);
#else
			input_report_abs(in_dev, ABS_MT_POSITION_X, position_x);
			input_report_abs(in_dev, ABS_MT_POSITION_Y, position_y);
#endif
			input_report_abs(in_dev, ABS_MT_TRACKING_ID, buf[buf_position + 2] >> 4);
			//printk("tracking_id: %d\n", buf[buf_position + 2] >> 4);

			input_mt_sync(in_dev);
			buf_position += 6;
		}
	} else {
		//printk("mt sync\n");
		input_mt_sync(in_dev);
	}

	input_sync(in_dev);
	return;
out:
	//printk("ft5306 exit work.\n");
	input_mt_sync(in_dev);
	input_sync(in_dev);
//	enable_irq(ft5306_data->irq);

	return;
}

static irqreturn_t ft5306_interrupt(int irq, void *dev_id)
{
//	disable_irq_nosync(ft5306_data->irq);
	queue_work(ft5306_data->workqueue, &ft5306_data->work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5306_early_suspend(struct early_suspend *data)
{
    pr_alert("%s, in\n", __func__);
	ft5306_data->sleeping = TRUE;
	disable_irq(ft5306_data->irq);
	ft5306_write_reg(ft5306_data, ID_G_PMODE, PMODE_HIBERNATE);
	//regulator_disable(ft5306_data->regulator);
    pr_alert("%s, exit\n", __func__);
}

static void ft5306_late_resume(struct early_suspend *data)
{
	char fw_id = 0;

    pr_alert("%s, in\n", __func__);
	//regulator_enable(ft5306_data->regulator);
	(ft5306_data->pdata->wake_up)();
	enable_irq(ft5306_data->irq);
	ft5306_read_reg(ft5306_data, ID_G_FIRMWARE_ID, &fw_id, 1);
    msleep(1000);
	//printk("%s firmware id:0x%x\n", __func__, fw_id);
	ft5306_data->sleeping = FALSE;
    pr_alert("%s, exit\n", __func__);
}
#endif



static ssize_t ft5306_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	/* center x:center y:width:height */
	return sprintf(buf,
		       __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":65:1000:130:80"
		       ":" __stringify(EV_KEY) ":"__stringify(KEY_HOME) ":270:1000:130:80"
		       ":" __stringify(EV_KEY) ":"__stringify(KEY_MENU) ":475:1000:130:80"
		       "\n");
}

static struct kobj_attribute ft5306_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.tp_ft5306",
		.mode = S_IRUGO,
	},
	.show = ft5306_virtual_keys_show,
};

static struct attribute *ft5306_properties_attrs[] = {
	&ft5306_virtual_keys_attr.attr,
	NULL,
};

static struct attribute_group ft5306_properties_attr_group = {
	.attrs = ft5306_properties_attrs,
};

static void ft5306_virtual_keys_set(void)
{
	struct kobject *properties_kobj;
	int ret;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj, &ft5306_properties_attr_group);
}


static int __devinit ft5306_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	u8 reg_val;

	printk("%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ft5306_data = (struct ft5306_device *)kmalloc(sizeof(*ft5306_data), GFP_KERNEL);
	if (NULL == ft5306_data)
		return -ENOMEM;

	ft5306_data->client = client;
	ft5306_data->irq = client->irq;
	ft5306_data->pdata = client->dev.platform_data;

	dev_set_name(&client->dev, FT5306_NAME);
	ft5306_data->regulator = regulator_get(&client->dev, "avdd");
	if (IS_ERR(ft5306_data->regulator)) {
		dev_warn(&client->dev, "regulator_get failed\n");
		ft5306_data->regulator = NULL;
	}
	if (ft5306_data->regulator)
		regulator_enable(ft5306_data->regulator);

	ret = ft5x0x_read_reg(0x00, (u8 *)&reg_val);
	if (ret < 0) {
		printk(KERN_WARNING "ft5306 detect fail!\n");
		//touch->i2c = NULL;
		ret = -ENXIO;
		goto err_input_alloc;
	} else {
		//printk(KERN_INFO "ft5306 detect success.\n");
		ft5306_virtual_keys_set();
	}


	if (ft5306_data->pdata->wake_up)
		(ft5306_data->pdata->wake_up)();


	in_dev = input_allocate_device();
	if (!in_dev) {
		ret = -ENOMEM;
		goto err_input_alloc;
	}
	in_dev->name = FT5306_NAME;
	in_dev->id.bustype = BUS_I2C;
	in_dev->dev.parent = &client->dev;

	__set_bit(EV_SYN, in_dev->evbit);
	__set_bit(EV_KEY, in_dev->evbit);
	__set_bit(EV_ABS, in_dev->evbit);
	__set_bit(BTN_MISC, in_dev->evbit);
	__set_bit(EV_KEY, in_dev->keybit);
#if defined(CONFIG_PAD_C7)
	input_set_abs_params(in_dev, ABS_MT_POSITION_X, 0,
						ft5306_data->pdata->res_y, 0, 0);
	input_set_abs_params(in_dev, ABS_MT_POSITION_Y, 0,
						ft5306_data->pdata->res_x, 0, 0);
	input_set_abs_params(in_dev, ABS_MT_TOUCH_MAJOR, 0,
				max(ft5306_data->pdata->res_y, ft5306_data->pdata->res_x), 0, 0);
#else
	input_set_abs_params(in_dev, ABS_MT_POSITION_X, 0,
						ft5306_data->pdata->res_x, 0, 0);
	input_set_abs_params(in_dev, ABS_MT_POSITION_Y, 0,
						ft5306_data->pdata->res_y, 0, 0);
	input_set_abs_params(in_dev, ABS_MT_TOUCH_MAJOR, 0,
				max(ft5306_data->pdata->res_x, ft5306_data->pdata->res_y), 0, 0);
#endif
	input_set_drvdata(in_dev, ft5306_data);

	ret = input_register_device(in_dev);
	if (ret)
		goto err_input_register;


	INIT_WORK(&ft5306_data->work, mutitouch_ts_work);
	ft5306_data->workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5306_data->workqueue) {
	  ret = -ESRCH;
	  goto err_creat_workqueue;
	}

	ft5306_data->sleeping = TRUE;
	ret = request_irq(ft5306_data->irq, ft5306_interrupt,
					IRQF_TRIGGER_FALLING, FT5306_NAME, NULL);
	if (ret) {
		printk("%s can't get irq  %d \n", __func__, ft5306_data->irq);
		goto err_request_irq;
	}
	disable_irq(ft5306_data->irq);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5306_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5306_data->early_suspend.resume = ft5306_late_resume;
	ft5306_data->early_suspend.suspend = ft5306_early_suspend;
	register_early_suspend(&ft5306_data->early_suspend);
#endif

	//fts_ctpm_auto_upg();

	enable_irq(ft5306_data->irq);
	ft5306_data->sleeping = FALSE;

	return ret;

err_request_irq:
	destroy_workqueue(ft5306_data->workqueue);
err_creat_workqueue:
	input_unregister_device(in_dev);
err_input_register:
	input_free_device(in_dev);
err_input_alloc:
	regulator_put(ft5306_data->regulator);
	kfree(ft5306_data);
err_check_functionality_failed:

	return ret;
}

static int __devexit ft5306_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5306_data->early_suspend);
#endif
	free_irq(ft5306_data->irq, NULL);
	regulator_put(ft5306_data->regulator);
	destroy_workqueue(ft5306_data->workqueue);
	input_unregister_device(in_dev);
	input_free_device(in_dev);
	kfree(ft5306_data);

	return 0;
}

static const struct i2c_device_id ft5306_id[] = {
	{FT5306_NAME, 0},
	{ }
};

static struct i2c_driver ft5306_driver = {
	.driver = {
		.name = FT5306_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ft5306_probe,
	.remove = ft5306_remove,
	.id_table = ft5306_id,
};


static int __init ft5306_init(void)
{
	return i2c_add_driver(&ft5306_driver);
}

static void __exit ft5306_exit(void)
{
	i2c_del_driver(&ft5306_driver);
}

module_init(ft5306_init);
module_exit(ft5306_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("fengxiaoqi <xfeng@via-telecom.com>");
MODULE_DESCRIPTION("ft5306 driver");
