// Author: Dong Jinxing
// Data: 2017-1-24 AM 10:10

#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define VERSION 0x55

#define SYSTEM_EVENT_DI            0x01
#define SYSTEM_EVENT_MODBUS        0x02
#define SYSTEM_EVENT_COLLECTION    0x04
#define SYSTEM_EVENT
#define SYSTEM_EVENT_ALL SYSTEM_EVENT_DI|SYSTEM_EVENT_MODBUS|SYSTEM_EVENT_COLLECTION


enum system_startup_error{
	SYSTEM_STARTUP_ERROR_NOT,
	SYSTEM_STARTUP_ERROR_FRAM,
	SYSTEM_STARTUP_ERROR_FALSH,
};

enum senser_state
{
    SENSER_STATE_ONLINE = 0,
    SENSER_STATE_OFFLINE,
};

enum operation_mode
{
	OPERATION_MODE_UNKNOW = 0,
	OPERATION_MODE_LOWER_LIMIT,
	OPERATION_MODE_SUITABLE,
	OPERATION_MODE_HIGH,
	OPERATION_MODE_UPPER_LIMIT,
	OPERATION_MODE_WARNING,
};

enum ac_run_state
{
	AC_RUN_STATE_UNKNOW = 0,
	AC_RUN_STATE_OFF,
	AC_RUN_STATE_STANDBY,
	AC_RUN_STATE_REFRIGERATION,
	AC_RUN_STATE_HEATING,
};

struct time_s
{
    uint16_t hour;
    uint8_t min;
    uint8_t sec;
};

/* 环境数据 */
struct env_status
{
	enum senser_state senser_state;
    float temp;
    float hum;
};
typedef struct env_status *env_status_t;

/* 电能数据 */
struct energy_data
{
    float ua;           /* Ua相电压(V) */
    float uca;          /* Uca线电压(V) */
    float ia;           /* A相电流 */
    float pa;           /* A相有功功率 */
    float pfa;          /* A相功率因数 */
    float qa;           /* A相无功功率 */
    float sa;           /* A相视在功率 */

    float ub;           /* Ub相电压(V) */
    float uab;          /* Uab线电压(V) */
    float ib;           /* B相电流 */
    float pb;           /* B相有功功率 */
    float pfb;          /* B相功率因数 */
    float qb;           /* B相无功功率 */
    float sb;           /* B相视在功率 */

    float uc;           /* Uc相电压(V) */
    float ubc;          /* Ubc线电压(V) */
    float ic;           /* B相电流 */
    float pc;           /* B相有功功率 */
    float pfc;          /* B相功率因数 */
    float qc;           /* B相无功功率 */
    float sc;           /* B相视在功率 */

    float freq;         /* 频率(Hz) */
    float psum;         /* 三相有功功率 */
    float pfav;         /* 三相总功率因数 */
    float qsum;         /* 三相无功功率 */
    float ssum;         /* 三相视在公率 */

    /* 当日能耗 */
    float pwhday;       /* 正向有功电能 */
    float nwhday;       /* 负向有功电能 */
    float pvarhday;     /* 正向无功电能 */
    float nvarhday;     /* 负向无功电能 */

    /* 总计电能 */
    float total_pwh;    /* 正向有功电能 */
    float total_nwh;    /* 负向有功电能 */
    float total_pvarh;  /* 正向无功电能 */
    float total_nvarh;  /* 负向无功电能 */
};
typedef struct energy_data *energy_data_t;

/**
 * 空调状态
 */
struct ac_status
{
	struct{
		float         temp;            // 送风温度
	    rt_uint8_t    err:1;           // 送风传感器状态
	    rt_uint8_t    count:7;
	}ntc;

    rt_uint8_t       run_state:4;
    rt_uint8_t       err_state:4;
    rt_uint8_t       err_count:4;
    rt_uint8_t       execute:4;
    rt_uint8_t       seting_temperation;
    time_t        run_time;
    time_t        total_run_time;
};
typedef struct ac_status *ac_status_t;

struct di_status
{
    rt_uint8_t    normally_open:1;
	rt_uint8_t    keep_time;

};
typedef struct di_status *di_status_t;

struct dev_status
{
	rt_uint8_t operation_mode;
    rt_uint8_t normal_run_number:2;// 正常时运行空调数量
    rt_uint8_t alarm_run_number:2; // 告警时运行空调数量


    struct env_status  indoor;
    struct env_status  outdoor;

    struct ac_status   ac[4];

    rt_uint8_t fac_running;

    struct energy_data energy;
};
typedef struct dev_status *dev_status_t;

struct comm_conf
{
    uint8_t addr;
    uint8_t baudrate;
};
typedef struct comm_conf *comm_conf_t;

struct ac_conf
{
    rt_uint8_t   enabled:1;         	// 启用空调
    rt_uint8_t   heat_function:1;       // 支持制热功能
    rt_uint8_t   refrigeration_grap:4;  // 制冷温差
	rt_uint8_t   heat_grap:4;           // 制热温差
	rt_uint8_t   refrigeration_temperation; // 制冷温度
    time_t       longest_run_time;   // 单次开机最长运行时间
};
typedef struct ac_conf *ac_conf_t;

struct fan_conf
{
    rt_uint8_t enabled:1;       // 启用风机
    rt_uint8_t do_number:2;     // DO序号
    rt_uint8_t gap:5;           // 开风机温差
    rt_uint8_t hum_upper;       // 湿度上限
	rt_uint8_t hum_lower;       // 湿度下限
};
typedef struct fan_conf *fan_conf_t;

/**
 * 设备地址
 * 波特率
 * 来电启动等待时间
 */
struct device_conf
{
    // 通讯配置
    struct comm_conf comm;

    rt_uint8_t run_number:2;
    rt_uint8_t heat_function:1;

    // 自动模式

    rt_bool_t auto_mode;
    struct {
        // 温度设定点
        int8_t tempset_max; // 温度上限
        int8_t tempset_high;// 温度偏高
        int8_t tempset_low; // 温度偏低
        int8_t tempset_min; // 温度下限

        // 湿度设定点
        int8_t humset_high;
        int8_t humset_low;

        // 偏移量
        uint8_t temp_offset;
        uint8_t hum_offset;
    }auto_conf;

    // 空调配置
    struct ac_conf ac[4];

    // 风机配置
    struct fan_conf fan;
};
typedef struct device_conf *device_conf_t;

// 空调调度模式
struct mmi_conf
{
    uint8_t language;        // 语言
    uint8_t backlight_time;  // 背光时间
    uint8_t backlight_level; // 背光等级
};
typedef struct mmi_conf *mmi_conf_t;

// 空调编号、模式、温度、运行时间
struct ac_ctrl_cmd
{
    uint8_t delay;      // 命令执行延迟时间(预约)
    uint8_t number;     // 设备编号
    uint8_t mode;       // 工作模式
    uint8_t tempset;    // 设定温度
    uint8_t runtime;    // 运行时间
};
typedef struct ac_ctrl_cmd *ac_ctrl_cmd_t;

struct ac_ctrl_conf
{
    uint8_t number;
    struct rt_mailbox mb;
};
typedef struct ac_ctrl_conf *ac_ctrl_conf_t;



// 时间任务
struct task
{
	rt_uint32_t valid :1;       // 任务时候为空
	rt_uint32_t enable:1;       // 任务使能
	rt_uint32_t hour:  5;       // 时间
	rt_uint32_t min:   6;       // 分钟
	rt_uint32_t day:   7;       // 周
	rt_uint32_t ac1:   1;       // 空调1
	rt_uint32_t ac1_enable : 1; // 空调1使能
	rt_uint32_t ac2:   1;       // 空调2
	rt_uint32_t ac2_enable : 1; // 空调2使能
	rt_uint32_t ac3:   1;       // 空调3
	rt_uint32_t ac3_enable : 1; // 空调2使能
	rt_uint32_t ac4:   1;       // 空调4
	rt_uint32_t ac4_enable : 1; // 空调4
	rt_uint32_t do1:   1;       // do1

};

// 状态存储
struct dev_status_save {
	rt_uint8_t version;
	struct dev_status rt_status;
	rt_uint8_t verify;
};
typedef struct dev_status_save *dev_status_save_t;

/* External function */
extern rt_err_t rt_status_thread_init(void);
extern rt_err_t ac_manage_thread_init(void);
extern rt_err_t modbus_thread_init(void);
extern void control_thread_init(void);
rt_err_t get_rt_status(dev_status_t data);
void save_thread_init(void);

extern struct dev_status rt_status;
extern struct rt_event ac_event;
extern struct rt_event system_event;
extern struct device_conf rt_conf;

#endif /* _INCLUDE_H_ */
