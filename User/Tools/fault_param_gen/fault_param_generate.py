#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FaultParam CSV 转 C 语言模块脚本

从 fault_param.csv 生成故障管理模块的码表定义 fault_def.h / fault_def.c:
  1. 故障来源/级别/动作/状态枚举 + 全量故障码枚举 fault_code_e (0xSLNN 16bit)
  2. 故障元数据表 fault_meta_table[]: 级别/动作/降功率档/实现期, 供 fault_manager 运行时查表
  3. fault_code_to_index() 二分查找 (码表按升序排序) + 优先级比较接口

CSV 列: 故障码,故障来源,故障级别,故障名称,触发条件,处理方式,异常命名,
        处理动作,实现期,降功率档,信号源,参数Key
  - 异常命名: 枚举名, 生成 FAULT_<命名>
  - 实现期:   0=预留(硬件缺失/未实现, 永不触发) 1=一期 2=二期
  - 降功率档: 0=无 1=中度(70%) 2=重度(50%), 实际系数由 FaultParam 参数块配置
  - 参数Key:  触发阈值绑定的配置字段名, 供人工核对阈值来源

用法:
    python fault_param_generate.py [csv] [out_dir] [选项]
    选项:
      --validate-only        仅校验不生成
      --no-format            生成后不执行 clang-format
      --clang-format=PATH    指定 clang-format 可执行文件路径
默认:
    csv=fault_param.csv  out_dir=../../AppServices/FaultManager (相对脚本目录)
"""

import csv
import os
import re
import sys
import subprocess
from datetime import datetime

# Windows 控制台默认 GBK，重配为 UTF-8（Python 3.7+）
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding='utf-8')
    except Exception:
        pass

# ----------------------------------------------------------------------------
# 常量定义
# ----------------------------------------------------------------------------
REQUIRED_COLUMNS = [
    '故障码', '故障来源', '故障级别', '故障名称', '触发条件', '处理方式',
    '异常命名', '处理动作', '实现期', '降功率档', '信号源', '参数Key',
]

# 故障来源 -> 枚举名 (高4位, 编码顺序=优先级顺序)
SOURCE_ENUM = {
    '安全系统':       'FAULT_SOURCE_SAFETY',
    '电源系统':       'FAULT_SOURCE_POWER',
    '驱动器':         'FAULT_SOURCE_DRIVER',
    '电机本体':       'FAULT_SOURCE_MOTOR',
    '编码器/传感器':  'FAULT_SOURCE_ENCODER',
    '机械传动系统':   'FAULT_SOURCE_MECHANICAL',
    '抱闸/刹车系统':  'FAULT_SOURCE_BRAKE',
    '软件与算法':     'FAULT_SOURCE_SOFTWARE',
    '通信系统':       'FAULT_SOURCE_COMMUNICATION',
    '环境因素':       'FAULT_SOURCE_ENVIRONMENT',
}

# 故障来源 -> 高4位编码值 (与 SOURCE_ENUM 键一一对应)
SOURCE_VALUE = {
    '安全系统': 0x1, '电源系统': 0x2, '驱动器': 0x3, '电机本体': 0x4,
    '编码器/传感器': 0x5, '机械传动系统': 0x6, '抱闸/刹车系统': 0x7,
    '软件与算法': 0x8, '通信系统': 0x9, '环境因素': 0xA,
}

# 故障级别 -> 枚举名 (中4位)
LEVEL_ENUM = {
    '故障级': 'FAULT_LEVEL_CRITICAL',
    '异常级': 'FAULT_LEVEL_EXCEPTION',
    '警告级': 'FAULT_LEVEL_WARNING',
}

# 故障级别 -> 中4位编码值
LEVEL_VALUE = {'故障级': 1, '异常级': 2, '警告级': 3}

# 处理动作 -> 枚举名 (数值即 fault_action_e 取值)
ACTION_ENUM = {
    'STOP_POWER': 'FAULT_ACTION_STOP_POWER',  # 立即停机, 切断电源语义
    'STOP_BRAKE': 'FAULT_ACTION_STOP_BRAKE',  # 立即停机, 动态刹车
    'STOP_IDLE':  'FAULT_ACTION_STOP_IDLE',   # 立即停机, 可软件清障恢复
    'DERATE':     'FAULT_ACTION_DERATE',      # 降功率运行
    'CLAMP_POS':  'FAULT_ACTION_CLAMP_POS',   # 软限位钳制(允许反向)
    'DENY':       'FAULT_ACTION_DENY',        # 拒绝操作(禁止运行/使能)
    'RECAL':      'FAULT_ACTION_RECAL',       # 提示重新标定
    'LOG':        'FAULT_ACTION_LOG',         # 仅记录
    'RESET':      'FAULT_ACTION_RESET',       # 系统复位
}

# 动作数值(与 fault_action_e 枚举顺序一致, 供元数据表打包)
ACTION_VALUE = {k: i for i, k in enumerate(ACTION_ENUM.keys())}

# 级别-动作一致性矩阵: 各级别允许的动作 (超出仅告警不阻断)
LEVEL_ALLOWED_ACTIONS = {
    '故障级': {'STOP_POWER', 'STOP_BRAKE', 'RESET', 'DENY'},
    '异常级': {'STOP_IDLE', 'STOP_BRAKE', 'DERATE', 'CLAMP_POS', 'DENY', 'RECAL', 'LOG'},
    '警告级': {'LOG', 'DERATE'},
}

FAULT_CODE_RE = re.compile(r'^0x([1-9A])([1-3])([0-9A-F]{2})$')
ENUM_NAME_RE = re.compile(r'^[A-Z][A-Z0-9_]*$')


# ----------------------------------------------------------------------------
# CSV 加载与校验
# ----------------------------------------------------------------------------
def load_csv(path):
    """读取并校验 CSV, 返回记录列表; 校验失败抛 SystemExit"""
    if not os.path.isfile(path):
        fail(f'CSV 文件不存在: {path}')

    with open(path, 'r', encoding='utf-8-sig', newline='') as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames or []
        missing = [c for c in REQUIRED_COLUMNS if c not in cols]
        if missing:
            fail(f'CSV 缺少必需列: {missing}')

        rows = []
        for lineno, row in enumerate(reader, start=2):
            rec = {k: (row.get(k) or '').strip() for k in REQUIRED_COLUMNS}
            if not any(rec.values()):
                continue  # 跳过空行
            rows.append((lineno, rec))

    if not rows:
        fail('CSV 无有效数据行')
    return rows


def assign_level_bits(rows):
    """计算级别内序号(level_bit): 同级别前驱计数(码表升序)。
    该序号即三级 64bit 使能掩码(mask_critical/exception/warning)的 bit 位,
    固件 fault_mgr_is_enabled 与上位机 UI 均按此绑定, 规则改动需两端同步。"""
    cnt = {}
    for _, r in rows:
        bit = cnt.get(r['故障级别'], 0)
        r['_level_bit'] = bit
        cnt[r['故障级别']] = bit + 1


def validate(rows):
    """全量校验, 返回告警列表"""
    warns = []
    codes_seen = {}
    enums_seen = {}

    for lineno, r in rows:
        tag = f'行{lineno} [{r["故障码"]} {r["故障名称"]}]'

        # 1. 故障码格式与来源/级别一致性
        m = FAULT_CODE_RE.match(r['故障码'])
        if not m:
            fail(f'{tag} 故障码格式非法: {r["故障码"]} (须为 0xSLNN, S=1~A, L=1~3)')
        src_nibble = int(m.group(1), 16)
        lvl_nibble = int(m.group(2), 16)

        src_enum = SOURCE_ENUM.get(r['故障来源'])
        if src_enum is None:
            fail(f'{tag} 未知故障来源: {r["故障来源"]}')
        if SOURCE_VALUE[r['故障来源']] != src_nibble:
            fail(f'{tag} 故障码高4位(0x{src_nibble:X})与故障来源({r["故障来源"]})不匹配')

        lvl_enum = LEVEL_ENUM.get(r['故障级别'])
        if lvl_enum is None:
            fail(f'{tag} 未知故障级别: {r["故障级别"]}')
        if LEVEL_VALUE[r['故障级别']] != lvl_nibble:
            fail(f'{tag} 故障码中4位({lvl_nibble})与故障级别({r["故障级别"]})不匹配')

        # 2. 枚举名合法性与唯一性
        if not ENUM_NAME_RE.match(r['异常命名']):
            fail(f'{tag} 异常命名非法(须大写字母/数字/下划线): {r["异常命名"]}')
        if r['异常命名'] in enums_seen:
            fail(f'{tag} 异常命名重复: {r["异常命名"]} (首见于行{enums_seen[r["异常命名"]]})')
        enums_seen[r['异常命名']] = lineno

        # 3. 码值唯一性
        code_val = int(r['故障码'], 16)
        if code_val in codes_seen:
            fail(f'{tag} 故障码重复 (首见于行{codes_seen[code_val]})')
        codes_seen[code_val] = lineno

        # 4. 动作/实现期/降功率档合法性
        if r['处理动作'] not in ACTION_ENUM:
            fail(f'{tag} 未知处理动作: {r["处理动作"]}')
        if r['实现期'] not in ('0', '1', '2'):
            fail(f'{tag} 实现期须为 0/1/2: {r["实现期"]}')
        if r['降功率档'] not in ('0', '1', '2'):
            fail(f'{tag} 降功率档须为 0/1/2: {r["降功率档"]}')
        if r['降功率档'] != '0' and r['处理动作'] != 'DERATE':
            warns.append(f'{tag} 降功率档={r["降功率档"]} 但动作非 DERATE')

        # 5. 级别-动作一致性
        if r['处理动作'] not in LEVEL_ALLOWED_ACTIONS[r['故障级别']]:
            warns.append(f'{tag} {r["故障级别"]}配动作{r["处理动作"]} 语义可疑')

        # 6. 可实现项须有信号源与参数Key说明
        if r['实现期'] in ('1', '2'):
            if not r['信号源']:
                warns.append(f'{tag} 实现期{r["实现期"]}但信号源为空')
            if r['参数Key'] == '-':
                warns.append(f'{tag} 实现期{r["实现期"]}但参数Key为空(无量化阈值可豁免)')

    # 7. 码表升序排序(生成物依赖)
    codes = [int(r['故障码'], 16) for _, r in rows]
    if codes != sorted(codes):
        fail('CSV 未按故障码升序排列(生成物二分查找依赖升序)')
    return warns


def fail(msg):
    print(f'[错误] {msg}')
    sys.exit(1)


# ----------------------------------------------------------------------------
# 代码生成
# ----------------------------------------------------------------------------
def gen_header(rows, csv_name):
    """生成 fault_def.h 内容"""
    n = len(rows)
    p1 = sum(1 for _, r in rows if r['实现期'] == '1')
    p2 = sum(1 for _, r in rows if r['实现期'] == '2')
    today = datetime.now().strftime('%Y-%m-%d')

    enum_body = '\n'.join(
        f'\tFAULT_{r["异常命名"]} = {r["故障码"]},' +
        (f' /* {r["故障名称"]} */' if r['实现期'] == '0' else f' /* {r["故障名称"]} P{r["实现期"]} */')
        for _, r in rows
    )

    # 三级使能掩码 bit 枚举: 按级别分组显式列出 位号=故障码, 与 meta 表同源
    _LEVEL_GROUP_CN = (
        ('故障级',   '故障级 -> mask_critical1/2 (PID154/155),  bit0~31 在低位字'),
        ('异常级',   '异常级 -> mask_exception1/2 (PID156/157), bit0~31 在低位字'),
        ('警告级',   '警告级 -> mask_warning1/2 (PID158/159),   bit0~31 在低位字'),
    )
    group_bodies = []
    for lv, header_cn in _LEVEL_GROUP_CN:
        lines = [f'\t/* ---- {header_cn} ---- */']
        lines += [
            f'\tFAULT_BIT_{r["异常命名"]} = {r["_level_bit"]},'
            f' /* {r["故障码"]} {r["故障名称"]} */'
            for _, r in rows if r['故障级别'] == lv
        ]
        group_bodies.append('\n'.join(lines))
    bit_enum_body = '\n'.join(group_bodies)

    return f'''/**
 * @file        fault_def.h
 * @brief \t\t故障码表定义(由 {csv_name} 生成, 勿手改)
 *
 * @author      fault_param_generate.py
 * @version     1.0
 * @date        {today}
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容   |
 * |------------|------|-------|------------|
 * | {today} | 1.0  | auto  | 初始生成   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *
 * @details 故障码编码 0xSLNN: S=故障来源(高4位, 数值越小优先级越高),
 *          L=故障级别(中4位), NN=编号(低8位)。
 *          优先级 = 码值数值比较(先来源后级别), 码值越小优先级越高。
 *          共 {n} 项: 一期 {p1} + 二期 {p2} + 预留 {n - p1 - p2}(硬件缺失/未实现, 永不触发)。
 *          阈值参数经 motor_info FaultParam 块配置, 字段名见 CSV 参数Key列。
 */

#ifndef __FAULT_DEF_H__
#define __FAULT_DEF_H__

#include <stdint.h>
#include <stddef.h> /* NULL */

#ifdef __cplusplus
extern "C"
{{
#endif

/* ===================== 故障来源 (0xSLNN 高4位, 编码顺序=优先级顺序) ===================== */
typedef enum
{{
\tFAULT_SOURCE_SAFETY = 0x1,        /* 安全系统(优先级最高) */
\tFAULT_SOURCE_POWER = 0x2,         /* 电源系统 */
\tFAULT_SOURCE_DRIVER = 0x3,        /* 驱动器 */
\tFAULT_SOURCE_MOTOR = 0x4,         /* 电机本体 */
\tFAULT_SOURCE_ENCODER = 0x5,       /* 编码器/传感器 */
\tFAULT_SOURCE_MECHANICAL = 0x6,    /* 机械传动系统 */
\tFAULT_SOURCE_BRAKE = 0x7,         /* 抱闸/刹车系统 */
\tFAULT_SOURCE_SOFTWARE = 0x8,      /* 软件与算法 */
\tFAULT_SOURCE_COMMUNICATION = 0x9, /* 通信系统 */
\tFAULT_SOURCE_ENVIRONMENT = 0xA,   /* 环境因素(优先级最低) */
}} fault_source_e;

/* ===================== 故障级别 (中4位, 恢复策略) =====================
 * 故障级: 立即停机, 需断电排查/人工清障
 * 异常级: 按处理动作降功率或停机, 故障消除后手动清除
 * 警告级: 正常运行仅记录, 自动清除或手动清除
 * 注: 即时响应对应"处理动作"字段(独立于级别), 级别决定恢复与展示语义 */
typedef enum
{{
\tFAULT_LEVEL_CRITICAL = 0x1,  /* 故障级 */
\tFAULT_LEVEL_EXCEPTION = 0x2, /* 异常级 */
\tFAULT_LEVEL_WARNING = 0x3,   /* 警告级 */
}} fault_level_e;

/* ===================== 处理动作 (触发时的即时响应) ===================== */
typedef enum
{{
\tFAULT_ACTION_STOP_POWER = 0, /* 立即停机, 切断电源语义 */
\tFAULT_ACTION_STOP_BRAKE = 1, /* 立即停机, 动态刹车 */
\tFAULT_ACTION_STOP_IDLE = 2,  /* 立即停机, 可软件清障恢复 */
\tFAULT_ACTION_DERATE = 3,     /* 降功率运行(系数经降功率档查 FaultParam) */
\tFAULT_ACTION_CLAMP_POS = 4,  /* 软限位钳制(停止运动允许反向) */
\tFAULT_ACTION_DENY = 5,       /* 拒绝操作(禁止运行/使能) */
\tFAULT_ACTION_RECAL = 6,      /* 提示重新标定 */
\tFAULT_ACTION_LOG = 7,        /* 仅记录 */
\tFAULT_ACTION_RESET = 8,      /* 系统复位 */
}} fault_action_e;

/* ===================== 故障记录状态 ===================== */
typedef enum
{{
\tFAULT_STATUS_INACTIVE = 0, /* 未激活 */
\tFAULT_STATUS_ACTIVE = 1,   /* 活动状态 */
\tFAULT_STATUS_CLEARED = 2,  /* 已清除(历史保留) */
}} fault_status_e;

/* ===================== 实现期 =====================
 * P0=预留(硬件缺失/未实现, 检测永不触发, 码值保留供协议兼容)
 * P1=一期实现, P2=二期实现 */
typedef enum
{{
\tFAULT_PHASE_RESERVED = 0,
\tFAULT_PHASE_P1 = 1,
\tFAULT_PHASE_P2 = 2,
}} fault_phase_e;

/* ===================== 故障码 (全量 {n} 项, 升序) ===================== */
typedef enum
{{
{enum_body}
}} fault_code_e;

#define FAULT_CODE_COUNT {n}

/* ===================== 三级使能掩码 bit 位定义 (级别内序号, 生成期绑定) =====================
 * 每个故障码在所属级别 64bit 使能掩码中的 bit 位, 与 fault_meta_table 的
 * FAULT_META_LEVEL_BIT 编译期同源(同一 _level_bit 生成)。
 * 掩码参数: 故障级=mask_critical1/2(PID154/155) 异常级=mask_exception1/2(PID156/157)
 *           警告级=mask_warning1/2(PID158/159), 位=1 使能该故障检测, 位=0 禁用 */
typedef enum
{{
{bit_enum_body}
}} fault_en_bit_e;

/* 故障码字段提取 */
#define FAULT_GET_SOURCE(code) ((uint8_t)(((uint16_t)(code) >> 12) & 0x0Fu))
#define FAULT_GET_LEVEL(code)  ((uint8_t)(((uint16_t)(code) >> 8) & 0x0Fu))

/* ===================== 故障元数据 (const 表, flash 存储) =====================
 * 由 CSV 生成, 字段打包为 4B/项:
 *   pack1(u8): [7:4]=fault_action_e [3:0]=level_bit 低4位
 *   pack2(u8): [7:6]=level_bit 高2位 [5:4]=fault_phase_e [3:2]=降功率档 [1:0]=fault_level_e
 * level_bit = 三级 64bit 使能掩码(mask_critical/exception/warning)的 bit 位,
 * 即该码在码表(升序)中同级别前驱的数量, 生成时直接绑定, 1=使能 0=禁用。
 * 经访问宏读取, 大小 {n}*4B */
typedef struct
{{
\tuint16_t code;   /* 故障码 0xSLNN */
\tuint8_t pack1;   /* [7:4]=fault_action_e [3:0]=level_bit[3:0] */
\tuint8_t pack2;   /* [7:6]=level_bit[5:4] [5:4]=fault_phase_e [3:2]=降功率档 [1:0]=fault_level_e */
}} fault_meta_t;

#define FAULT_META_LEVEL(m)     ((fault_level_e)(((m)->pack2 >> 0) & 0x03u))
#define FAULT_META_ACTION(m)    ((fault_action_e)(((m)->pack1 >> 4) & 0x0Fu))
#define FAULT_META_LEVEL_BIT(m) ((uint8_t)(((((m)->pack2 >> 6) & 0x03u) << 4) | (((m)->pack1 >> 0) & 0x0Fu)))
#define FAULT_META_PHASE(m)     ((fault_phase_e)(((m)->pack2 >> 4) & 0x03u))
#define FAULT_META_DERATE(m)    ((uint8_t)(((m)->pack2 >> 2) & 0x03u))

extern const fault_meta_t fault_meta_table[FAULT_CODE_COUNT];

/* 故障码 -> 元数据表索引, 未定义码返回 -1 (表升序, 二分查找) */
int8_t fault_code_to_index(uint16_t code);

/* 取故障码元数据, 未定义码返回 NULL */
const fault_meta_t *fault_meta_get(uint16_t code);

/* 优先级比较: 返回 1 表示 f1 优先级高于 f2 (码值数值小者优先) */
uint8_t fault_is_higher_priority(uint16_t f1, uint16_t f2);

#ifdef __cplusplus
}}
#endif
#endif /* __FAULT_DEF_H__ */
'''


def gen_source(rows, csv_name):
    """生成 fault_def.c 内容"""
    today = datetime.now().strftime('%Y-%m-%d')
    n = len(rows)

    table_body = '\n'.join(
        f'\t{{FAULT_{r["异常命名"]}, '
        f'0x{((ACTION_VALUE[r["处理动作"]] << 4) | (r["_level_bit"] & 0x0F)):02X}u, '
        f'0x{(((r["_level_bit"] >> 4) & 0x03) << 6) | (int(r["实现期"]) << 4) | (int(r["降功率档"]) << 2) | LEVEL_VALUE[r["故障级别"]]:02X}u}},'
        f' /* {r["故障名称"]} L{LEVEL_VALUE[r["故障级别"]]}A{ACTION_VALUE[r["处理动作"]]}'
        f'D{r["降功率档"]}P{r["实现期"]} BIT{r["_level_bit"]} */'
        for _, r in rows
    )

    return f'''/**
 * @file        fault_def.c
 * @brief \t\t故障码表数据(由 {csv_name} 生成, 勿手改)
 *
 * @author      fault_param_generate.py
 * @version     1.0
 * @date        {today}
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容   |
 * |------------|------|-------|------------|
 * | {today} | 1.0  | auto  | 初始生成   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "fault_def.h"

/* 故障元数据表: 按故障码升序排列(二分查找依赖), 字段含义见 fault_def.h */
const fault_meta_t fault_meta_table[FAULT_CODE_COUNT] = {{
{table_body}
}};

/**
 * @brief 故障码 -> 元数据表索引
 * @param code 故障码 0xSLNN
 * @retval 表索引(0~FAULT_CODE_COUNT-1), 未定义码返回 -1
 * @note  表按码值升序, 采用二分查找({n}项约7次比较)
 */
int8_t fault_code_to_index(uint16_t code)
{{
\tint8_t lo = 0;
\tint8_t hi = (int8_t)FAULT_CODE_COUNT - 1;

\twhile (lo <= hi)
\t{{
\t\tint8_t mid = (int8_t)((lo + hi) / 2);
\t\tuint16_t v = fault_meta_table[mid].code;
\t\tif (v == code)
\t\t\treturn mid;
\t\tif (v < code)
\t\t\tlo = (int8_t)(mid + 1);
\t\telse
\t\t\thi = (int8_t)(mid - 1);
\t}}
\treturn -1;
}}

/**
 * @brief 取故障码元数据
 * @param code 故障码 0xSLNN
 * @retval 元数据指针, 未定义码返回 NULL
 */
const fault_meta_t *fault_meta_get(uint16_t code)
{{
\tint8_t idx = fault_code_to_index(code);
\treturn (idx < 0) ? NULL : &fault_meta_table[idx];
}}

/**
 * @brief 故障优先级比较
 * @param f1/f2 故障码
 * @retval 1=f1优先级高, 0=不高于(含相等)
 * @note  先比故障来源(高4位小者优先), 同来源比级别(中4位小者优先)
 */
uint8_t fault_is_higher_priority(uint16_t f1, uint16_t f2)
{{
\tuint8_t s1 = FAULT_GET_SOURCE(f1);
\tuint8_t s2 = FAULT_GET_SOURCE(f2);

\tif (s1 != s2)
\t\treturn (s1 < s2) ? 1u : 0u;
\treturn (FAULT_GET_LEVEL(f1) < FAULT_GET_LEVEL(f2)) ? 1u : 0u;
}}
'''


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def run_clang_format(paths, exe):
    for p in paths:
        try:
            subprocess.run([exe, '-i', p], check=False,
                           capture_output=True, timeout=30)
        except Exception as e:
            print(f'[告警] clang-format 执行失败: {e}')


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    opts = [a for a in sys.argv[1:] if a.startswith('--')]

    csv_path = args[0] if len(args) > 0 else os.path.join(script_dir, 'fault_param.csv')
    out_dir = args[1] if len(args) > 1 else os.path.join(script_dir, '..', '..', 'AppServices', 'FaultManager')

    validate_only = '--validate-only' in opts
    no_format = '--no-format' in opts
    cf_exe = 'clang-format'
    for o in opts:
        if o.startswith('--clang-format='):
            cf_exe = o.split('=', 1)[1]

    rows = load_csv(csv_path)
    warns = validate(rows)
    assign_level_bits(rows)
    csv_name = os.path.basename(csv_path)

    n = len(rows)
    p1 = sum(1 for _, r in rows if r['实现期'] == '1')
    p2 = sum(1 for _, r in rows if r['实现期'] == '2')
    p0 = n - p1 - p2
    print(f'[信息] 共 {n} 项故障码: 一期 {p1} / 二期 {p2} / 预留 {p0}')
    for w in warns:
        print(f'[告警] {w}')
    if validate_only:
        print('[信息] 仅校验模式, 未生成文件')
        return

    os.makedirs(out_dir, exist_ok=True)
    h_path = os.path.join(out_dir, 'fault_def.h')
    c_path = os.path.join(out_dir, 'fault_def.c')
    with open(h_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(gen_header(rows, csv_name))
    with open(c_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(gen_source(rows, csv_name))
    print(f'[信息] 已生成: {h_path}')
    print(f'[信息] 已生成: {c_path}')

    if not no_format:
        run_clang_format([h_path, c_path], cf_exe)


if __name__ == '__main__':
    main()
