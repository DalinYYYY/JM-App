#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MotorInfo 配置 CSV 转 C 语言模块脚本

参考 motor_param_generate_v9.py，针对 Flash/EEPROM 持久化场景的三大差异：
  1. 定义一整块 1024B 空间：motor_info_t 联合体 = raw[1024] + blocks 结构体
     (含 ParamHeader_t 头部 + 6 个子块 + 192B 预留)，可直接 memcpy 到 Flash
  2. 字节对齐：所有结构体/联合体加 __ALIGNED_4，保证 RAM 布局与 Flash 字节布局一致
  3. 每个参数单独的 get/set 读写函数，访问路径 cfg->blocks.<member>.<param>

CSV 列：VariableName, NameZh, Category, Access, DataType, DefaultValue, Min, Max,
       Unit, BlockOffset, Index, Remarks

用法:
    python motor_info_generate.py [csv] [out.h] [out.c] [选项]
    选项:
      --no-format            生成后不执行 clang-format
      --clang-format=PATH    指定 clang-format 可执行文件路径
默认:
    csv=motor_info.csv  out.h=motor_info.h  out.c=motor_info.c (当前目录)
"""

import csv
import os
import sys
import shutil
import subprocess
from datetime import datetime

# Windows 控制台默认 GBK，重配为 UTF-8 以输出中文/emoji（Python 3.7+）
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding='utf-8')
    except Exception:
        pass

# ----------------------------------------------------------------------------
# 布局常量：与 MotorInfo_readme.md 的 1024B 空间分配严格一致
# ----------------------------------------------------------------------------
PARAM_AREA_SIZE = 1024
PARAM_MAGIC = 0x53455256  # "SERVO_V2" 低 32 位
MAX_BLOCK_COUNT = 6
HEADER_SIZE = 64
TAIL_RESERVED_SIZE = 192  # 末尾预留 0x0340-0x03FF

# Category -> (结构体类型名, 成员名, 块大小B, 块偏移)
# 块偏移/块大小必须让自然累加 = 1024：64+64+128+64+320+128+64+192 = 1024
CATEGORY_MAP = {
    'SystemParam':       ('SystemParam_t',       'system',       64,  0x0040),
    'MotorCalibParam':  ('MotorCalibParam_t',  'motor_calib', 128, 0x0080),
    'DeviceParam':       ('DeviceParam_t',       'device',       64,  0x0100),
    'ControlParam':      ('ControlParam_t',      'control',     320, 0x0140),
    'ProtectCommParam':  ('ProtectCommParam_t',  'protect_comm',128, 0x0280),
    'AdvancedAlgoParam': ('AdvancedAlgoParam_t', 'advanced',     64,  0x0300),
}

CATEGORY_COMMENTS = {
    'SystemParam':       '系统级参数',
    'MotorCalibParam':  '电机标定参数（含减速器/编码器/功率级/电流采样）',
    'DeviceParam':       '设备参数（CAN/UART）',
    'ControlParam':      '控制参数（三环PID+前馈+滤波）',
    'ProtectCommParam':  '保护与通信参数',
    'AdvancedAlgoParam': '高级算法参数（MIT/力控/回零）',
}

CATEGORY_COMMENTS_EN = {
    'SystemParam':       'System Parameters',
    'MotorCalibParam':  'Motor Calibration Parameters',
    'DeviceParam':       'Device Parameters',
    'ControlParam':      'Control Parameters',
    'ProtectCommParam':  'Protection & Comm Parameters',
    'AdvancedAlgoParam': 'Advanced Algorithm Parameters',
}

# CSV 必需列
REQUIRED_COLUMNS = {
    'VariableName', 'NameZh', 'Category', 'Access', 'DataType',
    'DefaultValue', 'Min', 'Max', 'Unit', 'BlockOffset', 'Index', 'Remarks',
}

# 支持的标量数据类型 -> 字节大小
SCALAR_TYPES = {
    'float':    4,
    'int8_t':   1, 'int16_t': 2, 'int32_t': 4, 'int64_t': 8,
    'uint8_t':  1, 'uint16_t': 2, 'uint32_t': 4, 'uint64_t': 8,
}

# DataType -> 协议类型码(与 jm_cmd_def.h 的 JM_PT_* / 上位机 JmParamType 一致)
# u8=0 i8=1 u16=2 i16=3 u32=4 i32=5 f32=6
TYPE_CODE = {
    'uint8_t': 0, 'int8_t': 1, 'uint16_t': 2, 'int16_t': 3,
    'uint32_t': 4, 'int32_t': 5, 'float': 6,
    'uint64_t': 4, 'int64_t': 5,  # 64位按32位传输(截断), 实际CSV未使用
}


# ----------------------------------------------------------------------------
# 辅助函数
# ----------------------------------------------------------------------------
def format_float(val_str):
    """格式化浮点字面量，整数值补 .0f，其余补 f（兼容科学计数法）"""
    num_val = float(val_str)
    if num_val.is_integer():
        return f'{int(num_val)}.0f'
    return f'{val_str}f'


def format_default(data_type, default_val):
    """按类型格式化默认值字面量"""
    if not default_val:
        return '0.0f' if data_type == 'float' else '0'
    if data_type == 'float':
        return format_float(default_val)
    v = int(float(default_val))
    if data_type.startswith('uint'):
        return f'{v}U'
    if data_type == 'int64_t':
        return f'{v}LL'
    if data_type == 'uint64_t':
        return f'{v}ULL'
    return str(v)


def is_readonly(param):
    return param.get('Access', '').strip().upper() in ('RO', 'R', 'READONLY')


def printf_fmt_and_cast(data_type):
    if data_type == 'float':
        return '%f', ''
    if data_type == 'int64_t':
        return '%lld', '(long long)'
    if data_type == 'uint64_t':
        return '%llu', '(unsigned long long)'
    if data_type.startswith('uint'):
        return '%u', '(unsigned)'
    return '%d', '(int)'


def range_check_expr(data_type, min_val, max_val, lhs):
    """生成范围检查的"越界条件"表达式（越界为真）。无需检查返回 None。"""
    if not min_val or not max_val:
        return None
    min_f = float(min_val)
    max_f = float(max_val)
    if min_f == 0 and max_f == 0:
        return None
    skip_min = data_type.startswith('uint') and min_f == 0

    if data_type == 'float':
        mn, mx = format_float(min_val), format_float(max_val)
        return f'{lhs} > {mx}' if skip_min else f'{lhs} < {mn} || {lhs} > {mx}'
    if data_type == 'int64_t':
        mn, mx = int(min_f), int(max_f)
        return f'{lhs} > {mx}LL' if skip_min else f'{lhs} < {mn}LL || {lhs} > {mx}LL'
    mn, mx = int(min_f), int(max_f)
    if skip_min:
        return f'{lhs} > ({data_type}){mx}'
    return f'{lhs} < ({data_type}){mn} || {lhs} > ({data_type}){mx}'


def file_banner(filename, brief):
    return [
        '/**',
        f' * @file    {filename}',
        f' * @brief   {brief}',
        f' * @date    {datetime.now().strftime("%Y-%m-%d")}',
        ' *',
        ' * @warning 【自动生成文件，请勿手动修改】',
        f' *          本文件由脚本 {os.path.basename(__file__)} 根据 motor_info.csv 自动生成，',
        ' *          任何手动改动都会在下次运行脚本时被覆盖。',
        ' *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。',
        ' */',
    ]


# ----------------------------------------------------------------------------
# CSV 读取与校验
# ----------------------------------------------------------------------------
def load_csv(csv_file):
    if not os.path.isfile(csv_file):
        raise ValueError(f'找不到 CSV 文件: {csv_file}')
    with open(csv_file, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError('CSV 为空或无表头')
        missing = REQUIRED_COLUMNS - set(reader.fieldnames)
        if missing:
            raise ValueError(f'CSV 缺少必需列: {", ".join(sorted(missing))}')
        params = list(reader)
    if not params:
        raise ValueError('CSV 无数据行')

    errors = []
    seen_names = set()
    seen_index = set()
    for i, p in enumerate(params, start=2):
        name = p.get('VariableName', '').strip()
        if not name:
            errors.append(f'第 {i} 行: VariableName 为空')
            continue
        if name in seen_names:
            errors.append(f'第 {i} 行: VariableName 重复 "{name}"')
        seen_names.add(name)
        if p['Category'] not in CATEGORY_MAP:
            errors.append(f'第 {i} 行[{name}]: 未知 Category "{p["Category"]}"')
        if p['DataType'] not in SCALAR_TYPES:
            errors.append(f'第 {i} 行[{name}]: 未知 DataType "{p["DataType"]}"')
        try:
            idx = int(p['Index'])
        except (ValueError, TypeError):
            errors.append(f'第 {i} 行[{name}]: Index 非法 "{p.get("Index")}"')
            continue
        if idx in seen_index:
            errors.append(f'第 {i} 行[{name}]: Index 重复 {idx}')
        seen_index.add(idx)
    if errors:
        raise ValueError('CSV 校验失败:\n  - ' + '\n  - '.join(errors))
    return params


def group_params(params):
    """按 Category 分组，保持首次出现顺序"""
    groups = {}
    for p in params:
        groups.setdefault(p['Category'], []).append(p)
    return groups


def compute_block_layout(grouped):
    """
    校验每个块：已用字节 + reserved 必须 == 块大小。
    返回 {Category: reserved_uint32_count}（要求 used 是 4 的倍数）。
    """
    layout = {}
    for cat, gp in grouped.items():
        _, _, block_size, _ = CATEGORY_MAP[cat]
        used = sum(SCALAR_TYPES[p['DataType']] for p in gp)
        if used > block_size:
            raise ValueError(f'块 {cat} 参数总占用 {used}B 超过块大小 {block_size}B')
        reserved_bytes = block_size - used
        if reserved_bytes % 4 != 0:
            raise ValueError(f'块 {cat} 预留 {reserved_bytes}B 非 4 字节对齐，请检查参数类型')
        layout[cat] = reserved_bytes // 4
    return layout


# ----------------------------------------------------------------------------
# 头文件生成
# ----------------------------------------------------------------------------
def generate_h(params, grouped, layout, output_h, module_name, config_type):
    L = []
    guard = f'__{module_name.upper()}_H__'
    prefix = module_name.upper()

    L += file_banner(os.path.basename(output_h), 'MotorInfo 配置参数 API 接口（1024B 整块空间）')
    L.append('')
    L.append(f'#ifndef {guard}')
    L.append(f'#define {guard}')
    L.append('')
    L.append('#ifdef __cplusplus')
    L.append('extern "C" {')
    L.append('#endif')
    L.append('')
    L.append('#include <stdint.h>')
    L.append('#include <stdbool.h>')
    L.append('')
    L.append('/* 字节对齐宏：保证 RAM 布局与 Flash 字节布局完全一致，可整块 memcpy */')
    L.append('#ifndef __ALIGNED_4')
    L.append('#define __ALIGNED_4 __attribute__((aligned(4)))')
    L.append('#endif')
    L.append('')

    # 元信息
    L.append('/* ===== 自动生成元信息 ===== */')
    L.append(f'#define {prefix}_GEN_DATE     "{datetime.now().strftime("%Y-%m-%d")}"')
    L.append(f'#define {prefix}_PARAM_COUNT   {len(params)}')
    L.append(f'#define {prefix}_VERSION_MAJOR 1')
    L.append(f'#define {prefix}_VERSION_MINOR 0')
    L.append('')

    # Flash 区域布局常量
    L.append('/* ===== Flash 区域布局常量 ===== */')
    L.append(f'#define PARAM_MAGIC       0x{PARAM_MAGIC:08X}u  /* "SERVO_V2" */')
    L.append(f'#define PARAM_AREA_SIZE    {PARAM_AREA_SIZE}')
    L.append(f'#define MAX_BLOCK_COUNT    {MAX_BLOCK_COUNT}')
    L.append('')

    # 各块偏移/大小常量
    L.append('/* 各子块偏移/大小常量（供 Flash 读写寻址） */')
    for cat in grouped:
        struct_name, _, block_size, block_offset = CATEGORY_MAP[cat]
        cat_up = cat.upper()
        L.append(f'#define {prefix}_BLOCK_{cat_up}_OFFSET 0x{block_offset:04X}u')
        L.append(f'#define {prefix}_BLOCK_{cat_up}_SIZE   {block_size}u')
    L.append('')

    # 头部与块索引
    L.append('/* ===== 全局头部与子块索引表 ===== */')
    L.append('typedef struct __ALIGNED_4')
    L.append('{')
    L.append('    uint32_t offset;  /* 子块偏移(字节) */')
    L.append('    uint32_t size;    /* 子块大小(字节) */')
    L.append('} BlockIndex_t;')
    L.append('')
    L.append('typedef struct __ALIGNED_4')
    L.append('{')
    L.append('    uint32_t      magic;                     /* 魔数 PARAM_MAGIC */')
    L.append('    uint16_t      version_major;             /* 主版本 */')
    L.append('    uint16_t      version_minor;             /* 次版本 */')
    L.append('    uint32_t      crc32;                     /* 参数区 CRC32(校验范围跳过本字段) */')
    L.append(f'    BlockIndex_t  blocks[MAX_BLOCK_COUNT];   /* {MAX_BLOCK_COUNT}*8 = 48B */')
    L.append('    uint32_t      reserved;                  /* 4B 填充凑足 64B */')
    L.append('} ParamHeader_t;  /* 64B */')
    L.append('')

    # 子块结构体
    L.append('/* ===== 各子块数据结构（字段顺序与 CSV 一致，4 字节对齐）===== */')
    for cat, gp in grouped.items():
        struct_name, _, _, _ = CATEGORY_MAP[cat]
        res_cnt = layout[cat]
        L.append('/**')
        L.append(f' * @brief   {CATEGORY_COMMENTS[cat]}')
        L.append(f' * @details 块大小 {CATEGORY_MAP[cat][2]}B，已用 {sum(SCALAR_TYPES[p["DataType"]] for p in gp)}B，'
                 f'预留 {res_cnt * 4}B')
        L.append(' */')
        L.append('typedef struct __ALIGNED_4')
        L.append('{')
        for p in gp:
            comment = p['NameZh'] + (f' ({p["Unit"]})' if p['Unit'] else '')
            if p['Remarks']:
                comment += f'  [{p["Remarks"]}]'
            L.append(f'    {p["DataType"]:<8} {p["VariableName"]:<32}; /* {comment} */')
        if res_cnt > 0:
            L.append(f'    uint32_t reserved[{res_cnt}];                    /* 预留 {res_cnt * 4}B */')
        L.append(f'}} {struct_name};')
        L.append('')

    # 主联合体
    L.append('/* ===== 主参数区联合体：1024B 整块空间 ===== */')
    L.append('typedef union __ALIGNED_4')
    L.append('{')
    L.append('    uint8_t raw[PARAM_AREA_SIZE];  /* 原始字节数组，可直接 memcpy 到 Flash */')
    L.append('    struct __ALIGNED_4')
    L.append('    {')
    L.append('        ParamHeader_t header;            /* 0x0000  64B */')
    for cat in grouped:
        struct_name, member_name, block_size, block_offset = CATEGORY_MAP[cat]
        L.append(f'        {struct_name:<20} {member_name:<14}; /* 0x{block_offset:04X}  {block_size}B  {CATEGORY_COMMENTS[cat]} */')
    L.append(f'        uint8_t reserved[{TAIL_RESERVED_SIZE}];            /* 0x0340  {TAIL_RESERVED_SIZE}B  末尾预留 */')
    L.append(f'    }} blocks;')
    L.append(f'}} {config_type};  /* {PARAM_AREA_SIZE}B */')
    L.append('')

    # 基础 API
    L.append('/******************************************************************************')
    L.append(' * @brief   基础 API')
    L.append(' ******************************************************************************/')
    L.append('/**')
    L.append(f' * @brief   初始化为默认值（含头部魔数/版本/块索引表 + 各参数默认值）')
    L.append(f' * @param   cfg 参数区指针')
    L.append(' * @return  0=成功, -EINVAL=空指针')
    L.append(' */')
    L.append(f'int  {module_name}_init({config_type} *cfg);')
    L.append('')
    L.append('/**')
    L.append(' * @brief   校验所有参数范围')
    L.append(' * @param   cfg 参数区指针')
    L.append(' * @return  0=全部通过, >0=首个越界参数的 Index(见CSV), -EINVAL=空指针')
    L.append(' */')
    L.append(f'int  {module_name}_validate(const {config_type} *cfg);')
    L.append('')
    L.append('/**')
    L.append(' * @brief   打印所有参数')
    L.append(' * @param   cfg 参数区指针')
    L.append(' */')
    L.append(f'void {module_name}_print(const {config_type} *cfg);')
    L.append('')

    # 每个参数的 get/set 声明
    for cat, gp in grouped.items():
        _, member_name, _, _ = CATEGORY_MAP[cat]
        L.append('/******************************************************************************')
        L.append(f' * @brief   {CATEGORY_COMMENTS[cat]}')
        L.append(' ******************************************************************************/')
        for p in gp:
            name = p['VariableName']
            data_type = p['DataType']
            desc = p['NameZh']
            unit = f' ({p["Unit"]})' if p['Unit'] else ''
            L.append('/**')
            L.append(f' * @brief   读取 {desc}{unit}')
            L.append(f' * @param   cfg 参数区指针')
            L.append(f' * @return  {desc}')
            L.append(' */')
            L.append(f'{data_type} {module_name}_get_{name}(const {config_type} *cfg);')
            if not is_readonly(p):
                L.append('/**')
                L.append(f' * @brief   设置 {desc}{unit}')
                L.append(f' * @param   cfg 参数区指针')
                L.append(f' * @param   value 要设置的值')
                L.append(f' * @return  0=成功, -EINVAL=空指针或越界')
                L.append(' */')
                L.append(f'int {module_name}_set_{name}({config_type} *cfg, {data_type} value);')
            L.append('')

    # 协议分发表声明(并入头文件末尾, 在 extern "C" 闭合前)
    L += build_dispatch_h_lines(params, module_name, config_type)

    L.append('#ifdef __cplusplus')
    L.append('}')
    L.append('#endif')
    L.append('')
    L.append(f'#endif /* {guard} */')
    L.append('')

    with open(output_h, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L))
    print(f'✅ 生成头文件: {output_h}')


# ----------------------------------------------------------------------------
# 实现文件生成
# ----------------------------------------------------------------------------
def generate_c(params, grouped, layout, output_c, output_h, module_name, config_type):
    L = []
    prefix = module_name.upper()

    L += file_banner(os.path.basename(output_c), 'MotorInfo 配置参数 API 实现')
    L.append('')
    L.append(f'#include "{os.path.basename(output_h)}"')
    L.append('#include <stdio.h>')
    L.append('#include <string.h>')
    L.append('#include <errno.h>')
    L.append('')

    # init
    L.append(f'int {module_name}_init({config_type} *cfg)')
    L.append('{')
    L.append('    if (cfg == NULL) return -EINVAL;')
    L.append('    memset(cfg->raw, 0, PARAM_AREA_SIZE);')
    L.append('')
    L.append('    /* ---- 头部 ---- */')
    L.append('    cfg->blocks.header.magic = PARAM_MAGIC;')
    L.append(f'    cfg->blocks.header.version_major = {prefix}_VERSION_MAJOR;')
    L.append(f'    cfg->blocks.header.version_minor = {prefix}_VERSION_MINOR;')
    L.append('    cfg->blocks.header.crc32 = 0;')
    for i, cat in enumerate(grouped):
        _, _, block_size, block_offset = CATEGORY_MAP[cat]
        L.append(f'    cfg->blocks.header.blocks[{i}].offset = 0x{block_offset:04X}u;')
        L.append(f'    cfg->blocks.header.blocks[{i}].size   = {block_size}u;')
    L.append('    cfg->blocks.header.reserved = 0;')
    L.append('')

    # 各块默认值
    for cat, gp in grouped.items():
        _, member_name, _, _ = CATEGORY_MAP[cat]
        L.append(f'    /* ---- {CATEGORY_COMMENTS[cat]} ---- */')
        for p in gp:
            val = format_default(p['DataType'], p['DefaultValue'])
            L.append(f'    cfg->blocks.{member_name}.{p["VariableName"]} = {val};')
        L.append('')

    L.append('    return 0;')
    L.append('}')
    L.append('')

    # validate
    L.append(f'int {module_name}_validate(const {config_type} *cfg)')
    L.append('{')
    L.append('    if (cfg == NULL) return -EINVAL;')
    L.append('')
    L.append('    /* 头部魔数校验 */')
    L.append('    if (cfg->blocks.header.magic != PARAM_MAGIC) return -1;')
    L.append('')
    for cat, gp in grouped.items():
        _, member_name, _, _ = CATEGORY_MAP[cat]
        L.append(f'    /* {CATEGORY_COMMENTS[cat]} */')
        for p in gp:
            lhs = f'cfg->blocks.{member_name}.{p["VariableName"]}'
            cond = range_check_expr(p['DataType'], p['Min'], p['Max'], lhs)
            if cond is None:
                continue
            L.append(f'    if ({cond}) return {int(p["Index"])};  /* {p["VariableName"]} */')
        L.append('')
    L.append('    return 0;')
    L.append('}')
    L.append('')

    # print
    L.append(f'void {module_name}_print(const {config_type} *cfg)')
    L.append('{')
    L.append('    if (cfg == NULL) return;')
    L.append('    printf("========== MotorInfo Config (%d params) ==========\\n",'
             f' {prefix}_PARAM_COUNT);')
    L.append('    printf("magic=0x%08X  v%d.%d  crc32=0x%08X\\n",')
    L.append('           (unsigned)cfg->blocks.header.magic,')
    L.append('           cfg->blocks.header.version_major,')
    L.append('           cfg->blocks.header.version_minor,')
    L.append('           (unsigned)cfg->blocks.header.crc32);')
    for cat, gp in grouped.items():
        _, member_name, _, _ = CATEGORY_MAP[cat]
        L.append('')
        L.append(f'    printf("\\n--- {CATEGORY_COMMENTS_EN[cat]} ---\\n");')
        for p in gp:
            name = p['VariableName']
            data_type = p['DataType']
            unit = p['Unit']
            member = f'cfg->blocks.{member_name}.{name}'
            unit_str = f' {unit}' if unit else ''
            fmt, cast = printf_fmt_and_cast(data_type)
            arg = f'{cast}{member}' if cast else member
            L.append(f'    printf("{name}: {fmt}{unit_str}\\n", {arg});')
    L.append('')
    L.append('    printf("\\n=============================================\\n");')
    L.append('}')
    L.append('')

    # 每个参数的 get/set 实现
    for cat, gp in grouped.items():
        _, member_name, _, _ = CATEGORY_MAP[cat]
        L.append('/******************************************************************************')
        L.append(f' * {CATEGORY_COMMENTS[cat]}')
        L.append(' ******************************************************************************/')
        for p in gp:
            name = p['VariableName']
            data_type = p['DataType']
            member = f'cfg->blocks.{member_name}.{name}'

            # get
            L.append(f'{data_type} {module_name}_get_{name}(const {config_type} *cfg)')
            L.append('{')
            L.append(f'    return {member};')
            L.append('}')
            L.append('')

            # set (只读跳过)
            if is_readonly(p):
                continue
            L.append(f'int {module_name}_set_{name}({config_type} *cfg, {data_type} value)')
            L.append('{')
            L.append('    if (cfg == NULL) return -EINVAL;')
            cond = range_check_expr(data_type, p['Min'], p['Max'], 'value')
            if cond is not None:
                L.append(f'    if ({cond}) return -EINVAL;  /* 越界 */')
            L.append(f'    {member} = value;')
            L.append('    return 0;')
            L.append('}')
            L.append('')

    # 协议分发表实现(并入实现文件末尾)
    L += build_dispatch_c_lines(params, module_name, config_type)

    with open(output_c, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L))
    print(f'✅ 生成实现文件: {output_c}')


# ----------------------------------------------------------------------------
# clang-format
# ----------------------------------------------------------------------------
def run_clang_format(files, clang_format_path=None):
    exe = clang_format_path or shutil.which('clang-format')
    if not exe:
        print('')
        print('⚠ 未找到 clang-format，已跳过自动格式化。请手动执行：')
        print(f'   clang-format -i -style=file {" ".join(files)}')
        return False
    ok = True
    for f in files:
        try:
            subprocess.run([exe, '-i', '-style=file', f], check=True)
            print(f'✨ 已格式化: {f}')
        except subprocess.CalledProcessError as e:
            ok = False
            print(f'⚠ 格式化失败 {f}: {e}')
    return ok


# ----------------------------------------------------------------------------
# 分发表生成: param_id(Index) -> get/set 调用 + 类型码, 供协议层 0xE6-0xEB 使用
# 直接并入 motor_info.h/.c, 不单独生成 dispatch 文件(避免重复维护)
# ----------------------------------------------------------------------------
DISPATCH_OK = 0
DISPATCH_E_BAD_ID = -1
DISPATCH_E_BOUNDS = -2
DISPATCH_E_RO = -3


def build_dispatch_h_lines(params, module_name, config_type):
    """返回 dispatch 声明行列表(并入 motor_info.h 末尾, 在 extern "C" 闭合前)"""
    L = []
    L.append('/******************************************************************************')
    L.append(' * @brief   协议分发表 (param_id -> get/set), 供 0xE6-0xEB 单参读写')
    L.append(' ******************************************************************************/')
    L.append('/* dispatch 返回码 */')
    L.append(f'#define {module_name.upper()}_DISPATCH_OK       {DISPATCH_OK}')
    L.append(f'#define {module_name.upper()}_DISPATCH_E_BAD_ID {DISPATCH_E_BAD_ID}')
    L.append(f'#define {module_name.upper()}_DISPATCH_E_BOUNDS {DISPATCH_E_BOUNDS}')
    L.append(f'#define {module_name.upper()}_DISPATCH_E_RO     {DISPATCH_E_RO}')
    L.append('')
    L.append('/**')
    L.append(f' * @brief 按 param_id 读单个参数, 值写入 out4(固定4B, 零填充)')
    L.append(' * @param  pid      参数ID(CSV Index)')
    L.append(f' * @param  cfg      {config_type} 指针')
    L.append(' * @param  out4     输出缓冲(4字节)')
    L.append(' * @param  out_type 输出协议类型码(0=u8..6=f32)')
    L.append(' * @param  out_len  输出实际值字节数(1/2/4)')
    L.append(f' * @return {DISPATCH_OK}=成功, {DISPATCH_E_BAD_ID}=未知pid/空指针')
    L.append(' */')
    L.append(f'int {module_name}_dispatch_read(uint16_t pid, const {config_type} *cfg, '
             f'uint8_t out4[4], uint8_t *out_type, uint8_t *out_len);')
    L.append('')
    L.append('/**')
    L.append(f' * @brief 按 param_id 写单个参数, 值取自 in4(固定4B)')
    L.append(' * @param  pid  参数ID(CSV Index)')
    L.append(f' * @param  cfg  {config_type} 指针')
    L.append(' * @param  in4  输入缓冲(4字节)')
    L.append(' * @param  len  实际有效字节数(仅校验, 取低4B)')
    L.append(f' * @return {DISPATCH_OK}=成功, {DISPATCH_E_BAD_ID}=未知pid/空指针, '
             f'{DISPATCH_E_BOUNDS}=越界, {DISPATCH_E_RO}=只读')
    L.append(' */')
    L.append(f'int {module_name}_dispatch_write(uint16_t pid, {config_type} *cfg, '
             f'const uint8_t in4[4], uint8_t len);')
    L.append('')
    return L


def build_dispatch_c_lines(params, module_name, config_type):
    """返回 dispatch 实现行列表(并入 motor_info.c 末尾)"""
    L = []
    L.append('/******************************************************************************')
    L.append(' * 协议分发表实现 (param_id -> get/set), 供 0xE6-0xEB 单参读写')
    L.append(' ******************************************************************************/')
    L.append('')
    # ---- dispatch_read ----
    L.append(f'int {module_name}_dispatch_read(uint16_t pid, const {config_type} *cfg, '
             f'uint8_t out4[4], uint8_t *out_type, uint8_t *out_len)')
    L.append('{')
    L.append('    if (cfg == NULL || out4 == NULL || out_type == NULL || out_len == NULL)')
    L.append(f'        return {DISPATCH_E_BAD_ID};')
    L.append('    /* 固定4B, 先清零(短类型高字节补零) */')
    L.append('    out4[0] = out4[1] = out4[2] = out4[3] = 0;')
    L.append('    switch (pid)')
    L.append('    {')
    for p in params:
        name = p['VariableName']
        dt = p['DataType']
        tcode = TYPE_CODE.get(dt, 6)
        tlen = SCALAR_TYPES.get(dt, 4)
        if tlen > 4:
            tlen = 4
        L.append(f'    case {int(p["Index"])}: {{ /* {name} ({dt}) */')
        L.append(f'        {dt} v = {module_name}_get_{name}(cfg);')
        L.append(f'        memcpy(out4, &v, {tlen});')
        L.append(f'        *out_type = {tcode}; *out_len = {tlen};')
        L.append(f'        return {DISPATCH_OK}; }}')
    L.append('    default:')
    L.append(f'        return {DISPATCH_E_BAD_ID};')
    L.append('    }')
    L.append('}')
    L.append('')
    # ---- dispatch_write ----
    L.append(f'int {module_name}_dispatch_write(uint16_t pid, {config_type} *cfg, '
             f'const uint8_t in4[4], uint8_t len)')
    L.append('{')
    L.append('    (void)len; /* 帧内固定4B, 仅取低4B */')
    L.append('    if (cfg == NULL || in4 == NULL)')
    L.append(f'        return {DISPATCH_E_BAD_ID};')
    L.append('    switch (pid)')
    L.append('    {')
    for p in params:
        name = p['VariableName']
        dt = p['DataType']
        tlen = SCALAR_TYPES.get(dt, 4)
        if tlen > 4:
            tlen = 4
        if is_readonly(p):
            L.append(f'    case {int(p["Index"])}: /* {name} 只读 */')
            L.append(f'        return {DISPATCH_E_RO};')
        else:
            L.append(f'    case {int(p["Index"])}: {{ /* {name} ({dt}) */')
            L.append(f'        {dt} v;')
            L.append(f'        memcpy(&v, in4, {tlen});')
            L.append(f'        int rc = {module_name}_set_{name}(cfg, v);')
            L.append(f'        return (rc == 0) ? {DISPATCH_OK} : {DISPATCH_E_BOUNDS}; }}')
    L.append('    default:')
    L.append(f'        return {DISPATCH_E_BAD_ID};')
    L.append('    }')
    L.append('}')
    L.append('')
    return L


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def generate(csv_file, output_h, output_c, do_format=True, clang_format_path=None):
    params = load_csv(csv_file)
    grouped = group_params(params)
    layout = compute_block_layout(grouped)

    module_name = os.path.splitext(os.path.basename(output_h))[0]
    config_type = f'{module_name}_t'

    generate_h(params, grouped, layout, output_h, module_name, config_type)
    generate_c(params, grouped, layout, output_c, output_h, module_name, config_type)

    # 统计
    get_count = len(params)
    set_count = sum(1 for p in params if not is_readonly(p))
    print('')
    print('📋 生成统计:')
    print(f'   - 联合体类型 : {config_type}  ({PARAM_AREA_SIZE}B 整块空间)')
    print(f'   - 子块数量   : {len(grouped)}')
    for cat, gp in grouped.items():
        used = sum(SCALAR_TYPES[p['DataType']] for p in gp)
        _, _, block_size, block_offset = CATEGORY_MAP[cat]
        print(f'       · {cat:<20} 0x{block_offset:04X}  {block_size}B  '
              f'已用{used}B 预留{block_size - used}B  参数{len(gp)}个')
    print(f'   - 参数总数   : {len(params)}')
    print(f'   - Get 函数   : {get_count}')
    print(f'   - Set 函数   : {set_count}')
    print(f'   - 基础函数   : 3 (init/validate/print)')
    print(f'   - 分发表     : dispatch_read + dispatch_write (switch {len(params)} cases, 已并入本文件)')

    if do_format:
        run_clang_format([output_h, output_c], clang_format_path)


def main(argv):
    csv_file = 'motor_info.csv'
    output_h = 'motor_info.h'
    output_c = 'motor_info.c'
    do_format = True
    clang_format_path = None

    positional = []
    for arg in argv:
        if arg == '--no-format':
            do_format = False
        elif arg.startswith('--clang-format='):
            clang_format_path = arg.split('=', 1)[1]
        elif arg in ('-h', '--help'):
            print(__doc__)
            return 0
        else:
            positional.append(arg)

    if len(positional) > 0:
        csv_file = positional[0]
    if len(positional) > 1:
        output_h = positional[1]
    if len(positional) > 2:
        output_c = positional[2]

    try:
        generate(csv_file, output_h, output_c, do_format, clang_format_path)
    except ValueError as e:
        print(f'❌ {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
