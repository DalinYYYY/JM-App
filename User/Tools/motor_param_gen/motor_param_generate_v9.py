#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
关节电机配置CSV转C语言头文件脚本 V9

相对 V8 的改进：
  1. 生成后自动按项目 .clang-format 执行一次格式化（找不到 clang-format 时优雅降级）
  2. 修复 Windows 控制台 GBK 编码导致的崩溃（启动即重配 stdout 为 UTF-8）
  3. CSV 输入校验：缺列 / 未知 group / 未知数据类型时给出友好报错而非堆栈崩溃
  4. 支持 access 字段：只读(RO)参数不再生成 Set 接口
  5. char 数组参数（如电机名称）补齐字符串版 get/set 与 %s 打印（V8 直接忽略数组）
  6. 校验函数返回首个越界参数的 CSV id（可追溯），0 表示全部通过
  7. 抽取重复的范围检查逻辑为单一辅助函数，便于维护
  8. 打印函数按类型选用 %u/%lld/%llu 并显式 cast，消除 -Wformat 警告
  9. 头文件增加生成日期、参数数量等元信息宏

用法:
    python generate_config_header_v9.py [csv] [out.h] [out.c] [选项]
    选项:
      --no-format            生成后不执行 clang-format
      --clang-format=PATH    指定 clang-format 可执行文件路径
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

# Group 结构体名称映射: group -> (结构体类型名, 成员名)
GROUP_STRUCT_NAMES = {
    'MotorInstance':    ('motor_instance_t', 'motor_instance'),
    'MotorConfig':      ('motor_base_t', 'motor_base'),
    'GearboxConfig':    ('gearbox_param_t', 'gearbox_param'),
    'EncoderConfig':    ('encoder_param_t', 'encoder_param'),
    'PositionConfig':   ('position_limit_t', 'position_limit'),
    'HomingConfig':     ('homing_param_t', 'homing_param'),
    'CurrentLoop':      ('current_loop_t', 'current_loop'),
    'PositionLoop':     ('position_loop_t', 'position_loop'),
    'ImpedanceControl': ('impedance_ctrl_t', 'impedance_ctrl'),
    'ThermalConfig':    ('thermal_model_t', 'thermal_model'),
    'Protection':       ('protection_param_t', 'protection_param'),
}

GROUP_COMMENTS = {
    'MotorInstance':    '电机实例标识',
    'MotorConfig':      '电机本体参数',
    'GearboxConfig':    '减速器参数',
    'EncoderConfig':    '编码器参数',
    'PositionConfig':   '位置限位配置',
    'HomingConfig':     '回零配置参数',
    'CurrentLoop':      '电流环控制参数',
    'PositionLoop':     '位置速度环控制参数',
    'ImpedanceControl': '阻抗控制参数',
    'ThermalConfig':    '热模型参数',
    'Protection':       '保护参数配置',
}

GROUP_COMMENTS_EN = {
    'MotorInstance':    'Motor Instance',
    'MotorConfig':      'Motor Base Parameters',
    'GearboxConfig':    'Gearbox Parameters',
    'EncoderConfig':    'Encoder Parameters',
    'PositionConfig':   'Position Limit Config',
    'HomingConfig':     'Homing Config',
    'CurrentLoop':      'Current Loop Control',
    'PositionLoop':     'Position/Velocity Loop',
    'ImpedanceControl': 'Impedance Control',
    'ThermalConfig':    'Thermal Model',
    'Protection':       'Protection Config',
}

# CSV 必需列
REQUIRED_COLUMNS = {
    'id', 'param_name', 'description', 'group', 'access', 'data_type',
    'array_size', 'default_value', 'unit', 'min_value', 'max_value',
}

# 支持的标量数据类型
SCALAR_TYPES = {
    'float', 'char',
    'int8_t', 'int16_t', 'int32_t', 'int64_t',
    'uint8_t', 'uint16_t', 'uint32_t', 'uint64_t',
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


def is_readonly(param):
    """access 为 RO/R/READONLY 视为只读，不生成 Set 接口"""
    return param.get('access', '').strip().upper() in ('RO', 'R', 'READONLY')


def is_char_array(param, array_size):
    """char 数组视为字符串"""
    return param['data_type'] == 'char' and array_size > 1


def printf_fmt_and_cast(data_type):
    """返回 (printf格式符, 强制转换前缀) ，消除 -Wformat 警告"""
    if data_type == 'float':
        return '%f', ''  # float 在可变参中提升为 double
    if data_type == 'int64_t':
        return '%lld', '(long long)'
    if data_type == 'uint64_t':
        return '%llu', '(unsigned long long)'
    if data_type.startswith('uint'):
        return '%u', '(unsigned)'
    return '%d', '(int)'


def range_check_expr(data_type, min_val, max_val, lhs):
    """
    生成范围检查的"越界条件"表达式（越界为真）。
    无需检查（无范围或全 0）时返回 None。
    lhs 为被检查的左值（validate 用结构体成员，set 用 value）。
    """
    if not min_val or not max_val:
        return None

    min_f = float(min_val)
    max_f = float(max_val)
    if min_f == 0 and max_f == 0:
        return None

    # 无符号类型且下限为 0 时，省去下限检查（恒为真，避免 -Wtype-limits 警告）
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
    """生成带防修改警告的文件头注释"""
    return [
        '/**',
        f' * @file    {filename}',
        f' * @brief   {brief}',
        f' * @date    {datetime.now().strftime("%Y-%m-%d")}',
        ' *',
        ' * @warning 【自动生成文件，请勿手动修改】',
        f' *          本文件由脚本 {os.path.basename(__file__)} 根据配置表自动生成，',
        ' *          任何手动改动都会在下次运行脚本时被覆盖。',
        ' *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。',
        ' */',
    ]


# ----------------------------------------------------------------------------
# CSV 读取与校验
# ----------------------------------------------------------------------------
def load_csv(csv_file):
    """读取并校验 CSV，返回参数列表，发现错误时抛出 ValueError"""
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
    for i, p in enumerate(params, start=2):  # 行号从 2 起（含表头）
        name = p.get('param_name', '').strip()
        if not name:
            errors.append(f'第 {i} 行: param_name 为空')
            continue
        if p['group'] not in GROUP_STRUCT_NAMES:
            errors.append(f'第 {i} 行[{name}]: 未知 group "{p["group"]}"')
        if p['data_type'] not in SCALAR_TYPES:
            errors.append(f'第 {i} 行[{name}]: 未知 data_type "{p["data_type"]}"')
        try:
            int(float(p['array_size']))
        except (ValueError, KeyError):
            errors.append(f'第 {i} 行[{name}]: array_size 非法 "{p.get("array_size")}"')

    if errors:
        raise ValueError('CSV 校验失败:\n  - ' + '\n  - '.join(errors))

    return params


def group_params(params):
    """按 group 分组，保持首次出现顺序"""
    groups = {}
    for p in params:
        groups.setdefault(p['group'], []).append(p)
    return groups


# ----------------------------------------------------------------------------
# 头文件生成
# ----------------------------------------------------------------------------
def generate_h(params, groups, output_h, module_name, config_type):
    L = []
    guard = f'__{module_name.upper()}_H__'
    prefix = module_name.upper()

    L += file_banner(os.path.basename(output_h), '关节电机配置参数API接口')
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

    # 元信息宏
    L.append('/* ===== 自动生成元信息 ===== */')
    L.append(f'#define {prefix}_GEN_DATE   "{datetime.now().strftime("%Y-%m-%d")}"')
    L.append(f'#define {prefix}_PARAM_COUNT {len(params)}')
    L.append('')

    # 前向声明
    L.append('/* 配置结构体前向声明 */')
    L.append(f'typedef struct {module_name} {config_type};')
    L.append('')

    # 子结构体
    for group_name, gp in groups.items():
        struct_name, _ = GROUP_STRUCT_NAMES[group_name]
        L.append('/**')
        L.append(f' * @brief   {GROUP_COMMENTS[group_name]}')
        L.append(' */')
        L.append('typedef struct')
        L.append('{')
        for p in gp:
            data_type = p['data_type']
            array_size = int(float(p['array_size']))
            comment = p['description'] + (f' ({p["unit"]})' if p['unit'] else '')
            if array_size > 1:
                L.append(f'    {data_type} {p["param_name"]}[{array_size}]; /* {comment} */')
            else:
                L.append(f'    {data_type} {p["param_name"]}; /* {comment} */')
        L.append(f'}} {struct_name};')
        L.append('')

    # 主结构体
    L.append('/**')
    L.append(' * @brief   关节电机完整配置结构体')
    L.append(' */')
    L.append(f'struct {module_name}')
    L.append('{')
    for group_name in groups:
        struct_name, member_name = GROUP_STRUCT_NAMES[group_name]
        L.append(f'    {struct_name} {member_name}; /* {GROUP_COMMENTS[group_name]} */')
    L.append('};')
    L.append('')

    # 基础 API
    L.append('/******************************************************************************')
    L.append(' * @brief   基础API接口')
    L.append(' ******************************************************************************/')
    L.append('')
    L.append('/**')
    L.append(' * @brief   初始化电机配置为默认值')
    L.append(' * @param   cfg 电机配置指针')
    L.append(' * @return  0=成功, -EINVAL=参数错误')
    L.append(' */')
    L.append(f'int {module_name}_init({config_type} *cfg);')
    L.append('')
    L.append('/**')
    L.append(' * @brief   校验电机配置参数范围')
    L.append(' * @param   cfg 电机配置指针')
    L.append(' * @return  0=全部通过, >0=首个越界参数的 id(见CSV), -EINVAL=空指针')
    L.append(' */')
    L.append(f'int {module_name}_validate(const {config_type} *cfg);')
    L.append('')
    L.append('/**')
    L.append(' * @brief   打印电机配置所有参数')
    L.append(' * @param   cfg 电机配置指针')
    L.append(' */')
    L.append(f'void {module_name}_print(const {config_type} *cfg);')
    L.append('')

    # Get/Set 声明
    for group_name, gp in groups.items():
        L.append('/******************************************************************************')
        L.append(f' * @brief   {GROUP_COMMENTS[group_name]}')
        L.append(' ******************************************************************************/')
        L.append('')
        for p in gp:
            name = p['param_name']
            data_type = p['data_type']
            desc = p['description']
            array_size = int(float(p['array_size']))
            char_arr = is_char_array(p, array_size)

            # 数值数组无标量访问语义，跳过
            if array_size > 1 and not char_arr:
                continue

            if char_arr:
                # 字符串 get/set
                L.append('/**')
                L.append(f' * @brief   获取{desc}')
                L.append(' * @param   cfg 电机配置指针')
                L.append(f' * @return  指向{desc}字符串的指针')
                L.append(' */')
                L.append(f'const char *{module_name}_get_{name}(const {config_type} *cfg);')
                L.append('')
                if not is_readonly(p):
                    L.append('/**')
                    L.append(f' * @brief   设置{desc}')
                    L.append(' * @param   cfg 电机配置指针')
                    L.append(' * @param   value 源字符串(以\\0结尾)')
                    L.append(' * @return  0=成功, -EINVAL=参数错误')
                    L.append(' */')
                    L.append(f'int {module_name}_set_{name}({config_type} *cfg, const char *value);')
                    L.append('')
                continue

            # 标量 get
            L.append('/**')
            L.append(f' * @brief   获取{desc}')
            L.append(' * @param   cfg 电机配置指针')
            L.append(f' * @return  {desc}')
            L.append(' */')
            L.append(f'{data_type} {module_name}_get_{name}(const {config_type} *cfg);')
            L.append('')

            # 标量 set（只读跳过）
            if not is_readonly(p):
                L.append('/**')
                L.append(f' * @brief   设置{desc}')
                L.append(' * @param   cfg 电机配置指针')
                L.append(' * @param   value 要设置的值')
                L.append(' * @return  0=成功, -EINVAL=参数错误')
                L.append(' */')
                L.append(f'int {module_name}_set_{name}({config_type} *cfg, {data_type} value);')
                L.append('')

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
def generate_c(params, groups, output_c, output_h, module_name, config_type):
    L = []

    L += file_banner(os.path.basename(output_c), '关节电机配置参数API实现')
    L.append('')
    L.append(f'#include "{os.path.basename(output_h)}"')
    L.append('#include <stdio.h>')
    L.append('#include <string.h>')
    L.append('#include <errno.h>')
    L.append('')

    # 默认值常量
    L.append('/* 默认值常量 */')
    L.append(f'static const {config_type} g_default_config =')
    L.append('{')
    for group_name, gp in groups.items():
        _, member_name = GROUP_STRUCT_NAMES[group_name]
        L.append(f'    .{member_name} =')
        L.append('    {')
        for p in gp:
            data_type = p['data_type']
            default_val = p['default_value']
            array_size = int(float(p['array_size']))
            if data_type == 'char':
                val = '{0}'  # 数组/字符全部清零
            elif data_type == 'float':
                val = format_float(default_val) if default_val else '0.0f'
            else:
                val = str(int(float(default_val))) if default_val else '0'
            L.append(f'        .{p["param_name"]} = {val},')
        L.append('    },')
    L.append('};')
    L.append('')

    # init
    L.append(f'int {module_name}_init({config_type} *cfg)')
    L.append('{')
    L.append('    if (cfg == NULL) return -EINVAL;')
    L.append(f'    memcpy(cfg, &g_default_config, sizeof({config_type}));')
    L.append('    return 0;')
    L.append('}')
    L.append('')

    # validate（返回首个越界参数 id）
    L.append(f'int {module_name}_validate(const {config_type} *cfg)')
    L.append('{')
    L.append('    if (cfg == NULL) return -EINVAL;')
    L.append('')
    for group_name, gp in groups.items():
        _, member_name = GROUP_STRUCT_NAMES[group_name]
        for p in gp:
            array_size = int(float(p['array_size']))
            if array_size > 1:  # 数组（含字符串）不做范围校验
                continue
            lhs = f'cfg->{member_name}.{p["param_name"]}'
            cond = range_check_expr(p['data_type'], p['min_value'], p['max_value'], lhs)
            if cond is None:
                continue
            L.append(f'    if ({cond}) return {int(p["id"])};')
    L.append('')
    L.append('    return 0;')
    L.append('}')
    L.append('')

    # print
    L.append(f'void {module_name}_print(const {config_type} *cfg)')
    L.append('{')
    L.append('    if (cfg == NULL) return;')
    L.append('    printf("========== Motor Config ==========\\n");')
    for group_name, gp in groups.items():
        _, member_name = GROUP_STRUCT_NAMES[group_name]
        L.append('')
        L.append(f'    printf("\\n--- {GROUP_COMMENTS_EN[group_name]} ---\\n");')
        for p in gp:
            name = p['param_name']
            data_type = p['data_type']
            unit = p['unit']
            array_size = int(float(p['array_size']))
            member = f'cfg->{member_name}.{name}'
            unit_str = f' {unit}' if unit else ''

            if is_char_array(p, array_size):
                L.append(f'    printf("{name}: %s{unit_str}\\n", {member});')
            elif array_size > 1:
                continue  # 数值数组不打印
            else:
                fmt, cast = printf_fmt_and_cast(data_type)
                arg = f'{cast}{member}' if cast else member
                L.append(f'    printf("{name}: {fmt}{unit_str}\\n", {arg});')
    L.append('')
    L.append('    printf("\\n==================================\\n");')
    L.append('}')
    L.append('')

    # Get/Set 实现
    for group_name, gp in groups.items():
        _, member_name = GROUP_STRUCT_NAMES[group_name]
        for p in gp:
            name = p['param_name']
            data_type = p['data_type']
            array_size = int(float(p['array_size']))
            member = f'cfg->{member_name}.{name}'
            char_arr = is_char_array(p, array_size)

            if array_size > 1 and not char_arr:
                continue

            if char_arr:
                # 字符串 get
                L.append(f'const char *{module_name}_get_{name}(const {config_type} *cfg)')
                L.append('{')
                L.append(f'    return {member};')
                L.append('}')
                L.append('')
                # 字符串 set（strncpy + 强制 null 结尾）
                if not is_readonly(p):
                    L.append(f'int {module_name}_set_{name}({config_type} *cfg, const char *value)')
                    L.append('{')
                    L.append('    if (cfg == NULL || value == NULL) return -EINVAL;')
                    L.append(f'    strncpy({member}, value, sizeof({member}) - 1);')
                    L.append(f'    {member}[sizeof({member}) - 1] = \'\\0\';')
                    L.append('    return 0;')
                    L.append('}')
                    L.append('')
                continue

            # 标量 get
            L.append(f'{data_type} {module_name}_get_{name}(const {config_type} *cfg)')
            L.append('{')
            L.append(f'    return {member};')
            L.append('}')
            L.append('')

            # 标量 set（只读跳过）
            if not is_readonly(p):
                L.append(f'int {module_name}_set_{name}({config_type} *cfg, {data_type} value)')
                L.append('{')
                L.append('    if (cfg == NULL) return -EINVAL;')
                cond = range_check_expr(data_type, p['min_value'], p['max_value'], 'value')
                if cond is not None:
                    L.append('')
                    L.append(f'    if ({cond}) return -EINVAL;')
                L.append('')
                L.append(f'    {member} = value;')
                L.append('    return 0;')
                L.append('}')
                L.append('')

    with open(output_c, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L))
    print(f'✅ 生成实现文件: {output_c}')


# ----------------------------------------------------------------------------
# clang-format
# ----------------------------------------------------------------------------
def run_clang_format(files, clang_format_path=None):
    """按项目 .clang-format 就地格式化生成文件；找不到工具时优雅降级"""
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
# 主流程
# ----------------------------------------------------------------------------
def generate(csv_file, output_h, output_c, do_format=True, clang_format_path=None):
    params = load_csv(csv_file)
    groups = group_params(params)
    module_name = os.path.splitext(os.path.basename(output_h))[0]
    config_type = f'{module_name}_t'

    generate_h(params, groups, output_h, module_name, config_type)
    generate_c(params, groups, output_c, output_h, module_name, config_type)

    # 统计
    scalar = sum(1 for p in params if int(float(p['array_size'])) == 1)
    char_arr = sum(1 for p in params
                   if is_char_array(p, int(float(p['array_size']))))
    num_arr = sum(1 for p in params
                  if int(float(p['array_size'])) > 1
                  and p['data_type'] != 'char')
    readonly = sum(1 for p in params if is_readonly(p)
                   and not (int(float(p['array_size'])) > 1 and p['data_type'] != 'char'))
    accessible = scalar + char_arr
    set_count = accessible - readonly

    print('')
    print('📋 API统计:')
    print(f'   - 结构体类型 : {config_type}')
    print(f'   - 参数总数   : {len(params)}')
    print(f'   - 标量参数   : {scalar}  (字符串数组 {char_arr}, 数值数组 {num_arr} 已跳过访问接口)')
    print(f'   - 只读参数   : {readonly} (不生成 Set)')
    print(f'   - Get 函数   : {accessible}')
    print(f'   - Set 函数   : {set_count}')
    print(f'   - 基础函数   : 3 (init/validate/print)')
    print(f'   - 合计 API   : {accessible + set_count + 3}')

    if do_format:
        run_clang_format([output_h, output_c], clang_format_path)


def main(argv):
    csv_file = 'motor_param.csv'
    output_h = 'motor_param.h'
    output_c = 'motor_param.c'
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
