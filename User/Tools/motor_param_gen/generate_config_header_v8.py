#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
关节电机配置CSV转C语言头文件脚本 V8
- 头文件结构体参数加中文注释和单位
- 修复所有编译警告
"""

import csv
import os
import sys
from datetime import datetime

# Group 结构体名称映射
GROUP_STRUCT_NAMES = {
    'MotorInstance':      ('motor_instance_t', 'motor_instance'),
    'MotorConfig':        ('motor_base_t', 'motor_base'),
    'GearboxConfig':      ('gearbox_param_t', 'gearbox_param'),
    'EncoderConfig':      ('encoder_param_t', 'encoder_param'),
    'PositionConfig':     ('position_limit_t', 'position_limit'),
    'HomingConfig':       ('homing_param_t', 'homing_param'),
    'CurrentLoop':        ('current_loop_t', 'current_loop'),
    'PositionLoop':       ('position_loop_t', 'position_loop'),
    'ImpedanceControl':   ('impedance_ctrl_t', 'impedance_ctrl'),
    'ThermalConfig':      ('thermal_model_t', 'thermal_model'),
    'Protection':         ('protection_param_t', 'protection_param'),
}

GROUP_COMMENTS = {
    'MotorInstance':      '电机实例标识',
    'MotorConfig':        '电机本体参数',
    'GearboxConfig':      '减速器参数',
    'EncoderConfig':      '编码器参数',
    'PositionConfig':     '位置限位配置',
    'HomingConfig':       '回零配置参数',
    'CurrentLoop':        '电流环控制参数',
    'PositionLoop':       '位置速度环控制参数',
    'ImpedanceControl':   '阻抗控制参数',
    'ThermalConfig':      '热模型参数',
    'Protection':         '保护参数配置',
}

GROUP_COMMENTS_EN = {
    'MotorInstance':      'Motor Instance',
    'MotorConfig':        'Motor Base Parameters',
    'GearboxConfig':      'Gearbox Parameters',
    'EncoderConfig':      'Encoder Parameters',
    'PositionConfig':     'Position Limit Config',
    'HomingConfig':       'Homing Config',
    'CurrentLoop':        'Current Loop Control',
    'PositionLoop':       'Position/Velocity Loop',
    'ImpedanceControl':   'Impedance Control',
    'ThermalConfig':      'Thermal Model',
    'Protection':         'Protection Config',
}

def format_float(val_str):
    """格式化浮点数，确保有小数点"""
    num_val = float(val_str)
    if num_val.is_integer():
        return f'{int(num_val)}.0f'
    return f'{val_str}f'

def generate_c_header(csv_file, output_h='motor_param.h', output_c='motor_param.c'):
    """读取CSV并生成完整API"""
    
    params = []
    with open(csv_file, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for row in reader:
            params.append(row)
    
    groups = {}
    for p in params:
        group = p['group']
        if group not in groups:
            groups[group] = []
        groups[group].append(p)
    
    module_name = os.path.splitext(os.path.basename(output_h))[0]
    config_type = f'{module_name}_t'
    
    # ========== 生成头文件 ==========
    lines_h = []
    guard = f'__{module_name.upper()}_H__'
    
    lines_h.append('/**')
    lines_h.append(f' * @file    {os.path.basename(output_h)}')
    lines_h.append(' * @brief   关节电机配置参数API接口')
    lines_h.append(f' * @date    {datetime.now().strftime("%Y-%m-%d")}')
    lines_h.append(' *')
    lines_h.append(' * @warning 【自动生成文件，请勿手动修改】')
    lines_h.append(f' *          本文件由脚本 {os.path.basename(__file__)} 根据配置表自动生成，')
    lines_h.append(' *          任何手动改动都会在下次运行脚本时被覆盖。')
    lines_h.append(' *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。')
    lines_h.append(' */')
    lines_h.append('')
    lines_h.append(f'#ifndef {guard}')
    lines_h.append(f'#define {guard}')
    lines_h.append('')
    lines_h.append('#ifdef __cplusplus')
    lines_h.append('extern "C" {')
    lines_h.append('#endif')
    lines_h.append('')
    lines_h.append('#include <stdint.h>')
    lines_h.append('#include <stdbool.h>')
    lines_h.append('')
    
    lines_h.append('/* 配置结构体前向声明 */')
    lines_h.append(f'typedef struct {module_name} {config_type};')
    lines_h.append('')
    
    # 生成每个group的子结构体 - 带中文注释和单位
    for group_name, group_params in groups.items():
        struct_name, _ = GROUP_STRUCT_NAMES[group_name]
        lines_h.append('/**')
        lines_h.append(f' * @brief   {GROUP_COMMENTS[group_name]}')
        lines_h.append(' */')
        lines_h.append(f'typedef struct {{')
        
        for p in group_params:
            param_name = p['param_name']
            data_type = p['data_type']
            array_size = int(float(p['array_size']))
            desc = p['description']
            unit = p['unit']
            
            comment = desc
            if unit:
                comment += f' ({unit})'
            
            if array_size > 1:
                lines_h.append(f'    {data_type:<8s} {param_name}[{array_size}];    /* {comment} */')
            else:
                lines_h.append(f'    {data_type:<8s} {param_name};    /* {comment} */')
        
        lines_h.append(f'}} {struct_name};')
        lines_h.append('')
    
    # 主结构体
    lines_h.append('/**')
    lines_h.append(' * @brief   关节电机完整配置结构体')
    lines_h.append(' */')
    lines_h.append(f'struct {module_name} {{')
    for group_name in groups.keys():
        struct_name, member_name = GROUP_STRUCT_NAMES[group_name]
        lines_h.append(f'    {struct_name:<20s} {member_name};    /* {GROUP_COMMENTS[group_name]} */')
    lines_h.append('};')
    lines_h.append('')
    
    # API函数声明
    lines_h.append('/******************************************************************************')
    lines_h.append(' * @brief   基础API接口')
    lines_h.append(' ******************************************************************************/')
    lines_h.append('')
    lines_h.append('/**')
    lines_h.append(' * @brief   初始化电机配置为默认值')
    lines_h.append(f' * @param   cfg 电机配置指针')
    lines_h.append(' * @return  0=成功, -EINVAL=参数错误')
    lines_h.append(' */')
    lines_h.append(f'int {module_name}_init({config_type} *cfg);')
    lines_h.append('')
    
    lines_h.append('/**')
    lines_h.append(' * @brief   校验电机配置参数范围')
    lines_h.append(f' * @param   cfg 电机配置指针')
    lines_h.append(' * @return  0=成功, 其他=错误码')
    lines_h.append(' */')
    lines_h.append(f'int {module_name}_validate(const {config_type} *cfg);')
    lines_h.append('')
    
    lines_h.append('/**')
    lines_h.append(' * @brief   打印电机配置所有参数')
    lines_h.append(f' * @param   cfg 电机配置指针')
    lines_h.append(' */')
    lines_h.append(f'void {module_name}_print(const {config_type} *cfg);')
    lines_h.append('')
    
    # 每个参数的Get/Set函数 - 带中文注释
    for group_name, group_params in groups.items():
        _, member_name = GROUP_STRUCT_NAMES[group_name]
        lines_h.append(f'/******************************************************************************')
        lines_h.append(f' * @brief   {GROUP_COMMENTS[group_name]}')
        lines_h.append(' ******************************************************************************/')
        lines_h.append('')
        
        for p in group_params:
            param_name = p['param_name']
            data_type = p['data_type']
            desc = p['description']
            array_size = int(float(p['array_size']))
            
            if array_size > 1:
                continue
            
            # Get函数
            lines_h.append('/**')
            lines_h.append(f' * @brief   获取{desc}')
            lines_h.append(f' * @param   cfg 电机配置指针')
            lines_h.append(f' * @return  {desc}')
            lines_h.append(' */')
            lines_h.append(f'{data_type} {module_name}_get_{param_name}(const {config_type} *cfg);')
            lines_h.append('')
            
            # Set函数
            lines_h.append('/**')
            lines_h.append(f' * @brief   设置{desc}')
            lines_h.append(f' * @param   cfg 电机配置指针')
            lines_h.append(f' * @param   value 要设置的值')
            lines_h.append(' * @return  0=成功, -EINVAL=参数错误')
            lines_h.append(' */')
            lines_h.append(f'int {module_name}_set_{param_name}({config_type} *cfg, {data_type} value);')
            lines_h.append('')
    
    lines_h.append('#ifdef __cplusplus')
    lines_h.append('}')
    lines_h.append('#endif')
    lines_h.append('')
    lines_h.append(f'#endif /* {guard} */')
    lines_h.append('')
    
    with open(output_h, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines_h))
    
    print(f'✅ 生成头文件: {output_h}')
    
    generate_c_file(params, groups, output_c, output_h, module_name, config_type)

def generate_c_file(params, groups, output_file, output_h, module_name, config_type):
    """生成C实现文件 - 修复所有编译警告"""
    lines_c = []

    lines_c.append('/**')
    lines_c.append(f' * @file    {os.path.basename(output_file)}')
    lines_c.append(' * @brief   关节电机配置参数API实现')
    lines_c.append(f' * @date    {datetime.now().strftime("%Y-%m-%d")}')
    lines_c.append(' *')
    lines_c.append(' * @warning 【自动生成文件，请勿手动修改】')
    lines_c.append(f' *          本文件由脚本 {os.path.basename(__file__)} 根据配置表自动生成，')
    lines_c.append(' *          任何手动改动都会在下次运行脚本时被覆盖。')
    lines_c.append(' *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。')
    lines_c.append(' */')
    lines_c.append('')
    lines_c.append(f'#include "{os.path.basename(output_h)}"')
    lines_c.append('#include <stdio.h>')
    lines_c.append('#include <string.h>')
    lines_c.append('#include <errno.h>')
    lines_c.append('')
    
    # 默认值定义
    lines_c.append('/* 默认值常量 */')
    lines_c.append(f'static const {config_type} g_default_config = {{')
    for group_name, group_params in groups.items():
        struct_name, member_name = GROUP_STRUCT_NAMES[group_name]
        lines_c.append(f'    .{member_name} = {{')
        
        for p in group_params:
            data_type = p['data_type']
            default_val = p['default_value']
            array_size = int(float(p['array_size']))
            
            if data_type == 'float':
                if default_val:
                    val = format_float(default_val)
                else:
                    val = '0.0f'
            elif data_type == 'char':
                val = '{0}'
            else:
                if default_val:
                    val = str(int(float(default_val)))
                else:
                    val = '0'
            
            lines_c.append(f'        .{p["param_name"]} = {val},')
        
        lines_c.append(f'    }},')
    lines_c.append('};')
    lines_c.append('')
    
    # 1. 初始化函数
    lines_c.append(f'int {module_name}_init({config_type} *cfg)')
    lines_c.append('{')
    lines_c.append('    if (cfg == NULL) return -EINVAL;')
    lines_c.append(f'    memcpy(cfg, &g_default_config, sizeof({config_type}));')
    lines_c.append('    return 0;')
    lines_c.append('}')
    lines_c.append('')
    
    # 2. 校验函数 - 修复无符号比较警告
    lines_c.append(f'int {module_name}_validate(const {config_type} *cfg)')
    lines_c.append('{')
    lines_c.append('    if (cfg == NULL) return -EINVAL;')
    lines_c.append('')
    
    err_code = 1
    for group_name, group_params in groups.items():
        _, member_name = GROUP_STRUCT_NAMES[group_name]
        
        for p in group_params:
            param_name = p['param_name']
            data_type = p['data_type']
            min_val = p['min_value']
            max_val = p['max_value']
            array_size = int(float(p['array_size']))
            
            if array_size > 1:
                continue
            if not min_val or not max_val:
                continue
            
            min_f = float(min_val)
            max_f = float(max_val)
            if min_f == 0 and max_f == 0:
                continue
            
            # 无符号类型且最小值为0时，跳过最小值检查
            is_unsigned = data_type.startswith('uint')
            skip_min = is_unsigned and min_f == 0
            
            if data_type == 'float':
                min_str = format_float(min_val)
                max_str = format_float(max_val)
                if skip_min:
                    lines_c.append(f'    if (cfg->{member_name}.{param_name} > {max_str}) return {err_code};')
                else:
                    lines_c.append(f'    if (cfg->{member_name}.{param_name} < {min_str} || cfg->{member_name}.{param_name} > {max_str}) return {err_code};')
            elif data_type == 'int64_t':
                min_i = int(min_f)
                max_i = int(max_f)
                if skip_min:
                    lines_c.append(f'    if (cfg->{member_name}.{param_name} > {max_i}LL) return {err_code};')
                else:
                    lines_c.append(f'    if (cfg->{member_name}.{param_name} < {min_i}LL || cfg->{member_name}.{param_name} > {max_i}LL) return {err_code};')
            else:
                min_i = int(min_f)
                max_i = int(max_f)
                if skip_min:
                    lines_c.append(f'    if (cfg->{member_name}.{param_name} > ({data_type}){max_i}) return {err_code};')
                else:
                    lines_c.append(f'    if (cfg->{member_name}.{param_name} < ({data_type}){min_i} || cfg->{member_name}.{param_name} > ({data_type}){max_i}) return {err_code};')
            
            err_code += 1
    
    lines_c.append('')
    lines_c.append('    return 0;')
    lines_c.append('}')
    lines_c.append('')
    
    # 3. 打印函数 - 使用英文避免编码问题，修复格式
    lines_c.append(f'void {module_name}_print(const {config_type} *cfg)')
    lines_c.append('{')
    lines_c.append('    if (cfg == NULL) return;')
    lines_c.append('    printf("========== Motor Config ==========\\n");')
    
    for group_name, group_params in groups.items():
        _, member_name = GROUP_STRUCT_NAMES[group_name]
        lines_c.append('')
        lines_c.append(f'    printf("\\n--- {GROUP_COMMENTS_EN[group_name]} ---\\n");')
        
        for p in group_params:
            param_name = p['param_name']
            data_type = p['data_type']
            unit = p['unit']
            array_size = int(float(p['array_size']))
            
            if array_size > 1:
                continue
            
            if data_type == 'float':
                fmt = '%f'
            elif data_type == 'int64_t':
                fmt = '%lld'  # int64_t 用 %lld
            else:
                fmt = '%d'
            
            unit_str = f' {unit}' if unit else ''
            
            lines_c.append(f'    printf("{param_name}: {fmt}{unit_str}\\n", cfg->{member_name}.{param_name});')
    
    lines_c.append('')
    lines_c.append('    printf("\\n==================================\\n");')
    lines_c.append('}')
    lines_c.append('')
    
    # 4. 每个参数的Get/Set实现
    for group_name, group_params in groups.items():
        _, member_name = GROUP_STRUCT_NAMES[group_name]
        
        for p in group_params:
            param_name = p['param_name']
            data_type = p['data_type']
            min_val = p['min_value']
            max_val = p['max_value']
            array_size = int(float(p['array_size']))
            
            if array_size > 1:
                continue
            
            # Get函数
            lines_c.append(f'{data_type} {module_name}_get_{param_name}(const {config_type} *cfg)')
            lines_c.append('{')
            lines_c.append(f'    return cfg->{member_name}.{param_name};')
            lines_c.append('}')
            lines_c.append('')
            
            # Set函数 - 修复无符号比较警告
            lines_c.append(f'int {module_name}_set_{param_name}({config_type} *cfg, {data_type} value)')
            lines_c.append('{')
            lines_c.append('    if (cfg == NULL) return -EINVAL;')
            lines_c.append('')
            
            if min_val and max_val and not (float(min_val) == 0 and float(max_val) == 0):
                min_f = float(min_val)
                max_f = float(max_val)
                
                is_unsigned = data_type.startswith('uint')
                skip_min = is_unsigned and min_f == 0
                
                if data_type == 'float':
                    min_str = format_float(min_val)
                    max_str = format_float(max_val)
                    if skip_min:
                        lines_c.append(f'    if (value > {max_str}) return -EINVAL;')
                    else:
                        lines_c.append(f'    if (value < {min_str} || value > {max_str}) return -EINVAL;')
                elif data_type == 'int64_t':
                    min_i = int(min_f)
                    max_i = int(max_f)
                    if skip_min:
                        lines_c.append(f'    if (value > {max_i}LL) return -EINVAL;')
                    else:
                        lines_c.append(f'    if (value < {min_i}LL || value > {max_i}LL) return -EINVAL;')
                else:
                    min_i = int(min_f)
                    max_i = int(max_f)
                    if skip_min:
                        lines_c.append(f'    if (value > ({data_type}){max_i}) return -EINVAL;')
                    else:
                        lines_c.append(f'    if (value < ({data_type}){min_i} || value > ({data_type}){max_i}) return -EINVAL;')
                lines_c.append('')
            
            lines_c.append(f'    cfg->{member_name}.{param_name} = value;')
            lines_c.append('    return 0;')
            lines_c.append('}')
            lines_c.append('')
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines_c))
    
    print(f'✅ 生成实现文件: {output_file}')
    print(f'')
    print(f'📋 API统计:')
    total_getset = sum(1 for p in params if int(float(p['array_size'])) == 1)
    print(f'   - 结构体类型: {config_type}')
    print(f'   - 初始化函数: 1')
    print(f'   - 校验函数: 1')
    print(f'   - 打印函数: 1')
    print(f'   - Get函数: {total_getset}')
    print(f'   - Set函数: {total_getset}')
    print(f'   - 总计: {total_getset * 2 + 3} 个API函数')

if __name__ == '__main__':
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'motor_param.csv'
    output_h = sys.argv[2] if len(sys.argv) > 2 else 'motor_param.h'
    output_c = sys.argv[3] if len(sys.argv) > 3 else 'motor_param.c'
    generate_c_header(csv_file, output_h, output_c)
