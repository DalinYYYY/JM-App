#!/usr/bin/env python3
"""DBC格式验证测试"""
import cantools
import sys

# 测试1: 最简DBC含SIG_VALTYPE_
dbc1 = '''VERSION ""
NS_ :
BS_:
BU_: Host Motor
BO_ 1 Test: 8 Host
 SG_ sig : 0|32@1+ (1,0) [0|4294967295] "" Motor
SIG_VALTYPE_ 1 sig 1;
'''
try:
    db = cantools.database.load_string(dbc1)
    print('Test1 SIG_VALTYPE_: OK')
except Exception as e:
    print('Test1 FAIL:', e)

# 测试2: 含VFrameFormat
dbc2 = '''VERSION ""
NS_ :
BS_:
BU_: Host Motor
BA_DEF_ BO_ "VFrameFormat" ENUM "Standard","Extended";
BO_ 1 Test: 8 Host
 SG_ sig : 0|32@1+ (1,0) [0|4294967295] "" Motor
BA_ "VFrameFormat" BO_ 1 1;
SIG_VALTYPE_ 1 sig 1;
'''
try:
    db = cantools.database.load_string(dbc2)
    print('Test2 VFrameFormat: OK')
except Exception as e:
    print('Test2 FAIL:', e)

# 测试3: 含中文注释
dbc3 = '''VERSION ""
NS_ :
BS_:
BU_: Host Motor
BO_ 1 Test: 8 Host
 SG_ sig : 0|32@1+ (1,0) [0|4294967295] "" Motor
CM_ BO_ 1 "中文注释";
SIG_VALTYPE_ 1 sig 1;
'''
try:
    db = cantools.database.load_string(dbc3)
    print('Test3 Chinese comment: OK')
except Exception as e:
    print('Test3 FAIL:', e)

# 测试4: 含DLC=0消息
dbc4 = '''VERSION ""
NS_ :
BS_:
BU_: Host Motor
BO_ 1 Test: 0 Host
SIG_VALTYPE_ 1 sig 1;
'''
try:
    db = cantools.database.load_string(dbc4)
    print('Test4 DLC=0: OK')
except Exception as e:
    print('Test4 FAIL:', e)

# 测试5: BA_DEF_后无BA_DEF_DEF_
dbc5 = '''VERSION ""
NS_ :
BS_:
BU_: Host Motor
BA_DEF_ BO_ "VFrameFormat" ENUM "Standard","Extended";
BA_DEF_DEF_ "VFrameFormat" "Standard";
BO_ 1 Test: 8 Host
 SG_ sig : 0|32@1+ (1,0) [0|4294967295] "" Motor
BA_ "VFrameFormat" BO_ 1 1;
SIG_VALTYPE_ 1 sig 1;
'''
try:
    db = cantools.database.load_string(dbc5)
    print('Test5 BA_DEF_DEF_: OK')
except Exception as e:
    print('Test5 FAIL:', e)

# 测试6: 实际DBC文件
print('\n--- 测试实际DBC文件 ---')
try:
    db = cantools.database.load_file(r'd:\AAWorkSpace\001_JointMotor\SW\JointMotor\User\Protocol\docs\joint_motor_can.dbc')
    print(f'实际DBC: OK, 消息数={len(db.messages)}')
except Exception as e:
    print(f'实际DBC FAIL: {e}')
