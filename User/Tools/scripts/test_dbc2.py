#!/usr/bin/env python3
"""用cantools API生成DBC，查看正确的SIG_VALTYPE_格式"""
import cantools
from cantools.database.can import Message, Signal
import inspect

# 检查Signal构造函数签名
print('Signal.__init__ 签名:', inspect.signature(Signal.__init__))

# 创建一个包含float信号的数据库
db = cantools.database.Database()

sig = Signal(name='sig', start=0, length=32, byte_order='little_endian')
sig.is_float = True

msg = Message(
    frame_id=1,
    name='Test',
    length=8,
    signals=[sig],
    senders=['Host'],
)
db.messages.append(msg)

# 导出为DBC字符串
dbc_str = db.as_dbc_string()
print('=== cantools生成的DBC ===')
print(dbc_str)
print('=== END ===')

# 检查SIG_VALTYPE_行
for line in dbc_str.split('\n'):
    if 'SIG_VALTYPE' in line:
        print(f'SIG_VALTYPE_行: {repr(line)}')
