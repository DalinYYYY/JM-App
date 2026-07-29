#!/usr/bin/env python3
"""DBC文件验证脚本"""
import cantools

dbc_path = r'd:\AAWorkSpace\001_JointMotor\SW\JointMotor\User\Protocol\docs\joint_motor_can.dbc'

try:
    db = cantools.database.load_file(dbc_path)
    print('=== cantools验证: OK ===')
    print(f'消息数: {len(db.messages)}')
    print(f'节点: {[n.name for n in db.nodes]}')

    # 检查扩展帧
    m = db.messages[7]  # OPEN_LOOP
    print(f'\n示例 {m.name}(0x{m.frame_id:X}): is_extended={m.is_extended_frame}')
    for s in m.signals:
        print(f'  信号 {s.name}: is_float={s.is_float}')

    # 检查HEARTBEAT
    m2 = db.get_message_by_name('HEARTBEAT')
    print(f'\nHEARTBEAT: DLC={m2.length}')
    for s in m2.signals:
        print(f'  信号 {s.name}: start={s.start} length={s.length}')

    # 检查NACK值表
    m3 = db.get_message_by_name('NACK')
    sig = m3.get_signal_by_name('err_code')
    print(f'\nNACK err_code 值表:')
    for v, lbl in sorted(sig.choices.items()):
        print(f'  {v}: {lbl}')

    # 检查MIT压缩
    m4 = db.get_message_by_name('MIT')
    print(f'\nMIT压缩: DLC={m4.length}')
    for s in m4.signals:
        print(f'  信号 {s.name}: start={s.start} length={s.length}')

    # 编解码测试
    print('\n=== 编解码测试 ===')
    # OPEN_LOOP: ud=1.0, uq=0.5
    m_ol = db.get_message_by_name('OPEN_LOOP')
    data = m_ol.encode({'ud': 1.0, 'uq': 0.5})
    decoded = m_ol.decode(data)
    print(f'OPEN_LOOP encode(ud=1.0, uq=0.5) = {data.hex()}')
    print(f'OPEN_LOOP decode = {decoded}')

    # NACK: cmd=0xE1, err_code=0x02
    data_nack = m3.encode({'cmd': 0xE1, 'err_code': 0x02})
    decoded_nack = m3.decode(data_nack)
    print(f'NACK encode(cmd=0xE1, err_code=0x02) = {data_nack.hex()}')
    print(f'NACK decode = {decoded_nack}')

    print('\n=== 全部验证通过 ===')

except Exception as e:
    print(f'验证失败: {e}')
    import traceback
    traceback.print_exc()
