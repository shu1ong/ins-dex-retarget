def parse_angle_data(data):
    if len(data) < 19:  # 7（头部信息） + 12（6个short值） = 19
        return "数据长度不足"

    result = {
        "包头": f"0x{data[0]:02X}{data[1]:02X}",
        "灵巧手ID": data[2],
        "寄存器长度": data[3] - 3,
        "命令回复": f"0x{data[4]:02X}",
        "寄存器起始地址": (data[6] << 8) | data[5],
        "手指角度": {}
    }

    finger_names = ["小拇指", "无名指", "中指", "食指", "大拇指弯曲", "大拇指旋转"]
    
    for i, name in enumerate(finger_names):
        angle = int.from_bytes(data[7+i*2:9+i*2], byteorder='little', signed=True)
        # 确保角度在0-1000范围内
        angle = max(0, min(1000, angle))
        result["手指角度"][name] = angle

    result["校验和"] = data[19]  # 假设校验和在所有数据之后

    return result

# 使用示例
# 注意：这里的数据是模拟的，您需要替换为实际的串口数据
data = b'\x90\xEB\x01\x0F\x11\x0A\x06\xE8\x03\xE6\x03\xE8\x03\xE3\x03\xE0\x03\xD3\x03\x8F'
parsed_data = parse_angle_data(data)
print(parsed_data)