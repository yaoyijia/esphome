# LD2402 with Heart Rate Detection for ESPHome

包含心率检测功能的HLK-LD2402毫米波雷达ESPHome组件。

## 特性

1. **基础功能**：
   - 检测状态：无人(0)、有人移动(1)、有人静止(2)
   - 距离测量：厘米精度
   - 自动配置：上电自动设置为工程模式

2. **心率检测**：
   - 只在检测到"有人静止"状态时进行
   - 基于Goertzel算法的频率分析
   - 同时估算呼吸率
   - 距离范围限制：0.5-4米

3. **智能控制**：
   - 移动状态下不进行心率检测
   - 无人状态自动重置数据
   - 内置有效性检查

## 安装

```yaml
external_components:
  - source: github://your-username/ld2402_with_hr
