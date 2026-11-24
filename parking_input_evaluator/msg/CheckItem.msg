# 单项检查结果
string id                # "A1"、"A3"、"F4" 等
string title             # 人类可读标题，如 "停车场覆盖率"

# 级别常量
uint8 UNKNOWN=0
uint8 PASS=1
uint8 WARN=2
uint8 FAIL=3
uint8 level

float64 value            # 实测值
float64 threshold_pass   # PASS 阈值，不可比时填 NaN
float64 threshold_warn   # WARN 阈值，不可比时填 NaN
string unit              # "m"、"Hz"、"ratio"，无单位时为空

string detail            # 完整实测描述，便于直接打印
string advice            # 修复建议，PASS 时为空
