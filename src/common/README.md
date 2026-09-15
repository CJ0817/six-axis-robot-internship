# 共享约定

[contract.json](contract.json) 保存单位、关节顺序和错误码；实现须从此读取或生成常量，避免各模块自行编号。
完整数据类型与校验规则见 [工程约定](../../docs/engineering-contract.md)。当前 JSON 是常量注册表，不是运行时校验器。
