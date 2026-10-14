# tf-route-audit

离线检查 TF 拓扑和观测时间范围，支持原生 URDF 与导出的 ROS TransformStamped JSON。它不替代实时 tf2，不模拟缓存或插值；活动 URDF 关节没有动态时间证据。

由 **Afloat16** 独立维护，MIT 许可。实验性工具，格式与限制详见 [English README](README.md) 和[需求证据](docs/EVIDENCE.md)。未声称修复上游运行时问题或获得上游认可。

## 安装与首个示例

Python 3.10+。克隆仓库、进入目录，创建并激活虚拟环境后：

```bash
python -m pip install -e '.[dev]'
tf-route-audit examples/transforms.json --query map base_link --at-ns 150
```

基础离线路径使用标准库，无需 GPU、ROS 运行环境或模型权重。输入保持只读，输出必须是新文件，不覆盖已有文件。审计退出码：0 无错误，1 有错误（strict 也把警告当作失败），2 输入或写入失败。

## 验证

```bash
python -m unittest discover -s tests -v
python -m compileall -q src tests
python -m build
```

CI 配置覆盖三种系统的 Python 3.10 / 3.13，实际状态以 Actions 为准。示例均为合成数据，不包含用户附件或私人数据。
