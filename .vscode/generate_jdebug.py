#!/usr/bin/env python3
import sys
import os
from datetime import datetime

def generate_jdebug(project_name, elf_path, device_name):
    # 计算关键路径
    project_dir = os.path.dirname(os.path.dirname(elf_path))
    debug_dir = os.path.join(project_dir, "debug")
    jdebug_path = os.path.join(debug_dir, f"{project_name}.jdebug")
    
    # 创建debug目录
    os.makedirs(debug_dir, exist_ok=True)

    # 设备配置映射（扩展支持更多型号）
    device_config = {
        "STM32F1": {
            "cpu_svd": "Cortex-M3.svd",
            "device_svd": "STM32F1xx.svd"
        },
        "STM32F4": {
            "cpu_svd": "Cortex-M4.svd",
            "device_svd": "STM32F4xx.svd"
        },
        "STM32H7": {
            "cpu_svd": "Cortex-M7.svd",
            "device_svd": "STM32H7xx.svd"
        }
    }

    # 自动检测设备系列
    device_family = device_name[:7]  # 提取STM32F4中的F4部分
    if device_family not in device_config:
        device_family = device_name[:8]  # 尝试STM32F40
        if device_family not in device_config:
            device_family = "STM32F4"  # 默认回退

    config = device_config.get(device_family, device_config["STM32F4"])

    # 生成文件内容
    header = f"""/*********************************************************************
*                 (c) SEGGER Microcontroller GmbH                    *
*                      The Embedded Experts                          *
*                         www.segger.com                             *
**********************************************************************

File          : {jdebug_path}
Created       : {datetime.now().strftime('%d %b %Y %H:%M')}
Ozone Version : V3.38g
*/"""

    content = f"""
void OnProjectLoad (void) {{
  // 路径替换
  Project.AddPathSubstitute("{project_dir}", "$(ProjectDir)");
  
  // 设备配置
  Project.SetDevice("{device_name}");
  Project.SetHostIF("USB", "");
  Project.SetTargetIF("SWD");
  Project.SetTIFSpeed("4 MHz");
  Project.AddSvdFile("$(InstallDir)/Config/CPU/{config['cpu_svd']}");
  Project.AddSvdFile("$(InstallDir)/Config/Peripherals/{config['device_svd']}");
  
  // 加载ELF
  File.Open("{os.path.normpath(elf_path)}");
}}

// 标准事件处理
void AfterTargetReset (void) {{
  _SetupTarget();
}}

void AfterTargetDownload (void) {{
  _SetupTarget();
}}

void _SetupTarget(void) {{
  unsigned int SP, PC, VectorTableAddr = Elf.GetBaseAddr();
  
  if (VectorTableAddr != 0xFFFFFFFF) {{
    SP = Target.ReadU32(VectorTableAddr);
    Target.SetReg("SP", (SP != 0xFFFFFFFF) ? SP : 0x20000000);
    
    PC = Elf.GetEntryPointPC();
    Target.SetReg("PC", (PC != 0xFFFFFFFF) ? PC : VectorTableAddr + 4);
  }}
}}
"""

    # 写入文件
    with open(jdebug_path, "w") as f:
        f.write(header + content)
    
    print(f"Successfully generated: {jdebug_path}")
    print(f"Device configuration: {device_name} -> {device_family}")
    print(f"Using SVDs: CPU={config['cpu_svd']}, Device={config['device_svd']}")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python3 generate_jdebug.py <project_name> <elf_path> <device_name>")
        print("Example: python3 generate_jdebug.py test02 build/test02.elf STM32F407VE")
        sys.exit(1)

    try:
        generate_jdebug(
            project_name=sys.argv[1],
            elf_path=sys.argv[2],
            device_name=sys.argv[3]
        )
    except Exception as e:
        print(f"Error: {str(e)}")
        print("Common fixes:")
        print("1. Check device name format (e.g. STM32F407VE)")
        print("2. Verify ELF file exists at specified path")
        sys.exit(1)