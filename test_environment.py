#!/usr/bin/env python3
"""
环境测试脚本 - 验证项目依赖是否正确安装
使用方法: python test_environment.py
"""

import importlib
import os
import sys


def test_module(module_name, min_version=None):
    """测试模块是否可以导入"""
    try:
        module = importlib.import_module(module_name)
        if hasattr(module, '__version__'):
            version = module.__version__
            print(f"✅ {module_name}: {version}")
            if min_version and version < min_version:
                print(f"   ⚠️  警告: 版本过低，推荐 >= {min_version}")
        else:
            print(f"✅ {module_name}: 安装成功")
        return True
    except ImportError as e:
        print(f"❌ {module_name}: 导入失败 - {e}")
        return False

def test_file_exists(file_path, description):
    """测试文件是否存在"""
    if os.path.exists(file_path):
        print(f"✅ {description}: 存在")
        return True
    else:
        print(f"❌ {description}: 不存在 - {file_path}")
        return False

def main():
    print("=" * 50)
    print("柔顺对接仿真项目 - 环境测试")
    print("=" * 50)
    
    # 测试Python版本
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    print(f"Python版本: {python_version}")
    
    if sys.version_info < (3, 8):
        print("⚠️  警告: Python版本过低，推荐使用3.8+")
    else:
        print("✅ Python版本符合要求")
    
    print("\n测试核心依赖...")
    
    # 测试必需的Python模块
    required_modules = [
        ("numpy", "1.19.0"),
        ("scipy", "1.5.0"),
        ("matplotlib", "3.3.0"),
        ("pinocchio", None),
        ("mujoco", None),
    ]
    
    all_passed = True
    for module, min_version in required_modules:
        if not test_module(module, min_version):
            all_passed = False
    
    print("\n测试项目文件...")
    
    # 测试项目文件
    project_files = [
        ("main_simulation.py", "主仿真程序"),
        ("muj_class.py", "MuJoCo接口类"),
        ("Relate_class.py", "控制算法类"),
        ("log_class.py", "数据记录类"),
        ("requirement.txt", "依赖文件"),
    ]
    
    for file_path, description in project_files:
        if not test_file_exists(file_path, description):
            all_passed = False
    
    print("\n测试模型文件...")
    
    # 测试模型文件
    model_files = [
        ("kuka_xml_urdf/iiwa14_dock.xml", "MuJoCo模型"),
        ("kuka_xml_urdf/iiwa14_dock.urdf", "URDF模型"),
    ]
    
    for file_path, description in model_files:
        if not test_file_exists(file_path, description):
            all_passed = False
    
    print("\n测试环境变量...")
    
    # 测试环境变量
    mujoco_gl = os.environ.get("MUJOCO_GL", "未设置")
    display = os.environ.get("DISPLAY", "未设置")
    
    print(f"MUJOCO_GL: {mujoco_gl}")
    if display == "未设置":
        print("DISPLAY: 未设置 (无头环境)")
        if mujoco_gl not in ["egl", "osmesa"]:
            print("⚠️  警告: 无头环境建议设置 MUJOCO_GL=egl")
    else:
        print(f"DISPLAY: {display}")
    
    print("\n测试基本功能...")
    
    # 测试基本功能
    try:
        import numpy as np
        import pinocchio as pin
        
        # 测试基本数值计算
        a = np.array([1, 2, 3])
        b = np.array([4, 5, 6])
        c = np.dot(a, b)
        print(f"✅ NumPy基本运算: {c}")
        
        # 测试Pinocchio基本功能
        try:
            model = pin.buildModelFromUrdf("kuka_xml_urdf/iiwa14_dock.urdf")
            print(f"✅ Pinocchio URDF加载: {model.nq} 关节")
        except Exception as e:
            print(f"❌ Pinocchio URDF加载失败: {e}")
            all_passed = False
            
    except Exception as e:
        print(f"❌ 基本功能测试失败: {e}")
        all_passed = False
    
    print("\n" + "=" * 50)
    if all_passed:
        print("🎉 所有测试通过！环境配置正确。")
        print("\n可以运行以下命令启动仿真:")
        print("python main_simulation.py")
    else:
        print("❌ 部分测试失败，请检查环境配置。")
        print("\n请参考以下文档:")
        print("- 环境配置指南.md")
        print("- 快速启动指南.md")
        print("\n或运行自动配置脚本:")
        print("bash setup.sh")
    
    print("=" * 50)
    
    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())