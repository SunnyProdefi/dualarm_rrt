from ..simplex import Support  # 从上级目录的simplex模块中导入Support类
from .GJK import GJK  # 从当前目录导入GJK模块


# 定义Collision类
class Collision:

    @staticmethod  # 使用@staticmethod装饰器，表明is_collision是一个静态方法
    def is_collision(shape0: Support, shape1: Support) -> bool:
        # 定义一个静态方法，接收两个几何形状作为参数，返回它们是否相交的布尔值
        return GJK.is_intersecting(
            shape0, shape1
        )  # 调用GJK模块的is_intersecting方法来检测两个形状是否相交，并返回检测结果
