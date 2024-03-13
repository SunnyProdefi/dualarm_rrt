import abc  # 导入abc模块，用于定义抽象基类
import multiprocessing  # 导入multiprocessing模块，支持程序进行多进程处理

from src.geometry import LineSegment  # 从src.geometry模块导入LineSegment类

# 定义ICheckCollision抽象基类
class ICheckCollision(abc.ABC):

    @abc.abstractmethod  # 使用@abstractmethod装饰器，标记下面的方法为抽象方法
    def check_collision(self, line_segment: LineSegment, pool: multiprocessing.Pool = None):
        # 定义一个抽象方法check_collision，该方法需要在子类中被实现
        # 方法接收一个LineSegment对象和可选的multiprocessing.Pool对象作为参数
        pass  # 抽象方法不提供实现（即方法体为空）

