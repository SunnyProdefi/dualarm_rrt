import copy  # 导入copy模块，用于对象的深复制
from typing import Union, List, Tuple  # 从typing模块导入Union, List, Tuple，用于类型注解

# 定义RRTMap类
class RRTMap:

    # 初始化函数
    def __init__(self, area: Union[List, Tuple],  # area参数可以是List或Tuple类型，表示地图的区域
                 obstacles: Union[List, Tuple]) -> None:  # obstacles参数也可以是List或Tuple类型，表示地图中的障碍物
        super().__init__()  # 调用父类的初始化方法，虽然这里没有显式的父类
        self.__area = copy.deepcopy(area)  # 使用深复制来保证area的副本不会受到外部修改的影响
        self.__obstacles = copy.deepcopy(obstacles)  # 使用深复制来保证obstacles的副本不会受到外部修改的影响

    # area属性的获取器，使用@property装饰器
    @property
    def area(self) -> Union[List, Tuple]:  # 返回area的深复制，保护内部数据不被外部直接修改
        return copy.deepcopy(self.__area)

    # obstacles属性的获取器，使用@property装饰器
    @property
    def obstacles(self) -> Union[List, Tuple]:  # 返回obstacles的深复制，同样保护内部数据不被外部直接修改
        return copy.deepcopy(self.__obstacles)
