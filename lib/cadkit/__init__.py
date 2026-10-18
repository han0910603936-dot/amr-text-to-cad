"""cadkit：這個專案共用的建模小工具。

    from cadkit import save, PROFILE
    from cadkit import fasteners as fx
"""

from cadkit.export import PROJECT_ROOT, save
from cadkit.printer import PROFILE

__all__ = ["PROFILE", "PROJECT_ROOT", "save"]
