import numpy as np

from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.types import ArticulationAction

class SO101PickController(BaseController):
   def __init__(self) -> None:
        super().__init__()