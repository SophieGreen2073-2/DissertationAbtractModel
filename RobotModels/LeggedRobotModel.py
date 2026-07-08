from DissertationAbtractModel.RobotModels.RobotModel import RobotModel
from AreaModel import AreaModel

class LeggedRobotModel(RobotModel):
    def __init__(self, x, y, area: AreaModel, robot_id, DisplayGrid):
        print("New Legged Robot")
        RobotModel.__init__(self, x, y, robot_id, area, DisplayGrid)

        self.type = "Legged"