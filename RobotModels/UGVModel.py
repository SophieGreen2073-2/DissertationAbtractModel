from DissertationAbtractModel.RobotModels.RobotModel import RobotModel
from AreaModel import AreaModel

class UGVModel(RobotModel):
    def __init__(self, x, y, robot_id, area: AreaModel, DisplayGrid):
        print("New UGV")
        RobotModel.__init__(self, x, y, robot_id, AreaModel, DisplayGrid)

        self.type = "UGV"