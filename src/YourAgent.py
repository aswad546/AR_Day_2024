from src.DriveInterface import DriveInterface
from src.DriveState import DriveState
from src.Constants import DriveMove, SensorData
from src.Utils import manhattan_dist_2D

class YourAgent(DriveInterface):

    def __init__(self, game_id: int, is_advanced_mode: bool):
        """
        Constructor for YourAgent

        Arguments:
        game_id -- a unique value passed to the player drive, you do not have to do anything with it, but will have access.
        is_advanced_mode -- boolean to indicate if the game is in advanced mode or not.
        """
        self.game_id = game_id
        self.need_to_find_target_pod = is_advanced_mode
        self.picked_up_pode = False
    # This is the main function the simulator will call each turn
    def get_next_move(self, sensor_data: dict) -> DriveMove:
        """
        Main function for YourAgent. The simulator will call this function each loop of the simulation to see what your agent's
        next move would be. You will have access to data about the field, your robot's location, other robots' locations and more
        in the sensor_data dict argument.

        Arguments:
        sensor_data -- a dict with state information about other objects in the game. The structure of sensor_data is shown below:
            sensor_data = {
                SensorData.FIELD_BOUNDARIES: [[-1, -1], [-1, 0], ...],
                SensorData.DRIVE_LOCATIONS: [[x1, y1], [x2, y2], ...],
                SensorData.POD_LOCATIONS: [[x1, y1], [x2, y2], ...],
                SensorData.PLAYER_LOCATION: [x, y],
                SensorData.GOAL_LOCATIONS: [[x1, y1], [x2, y2], ...],  # List of goal locations
                SensorData.GOAL_LOCATION: [x, y],  # Kept for compatibility
                SensorData.TARGET_POD_LOCATION: [x, y],  # Only used for Advanced mode
                SensorData.DRIVE_LIFTED_POD_PAIRS: [[drive_id_1, pod_id_1], [drive_id_2, pod_id_2], ...]  # Only used in Advanced mode for seeing which pods are currently lifted by drives
            }

        Returns:
        DriveMove - return value must be one of the enum values in the DriveMove class:
            DriveMove.NONE – Do nothing
            DriveMove.UP – Move 1 tile up (positive y direction)
            DriveMove.DOWN – Move 1 tile down (negative y direction)
            DriveMove.RIGHT – Move 1 tile right (positive x direction)
            DriveMove.LEFT – Move 1 tile left (negative x direction)

            (Advanced mode only)
            DriveMove.LIFT_POD – If a pod is in the same tile, pick it up. The pod will now move with the drive until it is dropped
            DriveMove.DROP_POD – If a pod is in the same tile, drop it. The pod will now stay in this position until it is picked up
        """

        if self.picked_up_pode == False and self.need_to_find_target_pod:
            currentLocation = sensor_data[SensorData.PLAYER_LOCATION]
            x, y = currentLocation
            if [x, y] == sensor_data[SensorData.TARGET_POD_LOCATION]:
                self.picked_up_pode = True
                return DriveMove.LIFT_POD

            stack = [(x, y, [(x, y)])]
            visited = set()
            visited.add((x, y))
            targetx, targety = -1, -1
            while stack:
                size = len(stack)
                temp = []
                for i in range(size):
                    curx, cury, curPath = stack.pop(0)
                    if [curx, cury] == sensor_data[SensorData.TARGET_POD_LOCATION]:
                        if len(curPath) > 0:
                            targetx, targety = curPath[1][0], curPath[1][1]
                            break
                    
                    nextX, nextY = curx+1, cury
                    extraFlag = True
                    if self.picked_up_pode and [nextX, nextY] in sensor_data[SensorData.POD_LOCATIONS]:
                        extraFlag = False
                    if [nextX, nextY] not in sensor_data[SensorData.FIELD_BOUNDARIES] and (nextX, nextY) not in visited and extraFlag:
                        flag = [nextX, nextY] in sensor_data[SensorData.DRIVE_LOCATIONS]
                        if flag and len(curPath) > 2:
                            newPath = curPath + [(nextX, nextY)]
                            temp.append((nextX, nextY, newPath))
                            visited.add((nextX, nextY))
                        
                        if not flag:
                            newPath = curPath + [(nextX, nextY)]
                            temp.append((nextX, nextY, newPath))
                            visited.add((nextX, nextY))
                    
                    nextX, nextY = curx-1, cury
                    extraFlag = True
                    if self.picked_up_pode and [nextX, nextY] in sensor_data[SensorData.POD_LOCATIONS]:
                        extraFlag = False
                    if [nextX, nextY] not in sensor_data[SensorData.FIELD_BOUNDARIES] and (nextX, nextY) not in visited and extraFlag:
                        flag = [nextX, nextY] in sensor_data[SensorData.DRIVE_LOCATIONS]
                        if flag and len(curPath) > 2:
                            newPath = curPath + [(nextX, nextY)]
                            temp.append((nextX, nextY, newPath))
                            visited.add((nextX, nextY))
                        
                        if not flag:
                            newPath = curPath + [(nextX, nextY)]
                            temp.append((nextX, nextY, newPath))
                            visited.add((nextX, nextY))

                    nextX, nextY = curx, cury + 1
                    extraFlag = True
                    if self.picked_up_pode and [nextX, nextY] in sensor_data[SensorData.POD_LOCATIONS]:
                        extraFlag = False
                    if [nextX, nextY] not in sensor_data[SensorData.FIELD_BOUNDARIES] and (nextX, nextY) not in visited and extraFlag:
                        flag = [nextX, nextY] in sensor_data[SensorData.DRIVE_LOCATIONS]
                        if flag and len(curPath) > 2:
                            newPath = curPath + [(nextX, nextY)]
                            temp.append((nextX, nextY, newPath))
                            visited.add((nextX, nextY))
                        
                        if not flag:
                            newPath = curPath + [(nextX, nextY)]
                            temp.append((nextX, nextY, newPath))
                            visited.add((nextX, nextY))
                    
                    nextX, nextY = curx, cury - 1
                    extraFlag = True
                    if self.picked_up_pode and [nextX, nextY] in sensor_data[SensorData.POD_LOCATIONS]:
                        extraFlag = False
                    if [nextX, nextY] not in sensor_data[SensorData.FIELD_BOUNDARIES] and (nextX, nextY) not in visited and extraFlag:
                        flag = [nextX, nextY] in sensor_data[SensorData.DRIVE_LOCATIONS]
                        if flag and len(curPath) > 2:
                            newPath = curPath + [(nextX, nextY)]
                            temp.append((nextX, nextY, newPath))
                            visited.add((nextX, nextY))
                        
                        if not flag:
                            newPath = curPath + [(nextX, nextY)]
                            temp.append((nextX, nextY, newPath))
                            visited.add((nextX, nextY))

                if targetx != -1 and targety != -1:
                    break
                
                stack = temp
            
            if targetx == -1 and targety == -1:
                return DriveMove.NONE

            if targetx > x:
                return DriveMove.RIGHT
            if targetx < x:
                return DriveMove.LEFT
            if targety > y:
                return DriveMove.UP
            if targety < y:
                return DriveMove.DOWN

        
        currentLocation = sensor_data[SensorData.PLAYER_LOCATION]
        x, y = currentLocation
        if [x, y] in sensor_data[SensorData.GOAL_LOCATIONS]:
            return DriveMove.DROP_POD
        stack = [(x, y, [(x, y)])]
        visited = set()
        visited.add((x, y))
        targetx, targety = -1, -1
        while stack:
            size = len(stack)
            temp = []
            for i in range(size):
                curx, cury, curPath = stack.pop(0)
                if [curx, cury] in sensor_data[SensorData.GOAL_LOCATIONS]:
                    if len(curPath) > 1:

                        targetx, targety = curPath[1][0], curPath[1][1]

                        break
                
                nextX, nextY = curx+1, cury
                extraFlag = True
                if self.picked_up_pode and [nextX, nextY] in sensor_data[SensorData.POD_LOCATIONS]:
                    extraFlag = False
                if [nextX, nextY] not in sensor_data[SensorData.FIELD_BOUNDARIES] and (nextX, nextY) not in visited and extraFlag:
                    flag = [nextX, nextY] in sensor_data[SensorData.DRIVE_LOCATIONS]
                    if flag and len(curPath) > 2:
                        newPath = curPath + [(nextX, nextY)]
                        temp.append((nextX, nextY, newPath))
                        visited.add((nextX, nextY))
                    
                    if not flag:
                        newPath = curPath + [(nextX, nextY)]
                        temp.append((nextX, nextY, newPath))
                        visited.add((nextX, nextY))
                
                nextX, nextY = curx-1, cury
                extraFlag = True
                if self.picked_up_pode and [nextX, nextY] in sensor_data[SensorData.POD_LOCATIONS]:
                    extraFlag = False
                if [nextX, nextY] not in sensor_data[SensorData.FIELD_BOUNDARIES] and (nextX, nextY) not in visited and extraFlag:
                    flag = [nextX, nextY] in sensor_data[SensorData.DRIVE_LOCATIONS]
                    if flag and len(curPath) > 2:
                        newPath = curPath + [(nextX, nextY)]
                        temp.append((nextX, nextY, newPath))
                        visited.add((nextX, nextY))
                    
                    if not flag:
                        newPath = curPath + [(nextX, nextY)]
                        temp.append((nextX, nextY, newPath))
                        visited.add((nextX, nextY))

                nextX, nextY = curx, cury + 1
                extraFlag = True
                if self.picked_up_pode and [nextX, nextY] in sensor_data[SensorData.POD_LOCATIONS]:
                    extraFlag = False
                if [nextX, nextY] not in sensor_data[SensorData.FIELD_BOUNDARIES] and (nextX, nextY) not in visited and extraFlag:
                    flag = [nextX, nextY] in sensor_data[SensorData.DRIVE_LOCATIONS]
                    if flag and len(curPath) > 2:
                        newPath = curPath + [(nextX, nextY)]
                        temp.append((nextX, nextY, newPath))
                        visited.add((nextX, nextY))
                    
                    if not flag:
                        newPath = curPath + [(nextX, nextY)]
                        temp.append((nextX, nextY, newPath))
                        visited.add((nextX, nextY))
                
                nextX, nextY = curx, cury - 1
                extraFlag = True
                if self.picked_up_pode and [nextX, nextY] in sensor_data[SensorData.POD_LOCATIONS]:
                    extraFlag = False
                if [nextX, nextY] not in sensor_data[SensorData.FIELD_BOUNDARIES] and (nextX, nextY) not in visited and extraFlag:
                    flag = [nextX, nextY] in sensor_data[SensorData.DRIVE_LOCATIONS]
                    if flag and len(curPath) > 2:
                        newPath = curPath + [(nextX, nextY)]
                        temp.append((nextX, nextY, newPath))
                        visited.add((nextX, nextY))
                    
                    if not flag:
                        newPath = curPath + [(nextX, nextY)]
                        temp.append((nextX, nextY, newPath))
                        visited.add((nextX, nextY))

            if targetx != -1 and targety != -1:
                break
            
            stack = temp
        
        if targetx == -1 and targety == -1:
            return DriveMove.NONE

        if targetx > x:
            return DriveMove.RIGHT
        if targetx < x:
            return DriveMove.LEFT
        if targety > y:
            return DriveMove.UP
        if targety < y:
            return DriveMove.DOWN
        
