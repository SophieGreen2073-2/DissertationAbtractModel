class BatteryChargeStation():
    def __init__(self, ChargeTime, NumBatteries, time_step):
        self.charge_time = ChargeTime
        self.num_batteries = NumBatteries
        self.batteries = []
        self.time_step = time_step

        # for _ in range(self.num_batteries):
            # self.batteries.append(Battery())

    def ChargeBatteries(self, time_step):
        for battery in self.batteries:
            battery.charge(time_step)

    def AddBattery(self, battery):
        self.batteries.append(battery)
        battery.location = "charge_station"

    def RemoveBattery(self, battery):
        self.batteries.remove(battery)
        # uav.battery = battery
        battery.location = "uav"
        battery.is_charged = False

class Battery():
    def __init__(self, battery_life, charge_time):
        self.is_charged = False
        self.total_charge_time = 0
        self.battery_life = battery_life
        self.charge_time = charge_time
        self.mission_time = 0

    def charge(self, time_step):
        if not self.is_charged:
            self.total_charge_time += time_step
            if round(self.total_charge_time, 1) >= self.charge_time:
                self.is_charged = True

    def drain(self, time_step):
        self.mission_time += time_step
        
    